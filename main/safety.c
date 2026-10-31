/* PRESENTER 5: safety.c
 * See presenters/ for explanation and demo notes.
 */
#include "shared.h"

/* ================= SECTION: state.c ================= */
#include "nvs.h"
#include "nvs_flash.h"

/* One versioned blob keeps users, schedule and occupancy in the same checkpoint.
   This is the single-ESP32 design; all state owners use state_mutex. */
typedef struct {
    uint32_t magic;
    uint32_t version;
    int open_min;
    int close_min;
    int rooms[2];
    user_t people[MAX_USERS];
} checkpoint_t;

static checkpoint_t saved;
static nvs_handle_t state_nvs;
static bool ready;

void state_lock(void) { xSemaphoreTakeRecursive(state_mutex, portMAX_DELAY); }
void state_unlock(void) { xSemaphoreGiveRecursive(state_mutex); }
bool state_healthy(void) { return ready; }

static checkpoint_t capture(void)
{
    checkpoint_t c = { .magic = 0x434F4349, .version = 1,
        .open_min = sched_open_min, .close_min = sched_close_min,
        .rooms = { master_room[0], master_room[1] } };
    memcpy(c.people, users, sizeof(users));
    return c;
}

static bool valid(const checkpoint_t *c)
{
    if (c->magic != 0x434F4349 || c->version != 1 ||
        c->open_min < 0 || c->open_min >= 1440 || c->close_min < 0 ||
        c->close_min >= 1440 || c->open_min == c->close_min ||
        c->rooms[0] < 0 || c->rooms[0] > MAX_USERS ||
        c->rooms[1] < 0 || c->rooms[1] > MAX_USERS) return false;
    for (int i = 0; i < MAX_USERS; i++) {
        if (!c->people[i].used) continue;
        if (!memchr(c->people[i].name, 0, sizeof(c->people[i].name))) return false;
        for (int j = 0; j < i; j++)
            if (c->people[j].used && !memcmp(c->people[j].uid, c->people[i].uid, 4)) return false;
    }
    /* Room sensors can miss movement. Preserve discrepancies for inspection;
       they must not invalidate a verified main-door exit checkpoint. */
    return true;
}

static void restore(const checkpoint_t *c)
{
    sched_open_min = c->open_min;
    sched_close_min = c->close_min;
    memcpy(master_room, c->rooms, sizeof(master_room));
    memcpy(users, c->people, sizeof(users));
}

bool state_commit(void)
{
    state_lock();
    checkpoint_t next = capture();
    esp_err_t err = ESP_FAIL;
    if (ready && valid(&next)) {
        err = nvs_set_blob(state_nvs, "checkpoint", &next, sizeof(next));
        if (err == ESP_OK) err = nvs_commit(state_nvs);
    }
    if (err == ESP_OK) saved = next;
    else {
        restore(&saved);
        /* A failed flash write requires inspection/restart. Do not keep granting
           doors while the durable outcome of a failed write is uncertain. */
        ready = false;
        passage_cancel_all("state checkpoint failed", true);
        door_close(&entry_door);
        door_close(&exit_door);
        printf("ERROR: state save failed (%s); change rolled back, grants disabled\n", esp_err_to_name(err));
    }
    state_unlock();
    return err == ESP_OK;
}

bool state_init(void)
{
    saved = capture();
    ready = false;
    esp_err_t err = nvs_flash_init();
    /* Never silently erase NVS when it cannot be read. */
    if (err != ESP_OK) return false;
    if (nvs_open("smartguard", NVS_READWRITE, &state_nvs) != ESP_OK) return false;
    checkpoint_t stored = {0};
    size_t size = sizeof(stored);
    err = nvs_get_blob(state_nvs, "checkpoint", &stored, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ready = true;
        return state_commit();
    }
    if (err != ESP_OK || size != sizeof(stored) || !valid(&stored)) return false;
    saved = stored;
    restore(&saved);
    ready = true;
    printf("STATE: restored %d people, rooms %d/%d and saved schedule\n",
           inside_count(), master_room[0], master_room[1]);
    return true;
}

/* ================= SECTION: storage.c ================= */

/* ================= STORAGE (SD card, owned by storage_task) ================= */
static bool sd_append_line(int file, const char *line);

static void store_append(int file, const char *line)
{
    bool ok = false;
    if (sd_ok) {
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        ok = sd_append_line(file, line);
        xSemaphoreGive(spi_mutex);
    }
    if (!ok) {
        atomic_fetch_add(&log_failures, 1);
        printf("LOG BACKUP (SD write unavailable): %s\n", line);
    }
}

bool store_request(uint8_t kind, int file, int n, const char *text)
{
    if (file < 0 || file >= 5 ||
        (kind != SQ_TAIL && kind != SQ_EXPORT && kind != SQ_FIND && kind != SQ_LOGLCD)) return false;
    store_item_t it = {
        .kind = kind,
        .file = (uint8_t)file,
        .n = n,
    };
    if (text) strncpy(it.text, text, sizeof(it.text) - 1);
    if (xQueueSend(storage_q, &it, pdMS_TO_TICKS(50)) != pdTRUE) {
        printf("ERROR: storage query queue full; try again\n");
        return false;
    }
    return true;
}

void store_write(int file, char *heap_text)
{
    if (!heap_text) return;
    bool ok = false;
    if (sd_ok && file >= 0 && file < 5) {
        xSemaphoreTake(spi_mutex, portMAX_DELAY);
        FILE *f = fopen(file_paths[file], "w");
        if (f) {
            ok = fputs(heap_text, f) >= 0;
            if (fclose(f) != 0) ok = false;
        }
        xSemaphoreGive(spi_mutex);
    }
    if (!ok) {
        atomic_fetch_add(&log_failures, 1);
        printf("WARN: SD export failed; authoritative configuration is retained in NVS\n");
    }
    free(heap_text);
}

void log_event(int file, const char *fmt, ...)
{
    char ts[24];
    fmt_time(ts, sizeof(ts));

    char body[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    char line[320];
    snprintf(line, sizeof(line), "%s,%s", ts, body);
    store_append(file, line);
}

static bool file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

static bool sd_append_line(int file, const char *line)
{
    if (file < 0 || file >= 5) return false;
    const char *path = file_paths[file];
    bool fresh = !file_exists(path);
    FILE *f = fopen(path, "a");
    if (!f) return false;
    bool ok = true;
    if (fresh && file < 3 && fprintf(f, "%s\n", file_headers[file]) < 0) ok = false;
    if (fprintf(f, "%s\n", line) < 0) ok = false;
    if (fclose(f) != 0) ok = false;
    return ok;
}

/* Long log queries lock SPI only for each filesystem operation. Printing an
   export must not monopolize the RFID bus or block room updates. */
static FILE *query_open(const char *path, const char *mode)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    FILE *f = fopen(path, mode);
    xSemaphoreGive(spi_mutex);
    return f;
}

static size_t query_read(void *data, size_t size, size_t n, FILE *f)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    size_t got = fread(data, size, n, f);
    xSemaphoreGive(spi_mutex);
    return got;
}

static int query_getc(FILE *f)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    int value = fgetc(f);
    xSemaphoreGive(spi_mutex);
    return value;
}

static int query_seek(FILE *f, long offset, int origin)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    int result = fseek(f, offset, origin);
    xSemaphoreGive(spi_mutex);
    return result;
}

static int query_close(FILE *f)
{
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    int result = fclose(f);
    xSemaphoreGive(spi_mutex);
    return result;
}

static void sd_tail(int file, int n)
{
    static uint32_t ring[100];
    FILE *f = query_open(file_paths[file], "r");
    if (!f) {
        printf("No data yet: %s does not exist\n", file_paths[file]);
        return;
    }
    if (n < 1) n = 1;
    if (n > 100) n = 100;

    uint32_t total = 0, pos = 0;
    bool at_start = true;
    uint8_t buf[128];
    int r;
    while ((r = (int)query_read(buf, 1, sizeof(buf), f)) > 0) {
        for (int i = 0; i < r; i++) {
            if (at_start) { ring[total % (uint32_t)n] = pos; total++; at_start = false; }
            if (buf[i] == '\n') at_start = true;
            pos++;
        }
    }
    if (total == 0) {
        printf("%s is empty\n", file_paths[file]);
        query_close(f);
        return;
    }
    uint32_t start = (total > (uint32_t)n) ? ring[total % (uint32_t)n] : ring[0];
    query_seek(f, (long)start, SEEK_SET);
    printf("--- %s (last %d lines) ---\n", file_paths[file],
           (int)(total < (uint32_t)n ? total : (uint32_t)n));
    while ((r = (int)query_read(buf, 1, sizeof(buf), f)) > 0) fwrite(buf, 1, (size_t)r, stdout);
    printf("--- end ---\n");
    query_close(f);
}

static void sd_export(int file)
{
    FILE *f = query_open(file_paths[file], "r");
    if (!f) {
        printf("No data yet: %s does not exist\n", file_paths[file]);
        return;
    }
    printf("--- BEGIN %s ---\n", file_paths[file]);
    uint8_t buf[128];
    int r;
    while ((r = (int)query_read(buf, 1, sizeof(buf), f)) > 0) fwrite(buf, 1, (size_t)r, stdout);
    printf("\n--- END %s ---\n", file_paths[file]);
    query_close(f);
}

static void sd_find(int file, const char *needle)
{
    FILE *f = query_open(file_paths[file], "r");
    if (!f) {
        printf("No data yet: %s does not exist\n", file_paths[file]);
        return;
    }
    printf("--- search '%s' in %s ---\n", needle, file_paths[file]);
    char line[512];
    int len = 0, hits = 0;
    int c;
    while ((c = query_getc(f)) != EOF) {
        if (c == '\r') continue;
        if (c == '\n' || len >= (int)sizeof(line) - 1) {
            line[len] = 0;
            if (len > 0 && strstr(line, needle)) { printf("%s\n", line); hits++; }
            len = 0;
        } else {
            line[len++] = (char)c;
        }
    }
    line[len] = 0;
    if (len > 0 && strstr(line, needle)) { printf("%s\n", line); hits++; }
    printf("--- %d match(es) ---\n", hits);
    query_close(f);
}

static void publish_lcd_log(int count, char rows[8][2][17])
{
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    lcdlog_n = count;
    if (rows) memcpy(lcdlog, rows, sizeof(lcdlog));
    lcdlog_ready = true;
    xSemaphoreGive(log_mutex);
}

/* Reads the newest access records and formats them for a 16x2 screen. */
static void sd_log_to_lcd(void)
{
    static char ring[8][320];
    int count = 0;
    char rows[8][2][17] = {0};

    FILE *f = query_open(file_paths[F_ACCESS], "r");
    if (!f) {
        publish_lcd_log(0, NULL);
        return;
    }

    char line[320];
    int len = 0;
    uint32_t total = 0;
    int c;
    while ((c = query_getc(f)) != EOF) {
        if (c == '\r') continue;
        if (c == '\n' || len >= (int)sizeof(line) - 1) {
            line[len] = 0;
            if (len > 0 && strncmp(line, "time,", 5) != 0) {
                strcpy(ring[total % 8], line);
                total++;
            }
            len = 0;
        } else {
            line[len++] = (char)c;
        }
    }
    line[len] = 0;
    if (len > 0 && strncmp(line, "time,", 5) != 0) {
        strcpy(ring[total % 8], line);
        total++;
    }
    query_close(f);

    int n = (total < 8) ? (int)total : 8;
    for (int i = 0; i < n; i++) {
        char tmp[320];
        strcpy(tmp, ring[(total - 1 - (uint32_t)i) % 8]);

        char *fld[7] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL };
        int nf = 0;
        char *p = tmp;
        while (nf < 7) {
            fld[nf++] = p;
            char *comma = strchr(p, ',');
            if (!comma) break;
            *comma = 0;
            p = comma + 1;
        }

        const char *ts     = fld[0] ? fld[0] : "";
        const char *point  = (nf > 1 && fld[1]) ? fld[1] : "";
        const char *name   = (nf > 3 && fld[3]) ? fld[3] : "";
        const char *result = (nf > 4 && fld[4]) ? fld[4] : "";
        const char *detail = (nf > 5 && fld[5]) ? fld[5] : "";

        char hm[6] = "--:--";
        if (strlen(ts) >= 16) {
            memcpy(hm, ts + 11, 5);
            hm[5] = 0;
        }
        bool ok = (strcmp(result, "GRANTED") == 0 || strcmp(result, "CONFIRMED") == 0);
        snprintf(rows[i][0], 17, "%s %s %s", hm,
                 strcmp(point, "EXIT") == 0 ? "OUT" : "IN", ok ? "OK" : (strcmp(result, "PENDING") == 0 ? "WAIT" : "CHECK"));
        snprintf(rows[i][1], 17, "%.16s", ok ? name : detail);
        count++;
    }
    publish_lcd_log(count, rows);
}

void storage_task(void *arg)
{
    (void)arg;
    for (;;) {
        store_item_t it;
        if (xQueueReceive(storage_q, &it, portMAX_DELAY) != pdTRUE) continue;
        if (!sd_ok) {
            if (it.kind == SQ_LOGLCD) publish_lcd_log(0, NULL);
            else printf("SD card not available\n");
            continue;
        }
        switch (it.kind) {
        case SQ_TAIL: sd_tail(it.file, (int)it.n); break;
        case SQ_EXPORT: sd_export(it.file); break;
        case SQ_FIND: sd_find(it.file, it.text); break;
        case SQ_LOGLCD: sd_log_to_lcd(); break;
        default: printf("ERROR: unsupported storage query\n"); break;
        }
    }
}

/* ================= SD CARD (sdspi + FATFS, mounted at /sdcard) ================= */
bool sd_mount(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI_HOST_ID;
    host.max_freq_khz = 4000;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = (gpio_num_t)SD_CS_PIN;
    slot.host_id = SPI_HOST_ID;

    esp_err_t err = esp_vfs_fat_sdspi_mount(SD_MOUNT, &host, &slot, &mc, &sd_card);
    if (err != ESP_OK) {
        printf("SD card: mount failed (%s)\n", esp_err_to_name(err));
        return false;
    }
    mkdir(sd_directory, 0775);
    return true;
}


/* ================= SECTION: emergency.c ================= */
#include "nvs.h"
#include "nvs_flash.h"

/* Alarm output never waits for state_mutex, NVS or SD logging. The high-priority
   task polls inputs and drives actuators; the entrance task services persistence.
   No prototype polling loop is a substitute for certified fire/egress hardware. */
typedef struct {
    uint32_t magic, version, active, uncertain, sources;
} alarm_record_t;
static _Atomic bool initialized;
static _Atomic bool latched;
static _Atomic bool uncertain;
static uint32_t sources, revision, serviced_revision, reported_revision, last_inputs;
static uint32_t last_save_attempt;
static bool save_attempted;
static _Atomic bool menu_visible;
static uint32_t clear_since;
static bool clear_timing;
static bool nvs_available;
static nvs_handle_t alarm_nvs;
static bool save_fault;

static uint32_t input_sources(void)
{
    uint32_t value = 0;
    if (pin_get(EMERGENCY_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL) value |= EM_SOURCE_BUTTON;
    if (pin_get(SMOKE_ALARM_PIN) == SMOKE_ACTIVE_LEVEL) value |= EM_SOURCE_SMOKE;
    return value;
}

bool emergency_active(void) { return atomic_load(&latched); }
bool emergency_counts_uncertain(void) { return atomic_load(&uncertain); }
bool emergency_exit_required(void)
{
    return emergency_active() || (atomic_load(&initialized) && input_sources() != 0);
}

void emergency_set_menu_visible(bool visible) { atomic_store(&menu_visible, visible); }
bool emergency_menu_visible(void) { return atomic_load(&menu_visible); }

static alarm_record_t snapshot(void)
{
    alarm_record_t r = { 0x454D4552, 1, emergency_active(), emergency_counts_uncertain(), sources };
    return r;
}

/* Only startup and the entrance task call NVS functions in this module. */
static bool persist(const alarm_record_t *r)
{
    if (!nvs_available) return false;
    esp_err_t err = nvs_set_blob(alarm_nvs, "evacuation", r, sizeof(*r));
    if (err == ESP_OK) err = nvs_commit(alarm_nvs);
    save_fault = err != ESP_OK;
    return !save_fault;
}

void emergency_poll(void)
{
    if (!atomic_load(&initialized)) return;
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    uint32_t inputs = input_sources();
    if (inputs) {
        if (!emergency_active()) {
            sources = inputs; /* A new incident must not inherit old trigger sources. */
            revision++;
        } else if (inputs & ~last_inputs) {
            sources |= inputs;
            revision++;
        }
        atomic_store(&latched, true);
        atomic_store(&uncertain, true);
        clear_timing = false;
    } else if (!clear_timing) {
        clear_since = now_ms();
        clear_timing = true;
    }
    last_inputs = inputs;
    xSemaphoreGive(emergency_mutex);
    /* These operations take only output_mutex, never a storage/state lock. */
    door_update(&entry_door);
    door_update(&exit_door);
    buzzer_update();
}

void emergency_task(void *arg)
{
    (void)arg;
    for (;;) {
        passage_sample();
        emergency_poll();
        TickType_t ticks = pdMS_TO_TICKS(EMERGENCY_POLL_MS);
        vTaskDelay(ticks ? ticks : 1);
    }
}

void emergency_init(void)
{
    atomic_store(&initialized, false);
    pin_input(EMERGENCY_BUTTON_PIN, BUTTON_ACTIVE_LEVEL == 0);
    pin_input(SMOKE_ALARM_PIN, SMOKE_ACTIVE_LEVEL == 0);
    if (BUTTON_ACTIVE_LEVEL != 0)
        ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)EMERGENCY_BUTTON_PIN, GPIO_PULLDOWN_ONLY));
    if (SMOKE_ACTIVE_LEVEL != 0)
        ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)SMOKE_ALARM_PIN, GPIO_PULLDOWN_ONLY));
    sources = 0;
    revision = 1;
    serviced_revision = 0;
    reported_revision = 0;
    last_inputs = 0;
    save_attempted = false;
    atomic_store(&menu_visible, false);
    clear_timing = false;
    save_fault = false;
    atomic_store(&latched, false);
    atomic_store(&uncertain, false);
    nvs_available = nvs_flash_init() == ESP_OK &&
        nvs_open("smartguard_em", NVS_READWRITE, &alarm_nvs) == ESP_OK;
    if (nvs_available) {
        alarm_record_t r = {0};
        size_t size = sizeof(r);
        esp_err_t err = nvs_get_blob(alarm_nvs, "evacuation", &r, &size);
        if (err == ESP_OK && size == sizeof(r) && r.magic == 0x454D4552 && r.version == 1 &&
            r.active <= 1 && r.uncertain <= 1 && (r.sources & ~7U) == 0 && (!r.active || r.uncertain)) {
            atomic_store(&latched, r.active != 0);
            atomic_store(&uncertain, r.uncertain != 0);
            sources = r.sources;
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            save_fault = true;
        }
    } else save_fault = true;
    if (save_fault) {
        /* Unknown saved alarm state must not force the exit shut. */
        atomic_store(&latched, true);
        atomic_store(&uncertain, true);
        sources |= EM_SOURCE_STORAGE;
    }
    atomic_store(&initialized, true);
    emergency_poll();
}

void emergency_service(void)
{
    if (!atomic_load(&initialized)) return;
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    uint32_t current_revision = revision;
    alarm_record_t r = snapshot();
    xSemaphoreGive(emergency_mutex);
    if (current_revision != reported_revision) {
        /* Audit even if NVS fails; this may fall back to the serial log. */
        log_event(F_SYSTEM, "EMERGENCY,%s,button=%u smoke=%u state_fault=%u,occupancy=%s",
                  r.active ? "ACTIVE" : (r.uncertain ? "STATE_UNVERIFIED" : "NORMAL"), !!(r.sources & EM_SOURCE_BUTTON),
                  !!(r.sources & EM_SOURCE_SMOKE), !!(r.sources & EM_SOURCE_STORAGE),
                  r.uncertain ? "UNVERIFIED" : "reconciled");
        reported_revision = current_revision;
    }
    if (current_revision == serviced_revision) return;
    if (save_fault && save_attempted && now_ms() - last_save_attempt < 1000U) return;
    save_attempted = true;
    last_save_attempt = now_ms();
    if (!persist(&r)) {
        save_fault = true;
        printf("WARN: emergency-state save failed; exit response remains active\n");
        return;
    }
    serviced_revision = current_revision;
}

bool emergency_reset(void)
{
    /* UI calls this under state_mutex; reset is never triggered by releasing a button. */
    if (!admin_check()) return false;
    emergency_poll();
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    bool clear = !input_sources() && clear_timing &&
        now_ms() - clear_since >= (uint32_t)EMERGENCY_CLEAR_MS;
    uint32_t before = revision;
    alarm_record_t r = snapshot();
    xSemaphoreGive(emergency_mutex);
    if (!r.active) { printf("Emergency is not active\n"); return false; }
    if (!clear) { printf("DENIED: release button and clear smoke input for %d ms\n", EMERGENCY_CLEAR_MS); return false; }
    r.active = 0; /* Preserve uncertainty and last-known occupants. */
    if (!persist(&r)) { printf("DENIED: cannot save emergency reset; exit stays open\n"); return false; }
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    if (input_sources() || revision != before || !clear_timing ||
        now_ms() - clear_since < (uint32_t)EMERGENCY_CLEAR_MS) {
        revision++; /* Ensure the active snapshot is written again. */
        xSemaphoreGive(emergency_mutex);
        printf("DENIED: emergency trigger changed during reset\n");
        return false;
    }
    atomic_store(&latched, false);
    revision++;
    xSemaphoreGive(emergency_mutex);
    buzzer_pattern(0, 0, 0);
    door_close(&exit_door);
    buzzer_update();
    log_event(F_SYSTEM, "EMERGENCY,admin reset; occupancy remains UNVERIFIED");
    printf("Emergency reset. Check the assembly point; counts remain UNVERIFIED.\n");
    return true;
}

bool emergency_reconcile_empty(void)
{
    if (!admin_check()) return false;
    emergency_poll();
    if (emergency_exit_required() || !state_healthy()) {
        printf("DENIED: reset emergency and resolve state fault first\n"); return false;
    }
    if (!passage_reconcile_clear()) return false;
    /* The command explicitly confirms a physical empty-building check. */
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    uint32_t before = revision;
    xSemaphoreGive(emergency_mutex);
    for (int i = 0; i < MAX_USERS; i++) { users[i].inside = false; users[i].since = 0; }
    master_room[0] = master_room[1] = 0;
    if (!state_commit()) return false;
    room1.count = room2.count = 0;
    room1.inject = room2.inject = 0;
    room1.seq = room2.seq = RS_CLEARING;
    room1.seq_since_ms = room2.seq_since_ms = now_ms();
    room1.fault_sent = room2.fault_sent = false;
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    alarm_record_t r = snapshot();
    xSemaphoreGive(emergency_mutex);
    if (r.active || emergency_exit_required()) {
        printf("DENIED: emergency reactivated; counts remain unverified\n"); return false;
    }
    r.uncertain = 0;
    if (!persist(&r)) { printf("ERROR: empty state saved, uncertainty flag could not be cleared\n"); return false; }
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    bool unchanged = revision == before && !input_sources() && !emergency_active();
    if (unchanged) atomic_store(&uncertain, false);
    revision++;
    xSemaphoreGive(emergency_mutex);
    if (!unchanged) { printf("WARNING: new emergency; occupancy remains unverified\n"); return false; }
    checklist_invalidate();
    log_event(F_SYSTEM, "EMERGENCY,admin confirmed building EMPTY; occupancy reconciled");
    printf("Occupancy reconciled after administrator empty-building confirmation.\n");
    return true;
}

void emergency_print_status(void)
{
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    alarm_record_t r = snapshot();
    bool smoke = pin_get(SMOKE_ALARM_PIN) == SMOKE_ACTIVE_LEVEL;
    bool button = pin_get(EMERGENCY_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL;
    xSemaphoreGive(emergency_mutex);
    printf("Emergency: %s; button=%s smoke=%s; saved sources: button=%u smoke=%u state_fault=%u\n",
           r.active ? "ACTIVE - EXIT HELD OPEN" : "inactive", button ? "PRESSED" : "clear",
           smoke ? "ALARM" : "clear", !!(r.sources & EM_SOURCE_BUTTON),
           !!(r.sources & EM_SOURCE_SMOKE), !!(r.sources & EM_SOURCE_STORAGE));
    printf("Occupancy: %s; alarm-state persistence: %s\n", r.uncertain ? "UNVERIFIED - reconcile records" : "recorded",
           save_fault ? "FAULT" : "OK");
}

bool emergency_display(int id, char line1[17], char line2[17])
{
    if (!emergency_exit_required() || (id == LCD_ENTRY && emergency_menu_visible())) return false;
    snprintf(line1, 17, "%-16.16s", id == LCD_EXIT ? "EMERGENCY EXIT" : "EVACUATE NOW");
    snprintf(line2, 17, "%-16.16s", id == LCD_EXIT ? "OPEN - NO RFID" : "USE MAIN EXIT");
    return true;
}

void occupancy_mark_uncertain(const char *reason)
{
    xSemaphoreTake(emergency_mutex, portMAX_DELAY);
    atomic_store(&uncertain, true);
    revision++;
    xSemaphoreGive(emergency_mutex);
    printf("OCCUPANCY UNVERIFIED: %s\n", reason);
    log_event(F_SYSTEM, "OCCUPANCY_UNVERIFIED,%s", reason);
}

/* ================= SECTION: checklist.c ================= */
static _Atomic int result;
static uint32_t due;
static uint32_t reasons;
enum { C_PEOPLE=1, C_ROOMS=2, C_LIGHTS=4, C_OFFLINE=8, C_UNVERIFIED=16, C_STATE=32, C_PASSAGE=64 };
int checklist_status(void) { return atomic_load(&result); }
void checklist_invalidate(void) { atomic_store(&result, CHECKLIST_IDLE); }
void checklist_start(void) { due = now_ms() + CHECKLIST_SETTLE_MS; atomic_store(&result, CHECKLIST_WAITING); }
const char *checklist_summary(void)
{
    if (reasons & C_UNVERIFIED) return "Count unverified";
    if (reasons & C_STATE) return "System down";
    if (reasons & C_PASSAGE) return "Doorway busy";
    if (reasons & C_PEOPLE) return "People recorded";
    if (reasons & C_ROOMS) return "Room mismatch";
    if (reasons & C_LIGHTS) return "Lights still ON";
    if (reasons & C_OFFLINE) return "Room offline";
    return "Records checked";
}
static bool room_online(int i)
{
    return !room_offline[i] && room_seen_ms[i] && now_ms() - room_seen_ms[i] <= (uint32_t)ROOM_OFFLINE_MS;
}
static void evaluate(void)
{
    reasons = 0;
    if (inside_count() != 0) reasons |= C_PEOPLE;
    if (master_room[0] || master_room[1] || room1.count || room2.count) reasons |= C_ROOMS;
    if (room_lights[0] || room_lights[1] || room1.lights_on || room2.lights_on) reasons |= C_LIGHTS;
    for (int i = 0; i < 2; i++) if (!room_online(i)) reasons |= C_OFFLINE;
    if (emergency_exit_required() || emergency_counts_uncertain()) reasons |= C_UNVERIFIED;
    if (!state_healthy() || !passage_healthy()) reasons |= C_STATE;
    if (passage_waiting(false) || passage_waiting(true) || passage_sensor_blocked(false) || passage_sensor_blocked(true)) reasons |= C_PASSAGE;
    atomic_store(&result, reasons ? CHECKLIST_REQUIRED : CHECKLIST_CLEAR);
}
static void publish(void)
{
    const char *headline = reasons ? "CHECK REQUIRED" : "BUILDING CLEAR";
    if (reasons) lcd_msg(LCD_ENTRY, headline, checklist_summary());
    lcd_msg(LCD_EXIT, headline, checklist_summary());
    printf("LAST PERSON OUT: %s (%s)\n", headline, checklist_summary());
    printf("  building=%d rooms=%d/%d cached=%d/%d lights=%s/%s room_online=%s/%s\n",
        inside_count(), master_room[0], master_room[1], room1.count, room2.count,
        room1.lights_on ? "ON" : "OFF", room2.lights_on ? "ON" : "OFF",
        room_online(0) ? "yes" : "no", room_online(1) ? "yes" : "no");
    if (reasons & C_PEOPLE) printf("  FAIL: people remain in the building records\n");
    if (reasons & C_ROOMS) printf("  FAIL: room counts are nonzero or disagree with the master\n");
    if (reasons & C_LIGHTS) printf("  FAIL: room light commands/cache are not all OFF (cache=%d/%d)\n", room_lights[0], room_lights[1]);
    if (reasons & C_OFFLINE) printf("  FAIL: a room heartbeat is missing, stale or offline\n");
    if (reasons & C_UNVERIFIED) printf("  FAIL: emergency or unverified occupancy requires reconciliation\n");
    if (reasons & C_STATE) printf("  FAIL: checkpoint or passage journal is unhealthy\n");
    if (reasons & C_PASSAGE) printf("  FAIL: a passage is pending or a beam is blocked\n");
    log_event(F_SYSTEM, "CHECKLIST,%s,flags=%lu,%s", headline, (unsigned long)reasons, checklist_summary());
}
void checklist_run(void) { evaluate(); publish(); }
void checklist_poll(void)
{
    if (checklist_status() == CHECKLIST_WAITING && (int32_t)(now_ms() - due) >= 0) checklist_run();
    else if (checklist_status() == CHECKLIST_CLEAR) {
        evaluate();
        if (checklist_status() == CHECKLIST_REQUIRED) publish();
    }
}

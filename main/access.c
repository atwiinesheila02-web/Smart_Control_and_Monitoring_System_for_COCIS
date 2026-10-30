/* PRESENTER 2: access.c
 * See presenters/ for explanation and demo notes.
 */
#include "shared.h"

/* ================= SECTION: rfid.c ================= */

/* ================= RFID (spi_master, bus shared with the SD card) ================= */
static void write_reg(reader_t *r, uint8_t reg, uint8_t value)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,
    };
    t.tx_data[0] = (uint8_t)((reg << 1) & 0x7E);
    t.tx_data[1] = value;
    spi_device_polling_transmit(r->dev, &t);
}

static uint8_t read_reg(reader_t *r, uint8_t reg)
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
    };
    t.tx_data[0] = (uint8_t)(0x80 | ((reg << 1) & 0x7E));
    t.tx_data[1] = 0;
    if (spi_device_polling_transmit(r->dev, &t) != ESP_OK) return 0;
    return t.rx_data[1];
}

void setup_reader(reader_t *r, const char *name, door_t *door,
                         bool is_exit, int cs)
{
    r->name = name;
    r->door = door;
    r->is_exit = is_exit;
    r->cs = cs;

    spi_device_interface_config_t dc = {
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = cs,
        .queue_size = 1,
    };
    if (spi_bus_add_device(SPI_HOST_ID, &dc, &r->dev) != ESP_OK) {
        printf("%s: cannot attach the reader to the SPI bus\n", r->name);
        r->available = false;
        return;
    }

    write_reg(r, COMMAND_REG, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t version = read_reg(r, VERSION_REG);
    printf("%s reader version: 0x%02X\n", r->name, version);

    r->available = version != 0x00 && version != 0xFF;
    if (!r->available) {
        printf("%s: no valid response. Check power, SPI wires, CS and RST.\n", r->name);
        return;
    }

    write_reg(r, T_MODE_REG, 0x80);
    write_reg(r, T_PRESCALER_REG, 0xA9);
    write_reg(r, T_RELOAD_H_REG, 0x03);
    write_reg(r, T_RELOAD_L_REG, 0xE8);
    write_reg(r, TX_ASK_REG, 0x40);
    write_reg(r, MODE_REG, 0x3D);
    uint8_t antenna = read_reg(r, TX_CONTROL_REG);
    write_reg(r, TX_CONTROL_REG, antenna | 0x03);
    printf("%s reader initialised.\n", r->name);
}

static int exchange(reader_t *r, const uint8_t *data, int length, uint8_t last_bits,
                    uint8_t *reply, int capacity)
{
    write_reg(r, COMMAND_REG, 0x00);
    write_reg(r, COM_IRQ_REG, 0x7F);
    write_reg(r, FIFO_LEVEL_REG, 0x80);
    for (int i = 0; i < length; i++) write_reg(r, FIFO_DATA_REG, data[i]);
    write_reg(r, BIT_FRAMING_REG, last_bits);
    write_reg(r, COMMAND_REG, 0x0C);
    write_reg(r, BIT_FRAMING_REG, 0x80 | last_bits);

    int64_t deadline = esp_timer_get_time() + 40000;
    bool received = false;
    while (esp_timer_get_time() < deadline) {
        uint8_t irq = read_reg(r, COM_IRQ_REG);
        if (irq & 0x20) { received = true; break; }
        if (irq & 0x01) { break; }
        vTaskDelay(1);
    }

    write_reg(r, BIT_FRAMING_REG, last_bits);
    write_reg(r, COMMAND_REG, 0x00);
    if (!received) return -1;
    if (read_reg(r, ERROR_REG) & 0x13) return -1;

    int count = read_reg(r, FIFO_LEVEL_REG);
    if (count <= 0 || count > capacity) return -1;
    for (int i = 0; i < count; i++) reply[i] = read_reg(r, FIFO_DATA_REG);
    return count;
}

/* 1 = 4-byte UID read, 0 = no usable response, -1 = longer UID */
static int read_uid(reader_t *r, uint8_t uid[4])
{
    uint8_t reply[10];
    write_reg(r, COLL_REG, read_reg(r, COLL_REG) & (uint8_t)~0x80);

    uint8_t cmd = 0x52;
    int count = exchange(r, &cmd, 1, 7, reply, sizeof(reply));
    if (count != 2) return 0;

    uint8_t anticollision[2] = { 0x93, 0x20 };
    count = exchange(r, anticollision, 2, 0, reply, sizeof(reply));
    if (count != 5) return 0;

    if (!SIM_NO_BCC) {
    uint8_t check = reply[0] ^ reply[1] ^ reply[2] ^ reply[3];
    if (check != reply[4]) return 0;
    }

    if (reply[0] == 0x88) return -1;
    memcpy(uid, reply, 4);
    return 1;
}

void poll_reader(reader_t *r)
{
    if (!r->available) return;

    uint8_t uid[4];
    xSemaphoreTake(spi_mutex, portMAX_DELAY);
    int result = read_uid(r, uid);
    xSemaphoreGive(spi_mutex);

    int64_t now = esp_timer_get_time();
    int lcd_id = r->is_exit ? LCD_EXIT : LCD_ENTRY;

    if (result == 1) {
        bool changed = memcmp(r->last_uid, uid, 4) != 0;
        if (!r->reported || changed) {
            printf("%s card detected: %02X:%02X:%02X:%02X\n", r->name,
                   uid[0], uid[1], uid[2], uid[3]);
            handle_scan(r->is_exit, uid);
        }
        memcpy(r->last_uid, uid, 4);
        r->reported = true;
        r->last_seen_us = now;
    } else if (result == -1) {
        if (!r->reported) {
            printf("%s: ACCESS DENIED (unsupported card)\n", r->name);
            lcd_msg(lcd_id, "ACCESS DENIED", "Bad card type");
            buzzer_pattern(1, DENIED_MS, 0);
            log_event(F_SYSTEM, "WARN,%s unsupported card", r->name);
        }
        r->reported = true;
        r->last_seen_us = now;
    } else if (now - r->last_seen_us > 500000) {
        r->reported = false;
    }
}


/* ================= SECTION: access.c ================= */

/* ================= SCHEDULE ================= */
static bool in_window(int m)
{
    if (sched_open_min < sched_close_min)
        return m >= sched_open_min && m < sched_close_min;
    return m >= sched_open_min || m < sched_close_min;
}

bool schedule_open(void)
{
    return in_window(minutes_now());
}

void sched_text(char *buf, size_t n)
{
    snprintf(buf, n, "Open %02d:%02d-%02d:%02d",
             sched_open_min / 60, sched_open_min % 60,
             sched_close_min / 60, sched_close_min % 60);
}

/* ================= USERS ================= */
void uid_to_hex(const uint8_t *uid, char *out)
{
    sprintf(out, "%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
}

bool parse_uid(const char *s, uint8_t out[4])
{
    char hex[9];
    int n = 0;
    for (; *s; s++) {
        if (*s == ':' || *s == '-') continue;
        if (!isxdigit((unsigned char)*s) || n >= 8) return false;
        hex[n++] = *s;
    }
    if (n != 8) return false;
    hex[8] = 0;
    for (int i = 0; i < 4; i++) {
        char b[3] = { hex[i * 2], hex[i * 2 + 1], 0 };
        out[i] = (uint8_t)strtol(b, NULL, 16);
    }
    return true;
}

int find_user(const uint8_t *uid)
{
    for (int i = 0; i < MAX_USERS; i++)
        if (users[i].used && memcmp(users[i].uid, uid, 4) == 0) return i;
    return -1;
}

/* returns index, -1 = table full, -2 = already registered */
int add_user(const uint8_t *uid, const char *name)
{
    if (find_user(uid) >= 0) return -2;
    for (int i = 0; i < MAX_USERS; i++) {
        if (!users[i].used) {
            memset(&users[i], 0, sizeof(user_t));
            users[i].used = true;
            memcpy(users[i].uid, uid, 4);
            strncpy(users[i].name, name, sizeof(users[i].name) - 1);
            return i;
        }
    }
    return -1;
}

int inside_count(void)
{
    int n = 0;
    for (int i = 0; i < MAX_USERS; i++)
        if (users[i].used && users[i].inside) n++;
    return n;
}

int users_total(void)
{
    int n = 0;
    for (int i = 0; i < MAX_USERS; i++)
        if (users[i].used) n++;
    return n;
}

int corridor_count(void)
{
    return inside_count() - master_room[0] - master_room[1];
}

bool users_save(void)
{
    if (!state_commit()) return false;
    size_t cap = (size_t)MAX_USERS * 40 + 24;
    char *buf = (char *)malloc(cap);
    if (!buf) { printf("WARN: SD user export allocation failed; NVS is saved\n"); return true; }
    int n = snprintf(buf, cap, "uid,name\n");
    for (int i = 0; i < MAX_USERS; i++) {
        if (!users[i].used) continue;
        n += snprintf(buf + n, cap - (size_t)n, "%02X%02X%02X%02X,%s\n",
                      users[i].uid[0], users[i].uid[1], users[i].uid[2],
                      users[i].uid[3], users[i].name);
    }
    store_write(F_USERS, buf);
    return true;
}

bool config_save(void)
{
    if (!state_commit()) return false;
    schedule_check();
    char *buf = (char *)malloc(48);
    if (!buf) { printf("WARN: SD config export allocation failed; NVS is saved\n"); return true; }
    snprintf(buf, 48, "open=%d\nclose=%d\n", sched_open_min, sched_close_min);
    store_write(F_CONFIG, buf);
    return true;
}

void users_defaults(void)
{
    memset(users, 0, sizeof(users));
    const uint8_t a[4] = { 0x11, 0x33, 0x44, 0x00 };
    const uint8_t b[4] = { 0x01, 0x03, 0x04, 0x00 };
    add_user(a, "Card A");
    add_user(b, "Card B");
}

/* A card scanned while the keypad menu is waiting to register a tag. */
static void reg_capture(const uint8_t *uid)
{
    char name[16];
    char uidtxt[9];
    snprintf(name, sizeof(name), "ID%s", reg_id);
    uid_to_hex(uid, uidtxt);

    int r = add_user(uid, name);
    if (r == -2) {
        lcd_msg(LCD_ENTRY, "ERROR", "Already in list");
    } else if (r == -1) {
        lcd_msg(LCD_ENTRY, "ERROR", "User table full");
    } else {
        if (!users_save()) { lcd_msg(LCD_ENTRY, "SAVE FAILED", "Change undone"); return; }
        printf("TAG REGISTERED: %s (%s)\n", name, uidtxt);
        log_event(F_SYSTEM, "INFO,tag registered %s (%s) via keypad", uidtxt, name);
        lcd_msg(LCD_ENTRY, "TAG REGISTERED", name);
    }
    menu_state = M_MAIN;
    menu_dirty = true;
}

/* strips trailing CR/LF/spaces */
static void trim_line(char *s)
{
    int n = (int)strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r' || s[n - 1] == ' ')) s[--n] = 0;
}

/* called from system_start(), before the tasks start */
void sd_load_users(void)
{
    FILE *f = fopen(file_paths[F_USERS], "r");
    if (!f) return;
    int loaded = 0;
    char line[64];
    while (fgets(line, sizeof(line), f)) {
        trim_line(line);
        if (strlen(line) < 9 || strncmp(line, "uid", 3) == 0) continue;
        if (line[8] != ',') continue;
        line[8] = 0;
        uint8_t uid[4];
        if (!parse_uid(line, uid)) continue;
        if (add_user(uid, line + 9) >= 0) loaded++;
    }
    fclose(f);
    printf("SD: loaded %d extra user(s) from users.csv\n", loaded);
}

void sd_load_config(void)
{
    FILE *f = fopen(file_paths[F_CONFIG], "r");
    if (!f) return;
    char line[32];
    while (fgets(line, sizeof(line), f)) {
        trim_line(line);
        if (strncmp(line, "open=", 5) == 0)  sched_open_min = atoi(line + 5);
        if (strncmp(line, "close=", 6) == 0) sched_close_min = atoi(line + 6);
    }
    fclose(f);
    if (sched_open_min < 0 || sched_open_min >= 1440 || sched_close_min < 0 ||
        sched_close_min >= 1440 || sched_open_min == sched_close_min) {
        sched_open_min = DEFAULT_OPEN_MIN;
        sched_close_min = DEFAULT_CLOSE_MIN;
        printf("WARN: invalid SD schedule; using defaults\n");
    }
    printf("SD: schedule loaded %02d:%02d-%02d:%02d\n", sched_open_min / 60,
           sched_open_min % 60, sched_close_min / 60, sched_close_min % 60);
}

/* ================= ENTRANCE TERMINAL: screens, access rules ================= */
void entrance_idle_refresh(void)
{
    bool open_now = schedule_open();

    /* the entry screen belongs to the keypad menu while it is open */
    if (menu_state == M_IDLE) {
        if (emergency_counts_uncertain()) lcd_idle(LCD_ENTRY, "ACCESS DENIED", "Contact admin");
        else if (!state_healthy()) lcd_idle(LCD_ENTRY, "SYSTEM DOWN", "Contact admin");
        else if (!entry_reader.available) lcd_idle(LCD_ENTRY, "ERROR", "Reader offline");
        else if (open_now)          lcd_idle(LCD_ENTRY, "ENTRANCE", "Scan your card");
        else                        lcd_idle(LCD_ENTRY, "BUILDING CLOSED", "");
    }
    if (menu_state == M_IDLE && checklist_status() >= CHECKLIST_CLEAR) {
        lcd_idle(LCD_ENTRY, checklist_status() == CHECKLIST_CLEAR ? "BUILDING CLEAR" : "CHECK REQUIRED", checklist_summary());
    }

    if (!state_healthy()) lcd_idle(LCD_EXIT, "SYSTEM DOWN", "Contact admin");
    else if (!exit_reader.available) lcd_idle(LCD_EXIT, "ERROR", "Reader offline");
    else if (open_now)          lcd_idle(LCD_EXIT, "EXIT", "Scan your card");
    else                        lcd_idle(LCD_EXIT, "BUILDING CLOSED", "");
    if (checklist_status() >= CHECKLIST_CLEAR)
        lcd_idle(LCD_EXIT, checklist_status() == CHECKLIST_CLEAR ? "BUILDING CLEAR" : "CHECK REQUIRED", checklist_summary());
}

static void deny_access(int lcd_id, const char *point, const char *uidtxt,
                        const char *name, const char *line2)
{
    /* These explanations use both rows so the 16-column LCD never cuts them off. */
    if (!strcmp(line2, "Card not registered")) lcd_msg(lcd_id, "Card not", "registered");
    else if (!strcmp(line2, "Card already used")) lcd_msg(lcd_id, "Card already", "used");
    else if (!strcmp(line2, "No entrance recorded")) lcd_msg(lcd_id, "No entrance", "recorded");
    else lcd_msg(lcd_id, "ACCESS DENIED", line2);
    buzzer_pattern(1, DENIED_MS, 0);
    printf("%s: ACCESS DENIED (%s) uid=%s\n", point, line2, uidtxt);
    log_event(F_ACCESS, "%s,%s,%s,DENIED,%s,%d", point, uidtxt, name, line2, inside_count());
}

/* Approval is provisional. Passage confirmation performs the occupancy commit. */
static void grant_access(bool is_exit, int ui, const char *uidtxt, const char *detail)
{
    (void)uidtxt; (void)detail;
    if (!passage_begin(is_exit, ui)) printf("Access window not opened; check passage status\n");
}

void handle_scan(bool is_exit, const uint8_t *uid)
{
    if (emergency_exit_required()) {
        printf("EMERGENCY: %s; no RFID accounting; occupancy UNVERIFIED\n",
               is_exit ? "exit is held open" : "use the designated exit");
        return;
    }
    if (!is_exit && emergency_counts_uncertain()) {
        lcd_msg(LCD_ENTRY, "ACCESS DENIED", "Contact admin");
        printf("ENTRY DENIED: reconcile occupancy after evacuation\n");
        return;
    }
    if (menu_state == M_REG_SCAN && !admin_active()) {
        menu_state = M_MAIN;
        menu_dirty = true;
        lcd_msg(LCD_ENTRY, "SESSION EXPIRED", "Log in again");
        return;
    }
    /* keypad menu is waiting for a new tag: capture it instead of opening a door */
    if (menu_state == M_REG_SCAN) {
        reg_capture(uid);
        return;
    }

    const char *point = is_exit ? "EXIT" : "ENTRY";
    int lcd_id = is_exit ? LCD_EXIT : LCD_ENTRY;
    char uidtxt[9];
    uid_to_hex(uid, uidtxt);

    int ui = find_user(uid);
    if (ui < 0) {
        deny_access(lcd_id, point, uidtxt, "UNKNOWN", "Card not registered");
        return;
    }
    const char *name = users[ui].name;
    if (!state_healthy()) {
        deny_access(lcd_id, point, uidtxt, name, "System down");
        return;
    }
    bool open_now = schedule_open();

    if (!is_exit) {
        if (!open_now)                 deny_access(lcd_id, point, uidtxt, name, "Building closed");
        else if (users[ui].inside)     deny_access(lcd_id, point, uidtxt, name, "Card already used");
        else                           grant_access(false, ui, uidtxt, "");
    } else {
        if (!open_now)      deny_access(lcd_id, point, uidtxt, name, "Building closed");
        else if (!users[ui].inside)           deny_access(lcd_id, point, uidtxt, name, "No entrance recorded");
        else grant_access(true, ui, uidtxt, "");
    }
}

static void closing_report(void)
{
    int n = inside_count();
    if (n == 0) {
        printf("CLOSING REPORT: building is empty\n");
        log_event(F_SYSTEM, "INFO,closing report: building empty");
        return;
    }
    char names[80] = "";
    printf("CLOSING REPORT: %d person(s) still inside:\n", n);
    for (int i = 0; i < MAX_USERS; i++) {
        if (users[i].used && users[i].inside) {
            printf("  - %s\n", users[i].name);
            if (strlen(names) + strlen(users[i].name) + 2 < sizeof(names)) {
                strcat(names, users[i].name);
                strcat(names, ";");
            }
        }
    }
    log_event(F_SYSTEM, "WARN,closing report: %d inside: %s", n, names);
}

/* Re-checks the schedule right now: called every second and immediately after
   an admin changes the opening or closing time, so the change applies at once. */
/* Remember the actual closing occurrence, including overnight schedules.
 * Volatile by design: rebooting within the warning window warns again. */
static time_t warned_closing;
static bool have_warned_closing;
static void closing_warning_check(bool open_now)
{
    if (!open_now || emergency_exit_required()) return;
    struct tm current;
    clock_get(&current);
    int remaining = (sched_close_min - (current.tm_hour * 60 + current.tm_min) + 1440) % 1440;
    if (remaining <= 0 || remaining > CLOSING_WARNING_MIN) return;
    struct tm closing = current;
    closing.tm_hour = sched_close_min / 60;
    closing.tm_min = sched_close_min % 60;
    closing.tm_sec = 0;
    if (sched_close_min <= current.tm_hour * 60 + current.tm_min) closing.tm_mday++;
    time_t occurrence = mktime(&closing);
    if (have_warned_closing && warned_closing == occurrence) return;
    warned_closing = occurrence;
    have_warned_closing = true;
    buzzer_closing_warning();
    char countdown[17];
    snprintf(countdown, sizeof(countdown), "IN %d MINUTE%s", remaining, remaining == 1 ? "" : "S");
    for (int i = 0; i < LCD_COUNT; i++)
        lcd_msg(i, "BUILDING CLOSING", countdown);
    printf("CLOSING WARNING: %d minute(s) remaining; %d inside\n", remaining, inside_count());
    log_event(F_SYSTEM, "WARN,closing in %d minute(s),%d inside", remaining, inside_count());
}

void schedule_check(void)
{
    bool open_now = schedule_open();
    if (!open_now) {
        passage_cancel_all("schedule closed", false);
        door_close(&entry_door); door_close(&exit_door);
    }
    closing_warning_check(open_now);
    if (open_now != last_open) {
        last_open = open_now;
        printf("SCHEDULE: building is now %s\n", open_now ? "OPEN" : "CLOSED");
        log_event(F_SYSTEM, "INFO,building %s", open_now ? "opened" : "closed");
        if (!open_now) {
            if (!emergency_exit_required()) {
                buzzer_closing_warning();
                for (int i = 0; i < LCD_COUNT; i++)
                    lcd_msg(i, "BUILDING CLOSED", "Please leave now");
                printf("CLOSING ALARM: building closed; %d inside\n", inside_count());
                log_event(F_SYSTEM, "WARN,closing alarm: building closed,%d inside", inside_count());
            }
            closing_report();
        }
    }
    entrance_idle_refresh();
}

/* ================= ENTRANCE TERMINAL: messages from the rooms ================= */
bool record_room_motion(room_t *room, bool entering)
{
    state_lock();
    if (emergency_exit_required() || emergency_counts_uncertain()) {
        state_unlock();
        return false; /* PIR events during evacuation cannot verify who escaped. */
    }
    int idx = room->term_id - TERM_ROOM1;
    bool allowed = idx >= 0 && idx < 2 && state_healthy() &&
        (entering ? corridor_count() > passage_reserved_exits() : master_room[idx] > 0);
    if (allowed) {
        master_room[idx] += entering ? 1 : -1;
        allowed = state_commit();
        if (allowed) {
            room->count = master_room[idx];
            printf("%s: %s, count=%d\n", room->name, entering ? "ENTERED" : "LEFT", room->count);
            log_event(F_ROOMS, "%s,%s,%d,%d", room->name, entering ? "ENTER" : "EXIT",
                      room->count, room->lights_on ? 1 : 0);
        }
    }
    if (!allowed) {
        printf("%s: occupancy update refused; check corridor/count/state\n", room->name);
        log_event(F_SYSTEM, "WARN,%s occupancy update refused", room->name);
        lcd_msg(LCD_ENTRY, "COUNT NOT UPDATED", "Check room/state");
    }
    state_unlock();
    return allowed;
}

void master_handle(const bus_msg_t *m)
{
    int idx = (int)m->src - TERM_ROOM1;
    if (idx < 0 || idx > 1) {
        printf("ERROR: message from unknown terminal %d\n", (int)m->src);
        return;
    }
    const char *rn = term_names[m->src];

    if (m->type == MSG_ROOM_ENTER_REQ || m->type == MSG_ROOM_EXIT_NOTICE) {
        printf("WARN: obsolete occupancy message ignored; use protected transaction\n");
    } else if (m->type == MSG_ROOM_STATE) {
        /* Queued telemetry may be old. Compare protected live state instead. */
        room_t *live = idx == 0 ? &room1 : &room2;
        room_lights[idx] = live->lights_on;
        if (room_offline[idx] && now_ms() - room_seen_ms[idx] <= (uint32_t)ROOM_OFFLINE_MS) {
            room_offline[idx] = false;
            printf("INFO: %s is back online\n", rn);
            log_event(F_SYSTEM, "INFO,%s back online", rn);
        }
        if (live->count != master_room[idx]) {
            if (++room_strikes[idx] >= 3) {
                room_strikes[idx] = 0;
                printf("ERROR: %s count mismatch (room=%ld, entrance=%d)\n",
                       rn, (long)live->count, master_room[idx]);
                log_event(F_SYSTEM, "ERROR,%s count mismatch room=%ld entrance=%d",
                          rn, (long)live->count, master_room[idx]);
                lcd_msg(LCD_ENTRY, "ERROR", "Count mismatch");
                lcd_msg(LCD_EXIT, "ERROR", "Count mismatch");
            }
        } else {
            room_strikes[idx] = 0;
        }
    } else if (m->type == MSG_ROOM_FAULT) {
        printf("ERROR: %s door PIR stuck (active too long)\n", rn);
        log_event(F_SYSTEM, "ERROR,%s door sensor stuck", rn);
        lcd_msg(LCD_ENTRY, "ERROR", "Room sensor");
        lcd_msg(LCD_EXIT, "ERROR", "Room sensor");
    }
}

void master_periodic(void)
{
    uint32_t now = now_ms();

    schedule_check();

    if (exit_override && (int32_t)(override_until_ms - now) <= 0) {
        exit_override = false;
        printf("INFO: emergency exit override expired\n");
        log_event(F_SYSTEM, "INFO,exit override expired");
    }

    for (int i = 0; i < 2; i++) {
        if (!room_offline[i] && (now - room_seen_ms[i]) > (uint32_t)ROOM_OFFLINE_MS) {
            room_offline[i] = true;
            printf("ERROR: %s is not responding\n", term_names[i + 1]);
            log_event(F_SYSTEM, "ERROR,%s not responding", term_names[i + 1]);
            lcd_msg(LCD_ENTRY, "ERROR", "Room offline");
            lcd_msg(LCD_EXIT, "ERROR", "Room offline");
        }
    }
}

/* ================= ADMIN SESSION (shared by serial menu and keypad menu) ================= */
bool eq(const char *a, const char *b)
{ return strcmp(a, b) == 0; }

void lower(char *s)
{ for (; *s; s++) *s = (char)tolower((unsigned char)*s); }

bool admin_active(void)
{
    if (admin_logged && (int32_t)(admin_until_ms - now_ms()) <= 0) {
        admin_logged = false;
        printf("Admin session expired.\n");
        log_event(F_SYSTEM, "INFO,admin session expired");
    }
    return admin_logged;
}

void admin_touch(void)
{
    admin_until_ms = now_ms() + ADMIN_SESSION_MS;
}

bool admin_check(void)
{
    if (!admin_active()) {
        printf("DENIED: admin login required. Use: login <PIN>\n");
        return false;
    }
    admin_touch();
    return true;
}

/* returns 1 = logged in, 0 = wrong PIN, -1 = locked out */
int admin_login_attempt(const char *pin, const char *via)
{
    if ((int32_t)(lockout_until_ms - now_ms()) > 0) return -1;

    if (strcmp(pin, ADMIN_PIN) == 0) {
        admin_logged = true;
        admin_touch();
        pin_fails = 0;
        log_event(F_SYSTEM, "INFO,admin login ok (%s)", via);
        return 1;
    }

    pin_fails++;
    log_event(F_SYSTEM, "WARN,admin login failed %d (%s)", pin_fails, via);
    if (pin_fails >= MAX_PIN_FAILS) {
        pin_fails = 0;
        lockout_until_ms = now_ms() + LOCKOUT_MS;
        log_event(F_SYSTEM, "ERROR,admin lockout %d s", LOCKOUT_MS / 1000);
        lcd_msg(LCD_ENTRY, "ERROR", "Admin lockout");
        lcd_msg(LCD_EXIT, "ERROR", "Admin lockout");
        return -1;
    }
    return 0;
}


/* ================= SECTION: controls.c ================= */

/* ================= SERIAL MANAGEMENT MENU ================= */
static int file_from_name(const char *s)
{
    if (eq(s, "access")) return F_ACCESS;
    if (eq(s, "rooms")) return F_ROOMS;
    if (eq(s, "sys") || eq(s, "system")) return F_SYSTEM;
    if (eq(s, "users")) return F_USERS;
    if (eq(s, "config")) return F_CONFIG;
    return -1;
}

static void print_help(void)
{
    printf("\n=== COMMANDS ===\n");
    printf(" emergency status | ADMIN: emergency reset | occupancy reconcile empty\n");
    printf(" checklist | passage status\n");
    printf(" status | time | occupancy | rooms | sensors | who | users\n");
    printf(" schedule show | login <PIN> | logout\n");
    printf(" ADMIN: log [n] | log sys [n] | log rooms [n] | log find <text>\n");
    printf(" ADMIN: export <access|rooms|system|users|config>\n");
    printf(" ADMIN: schedule set HH:MM HH:MM | adduser <uid> <name> | deluser <uid>\n");
    printf(" ADMIN: time set YYYY-MM-DD HH:MM[:SS] | unlock | lock | reboot (or reset)\n");
    printf(" ADMIN: person enter|leave 1|2   (test hook for the door PIRs)\n");
    printf(" KEYPAD: press * for the menu on the entrance screen, # = Enter,\n");
    printf("         2/8 = up/down, tag IDs accept digits 0-9 and letters A-D\n\n");
}

static void print_status(void)
{
    char ts[24], sched[24];
    fmt_time(ts, sizeof(ts));
    sched_text(sched, sizeof(sched));
    printf("\n=== STATUS ===\n");
    printf("Time      : %s (%s)\n", ts, rtc_present ? (rtc_ok ? "RTC DS1307" : "RTC read error") : "software clock");
    printf("Schedule  : %s -> building is %s%s\n", sched,
           schedule_open() ? "OPEN" : "CLOSED",
           exit_override ? " (exit override ACTIVE)" : "");
    emergency_print_status();
    passage_print_status();
    printf("%s: %d person(s), room1=%d, room2=%d, corridor=%d\n",
           emergency_counts_uncertain() ? "Last known" : "Inside    ", inside_count(), master_room[0], master_room[1], corridor_count());
    printf("Lights    : room1=%s, room2=%s\n", room_lights[0] ? "ON" : "OFF",
           room_lights[1] ? "ON" : "OFF");
    printf("SD card   : %s\n", sd_ok ? "OK" : "NOT AVAILABLE");
    printf("Rooms     : %s, %s\n", room_offline[0] ? "ROOM 1 OFFLINE" : "ROOM 1 online",
           room_offline[1] ? "ROOM 2 OFFLINE" : "ROOM 2 online");
    printf("State save: %s; log failures=%lu\n", state_healthy() ? "OK" : "FAULT", (unsigned long)atomic_load(&log_failures));
    printf("Admin     : %s\n\n", admin_logged ? "logged in" : "logged out");
}

static void cli_exec(char *line)
{
    char *tok[8];
    int nt = 0;
    char *p = line;
    while (*p && nt < 8) {
        while (*p == ' ') p++;
        if (!*p) break;
        tok[nt++] = p;
        while (*p && *p != ' ') p++;
        if (*p) *p++ = 0;
    }
    if (nt == 0) return;
    lower(tok[0]);
    if (nt > 1 && (eq(tok[0], "schedule") || eq(tok[0], "time") ||
                   eq(tok[0], "log") || eq(tok[0], "person") || eq(tok[0], "emergency") || eq(tok[0], "occupancy"))) lower(tok[1]);

    const char *c = tok[0];

    if (eq(c, "help") || eq(c, "?")) { print_help(); }
    else if (eq(c, "status")) { print_status(); }
    else if (eq(c, "checklist")) { checklist_run(); }
    else if (eq(c, "passage")) { passage_print_status(); }
    else if (eq(c, "time") && (nt == 1)) {
        char ts[24];
        fmt_time(ts, sizeof(ts));
        printf("Time: %s\n", ts);
    }
    else if (eq(c, "emergency")) {
        if (nt == 2 && eq(tok[1], "reset")) emergency_reset();
        else emergency_print_status();
    }
    else if (eq(c, "occupancy")) {
        if (nt == 3 && eq(tok[1], "reconcile") && eq(tok[2], "empty")) {
            emergency_reconcile_empty();
            return;
        }
        if (emergency_counts_uncertain()) printf("UNVERIFIED: values below are last-known records, not a live headcount\n");
        printf("Occupancy: R1=%d R2=%d TOT=%d (corridor=%d)\n",
               master_room[0], master_room[1], inside_count(), corridor_count());
    }
    else if (eq(c, "rooms")) {
        if (emergency_counts_uncertain()) printf("Room counts are LAST KNOWN, not verified after evacuation\n");
        for (int i = 0; i < 2; i++) {
            printf("%s: people=%d, lights=%s, light level=%d lux, %s\n", term_names[i + 1],
                   master_room[i], room_lights[i] ? "ON" : "OFF",
                   (i == 0 ? room1.lux : room2.lux), room_offline[i] ? "OFFLINE" : "online");
        }
    }
    else if (eq(c, "sensors")) {
        printf("ROOM 1: outside PIR=%s, inside PIR=%s\n",
               room1.a.active ? "MOTION" : "idle", room1.b.active ? "MOTION" : "idle");
        printf("ROOM 2: outside PIR=%s, inside PIR=%s\n",
               room2.a.active ? "MOTION" : "idle", room2.b.active ? "MOTION" : "idle");
        printf("LIGHT : room1=%d lux (%s), room2=%d lux (%s); on below %d, off above %d\n",
               room1.lux, room1.dark ? "dark" : "bright", room2.lux,
               room2.dark ? "dark" : "bright", LUX_ON, LUX_OFF);
    }
    else if (eq(c, "who")) {
        if (emergency_counts_uncertain()) printf("LAST KNOWN OCCUPANTS - confirm at assembly point\n");
        int n = 0;
        for (int i = 0; i < MAX_USERS; i++) {
            if (users[i].used && users[i].inside) {
                struct tm t;
                gmtime_r(&users[i].since, &t);
                printf("  %s (%02X%02X%02X%02X) since %02d:%02d:%02d\n", users[i].name,
                       users[i].uid[0], users[i].uid[1], users[i].uid[2], users[i].uid[3],
                       t.tm_hour, t.tm_min, t.tm_sec);
                n++;
            }
        }
        printf("%d person(s) inside the building\n", n);
    }
    else if (eq(c, "users")) {
        for (int i = 0; i < MAX_USERS; i++) {
            if (users[i].used) {
                printf("  %02X%02X%02X%02X  %-15s %s\n", users[i].uid[0], users[i].uid[1],
                       users[i].uid[2], users[i].uid[3], users[i].name,
                       users[i].inside ? "(inside)" : "");
            }
        }
    }
    else if (eq(c, "schedule")) {
        if (nt >= 2 && eq(tok[1], "set")) {
            if (!admin_check()) return;
            int oh, om, ch, cm;
            if (nt < 4 || sscanf(tok[2], "%d:%d", &oh, &om) != 2 ||
                sscanf(tok[3], "%d:%d", &ch, &cm) != 2 ||
                oh < 0 || oh > 23 || ch < 0 || ch > 23 ||
                om < 0 || om > 59 || cm < 0 || cm > 59 ||
                (oh * 60 + om) == (ch * 60 + cm)) {
                printf("Usage: schedule set HH:MM HH:MM (open close)\n");
                return;
            }
            sched_open_min = oh * 60 + om;
            sched_close_min = ch * 60 + cm;
            if (!config_save()) { printf("ERROR: schedule not saved; change undone\n"); return; }
            printf("Schedule changed to %02d:%02d - %02d:%02d\n", oh, om, ch, cm);
            log_event(F_SYSTEM, "INFO,schedule changed to %02d:%02d-%02d:%02d by admin",
                      oh, om, ch, cm);
            schedule_check();
            printf("Building is %s now\n", schedule_open() ? "OPEN" : "CLOSED");
        } else {
            char sched[24];
            sched_text(sched, sizeof(sched));
            printf("%s (building is %s now)\n", sched, schedule_open() ? "OPEN" : "CLOSED");
        }
    }
    else if (eq(c, "log")) {
        if (!admin_check()) return;
        int file = F_ACCESS, n = 10;
        if (nt >= 2 && eq(tok[1], "find")) {
            if (nt < 3) { printf("Usage: log find <text>\n"); return; }
            store_request(SQ_FIND, F_ACCESS, 0, tok[2]);
            return;
        }
        if (nt >= 2 && (eq(tok[1], "sys") || eq(tok[1], "system"))) {
            file = F_SYSTEM;
            if (nt >= 3) n = atoi(tok[2]);
        } else if (nt >= 2 && eq(tok[1], "rooms")) {
            file = F_ROOMS;
            if (nt >= 3) n = atoi(tok[2]);
        } else if (nt >= 2) {
            n = atoi(tok[1]);
        }
        if (n < 1) n = 10;
        store_request(SQ_TAIL, file, n, NULL);
    }
    else if (eq(c, "export")) {
        if (!admin_check()) return;
        int f = (nt >= 2) ? file_from_name(tok[1]) : -1;
        if (f < 0) { printf("Usage: export <access|rooms|system|users|config>\n"); return; }
        store_request(SQ_EXPORT, f, 0, NULL);
    }
    else if (eq(c, "login")) {
        if (nt < 2) { printf("Usage: login <PIN>\n"); return; }
        int r = admin_login_attempt(tok[1], "serial");
        if (r == 1) {
            printf("Admin login OK (session %d s)\n", ADMIN_SESSION_MS / 1000);
        } else if (r == 0) {
            printf("Wrong PIN (%d/%d)\n", pin_fails, MAX_PIN_FAILS);
        } else {
            printf("LOCKED OUT. Try again in %lu s\n",
                   (unsigned long)((lockout_until_ms - now_ms()) / 1000 + 1));
        }
    }
    else if (eq(c, "logout")) {
        admin_logged = false;
        printf("Logged out\n");
        log_event(F_SYSTEM, "INFO,admin logout");
    }
    else if (eq(c, "unlock")) {
        if (!admin_check()) return;
        printf("DENIED: this coursework build strictly enforces opening hours\n");
    }
    else if (eq(c, "lock")) {
        if (!admin_check()) return;
        exit_override = false;
        printf("Exit override cancelled\n");
        log_event(F_SYSTEM, "INFO,exit override cancelled");
    }
    else if (eq(c, "adduser")) {
        if (!admin_check()) return;
        uint8_t uid[4];
        if (nt < 3 || !parse_uid(tok[1], uid)) {
            printf("Usage: adduser <8-hex-digit uid> <name>   e.g. adduser 11334400 Alice\n");
            return;
        }
        char name[16] = "";
        for (int i = 2; i < nt; i++) {
            if (strlen(name) + strlen(tok[i]) + 2 >= sizeof(name)) break;
            if (i > 2) strcat(name, " ");
            strcat(name, tok[i]);
        }
        for (char *q = name; *q; q++) if (*q == ',') *q = ' ';
        int r = add_user(uid, name);
        if (r == -2) { printf("That card is already registered\n"); return; }
        if (r == -1) { printf("User table full (%d)\n", MAX_USERS); return; }
        if (!users_save()) { printf("ERROR: users not saved; change undone\n"); return; }
        printf("User added: %s\n", name);
        log_event(F_SYSTEM, "INFO,user added %s (%s)", tok[1], name);
    }
    else if (eq(c, "deluser")) {
        if (!admin_check()) return;
        uint8_t uid[4];
        if (nt < 2 || !parse_uid(tok[1], uid)) { printf("Usage: deluser <uid>\n"); return; }
        int ui = find_user(uid);
        if (ui < 0) { printf("No such user\n"); return; }
        if (users[ui].inside || passage_user_pending(ui)) { printf("DENIED: user is inside or has a pending passage\n"); return; }
        char removed_name[16];
        memcpy(removed_name, users[ui].name, sizeof(removed_name));
        memset(&users[ui], 0, sizeof(user_t));
        if (!users_save()) { printf("ERROR: users not saved; change undone\n"); return; }
        printf("User removed: %s\n", removed_name);
        log_event(F_SYSTEM, "INFO,user removed %s (%s)", tok[1], removed_name);
    }
    else if (eq(c, "time") && nt >= 2 && eq(tok[1], "set")) {
        if (!admin_check()) return;
        int y, mo, d, h, mi, s = 0;
        if (nt < 4 || sscanf(tok[2], "%d-%d-%d", &y, &mo, &d) != 3 ||
            sscanf(tok[3], "%d:%d:%d", &h, &mi, &s) < 2 ||
            y < 2024 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
            h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59) {
            printf("Usage: time set YYYY-MM-DD HH:MM[:SS]\n");
            return;
        }
        time_t value = make_epoch(y, mo, d, h, mi, s);
        struct tm checked;
        gmtime_r(&value, &checked);
        if (checked.tm_year != y - 1900 || checked.tm_mon != mo - 1 || checked.tm_mday != d) {
            printf("ERROR: invalid calendar date\n"); return;
        }
        if (!clock_set(value)) { printf("ERROR: RTC write failed; time unchanged\n"); return; }
        printf("Time set to %04d-%02d-%02d %02d:%02d:%02d\n", y, mo, d, h, mi, s);
        log_event(F_SYSTEM, "INFO,time set by admin to %04d-%02d-%02d %02d:%02d:%02d",
                  y, mo, d, h, mi, s);
    }
    else if (eq(c, "person")) {
        if (!admin_check()) return;
        int r = (nt >= 3) ? atoi(tok[2]) : 0;
        if (nt < 3 || (r != 1 && r != 2) ||
            !(eq(tok[1], "enter") || eq(tok[1], "leave"))) {
            printf("Usage: person enter|leave 1|2   (test hook: simulates the door PIRs)\n");
            return;
        }
        room_t *rm = (r == 1) ? &room1 : &room2;
        rm->inject = eq(tok[1], "enter") ? 1 : 2;
        printf("TEST: simulated person %s at %s\n", tok[1], rm->name);
        log_event(F_SYSTEM, "WARN,test person %s injected at %s by admin", tok[1], rm->name);
    }
    else if (eq(c, "reboot") || eq(c, "reset") || eq(c, "restart")) {
        if (!admin_check()) return;
        emergency_poll();
        passage_poll();
        if (passage_waiting(false) || passage_waiting(true) || passage_sensor_blocked(false) || passage_sensor_blocked(true)) {
            printf("DENIED: wait until the doorway is clear before reboot\n"); return;
        }
        emergency_service();
        if (emergency_exit_required()) {
            printf("DENIED: reboot cannot cancel an active emergency\n"); return;
        }
        printf("Rebooting...\n");
        log_event(F_SYSTEM, "INFO,reboot requested by admin");
        vTaskDelay(pdMS_TO_TICKS(1500));
        emergency_poll();
        passage_poll();
        emergency_service();
        if (passage_waiting(false) || passage_waiting(true) || passage_sensor_blocked(false) || passage_sensor_blocked(true)) {
            printf("REBOOT CANCELLED: doorway busy\n"); return;
        }
        if (emergency_exit_required()) {
            printf("REBOOT CANCELLED: emergency activated\n"); return;
        }
        esp_restart();
    }
    else {
        printf("Unknown command '%s'. Type 'help'.\n", c);
    }
}

void cli_poll(void)
{
    uint8_t in[32];
    int got = uart_read_bytes(CLI_UART, in, sizeof(in), 0);
    for (int i = 0; i < got; i++) {
        int ch = in[i];
        if (ch == '\r' || ch == '\n') {
            if (cli_len > 0) {
                cli_buf[cli_len] = 0;
                cli_len = 0;
                printf("> command received\n");   /* echo, proves the input arrived */
                cli_exec(cli_buf);
            }
        } else if (cli_len < (int)sizeof(cli_buf) - 1) {
            cli_buf[cli_len++] = (char)ch;
        }
    }
}

void keypad_init(void)
{
    for (int r = 0; r < 4; r++) pin_output(kp_rows[r], 1);
    for (int c = 0; c < 4; c++) pin_input(kp_cols[c], true);
}

/* returns the key that is down right now, or 0 */
static char keypad_scan(void)
{
    char found = 0;
    for (int r = 0; r < 4 && !found; r++) {
        pin_set(kp_rows[r], 0);
        delay_us(100);
        for (int c = 0; c < 4; c++) {
            if (pin_get(kp_cols[c]) == 0) found = kp_map[r][c];
        }
        pin_set(kp_rows[r], 1);
    }
    return found;
}

/* debounces and sends one event per key press to the entrance terminal */
void keypad_task(void *arg)
{
    (void)arg;
    char cand = 0, last = 0;
    int stable = 0;

    for (;;) {
        char k = keypad_scan();
        if (k == cand) {
            if (stable < 3) stable++;
        } else {
            cand = k;
            stable = 1;
        }
        if (stable >= 2 && cand != last) {
            last = cand;
            if (cand != 0 && xQueueSend(key_q, &cand, pdMS_TO_TICKS(20)) != pdTRUE)
                printf("WARN: keypad queue full; repeat key\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));   /* at least 1 tick, so the idle task can run */
    }
}

/* ================= KEYPAD MENU (runs inside the entrance terminal) ================= */
static void menu_build_items(void)
{
    menu_item_n = 0;
    menu_items[menu_item_n++] = IT_OCC;
    menu_items[menu_item_n++] = IT_ROOMS;
    menu_items[menu_item_n++] = IT_WHO;
    menu_items[menu_item_n++] = IT_SYS;
    if (admin_active()) {
        menu_items[menu_item_n++] = IT_LOG;      /* access log: admin only */
        menu_items[menu_item_n++] = IT_REG;
        menu_items[menu_item_n++] = IT_DEL;
        menu_items[menu_item_n++] = IT_OPEN;
        menu_items[menu_item_n++] = IT_CLOSE;
        if (emergency_active()) menu_items[menu_item_n++] = IT_OVR;
        menu_items[menu_item_n++] = IT_LOGOUT;
    } else {
        menu_items[menu_item_n++] = IT_LOGIN;
    }
    if (menu_cursor >= menu_item_n) menu_cursor = menu_item_n - 1;
    if (menu_cursor < 0) menu_cursor = 0;
}

static const char *item_label(int it)
{
    switch (it) {
    case IT_OCC:    return "Occupancy";
    case IT_ROOMS:  return "Room status";
    case IT_WHO:    return "Who inside";
    case IT_LOG:    return "Access log";
    case IT_SYS:    return "System info";
    case IT_LOGIN:  return "Admin login";
    case IT_REG:    return "Add tag";
    case IT_DEL:    return "Remove tag";
    case IT_OPEN:   return "Open time";
    case IT_CLOSE:  return "Close time";
    case IT_OVR:    return "Emergency reset";
    case IT_LOGOUT: return "Logout";
    }
    return "";
}

static void menu_back_to_main(void)
{
    menu_state = M_MAIN;
    menu_len = 0;
    menu_buf[0] = 0;
    menu_build_items();
    menu_dirty = true;
}

static void menu_close(void)
{
    menu_state = M_IDLE;
    menu_len = 0;
    menu_buf[0] = 0;
    entrance_idle_refresh();
    printf("MENU: closed\n");
}

static void build_who_list(void)
{
    who_n = 0;
    for (int i = 0; i < MAX_USERS; i++)
        if (users[i].used && users[i].inside) who_list[who_n++] = i;
}

static void build_del_list(void)
{
    del_n = 0;
    for (int i = 0; i < MAX_USERS; i++)
        if (users[i].used) del_list[del_n++] = i;
}

static void fmt_digits_time(char *out, size_t n)
{
    char d[5] = { '_', '_', '_', '_', 0 };
    for (int i = 0; i < menu_len && i < 4; i++) d[i] = menu_buf[i];
    snprintf(out, n, "%c%c:%c%c", d[0], d[1], d[2], d[3]);
}

static void menu_render(void)
{
    char l1[40] = "";
    char l2[40] = "";

    switch (menu_state) {
    case M_MAIN: {
        char pos[24];
        snprintf(pos, sizeof(pos), "%d/%d", menu_cursor + 1, menu_item_n);
        snprintf(l1, sizeof(l1), "%-11.11s%5s", item_label(menu_items[menu_cursor]), pos);
        snprintf(l2, sizeof(l2), "%s", admin_active() ? "ADMIN 2/8 #OK *X" : "2/8 Move #OK *X");
        break;
    }

    case M_INFO:
        if (info_kind == IT_OCC) {
            snprintf(l1, sizeof(l1), "%s", emergency_counts_uncertain() ? "LAST KNOWN COUNT" : "OCCUPANCY  *Back");
            snprintf(l2, sizeof(l2), "R1:%d R2:%d TOT:%d", master_room[0], master_room[1],
                     inside_count());
        } else if (info_kind == IT_ROOMS) {
            for (int i = 0; i < 2; i++) {
                char *l = (i == 0) ? l1 : l2;
                int lux = (i == 0) ? room1.lux : room2.lux;
                if (room_offline[i])
                    snprintf(l, sizeof(l1), "R%d: OFFLINE", i + 1);
                else
                    snprintf(l, sizeof(l1), "R%d:%-2d %-3s%5dlx", i + 1, master_room[i],
                             room_lights[i] ? "ON" : "OFF", lux);
            }
        } else {
            if (menu_page == 0) {
                struct tm t;
                clock_get(&t);
                snprintf(l1, sizeof(l1), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
                snprintf(l2, sizeof(l2), "%02d:%02d:%02d %s", t.tm_hour, t.tm_min, t.tm_sec,
                         rtc_present ? "RTC" : "SW");
            } else if (menu_page == 1) {
                sched_text(l1, sizeof(l1));
                snprintf(l2, sizeof(l2), "Now:%s%s", schedule_open() ? "OPEN" : "CLOSED",
                         exit_override ? " OVR" : "");
            } else {
                snprintf(l1, sizeof(l1), "SD:%s RTC:%s", sd_ok ? "OK" : "NO", rtc_ok ? "OK" : "NO");
                snprintf(l2, sizeof(l2), "Users:%d Adm:%s", users_total(),
                         admin_active() ? "yes" : "no");
            }
        }
        break;

    case M_WHO:
        build_who_list();
        if (who_n == 0) {
            snprintf(l1, sizeof(l1), "Nobody inside");
        } else {
            if (list_cursor >= who_n) list_cursor = who_n - 1;
            user_t *u = &users[who_list[list_cursor]];
            struct tm t;
            gmtime_r(&u->since, &t);
            snprintf(l1, sizeof(l1), "%d/%d %s", list_cursor + 1, who_n, u->name);
            if (emergency_counts_uncertain()) snprintf(l2, sizeof(l2), "LAST KNOWN ONLY");
            else snprintf(l2, sizeof(l2), "In since %02d:%02d", t.tm_hour, t.tm_min);
        }
        break;

    case M_LOG_WAIT:
        snprintf(l1, sizeof(l1), "ACCESS LOG");
        snprintf(l2, sizeof(l2), "Reading SD...");
        break;

    case M_LOG:
        xSemaphoreTake(log_mutex, portMAX_DELAY);
        if (lcdlog_n == 0) {
            snprintf(l1, sizeof(l1), "ACCESS LOG");
            snprintf(l2, sizeof(l2), "No records");
        } else {
            if (list_cursor >= lcdlog_n) list_cursor = lcdlog_n - 1;
            snprintf(l1, sizeof(l1), "%s", lcdlog[list_cursor][0]);
            snprintf(l2, sizeof(l2), "%s", lcdlog[list_cursor][1]);
        }
        xSemaphoreGive(log_mutex);
        break;

    case M_PIN: {
        char stars[10];
        int n = (menu_len < 8) ? menu_len : 8;
        for (int i = 0; i < n; i++) stars[i] = '*';
        stars[n] = 0;
        snprintf(l1, sizeof(l1), "ADMIN PIN:");
        snprintf(l2, sizeof(l2), "%s_", stars);
        break;
    }

    case M_REG_ID:
        snprintf(l1, sizeof(l1), "TAG ID (0-9,A-D)");
        snprintf(l2, sizeof(l2), "%s_", menu_buf);
        break;

    case M_REG_SCAN:
        snprintf(l1, sizeof(l1), "SCAN NEW CARD");
        snprintf(l2, sizeof(l2), "ID %s  *=cancel", reg_id);
        break;

    case M_DEL:
        build_del_list();
        if (del_n == 0) {
            snprintf(l1, sizeof(l1), "No users");
        } else {
            if (list_cursor >= del_n) list_cursor = del_n - 1;
            user_t *u = &users[del_list[list_cursor]];
            snprintf(l1, sizeof(l1), "%s", u->name);
            snprintf(l2, sizeof(l2), "%02X%02X%02X%02X%s", u->uid[0], u->uid[1],
                     u->uid[2], u->uid[3], u->inside ? " IN" : "");
        }
        break;

    case M_DEL_CONF:
        snprintf(l1, sizeof(l1), "Delete user?");
        snprintf(l2, sizeof(l2), "#=yes  *=no");
        break;

    case M_TIME_OPEN:
    case M_TIME_CLOSE: {
        char t[8];
        fmt_digits_time(t, sizeof(t));
        snprintf(l1, sizeof(l1), "%s", menu_state == M_TIME_OPEN ? "SET OPEN TIME" : "SET CLOSE TIME");
        snprintf(l2, sizeof(l2), "%s  (HHMM)", t);
        break;
    }

    default:
        return;
    }

    lcd_idle(LCD_ENTRY, l1, l2);
}

/* the user pressed # on a main-menu item */
static void menu_select(int it)
{

    bool admin_item = (it == IT_LOG || it == IT_REG || it == IT_DEL || it == IT_OPEN ||
                       it == IT_CLOSE || it == IT_OVR || it == IT_LOGOUT);
    if (admin_item) {
        if (!admin_active()) {
            lcd_msg(LCD_ENTRY, "SESSION EXPIRED", "Log in again");
            menu_build_items();
            return;
        }
        admin_touch();
    }

    switch (it) {
    case IT_OCC:
    case IT_ROOMS:
    case IT_SYS:
        info_kind = it;
        menu_page = 0;
        menu_state = M_INFO;
        break;

    case IT_WHO:
        list_cursor = 0;
        menu_state = M_WHO;
        break;

    case IT_LOG:
        xSemaphoreTake(log_mutex, portMAX_DELAY);
        lcdlog_ready = false;
        xSemaphoreGive(log_mutex);
        if (!store_request(SQ_LOGLCD, F_ACCESS, 0, NULL)) {
            lcd_msg(LCD_ENTRY, "LOG QUEUE BUSY", "Try again");
            return;
        }
        list_cursor = 0;
        menu_state = M_LOG_WAIT;
        break;

    case IT_LOGIN:
        menu_len = 0;
        menu_buf[0] = 0;
        menu_state = M_PIN;
        break;

    case IT_REG:
        menu_len = 0;
        menu_buf[0] = 0;
        menu_state = M_REG_ID;
        break;

    case IT_DEL:
        list_cursor = 0;
        menu_state = M_DEL;
        break;

    case IT_OPEN:
        menu_len = 0;
        menu_buf[0] = 0;
        menu_state = M_TIME_OPEN;
        break;

    case IT_CLOSE:
        menu_len = 0;
        menu_buf[0] = 0;
        menu_state = M_TIME_CLOSE;
        break;

    case IT_OVR:
        emergency_reset();
        break;

    case IT_LOGOUT:
        admin_logged = false;
        log_event(F_SYSTEM, "INFO,admin logout (keypad)");
        lcd_msg(LCD_ENTRY, "LOGGED OUT", "");
        menu_cursor = 0;
        menu_build_items();
        break;
    }
}

/* # pressed in a number-entry screen */
static void menu_submit(void)
{
    char buf[24];
    if (menu_len == 0) return;

    if (menu_state == M_PIN) {
        int r = admin_login_attempt(menu_buf, "keypad");
        menu_len = 0;
        menu_buf[0] = 0;
        if (r == 1) {
            lcd_msg(LCD_ENTRY, "ADMIN LOGIN", "Welcome");
            menu_cursor = 0;
            menu_back_to_main();
        } else if (r == 0) {
            snprintf(buf, sizeof(buf), "Attempt %d of %d", pin_fails, MAX_PIN_FAILS);
            lcd_msg(LCD_ENTRY, "WRONG PIN", buf);
        } else {
            lcd_msg(LCD_ENTRY, "LOCKED OUT", "Try again later");
            menu_back_to_main();
        }
        return;
    }

    /* everything below needs a live admin session */
    if (!admin_active()) {
        lcd_msg(LCD_ENTRY, "SESSION EXPIRED", "Log in again");
        menu_back_to_main();
        return;
    }
    admin_touch();

    if (menu_state == M_REG_ID) {
        strncpy(reg_id, menu_buf, sizeof(reg_id) - 1);
        reg_id[sizeof(reg_id) - 1] = 0;
        menu_len = 0;
        menu_buf[0] = 0;
        menu_state = M_REG_SCAN;
        printf("MENU: waiting for a new card (id %s)\n", reg_id);
    } else if (menu_state == M_TIME_OPEN || menu_state == M_TIME_CLOSE) {
        if (menu_len != 4) {
            lcd_msg(LCD_ENTRY, "ENTER 4 DIGITS", "like 0600");
            return;
        }
        int hh = (menu_buf[0] - '0') * 10 + (menu_buf[1] - '0');
        int mm = (menu_buf[2] - '0') * 10 + (menu_buf[3] - '0');
        bool is_open = (menu_state == M_TIME_OPEN);
        int val = hh * 60 + mm;
        int other = is_open ? sched_close_min : sched_open_min;
        menu_len = 0;
        menu_buf[0] = 0;

        if (hh > 23 || mm > 59 || val == other) {
            lcd_msg(LCD_ENTRY, "INVALID TIME", "Try again");
            return;
        }
        if (is_open) sched_open_min = val;
        else         sched_close_min = val;
        if (!config_save()) { lcd_msg(LCD_ENTRY, "SAVE FAILED", "Change undone"); return; }
        log_event(F_SYSTEM, "INFO,schedule changed to %02d:%02d-%02d:%02d (keypad)",
                  sched_open_min / 60, sched_open_min % 60,
                  sched_close_min / 60, sched_close_min % 60);
        printf("Schedule changed to %02d:%02d - %02d:%02d (keypad)\n",
               sched_open_min / 60, sched_open_min % 60,
               sched_close_min / 60, sched_close_min % 60);
        schedule_check();
        snprintf(buf, sizeof(buf), "%s %02d:%02d", is_open ? "Open" : "Close", hh, mm);
        lcd_msg(LCD_ENTRY, schedule_open() ? "SAVED: OPEN NOW" : "SAVED: CLOSED", buf);
        menu_back_to_main();
    }
}

void menu_key(char k)
{
    /* idle: only * does anything, it opens the menu */
    if (menu_state == M_IDLE) {
        if (k == '*') {
            menu_cursor = 0;
            menu_state = M_MAIN;
            menu_build_items();
            menu_last_key_ms = now_ms();
            menu_dirty = true;
            printf("MENU: opened (keypad)\n");
        }
        return;
    }

    menu_last_key_ms = now_ms();
    menu_dirty = true;

    switch (menu_state) {
    case M_MAIN:
        if (k == '2') {
            menu_cursor = (menu_cursor > 0) ? menu_cursor - 1 : menu_item_n - 1;
        } else if (k == '8') {
            menu_cursor = (menu_cursor + 1) % menu_item_n;
        } else if (k == '#') {
            menu_select(menu_items[menu_cursor]);
        } else if (k == '*') {
            menu_close();
        }
        break;

    case M_INFO:
        if (info_kind == IT_SYS && k == '2' && menu_page > 0) menu_page--;
        else if (info_kind == IT_SYS && k == '8' && menu_page < 2) menu_page++;
        else if (k == '*' || k == '#') menu_back_to_main();
        break;

    case M_WHO:
        if (k == '2' && list_cursor > 0) list_cursor--;
        else if (k == '8') list_cursor++;
        else if (k == '*' || k == '#') menu_back_to_main();
        break;

    case M_LOG_WAIT:
        if (k == '*') menu_back_to_main();
        break;

    case M_LOG:
        if (k == '2' && list_cursor > 0) list_cursor--;
        else if (k == '8') list_cursor++;
        else if (k == '*' || k == '#') menu_back_to_main();
        break;

    case M_PIN:
    case M_REG_ID:
    case M_TIME_OPEN:
    case M_TIME_CLOSE: {
        int maxlen = (menu_state == M_TIME_OPEN || menu_state == M_TIME_CLOSE) ? 4 : 6;
        /* the tag-ID screen also takes the letter keys A-D */
        bool letters_ok = (menu_state == M_REG_ID);
        bool is_digit = (k >= '0' && k <= '9');
        bool is_letter = (k >= 'A' && k <= 'D');

        if (is_digit || (letters_ok && is_letter)) {
            if (menu_len < maxlen) {
                menu_buf[menu_len++] = k;
                menu_buf[menu_len] = 0;
            }
        } else if (k == '*') {
            if (menu_len > 0) {
                menu_len--;
                menu_buf[menu_len] = 0;
            } else {
                menu_back_to_main();
            }
        } else if (k == '#') {
            menu_submit();
        }
        break;
    }

    case M_REG_SCAN:
        if (k == '*') {
            printf("MENU: tag registration cancelled\n");
            menu_back_to_main();
        }
        break;

    case M_DEL:
        build_del_list();
        if (k == '2' && list_cursor > 0) {
            list_cursor--;
        } else if (k == '8') {
            list_cursor++;
        } else if (k == '#') {
            if (del_n > 0 && list_cursor < del_n) {
                int ui = del_list[list_cursor];
                if (users[ui].inside || passage_user_pending(ui)) {
                    lcd_msg(LCD_ENTRY, "CANNOT REMOVE", "Inside/pending");
                } else {
                    del_target = ui;
                    menu_state = M_DEL_CONF;
                }
            }
        } else if (k == '*') {
            menu_back_to_main();
        }
        break;

    case M_DEL_CONF:
        if (k == '#') {
            if (!admin_active()) {
                lcd_msg(LCD_ENTRY, "SESSION EXPIRED", "Log in again");
                menu_back_to_main();
            } else if (del_target >= 0 && users[del_target].used && !users[del_target].inside && !passage_user_pending(del_target)) {
                admin_touch();
                char name[16];
                strncpy(name, users[del_target].name, sizeof(name) - 1);
                name[sizeof(name) - 1] = 0;
                char uidtxt[9];
                uid_to_hex(users[del_target].uid, uidtxt);
                memset(&users[del_target], 0, sizeof(user_t));
                if (!users_save()) { lcd_msg(LCD_ENTRY, "SAVE FAILED", "Change undone"); return; }
                log_event(F_SYSTEM, "INFO,user removed %s (%s) via keypad", uidtxt, name);
                printf("User removed: %s (keypad)\n", name);
                lcd_msg(LCD_ENTRY, "USER REMOVED", name);
                del_target = -1;
                menu_back_to_main();
            } else {
                menu_back_to_main();
            }
        } else if (k == '*') {
            menu_state = M_DEL;
        }
        break;
    }
}

/* called every loop: timeout, live refresh, SD log arrival */
void menu_tick(void)
{
    if (menu_state == M_IDLE) return;

    uint32_t now = now_ms();

    if (now - menu_last_key_ms > (uint32_t)MENU_TIMEOUT_MS) {
        menu_close();
        return;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    bool log_ready = lcdlog_ready;
    xSemaphoreGive(log_mutex);
    if (menu_state == M_LOG_WAIT && log_ready) {
        list_cursor = 0;
        menu_state = M_LOG;
        menu_dirty = true;
    }

    if (menu_dirty || (now - menu_refresh_ms) > 500) {
        menu_render();
        menu_dirty = false;
        menu_refresh_ms = now;
    }
}


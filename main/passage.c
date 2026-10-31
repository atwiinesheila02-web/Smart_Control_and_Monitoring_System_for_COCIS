#include "shared.h"

/* ================= SECTION: actuators.c ================= */

/* ================= BUZZER ================= */
static bool closing_buzz_active;
static uint32_t closing_buzz_start;
void buzzer_closing_warning(void)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    closing_buzz_start = now_ms();
    closing_buzz_active = true;
    xSemaphoreGive(output_mutex);
}

void buzzer_pattern(int count, int on_ms, int off_ms)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    pat_count = 0;
    pat_on_ms = (uint32_t)on_ms;
    pat_off_ms = (uint32_t)off_ms;
    pat_start_ms = now_ms();
    pat_count = count;
    xSemaphoreGive(output_mutex);
}

void buzzer_update(void)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    bool alarm = emergency_exit_required();
    bool want = alarm && ((now_ms() / (uint32_t)EMERGENCY_BUZZ_MS) % 2 == 0);
    uint32_t warning_elapsed = now_ms() - closing_buzz_start;
    if (closing_buzz_active && (alarm || warning_elapsed >= (uint32_t)CLOSING_BUZZ_MS))
        closing_buzz_active = false;
    if (!alarm && closing_buzz_active) {
        want = (warning_elapsed / (uint32_t)CLOSING_BEEP_MS) % 2 == 0;
    } else if (!alarm && pat_count > 0) {
        uint32_t elapsed = now_ms() - pat_start_ms;
        uint32_t period = pat_on_ms + pat_off_ms;
        if (period > 0) {
            uint32_t index = elapsed / period;
            want = (index < (uint32_t)pat_count) && ((elapsed % period) < pat_on_ms);
        }
    }
    if (want != buzzer_is_on) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, want ? 512 : 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        buzzer_is_on = want;
    }
    xSemaphoreGive(output_mutex);
}

/* ================= DOORS ================= */
static uint32_t angle_to_duty(int angle)
{
    uint32_t pulse_us = 544 + (uint32_t)angle * (2400 - 544) / 180;
    return pulse_us * 16384UL / 20000UL;
}

static void servo_write(door_t *door, int angle)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, door->channel, angle_to_duty(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, door->channel);
}

void door_init(door_t *door)
{
    ledc_channel_config_t ch = {
        .gpio_num = door->servo_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = door->channel,
        .timer_sel = LEDC_TIMER_0,
        .duty = angle_to_duty(DOOR_CLOSED_ANGLE),
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    pin_set(door->led_pin, 0);
    door->open = false;
}

static void hold_exit_locked(void)
{
    if (!exit_door.open) servo_write(&exit_door, DOOR_OPEN_ANGLE);
    exit_door.open = true;
    exit_door.close_at_us = 0;
    pin_set(exit_door.led_pin, (now_ms() / (uint32_t)EMERGENCY_FLASH_MS) % 2 == 0);
}

static void close_locked(door_t *door)
{
    if (!door->open || passage_door_hold(door)) return;
    servo_write(door, DOOR_CLOSED_ANGLE);
    pin_set(door->led_pin, 0);
    door->open = false;
    /* No logging while holding output_mutex: evacuation must not wait on UART. */
}

void door_open(door_t *door)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    if (emergency_exit_required()) {
        if (door == &exit_door) hold_exit_locked();
        else close_locked(door);
    } else {
        servo_write(door, DOOR_OPEN_ANGLE);
        pin_set(door->led_pin, 1);
        door->open = true;
        door->close_at_us = esp_timer_get_time() + (int64_t)DOOR_OPEN_MS * 1000;

    }
    xSemaphoreGive(output_mutex);
}

void door_close(door_t *door)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    if (door == &exit_door && emergency_exit_required()) hold_exit_locked();
    else close_locked(door);
    xSemaphoreGive(output_mutex);
}

void door_update(door_t *door)
{
    xSemaphoreTake(output_mutex, portMAX_DELAY);
    if (emergency_exit_required()) {
        if (door == &exit_door) hold_exit_locked();
        else close_locked(door);
    } else if (door->open && esp_timer_get_time() >= door->close_at_us) close_locked(door);
    xSemaphoreGive(output_mutex);
}

/* ================= SECTION: passage.c ================= */
#include "nvs.h"
#include "nvs_flash.h"

/* State mutations run in the entrance task under state_mutex. Only the priority
   input task calls passage_sample(); it never touches NVS or access records. */
typedef struct {
    bool candidate, stable;
    uint32_t candidate_since;
    _Atomic bool blocked;
    _Atomic uint32_t blocks, completed, completed_at;
} beam_t;
typedef struct {
    bool active;
    int user_index;
    uint8_t uid[4];
    uint32_t base, deadline;
} gate_t;
typedef struct { uint32_t magic, version, mask; uint8_t uid[2][4]; } intent_t;
static beam_t beams[2];
static gate_t gates[2];
static uint32_t seen[2];
static _Atomic bool initialized, holds[2];
static _Atomic uint32_t deadlines[2];
static bool journal_available, journal_ok;
static uint32_t unresolved;
static nvs_handle_t passage_nvs;

static int beam_pin(int i) { return i ? EXIT_PASSAGE_PIN : ENTRY_PASSAGE_PIN; }
bool passage_sensor_blocked(bool is_exit)
{
    int i = is_exit ? 1 : 0;
    return atomic_load(&initialized) && (pin_get(beam_pin(i)) == PASSAGE_ACTIVE_LEVEL || atomic_load(&beams[i].blocked));
}
bool passage_waiting(bool is_exit) { return atomic_load(&holds[is_exit ? 1 : 0]); }
bool passage_healthy(void) { return journal_ok; }
int passage_reserved_exits(void) { return passage_waiting(true) ? 1 : 0; }
bool passage_user_pending(int ui)
{
    return (gates[0].active && gates[0].user_index == ui) || (gates[1].active && gates[1].user_index == ui);
}
bool passage_door_hold(const door_t *door)
{
    if (!atomic_load(&initialized)) return false;
    int i = door == &exit_door ? 1 : 0;
    /* A broken beam only prevents closure of an already open door. It never
       opens a locked entrance by itself. Actuator callers enforce that rule. */
    return passage_sensor_blocked(i != 0) ||
        (atomic_load(&holds[i]) && (int32_t)(atomic_load(&deadlines[i]) - now_ms()) > 0);
}

static bool save_intents(void)
{
    intent_t r = { .magic = 0x50415353, .version = 1, .mask = unresolved };
    for (int i = 0; i < 2; i++) if (gates[i].active) {
        r.mask |= 1U << i;
        memcpy(r.uid[i], gates[i].uid, 4);
    }
    esp_err_t err = ESP_FAIL;
    if (journal_available) {
        err = nvs_set_blob(passage_nvs, "intents", &r, sizeof(r));
        if (err == ESP_OK) err = nvs_commit(passage_nvs);
    }
    journal_ok = err == ESP_OK;
    if (!journal_ok) occupancy_mark_uncertain("passage journal write failed");
    return journal_ok;
}

void passage_init(void)
{
    atomic_store(&initialized, false);
    memset(gates, 0, sizeof(gates));
    unresolved = 0;
    for (int i = 0; i < 2; i++) {
        pin_input(beam_pin(i), PASSAGE_ACTIVE_LEVEL == 0);
        if (PASSAGE_ACTIVE_LEVEL != 0)
            ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)beam_pin(i), GPIO_PULLDOWN_ONLY));
        bool active = pin_get(beam_pin(i)) == PASSAGE_ACTIVE_LEVEL;
        beams[i].candidate = beams[i].stable = active;
        beams[i].candidate_since = now_ms();
        atomic_store(&beams[i].blocked, active);
        atomic_store(&beams[i].blocks, active ? 1 : 0);
        atomic_store(&beams[i].completed, 0);
        atomic_store(&beams[i].completed_at, 0);
        atomic_store(&holds[i], false);
        atomic_store(&deadlines[i], 0);
        seen[i] = 0;
    }
    journal_available = nvs_flash_init() == ESP_OK &&
        nvs_open("smartguard_ps", NVS_READWRITE, &passage_nvs) == ESP_OK;
    journal_ok = journal_available;
    if (journal_available) {
        intent_t r = {0}; size_t size = sizeof(r);
        esp_err_t err = nvs_get_blob(passage_nvs, "intents", &r, &size);
        if (err == ESP_OK && size == sizeof(r) && r.magic == 0x50415353 && r.version == 1 && !(r.mask & ~3U)) {
            unresolved = r.mask;
        } else if (err != ESP_ERR_NVS_NOT_FOUND) journal_ok = false;
    }
    if (!journal_ok || unresolved)
        occupancy_mark_uncertain("restart with unresolved passage or unreadable passage journal");
    atomic_store(&initialized, true);
}

void passage_sample(void)
{
    if (!atomic_load(&initialized)) return;
    uint32_t now = now_ms();
    for (int i = 0; i < 2; i++) {
        beam_t *b = &beams[i];
        bool raw = pin_get(beam_pin(i)) == PASSAGE_ACTIVE_LEVEL;
        if (raw != b->candidate) { b->candidate = raw; b->candidate_since = now; }
        if (raw == b->stable || now - b->candidate_since < (uint32_t)PASSAGE_DEBOUNCE_MS) continue;
        b->stable = raw;
        atomic_store(&b->blocked, raw);
        if (raw) atomic_fetch_add(&b->blocks, 1);
        else {
            atomic_store(&b->completed_at, now);
            atomic_store(&b->completed, atomic_load(&b->blocks));
        }
    }
}

static void report_gate(int i, const gate_t *g, const char *result, const char *detail)
{
    char uid[9]; uid_to_hex(g->uid, uid);
    const char *name = g->user_index >= 0 && g->user_index < MAX_USERS && users[g->user_index].used ?
        users[g->user_index].name : "UNKNOWN";
    log_event(F_ACCESS, "%s,%s,%s,%s,%s,%d", i ? "EXIT" : "ENTRY", uid, name, result, detail, inside_count());
    printf("%s: %s (%s), recorded inside=%d\n", i ? "EXIT" : "ENTRY", result, detail, inside_count());
}

static void cancel_gate(int i, const char *reason, bool unknown)
{
    gate_t old = gates[i];
    gates[i].active = false;
    atomic_store(&holds[i], false);
    if (unknown) occupancy_mark_uncertain(reason);
    save_intents();
    report_gate(i, &old, unknown ? "UNVERIFIED" : "NO_PASSAGE", reason);
    lcd_msg(i ? LCD_EXIT : LCD_ENTRY, unknown ? "CHECK PASSAGE" : "NO PASSAGE", reason);
    door_close(i ? &exit_door : &entry_door);
    checklist_invalidate();
}

void passage_cancel_all(const char *reason, bool unknown)
{
    if (!atomic_load(&initialized)) return;
    for (int i = 0; i < 2; i++) if (gates[i].active) {
        bool activity = atomic_load(&beams[i].blocks) != gates[i].base || passage_sensor_blocked(i != 0);
        cancel_gate(i, reason, unknown || activity);
    }
}

bool passage_begin(bool is_exit, int ui)
{
    int i = is_exit ? 1 : 0;
    if (!atomic_load(&initialized) || !journal_ok || unresolved || ui < 0 || ui >= MAX_USERS ||
        !users[ui].used || emergency_exit_required() || emergency_counts_uncertain() ||
        !state_healthy() || !schedule_open() || users[ui].inside != is_exit) return false;
    /* Only a corridor occupant can reserve a normal building exit. Room PIR
       inner->outer must return occupants to the corridor first. */
    if (is_exit && corridor_count() <= passage_reserved_exits()) {
        lcd_msg(LCD_EXIT, "ACCESS DENIED", "Leave room first"); return false;
    }
    if (gates[i].active || passage_user_pending(ui)) {
        lcd_msg(i ? LCD_EXIT : LCD_ENTRY, "ACCESS DENIED", ""); return false;
    }
    if (passage_sensor_blocked(is_exit) || atomic_load(&beams[i].blocked) ||
        atomic_load(&beams[i].completed) != seen[i]) {
        lcd_msg(i ? LCD_EXIT : LCD_ENTRY, "ACCESS DENIED", ""); return false;
    }
    gate_t *g = &gates[i];
    g->active = true; g->user_index = ui; memcpy(g->uid, users[ui].uid, 4);
    g->base = atomic_load(&beams[i].blocks);
    g->deadline = now_ms() + PASSAGE_TIMEOUT_MS;
    if (!save_intents()) { g->active = false; return false; }
    if (emergency_exit_required()) { cancel_gate(i, "emergency", true); return false; }
    if (passage_sensor_blocked(is_exit) || atomic_load(&beams[i].blocks) != g->base ||
        atomic_load(&beams[i].completed) != seen[i]) {
        cancel_gate(i, "sensor activity before door opening", true); return false;
    }
    g->deadline = now_ms() + PASSAGE_TIMEOUT_MS;
    atomic_store(&deadlines[i], g->deadline);
    atomic_store(&holds[i], true);
    checklist_invalidate();
    door_open(is_exit ? &exit_door : &entry_door);
    lcd_msg(i ? LCD_EXIT : LCD_ENTRY, "ACCESS APPROVED", "Pass through");
    report_gate(i, g, "PENDING", "awaiting beam blocked then clear");
    return true;
}

static void confirm_gate(int i)
{
    gate_t old = gates[i]; int ui = old.user_index;
    if (ui < 0 || ui >= MAX_USERS || !users[ui].used || memcmp(users[ui].uid, old.uid, 4) ||
        users[ui].inside != (i != 0) || !state_healthy() || !schedule_open() ||
        emergency_counts_uncertain()) {
        cancel_gate(i, "passage could not be reconciled", true); return;
    }
    if (i && corridor_count() <= 0) {
        cancel_gate(i, "no corridor occupant for building exit", true); return;
    }
    users[ui].inside = i == 0; users[ui].since = 0;
    if (!i) {
        struct tm t; clock_get(&t);
        users[ui].since = make_epoch(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    }
    if (!state_commit()) {
        if (gates[i].active) cancel_gate(i, "state save failed after passage", true);
        return;
    }
    gates[i].active = false;
    atomic_store(&holds[i], false);
    bool saved = save_intents();
    door_close(i ? &exit_door : &entry_door);
    report_gate(i, &old, "CONFIRMED", "beam blocked then clear");
    lcd_msg(i ? LCD_EXIT : LCD_ENTRY, "PASSAGE VERIFIED", users[ui].name);
    if (saved && i && inside_count() == 0 && !emergency_counts_uncertain()) checklist_start();
}

void passage_poll(void)
{
    if (!atomic_load(&initialized)) return;
    if (emergency_exit_required()) { passage_cancel_all("emergency evacuation", true); return; }
    for (int i = 0; i < 2; i++) {
        uint32_t completed = atomic_load(&beams[i].completed);
        uint32_t completed_at = atomic_load(&beams[i].completed_at);
        bool new_cycle = completed != seen[i];
        if (new_cycle) seen[i] = completed;
        if (!gates[i].active) {
            if (new_cycle) {
                occupancy_mark_uncertain("passage without an authorized card window");
                checklist_invalidate();
            }
            continue;
        }
        gate_t *g = &gates[i];
        if (new_cycle) {
            if (completed != g->base + 1U || atomic_load(&beams[i].blocks) != completed ||
                (int32_t)(completed_at - g->deadline) > 0) {
                cancel_gate(i, "ambiguous or late passage", true);
            } else confirm_gate(i);
        } else if (!schedule_open() || !state_healthy() || emergency_counts_uncertain()) {
            cancel_gate(i, "authorization no longer valid", atomic_load(&beams[i].blocks) != g->base);
        } else if ((int32_t)(now_ms() - g->deadline) >= 0) {
            cancel_gate(i, "passage window expired", atomic_load(&beams[i].blocks) != g->base || passage_sensor_blocked(i != 0));
        }
    }
}

bool passage_reconcile_clear(void)
{
    if (!atomic_load(&initialized)) return true;
    if (gates[0].active || gates[1].active || passage_sensor_blocked(false) || passage_sensor_blocked(true)) {
        printf("DENIED: doorway busy/blocked; clear it before reconciliation\n"); return false;
    }
    uint32_t old = unresolved; unresolved = 0;
    if (!save_intents()) { unresolved = old; return false; }
    seen[0] = atomic_load(&beams[0].completed); seen[1] = atomic_load(&beams[1].completed);
    return true;
}

void passage_print_status(void)
{
    printf("Passage journal: %s; unresolved restart mask=%lu\n", journal_ok ? "OK" : "FAULT", (unsigned long)unresolved);
    for (int i = 0; i < 2; i++) printf("%s: %s, beam=%s\n", i ? "EXIT" : "ENTRY",
        gates[i].active ? "WAITING FOR PASSAGE" : "idle", passage_sensor_blocked(i != 0) ? "BLOCKED" : "clear");
}

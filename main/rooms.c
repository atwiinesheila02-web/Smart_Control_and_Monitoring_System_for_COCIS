#include "shared.h"

/* ================= SECTION: i2c_bus.c ================= */

/* ================= I2C BUS (open-drain GPIO, LCDs + RTC) =================
 * Software I2C on SDA=39 / SCL=40. Callers hold i2c_mutex once tasks run. */
static void i2c_line(int pin, int level)
{ gpio_set_level((gpio_num_t)pin, (uint32_t)level); }

void i2c_bus_init(void)
{
    const int pins[2] = { LCD_SDA_PIN, LCD_SCL_PIN };
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin((gpio_num_t)pins[i]);
        gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_pull_mode((gpio_num_t)pins[i], GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)pins[i], 1);
    }
}

static void i2c_scl_high(void)
{
    i2c_line(LCD_SCL_PIN, 1);
    for (int i = 0; i < 1000 && pin_get(LCD_SCL_PIN) == 0; i++) delay_us(1);  /* clock stretch */
    delay_us(I2C_HALF_US);
}

static void i2c_start(void)
{
    i2c_line(LCD_SDA_PIN, 1);
    i2c_scl_high();
    i2c_line(LCD_SDA_PIN, 0);
    delay_us(I2C_HALF_US);
    i2c_line(LCD_SCL_PIN, 0);
    delay_us(I2C_HALF_US);
}

static void i2c_stop(void)
{
    i2c_line(LCD_SDA_PIN, 0);
    delay_us(I2C_HALF_US);
    i2c_scl_high();
    i2c_line(LCD_SDA_PIN, 1);
    delay_us(I2C_HALF_US);
}

/* returns true when the device acknowledged */
static bool i2c_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        i2c_line(LCD_SDA_PIN, (b >> i) & 1);
        delay_us(I2C_HALF_US);
        i2c_scl_high();
        i2c_line(LCD_SCL_PIN, 0);
    }
    i2c_line(LCD_SDA_PIN, 1);
    delay_us(I2C_HALF_US);
    i2c_scl_high();
    bool ack = pin_get(LCD_SDA_PIN) == 0;
    i2c_line(LCD_SCL_PIN, 0);
    delay_us(I2C_HALF_US);
    return ack;
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t b = 0;
    i2c_line(LCD_SDA_PIN, 1);
    for (int i = 0; i < 8; i++) {
        delay_us(I2C_HALF_US);
        i2c_scl_high();
        b = (uint8_t)((b << 1) | (pin_get(LCD_SDA_PIN) & 1));
        i2c_line(LCD_SCL_PIN, 0);
    }
    i2c_line(LCD_SDA_PIN, ack ? 0 : 1);
    delay_us(I2C_HALF_US);
    i2c_scl_high();
    i2c_line(LCD_SCL_PIN, 0);
    i2c_line(LCD_SDA_PIN, 1);
    delay_us(I2C_HALF_US);
    return b;
}

/* len may be 0: then this only checks that the address answers */
bool i2c_write(uint8_t addr, const uint8_t *data, int len)
{
    i2c_start();
    bool ok = i2c_write_byte((uint8_t)(addr << 1));
    for (int i = 0; ok && i < len; i++) ok = i2c_write_byte(data[i]);
    i2c_stop();
    return ok;
}

bool i2c_read(uint8_t addr, uint8_t *data, int len)
{
    i2c_start();
    bool ok = i2c_write_byte((uint8_t)((addr << 1) | 1));
    if (ok) for (int i = 0; i < len; i++) data[i] = i2c_read_byte(i < len - 1);
    i2c_stop();
    return ok;
}


/* ================= SECTION: display.c ================= */

/* ================= LCD (I2C, PCF8574 backpack) ================= */
static bool lcd_raw(uint8_t addr, uint8_t data)
{
    return i2c_write(addr, &data, 1);
}

/* EN high then EN low in one I2C transaction (the PCF8574 latches every byte) */
static void lcd_pulse(uint8_t addr, uint8_t data)
{
    uint8_t seq[2] = { (uint8_t)(data | LCD_EN), (uint8_t)(data & ~LCD_EN) };
    i2c_write(addr, seq, 2);
    delay_us(60);
}

static void lcd_write4(uint8_t addr, uint8_t nibble, uint8_t rs)
{
    lcd_pulse(addr, (uint8_t)((nibble << 4) | LCD_BL | rs));
}

static void lcd_send(uint8_t addr, uint8_t value, uint8_t rs)
{
    lcd_write4(addr, (uint8_t)(value >> 4), rs);
    lcd_write4(addr, (uint8_t)(value & 0x0F), rs);
}

static void lcd_hw_init(uint8_t addr)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write4(addr, 0x03, 0);
    delay_us(5000);   /* busy waits: at 100 Hz tick, pdMS_TO_TICKS(<10) is 0 */
    lcd_write4(addr, 0x03, 0);
    delay_us(2000);
    lcd_write4(addr, 0x03, 0);
    lcd_write4(addr, 0x02, 0);
    lcd_send(addr, 0x28, 0);
    lcd_send(addr, 0x0C, 0);
    lcd_send(addr, 0x06, 0);
    lcd_send(addr, 0x01, 0);
    delay_us(3000);
}

static void lcd_draw(uint8_t addr, const char *l1, const char *l2)
{
    lcd_send(addr, 0x80, 0);
    for (int i = 0; i < 16; i++) lcd_send(addr, (uint8_t)l1[i], 1);
    lcd_send(addr, 0xC0, 0);
    for (int i = 0; i < 16; i++) lcd_send(addr, (uint8_t)l2[i], 1);
}

void lcd_idle(int id, const char *l1, const char *l2)
{
    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    snprintf(lcds[id].idle[0], 17, "%-16.16s", l1);
    snprintf(lcds[id].idle[1], 17, "%-16.16s", l2);
    xSemaphoreGive(lcd_mutex);
}

void lcd_msg(int id, const char *l1, const char *l2)
{
    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    snprintf(lcds[id].msg[0], 17, "%-16.16s", l1);
    snprintf(lcds[id].msg[1], 17, "%-16.16s", l2);
    lcds[id].msg_until_ms = now_ms() + LCD_MSG_MS;
    lcds[id].msg_active = true;
    xSemaphoreGive(lcd_mutex);
}

void lcd_init_all(void)
{
    for (int i = 0; i < LCD_COUNT; i++) {
        lcds[i].addr = lcd_addrs[i];
        lcds[i].present = lcd_raw(lcds[i].addr, LCD_BL);
        if (lcds[i].present) {
            lcd_hw_init(lcds[i].addr);
            printf("LCD %s at 0x%02X: OK\n", lcd_names[i], lcds[i].addr);
        } else {
            printf("LCD %s at 0x%02X: NOT FOUND (check i2cAddress, SDA=%d, SCL=%d)\n",
                   lcd_names[i], lcds[i].addr, LCD_SDA_PIN, LCD_SCL_PIN);
        }
        snprintf(lcds[i].idle[0], 17, "%-16.16s", lcd_names[i]);
        snprintf(lcds[i].idle[1], 17, "%-16.16s", "");
    }
}

/* Select the actual screen under the state lock, before touching I2C.
 * Transient access messages and the admin menu remain usable; emergency wins. */
void lcd_render_lines(int id, char line1[17], char line2[17])
{
    state_lock();
    lcd_t *d = &lcds[id];
    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    bool showing = d->msg_active && (int32_t)(d->msg_until_ms - now_ms()) > 0;
    if (!showing) d->msg_active = false;
    memcpy(line1, showing ? d->msg[0] : d->idle[0], 17);
    memcpy(line2, showing ? d->msg[1] : d->idle[1], 17);
    xSemaphoreGive(lcd_mutex);

    if (id != LCD_ENTRY || menu_state == M_IDLE) {
        if (!schedule_open()) {
            snprintf(line1, 17, "%-16.16s", "BUILDING CLOSED");
            snprintf(line2, 17, "%-16.16s", "");
        } else if (!showing) {
            int remaining = (sched_close_min - minutes_now() + 1440) % 1440;
            if (remaining > 0 && remaining <= CLOSING_WARNING_MIN) {
                char countdown[17];
                snprintf(countdown, sizeof(countdown), "IN %d MINUTE%s", remaining,
                         remaining == 1 ? "" : "S");
                snprintf(line1, 17, "%-16.16s", "BUILDING CLOSING");
                snprintf(line2, 17, "%-16.16s", countdown);
            }
        }
    }
    emergency_display(id, line1, line2);
    state_unlock();
}

void lcd_task(void *arg)
{
    (void)arg;
    for (;;) {
        for (int i = 0; i < LCD_COUNT; i++) {
            lcd_t *d = &lcds[i];
            if (!d->present) continue;

            char l1[17], l2[17];
            lcd_render_lines(i, l1, l2);

            if (!d->shown_valid || strcmp(l1, d->shown[0]) != 0 ||
                strcmp(l2, d->shown[1]) != 0) {
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                lcd_draw(d->addr, l1, l2);
                xSemaphoreGive(i2c_mutex);
                memcpy(d->shown[0], l1, 17);
                memcpy(d->shown[1], l2, 17);
                d->shown_valid = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


/* ================= SECTION: bus.c ================= */

/* ================= MESSAGE BUS ================= */
bool bus_send(int src, int dst, msg_type_t type, int32_t value, int32_t aux)
{
    if (src < 0 || src >= TERM_COUNT || dst < 0 || dst >= TERM_COUNT ||
        (int)type < 0 || (int)type >= 6) return false;
    bus_msg_t m;
    m.type = type;
    m.src = (uint8_t)src;
    m.dst = (uint8_t)dst;
    m.time_ms = now_ms();
    m.value = value;
    m.aux = aux;

    bool ok = xQueueSend(inbox[dst], &m, pdMS_TO_TICKS(50)) == pdTRUE;
    if (ok) {
        if (type != MSG_ROOM_STATE) {
            printf("BUS: %s -> %s : %s (%ld)\n", term_names[src], term_names[dst],
                   msg_names[(int)type], (long)value);
        }
    } else {
        printf("ERROR: inbox of %s is full, message %s lost\n",
               term_names[dst], msg_names[(int)type]);
    }
    return ok;
}


/* ================= SECTION: terminals.c ================= */

/* Keep doors and the buzzer responsive after each RFID poll. */
static void update_outputs(void)
{
    door_update(&entry_door);
    door_update(&exit_door);
    buzzer_update();
}

/* ================= ENTRANCE TERMINAL TASK ================= */
void entrance_terminal(void *arg)
{
    (void)arg;
    for (;;) {
        emergency_poll();
        state_lock();
        emergency_service();
        passage_poll();
        checklist_poll();
        poll_reader(&entry_reader);
        update_outputs();

        poll_reader(&exit_reader);
        update_outputs();

        bus_msg_t m;
        while (xQueueReceive(inbox[TERM_ENTRANCE], &m, 0) == pdTRUE) master_handle(&m);

        char key;
        while (xQueueReceive(key_q, &key, 0) == pdTRUE) {
            printf("KEY: %c\n", key);
            menu_key(key);
        }
        menu_tick();
        emergency_set_menu_visible(menu_state != M_IDLE);

        cli_poll();

        if (now_ms() - last_periodic_ms >= 1000) {
            last_periodic_ms = now_ms();
            master_periodic();
        }

        state_unlock();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================= ROOM TERMINALS (PIR door sensors) ================= */
static void room_set_lights(room_t *room, bool on)
{
    int level = (on == (RELAY_ACTIVE_HIGH != 0)) ? 1 : 0;
    pin_set(room->relay_pin, level);
    room->lights_on = on;
    room_lights[room->term_id - TERM_ROOM1] = on;
}

static void room_send_state(room_t *room)
{
    room_seen_ms[room->term_id - TERM_ROOM1] = now_ms();
    bus_send(room->term_id, TERM_ENTRANCE, MSG_ROOM_STATE, room->count,
             room->lights_on ? 1 : 0);
}

/* lights are on only while the room is occupied AND too dark */
static void room_apply(room_t *room)
{
    bool want = room->count > 0 && room->dark;
    if (want != room->lights_on) {
        room_set_lights(room, want);
        printf("%s: lights %s (people=%d, light=%d lux)\n", room->name,
               want ? "ON" : "OFF", room->count, room->lux);
        log_event(F_ROOMS, "%s,LIGHTS_%s,%d,%d", room->name, want ? "ON" : "OFF",
                  room->count, want ? 1 : 0);
        room_send_state(room);
    }
    char l1[24], l2[24];
    snprintf(l1, sizeof(l1), "%s Light %s", room->name, room->lights_on ? "ON" : "OFF");
    if (emergency_counts_uncertain()) snprintf(l2, sizeof(l2), "Last known:%d", room->count);
    else snprintf(l2, sizeof(l2), "People:%d %dlux", room->count, room->lux);
    lcd_idle(room->lcd_id, l1, l2);
}

/* LDR module: AO = VCC * R / (R + 10k); converts the ADC reading to lux */
static void room_read_light(room_t *room)
{
    int raw = 0;
    xSemaphoreTake(adc_mutex, portMAX_DELAY);
    esp_err_t err = adc_oneshot_read(adc1, room->ldr_ch, &raw);
    xSemaphoreGive(adc_mutex);
    if (err != ESP_OK) return;

    float v = (float)raw * LDR_VCC / 4095.0f;
    float lux;
    if (v >= LDR_VCC - 0.01f) {
        lux = 0.0f;
    } else if (v <= 0.001f) {
        lux = 100000.0f;
    } else {
        float r = 10000.0f * v / (LDR_VCC - v);
        lux = powf(LDR_RL10 * 1000.0f * powf(10.0f, LDR_GAMMA) / r, 1.0f / LDR_GAMMA);
        if (lux > 100000.0f) lux = 100000.0f;
    }
    room->lux = (int)(lux + 0.5f);

    if (!room->dark && room->lux < LUX_ON) room->dark = true;
    else if (room->dark && room->lux > LUX_OFF) room->dark = false;
}

static void sensor_init(sensor_t *s, int pin)
{
    memset((void *)s, 0, sizeof(sensor_t));
    s->pin = pin;
    pin_input(pin, false);     /* the PIR drives OUT high/low itself */
}

void room_init(room_t *room, const char *name, int term_id, int lcd_id,
                      int relay, int pir_out, int pir_in, adc_channel_t ldr_ch)
{
    memset((void *)room, 0, sizeof(room_t));
    room->name = name;
    room->term_id = term_id;
    room->lcd_id = lcd_id;
    room->relay_pin = relay;
    room->seq = RS_IDLE;
    room->count = master_room[term_id - TERM_ROOM1];
    sensor_init(&room->a, pir_out);
    sensor_init(&room->b, pir_in);
    room->ldr_ch = ldr_ch;
    pin_output(relay, 0);
    room_set_lights(room, false);
    room_read_light(room);
    room_apply(room);
}

/* Commit movement through the shared coordinator before changing room outputs. */
static void room_person_entering(room_t *room)
{
    if (record_room_motion(room, true)) room_apply(room);
}

/* Entry and exit use the same protected checkpoint operation. */
static void room_person_leaving(room_t *room)
{
    if (record_room_motion(room, false)) room_apply(room);
}

/* reads one PIR; returns true when it newly detects motion (LOW -> HIGH) */
static bool pir_update(sensor_t *s, const char *room, const char *label)
{
    bool level = pin_get(s->pin) == 1;
    bool newly = level && !s->active;
    if (newly) {
        s->since_ms = now_ms();
        printf("%s: %s PIR detected motion\n", room, label);
    }
    s->active = level;
    return newly;
}

/* Record the next PIR state and the time it began in one place. */
static void room_set_sequence(room_t *room, int state, uint32_t now)
{
    room->seq = state;
    room->seq_since_ms = now;
}

/* Outside then inside means entry; the reverse means exit. */
static void room_update_sequence(room_t *room, bool a_new, bool b_new, uint32_t now)
{
    switch (room->seq) {
    case RS_IDLE:
        if (a_new && b_new) room_set_sequence(room, RS_CLEARING, now);
        else if (a_new) room_set_sequence(room, RS_WAIT_B, now);
        else if (b_new) room_set_sequence(room, RS_WAIT_A, now);
        break;

    case RS_WAIT_B:
    case RS_WAIT_A: {
        bool entering = room->seq == RS_WAIT_B;
        bool second_sensor = entering ? b_new : a_new;
        if (now - room->seq_since_ms > (uint32_t)SEQ_TIMEOUT_MS) {
            printf("%s: %s sequence timed out (%s sensor not triggered)\n",
                   room->name, entering ? "entry" : "exit",
                   entering ? "inside" : "outside");
        } else if (second_sensor) {
            if (entering) room_person_entering(room);
            else room_person_leaving(room);
        } else {
            break; /* Keep waiting for the second sensor. */
        }
        room_set_sequence(room, RS_CLEARING, now);
        break;
    }

    case RS_CLEARING:
        if (!room->a.active && !room->b.active) {
            room->seq = RS_IDLE;
            room->fault_sent = false;
        } else if (!room->fault_sent && now - room->seq_since_ms > (uint32_t)STUCK_MS) {
            room->fault_sent = true;
            printf("%s: a door PIR has stayed active for too long\n", room->name);
            log_event(F_SYSTEM, "ERROR,%s door sensor stuck", room->name);
            lcd_msg(LCD_ENTRY, "ROOM SENSOR FAULT", room->name);
            bus_send(room->term_id, TERM_ENTRANCE, MSG_ROOM_FAULT, 1, 0);
        }
        break;
    }
}

static void room_receive_messages(room_t *room)
{
    bus_msg_t m;
    /* Occupancy is committed centrally, never incremented by a duplicate reply. */
    while (xQueueReceive(inbox[room->term_id], &m, 0) == pdTRUE) {
        printf("%s: unexpected control message %d ignored\n", room->name, (int)m.type);
    }
}

void room_terminal(void *arg)
{
    room_t *room = (room_t *)arg;
    room->next_hb_ms = now_ms();

    for (;;) {
        state_lock();
        /* test hook from the serial menu */
        int inj = room->inject;
        if (inj != 0) {
            room->inject = 0;
            if (inj == 1) room_person_entering(room);
            else if (inj == 2) room_person_leaving(room);
        }

        bool a_new = pir_update(&room->a, room->name, "outside");
        bool b_new = pir_update(&room->b, room->name, "inside");
        uint32_t now = now_ms();

        room_update_sequence(room, a_new, b_new, now);

        room_receive_messages(room);

        if ((int32_t)(now - room->next_ldr_ms) >= 0) {
            room->next_ldr_ms = now + LIGHT_READ_MS;
            room_read_light(room);
            room_apply(room);
        }

        if ((int32_t)(now - room->next_hb_ms) >= 0) {
            room_send_state(room);
            room->next_hb_ms = now + HB_MS;
        }

        state_unlock();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


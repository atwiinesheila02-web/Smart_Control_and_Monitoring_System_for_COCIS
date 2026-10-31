#ifndef SMARTGUARD_SHARED_H
#define SMARTGUARD_SHARED_H

/* CoCIS Smart Control and Monitoring System
 * Target : ESP32-S3-DevKitC-1 on Wokwi
 * File   : main/main.c of an ESP-IDF project, entry point app_main()
 * APIs   : ESP-IDF only (FreeRTOS, gpio, spi_master, sdspi + FATFS, ledc, uart,
 *          esp_timer).
 * Terminals: ENTRANCE (master), ROOM 1, ROOM 2 run as separate FreeRTOS tasks
 *          that talk over message queues and share resources through mutexes.
 * Rooms  : two PIR motion sensors per door (outside + inside); the order in
 *          which they fire tells entry from exit. A light sensor (LDR) per room
 *          keeps the lights off while the room is bright enough.
 * Clock  : the DS1307 holds local time (Wokwi starts it from the PC's clock).
 * Buzzer : denial tone, or continuous pulsing during an emergency.
 */
#include <stdio.h>
#include <assert.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

#if defined(_WIN32) || defined(__MINGW32__) || defined(__MINGW64__)
#include <direct.h>
#include <errno.h>
#include <stdlib.h>

#ifndef gmtime_r
static inline struct tm *gmtime_r(const time_t *timer, struct tm *result)
{
    if (!timer || !result) return NULL;
    struct tm *tmp = gmtime(timer);
    if (!tmp) return NULL;
    *result = *tmp;
    return result;
}
#endif

#ifndef setenv
static inline int setenv(const char *name, const char *value, int overwrite)
{
    if (!name || !value) return -1;
    if (!overwrite) {
        size_t needed = 0;
        if (getenv_s(&needed, NULL, 0, name) == 0 && needed != 0) return 0;
    }
    return _putenv_s(name, value);
}
#endif

#ifdef mkdtemp
#undef mkdtemp
#endif
#define mkdtemp smartguard_mkdtemp
static inline char *smartguard_mkdtemp(char *tmpl)
{
    if (!tmpl) {
        errno = EINVAL;
        return NULL;
    }
    size_t len = strlen(tmpl);
    if (len < 6 || strcmp(tmpl + len - 6, "XXXXXX") != 0) {
        errno = EINVAL;
        return NULL;
    }
    char *slash = strrchr(tmpl, '/');
    if (slash) {
        size_t keep = (size_t)(slash - tmpl) + 1;
        memmove(tmpl, slash + 1, len - keep + 1);
        len -= keep;
    }
    char *backslash = strrchr(tmpl, '\\');
    if (backslash) {
        size_t keep = (size_t)(backslash - tmpl) + 1;
        memmove(tmpl, backslash + 1, len - keep + 1);
        len -= keep;
    }
    const char *chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    srand((unsigned int)time(NULL) ^ (unsigned int)rand());
    for (size_t i = 0; i < 6; i++) {
        tmpl[len - 6 + i] = chars[rand() % 62];
    }
    if (_mkdir(tmpl) == 0) return tmpl;
    errno = EEXIST;
    return NULL;
}

#ifndef mkdir
#define mkdir(path, mode) _mkdir(path)
#endif
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_system.h"


#ifndef LCD_COUNT
#define LCD_COUNT 4
#endif
#ifndef LCD_ENTRY
#define LCD_ENTRY 0
#endif
#ifndef LCD_EXIT
#define LCD_EXIT 1
#endif
#ifndef LCD_ROOM1
#define LCD_ROOM1 2
#endif
#ifndef LCD_ROOM2
#define LCD_ROOM2 3
#endif
#ifndef TERM_ENTRANCE
#define TERM_ENTRANCE 0
#endif
#ifndef TERM_ROOM1
#define TERM_ROOM1 1
#endif
#ifndef TERM_ROOM2
#define TERM_ROOM2 2
#endif
#ifndef TERM_COUNT
#define TERM_COUNT 3
#endif
#ifndef MAX_USERS
#define MAX_USERS 16
#endif
#ifndef F_ACCESS
#define F_ACCESS 0
#endif
#ifndef F_ROOMS
#define F_ROOMS 1
#endif
#ifndef F_SYSTEM
#define F_SYSTEM 2
#endif
#ifndef F_USERS
#define F_USERS 3
#endif
#ifndef F_CONFIG
#define F_CONFIG 4
#endif
#ifndef SQ_APPEND
#define SQ_APPEND 0
#endif
#ifndef SQ_TAIL
#define SQ_TAIL 1
#endif
#ifndef SQ_EXPORT
#define SQ_EXPORT 2
#endif
#ifndef SQ_FIND
#define SQ_FIND 3
#endif
#ifndef SQ_WRITE
#define SQ_WRITE 4
#endif
#ifndef SQ_LOGLCD
#define SQ_LOGLCD 5
#endif
#ifndef RS_IDLE
#define RS_IDLE 0
#endif
#ifndef RS_WAIT_B
#define RS_WAIT_B 1
#endif
#ifndef RS_WAIT_A
#define RS_WAIT_A 2
#endif
#ifndef RS_CLEARING
#define RS_CLEARING 3
#endif
#ifndef M_IDLE
#define M_IDLE 0
#endif
#ifndef M_MAIN
#define M_MAIN 1
#endif
#ifndef M_INFO
#define M_INFO 2
#endif
#ifndef M_WHO
#define M_WHO 3
#endif
#ifndef M_LOG_WAIT
#define M_LOG_WAIT 4
#endif
#ifndef M_LOG
#define M_LOG 5
#endif
#ifndef M_PIN
#define M_PIN 6
#endif
#ifndef M_REG_ID
#define M_REG_ID 7
#endif
#ifndef M_REG_SCAN
#define M_REG_SCAN 8
#endif
#ifndef M_DEL
#define M_DEL 9
#endif
#ifndef M_DEL_CONF
#define M_DEL_CONF 10
#endif
#ifndef M_TIME_OPEN
#define M_TIME_OPEN 11
#endif
#ifndef M_TIME_CLOSE
#define M_TIME_CLOSE 12
#endif
#ifndef IT_OCC
#define IT_OCC 0
#endif
#ifndef IT_ROOMS
#define IT_ROOMS 1
#endif
#ifndef IT_WHO
#define IT_WHO 2
#endif
#ifndef IT_LOG
#define IT_LOG 3
#endif
#ifndef IT_SYS
#define IT_SYS 4
#endif
#ifndef IT_LOGIN
#define IT_LOGIN 5
#endif
#ifndef IT_REG
#define IT_REG 6
#endif
#ifndef IT_DEL
#define IT_DEL 7
#endif
#ifndef IT_OPEN
#define IT_OPEN 8
#endif
#ifndef IT_CLOSE
#define IT_CLOSE 9
#endif
#ifndef IT_EM_RESET
#define IT_EM_RESET 10
#endif
#ifndef IT_LOGOUT
#define IT_LOGOUT 11
#endif
/* ================= TYPES (keep above all functions) ================= */
typedef enum {
    MSG_ROOM_ENTER_REQ = 0,
    MSG_ROOM_EXIT_NOTICE,
    MSG_ROOM_STATE,
    MSG_ROOM_FAULT,
    MSG_ENTER_GRANTED,
    MSG_ENTER_DENIED
} msg_type_t;

typedef struct {
    msg_type_t type;
    uint8_t src;
    uint8_t dst;
    uint32_t time_ms;
    int32_t value;
    int32_t aux;
} bus_msg_t;

typedef struct {
    const char *name;
    ledc_channel_t channel;
    int servo_pin;
    int led_pin;
    bool open;
    int64_t close_at_us;
} door_t;

typedef struct {
    int cs;
    spi_device_handle_t dev;
    const char *name;
    door_t *door;
    bool is_exit;
    bool available;
    bool reported;
    uint8_t last_uid[4];
    int64_t last_seen_us;
} reader_t;

typedef struct {
    int pin;               /* PIR OUT pin: HIGH while motion is seen */
    bool active;           /* last level read */
    uint32_t since_ms;     /* when it last went HIGH */
} sensor_t;

typedef struct {
    const char *name;
    int term_id;
    int lcd_id;
    int relay_pin;
    sensor_t a;            /* outside the door */
    sensor_t b;            /* inside the door  */
    adc_channel_t ldr_ch;  /* light sensor */
    int lux;               /* last light reading */
    bool dark;             /* lux below LUX_ON (with hysteresis) */
    uint32_t next_ldr_ms;
    int count;             /* coordinator updates under state_mutex */
    bool lights_on;
    int seq;
    uint32_t seq_since_ms;
    bool fault_sent;
    uint32_t next_hb_ms;
    int inject;   /* test hook: 1 = person enters, 2 = person leaves */
} room_t;

typedef struct {
    uint8_t addr;
    bool present;
    char idle[2][17];
    char msg[2][17];
    bool msg_active;
    uint32_t msg_until_ms;
    char shown[2][17];
    bool shown_valid;
} lcd_t;

typedef struct {
    bool used;
    uint8_t uid[4];
    char name[16];
    bool inside;
    time_t since;
} user_t;

typedef struct {
    uint8_t kind;
    uint8_t file;
    int32_t n;
    char text[112];
    char *heap;
} store_item_t;



/* ACCESS PUBLIC API */

bool schedule_open(void);
void sched_text(char *buf, size_t n);
void uid_to_hex(const uint8_t *uid, char *out);
bool parse_uid(const char *s, uint8_t out[4]);
int find_user(const uint8_t *uid);
int add_user(const uint8_t *uid, const char *name);
int inside_count(void);
int users_total(void);
/* Building occupancy uses the whole-building total. The legacy helper remains
   for the room-entry validation logic and compatibility with existing tests. */
int building_count(void);
int corridor_count(void);
bool users_save(void);
bool config_save(void);
void users_defaults(void);
void sd_load_users(void);
void sd_load_config(void);
void entrance_idle_refresh(void);
void handle_scan(bool is_exit, const uint8_t *uid);
void schedule_check(void);
bool record_room_motion(room_t *room, bool entering);
void master_handle(const bus_msg_t *m);
void master_periodic(void);
bool eq(const char *a, const char *b);
void lower(char *s);
bool admin_active(void);
void admin_touch(void);
bool admin_check(void);
int admin_login_attempt(const char *pin, const char *via);

/* ACTUATORS PUBLIC API */

void buzzer_pattern(int count, int on_ms, int off_ms);
void buzzer_update(void);
void buzzer_closing_warning(void);
void door_init(door_t *door);
void door_open(door_t *door);
void door_close(door_t *door);
void door_update(door_t *door);

/* BUS PUBLIC API */

bool bus_send(int src, int dst, msg_type_t type, int32_t value, int32_t aux);

/* CHECKLIST PUBLIC API */
enum { CHECKLIST_IDLE, CHECKLIST_WAITING, CHECKLIST_CLEAR, CHECKLIST_REQUIRED };
void checklist_start(void);
void checklist_poll(void);
void checklist_invalidate(void);
int checklist_status(void);
void checklist_run(void);
const char *checklist_summary(void);

/* CLOCK PUBLIC API */

time_t make_epoch(int y, int mo, int d, int h, int mi, int s);
void clock_init(void);
bool clock_set(time_t value);
void clock_task(void *arg);
void clock_get(struct tm *out);
int minutes_now(void);
void fmt_time(char *buf, size_t n);

/* CONTROLS PUBLIC API */

void cli_poll(void);
void keypad_init(void);
void keypad_task(void *arg);
void menu_key(char k);
void menu_tick(void);

/* DISPLAY PUBLIC API */

void lcd_idle(int id, const char *l1, const char *l2);
void lcd_msg(int id, const char *l1, const char *l2);
void lcd_init_all(void);
void lcd_task(void *arg);
void lcd_render_lines(int id, char line1[17], char line2[17]);

/* EMERGENCY PUBLIC API */
enum { EM_SOURCE_BUTTON = 1, EM_SOURCE_SMOKE = 2, EM_SOURCE_STORAGE = 4 };
void emergency_init(void);
void emergency_poll(void);
void emergency_task(void *arg);
void emergency_service(void);
void occupancy_mark_uncertain(const char *reason);
bool emergency_active(void);
bool emergency_exit_required(void);
bool emergency_counts_uncertain(void);
bool emergency_reset(void);
bool emergency_reconcile_empty(void);
void emergency_print_status(void);
void emergency_set_menu_visible(bool visible);
bool emergency_menu_visible(void);
bool emergency_display(int id, char line1[17], char line2[17]);

/* HELPERS PUBLIC API */

uint32_t now_ms(void);
uint8_t bcd2dec(uint8_t v);
uint8_t dec2bcd(int v);
void delay_us(uint32_t us);
void pin_output(int pin, int level);
void pin_input(int pin, bool pullup);
void pin_set(int pin, int level);
int  pin_get(int pin);

/* I2C_BUS PUBLIC API */

void i2c_bus_init(void);
bool i2c_write(uint8_t addr, const uint8_t *data, int len);
bool i2c_read(uint8_t addr, uint8_t *data, int len);

/* PASSAGE PUBLIC API */
void passage_init(void);
void passage_sample(void);
void passage_poll(void);
bool passage_begin(bool is_exit, int user_index);
bool passage_waiting(bool is_exit);
bool passage_user_pending(int user_index);
int passage_reserved_exits(void);
bool passage_sensor_blocked(bool is_exit);
bool passage_door_hold(const door_t *door);
bool passage_healthy(void);
void passage_cancel_all(const char *reason, bool uncertain);
bool passage_reconcile_clear(void);
void passage_print_status(void);

/* RFID PUBLIC API */

void setup_reader(reader_t *r, const char *name, door_t *door,
                         bool is_exit, int cs);
void poll_reader(reader_t *r);

/* STATE PUBLIC API */
void state_lock(void);
void state_unlock(void);
bool state_init(void);
bool state_commit(void);
bool state_healthy(void);

/* STORAGE PUBLIC API */

bool store_request(uint8_t kind, int file, int n, const char *text);
void store_write(int file, char *heap_text);
void log_event(int file, const char *fmt, ...);
void storage_task(void *arg);
bool sd_mount(void);

/* TERMINALS PUBLIC API */

void entrance_terminal(void *arg);
void room_init(room_t *room, const char *name, int term_id, int lcd_id,
                      int relay, int pir_out, int pir_in, adc_channel_t ldr_ch);
void room_terminal(void *arg);

/* SHARED STATE AND CONFIGURATION ALIASES */

/* Definitions and values are owned by main.c. */
extern const int cfg_SIM_NO_BCC;
#ifndef SMARTGUARD_MAIN
#define SIM_NO_BCC cfg_SIM_NO_BCC
#endif
extern const int cfg_RELAY_ACTIVE_HIGH;
#ifndef SMARTGUARD_MAIN
#define RELAY_ACTIVE_HIGH cfg_RELAY_ACTIVE_HIGH
#endif
extern const char cfg_ADMIN_PIN[];
#ifndef SMARTGUARD_MAIN
#define ADMIN_PIN cfg_ADMIN_PIN
#endif
extern const int cfg_SD_CS_PIN;
#ifndef SMARTGUARD_MAIN
#define SD_CS_PIN cfg_SD_CS_PIN
#endif
extern const int cfg_LCD_SDA_PIN;
#ifndef SMARTGUARD_MAIN
#define LCD_SDA_PIN cfg_LCD_SDA_PIN
#endif
extern const int cfg_LCD_SCL_PIN;
#ifndef SMARTGUARD_MAIN
#define LCD_SCL_PIN cfg_LCD_SCL_PIN
#endif
extern const int cfg_I2C_HALF_US;
#ifndef SMARTGUARD_MAIN
#define I2C_HALF_US cfg_I2C_HALF_US
#endif
extern const int cfg_SPI_HOST_ID;
#ifndef SMARTGUARD_MAIN
#define SPI_HOST_ID cfg_SPI_HOST_ID
#endif
extern const char cfg_SD_MOUNT[];
#ifndef SMARTGUARD_MAIN
#define SD_MOUNT cfg_SD_MOUNT
#endif
extern const int cfg_CLI_UART;
#ifndef SMARTGUARD_MAIN
#define CLI_UART cfg_CLI_UART
#endif
extern const int cfg_RTC_ADDR;
#ifndef SMARTGUARD_MAIN
#define RTC_ADDR cfg_RTC_ADDR
#endif
extern const int cfg_LCD_MSG_MS;
#ifndef SMARTGUARD_MAIN
#define LCD_MSG_MS cfg_LCD_MSG_MS
#endif
extern const int cfg_LCD_BL;
#ifndef SMARTGUARD_MAIN
#define LCD_BL cfg_LCD_BL
#endif
extern const int cfg_LCD_EN;
#ifndef SMARTGUARD_MAIN
#define LCD_EN cfg_LCD_EN
#endif
extern const int cfg_DOOR_OPEN_ANGLE;
#ifndef SMARTGUARD_MAIN
#define DOOR_OPEN_ANGLE cfg_DOOR_OPEN_ANGLE
#endif
extern const int cfg_DOOR_CLOSED_ANGLE;
#ifndef SMARTGUARD_MAIN
#define DOOR_CLOSED_ANGLE cfg_DOOR_CLOSED_ANGLE
#endif
extern const int cfg_DOOR_OPEN_MS;
#ifndef SMARTGUARD_MAIN
#define DOOR_OPEN_MS cfg_DOOR_OPEN_MS
#endif
extern const int cfg_DENIED_MS;
#ifndef SMARTGUARD_MAIN
#define DENIED_MS cfg_DENIED_MS
#endif
extern const int cfg_SEQ_TIMEOUT_MS;
#ifndef SMARTGUARD_MAIN
#define SEQ_TIMEOUT_MS cfg_SEQ_TIMEOUT_MS
#endif
extern const int cfg_STUCK_MS;
#ifndef SMARTGUARD_MAIN
#define STUCK_MS cfg_STUCK_MS
#endif
extern const int cfg_LUX_ON;
#ifndef SMARTGUARD_MAIN
#define LUX_ON cfg_LUX_ON
#endif
extern const int cfg_LUX_OFF;
#ifndef SMARTGUARD_MAIN
#define LUX_OFF cfg_LUX_OFF
#endif
extern const int cfg_LIGHT_READ_MS;
#ifndef SMARTGUARD_MAIN
#define LIGHT_READ_MS cfg_LIGHT_READ_MS
#endif
extern const float cfg_LDR_VCC;
#ifndef SMARTGUARD_MAIN
#define LDR_VCC cfg_LDR_VCC
#endif
extern const float cfg_LDR_RL10;
#ifndef SMARTGUARD_MAIN
#define LDR_RL10 cfg_LDR_RL10
#endif
extern const float cfg_LDR_GAMMA;
#ifndef SMARTGUARD_MAIN
#define LDR_GAMMA cfg_LDR_GAMMA
#endif
extern const int cfg_HB_MS;
#ifndef SMARTGUARD_MAIN
#define HB_MS cfg_HB_MS
#endif
extern const int cfg_ROOM_OFFLINE_MS;
#ifndef SMARTGUARD_MAIN
#define ROOM_OFFLINE_MS cfg_ROOM_OFFLINE_MS
#endif
extern const int cfg_ADMIN_SESSION_MS;
#ifndef SMARTGUARD_MAIN
#define ADMIN_SESSION_MS cfg_ADMIN_SESSION_MS
#endif
extern const int cfg_LOCKOUT_MS;
#ifndef SMARTGUARD_MAIN
#define LOCKOUT_MS cfg_LOCKOUT_MS
#endif
extern const int cfg_MAX_PIN_FAILS;
#ifndef SMARTGUARD_MAIN
#define MAX_PIN_FAILS cfg_MAX_PIN_FAILS
#endif
extern const int cfg_MENU_TIMEOUT_MS;
#ifndef SMARTGUARD_MAIN
#define MENU_TIMEOUT_MS cfg_MENU_TIMEOUT_MS
#endif
extern const int cfg_COMMAND_REG;
#ifndef SMARTGUARD_MAIN
#define COMMAND_REG cfg_COMMAND_REG
#endif
extern const int cfg_COM_IRQ_REG;
#ifndef SMARTGUARD_MAIN
#define COM_IRQ_REG cfg_COM_IRQ_REG
#endif
extern const int cfg_ERROR_REG;
#ifndef SMARTGUARD_MAIN
#define ERROR_REG cfg_ERROR_REG
#endif
extern const int cfg_FIFO_DATA_REG;
#ifndef SMARTGUARD_MAIN
#define FIFO_DATA_REG cfg_FIFO_DATA_REG
#endif
extern const int cfg_FIFO_LEVEL_REG;
#ifndef SMARTGUARD_MAIN
#define FIFO_LEVEL_REG cfg_FIFO_LEVEL_REG
#endif
extern const int cfg_BIT_FRAMING_REG;
#ifndef SMARTGUARD_MAIN
#define BIT_FRAMING_REG cfg_BIT_FRAMING_REG
#endif
extern const int cfg_COLL_REG;
#ifndef SMARTGUARD_MAIN
#define COLL_REG cfg_COLL_REG
#endif
extern const int cfg_MODE_REG;
#ifndef SMARTGUARD_MAIN
#define MODE_REG cfg_MODE_REG
#endif
extern const int cfg_TX_CONTROL_REG;
#ifndef SMARTGUARD_MAIN
#define TX_CONTROL_REG cfg_TX_CONTROL_REG
#endif
extern const int cfg_TX_ASK_REG;
#ifndef SMARTGUARD_MAIN
#define TX_ASK_REG cfg_TX_ASK_REG
#endif
extern const int cfg_T_MODE_REG;
#ifndef SMARTGUARD_MAIN
#define T_MODE_REG cfg_T_MODE_REG
#endif
extern const int cfg_T_PRESCALER_REG;
#ifndef SMARTGUARD_MAIN
#define T_PRESCALER_REG cfg_T_PRESCALER_REG
#endif
extern const int cfg_T_RELOAD_H_REG;
#ifndef SMARTGUARD_MAIN
#define T_RELOAD_H_REG cfg_T_RELOAD_H_REG
#endif
extern const int cfg_T_RELOAD_L_REG;
#ifndef SMARTGUARD_MAIN
#define T_RELOAD_L_REG cfg_T_RELOAD_L_REG
#endif
extern const int cfg_VERSION_REG;
#ifndef SMARTGUARD_MAIN
#define VERSION_REG cfg_VERSION_REG
#endif

extern const char sd_directory[];
extern door_t entry_door;
extern door_t exit_door;
extern reader_t entry_reader;
extern reader_t exit_reader;
extern room_t room1;
extern room_t room2;
extern lcd_t lcds[LCD_COUNT];
extern const char *lcd_names[LCD_COUNT];
extern const uint8_t lcd_addrs[LCD_COUNT];
extern QueueHandle_t inbox[TERM_COUNT];
extern const char *term_names[TERM_COUNT];
extern const char *msg_names[6];
extern QueueHandle_t storage_q;
extern QueueHandle_t key_q;
extern const int cfg_DEFAULT_OPEN_MIN;
extern const int cfg_DEFAULT_CLOSE_MIN;
#ifndef SMARTGUARD_MAIN
#define DEFAULT_OPEN_MIN cfg_DEFAULT_OPEN_MIN
#define DEFAULT_CLOSE_MIN cfg_DEFAULT_CLOSE_MIN
#endif
extern const int cfg_EMERGENCY_BUTTON_PIN;
#ifndef SMARTGUARD_MAIN
#define EMERGENCY_BUTTON_PIN cfg_EMERGENCY_BUTTON_PIN
#endif
extern const int cfg_SMOKE_ALARM_PIN;
#ifndef SMARTGUARD_MAIN
#define SMOKE_ALARM_PIN cfg_SMOKE_ALARM_PIN
#endif
extern const int cfg_BUTTON_ACTIVE_LEVEL;
#ifndef SMARTGUARD_MAIN
#define BUTTON_ACTIVE_LEVEL cfg_BUTTON_ACTIVE_LEVEL
#endif
extern const int cfg_SMOKE_ACTIVE_LEVEL;
#ifndef SMARTGUARD_MAIN
#define SMOKE_ACTIVE_LEVEL cfg_SMOKE_ACTIVE_LEVEL
#endif
extern const int cfg_EMERGENCY_POLL_MS;
#ifndef SMARTGUARD_MAIN
#define EMERGENCY_POLL_MS cfg_EMERGENCY_POLL_MS
#endif
extern const int cfg_EMERGENCY_CLEAR_MS;
#ifndef SMARTGUARD_MAIN
#define EMERGENCY_CLEAR_MS cfg_EMERGENCY_CLEAR_MS
#endif
extern const int cfg_EMERGENCY_FLASH_MS;
#ifndef SMARTGUARD_MAIN
#define EMERGENCY_FLASH_MS cfg_EMERGENCY_FLASH_MS
#endif
extern const int cfg_EMERGENCY_BUZZ_MS;
#ifndef SMARTGUARD_MAIN
#define EMERGENCY_BUZZ_MS cfg_EMERGENCY_BUZZ_MS
#endif
extern const int cfg_ENTRY_PASSAGE_PIN;
#ifndef SMARTGUARD_MAIN
#define ENTRY_PASSAGE_PIN cfg_ENTRY_PASSAGE_PIN
#endif
extern const int cfg_EXIT_PASSAGE_PIN;
#ifndef SMARTGUARD_MAIN
#define EXIT_PASSAGE_PIN cfg_EXIT_PASSAGE_PIN
#endif
extern const int cfg_PASSAGE_ACTIVE_LEVEL;
#ifndef SMARTGUARD_MAIN
#define PASSAGE_ACTIVE_LEVEL cfg_PASSAGE_ACTIVE_LEVEL
#endif
extern const int cfg_PASSAGE_DEBOUNCE_MS;
#ifndef SMARTGUARD_MAIN
#define PASSAGE_DEBOUNCE_MS cfg_PASSAGE_DEBOUNCE_MS
#endif
extern const int cfg_PASSAGE_TIMEOUT_MS;
#ifndef SMARTGUARD_MAIN
#define PASSAGE_TIMEOUT_MS cfg_PASSAGE_TIMEOUT_MS
#endif
extern const int cfg_CHECKLIST_SETTLE_MS;
#ifndef SMARTGUARD_MAIN
#define CHECKLIST_SETTLE_MS cfg_CHECKLIST_SETTLE_MS
#endif
extern SemaphoreHandle_t output_mutex;
extern SemaphoreHandle_t emergency_mutex;
extern SemaphoreHandle_t state_mutex;
extern SemaphoreHandle_t log_mutex;
extern _Atomic uint32_t log_failures;
extern SemaphoreHandle_t lcd_mutex;
extern SemaphoreHandle_t i2c_mutex;
extern SemaphoreHandle_t spi_mutex;
extern SemaphoreHandle_t clock_mutex;
extern SemaphoreHandle_t adc_mutex;
extern adc_oneshot_unit_handle_t adc1;
extern const char *file_paths[5];
extern sdmmc_card_t *sd_card;
extern const char *file_headers[3];
extern bool sd_ok;
extern bool rtc_present;
extern bool rtc_ok;
extern time_t clock_epoch;
extern time_t sw_base_epoch;
extern uint32_t sw_base_ms;
extern int sched_open_min;
extern int sched_close_min;
extern bool last_open;
extern user_t users[MAX_USERS];
extern int master_room[2];
extern bool room_lights[2];
extern uint32_t room_seen_ms[2];
extern int room_strikes[2];
extern bool room_offline[2];
extern bool admin_logged;
extern uint32_t admin_until_ms;
extern int pin_fails;
extern uint32_t lockout_until_ms;
extern char cli_buf[96];
extern int cli_len;
extern int menu_state;
extern bool menu_dirty;
extern int menu_cursor;
extern int list_cursor;
extern int menu_page;
extern int info_kind;
extern int menu_items[16];
extern int menu_item_n;
extern char menu_buf[8];
extern int menu_len;
extern char reg_id[8];
extern uint32_t menu_last_key_ms;
extern uint32_t menu_refresh_ms;
extern int who_list[MAX_USERS];
extern int who_n;
extern int del_list[MAX_USERS];
extern int del_n;
extern int del_target;
extern bool lcdlog_ready;
extern int lcdlog_n;
extern char lcdlog[8][2][17];
extern volatile int pat_count;
extern volatile uint32_t pat_on_ms;
extern volatile uint32_t pat_off_ms;
extern volatile uint32_t pat_start_ms;
extern bool buzzer_is_on;
extern uint32_t last_periodic_ms;
extern const char kp_map[4][4];
extern const int kp_rows[4];
extern const int kp_cols[4];


extern const int cfg_CLOSING_WARNING_MIN, cfg_CLOSING_BUZZ_MS, cfg_CLOSING_BEEP_MS;
#ifndef SMARTGUARD_MAIN
#define CLOSING_WARNING_MIN cfg_CLOSING_WARNING_MIN
#define CLOSING_BUZZ_MS cfg_CLOSING_BUZZ_MS
#define CLOSING_BEEP_MS cfg_CLOSING_BEEP_MS

#endif

#endif /* SMARTGUARD_SHARED_H */

/* ================= SECTION: main.c ================= */
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

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

/* ================= BUILD OPTIONS ================= */
#define SIM_NO_BCC          1      /* Wokwi cards carry no real checksum */
#define RELAY_ACTIVE_HIGH   1
#define ADMIN_PIN           "2026"
#define DEFAULT_OPEN_MIN    (5 * 60)     /* 05:00 */
#define DEFAULT_CLOSE_MIN   (21 * 60)    /* 21:00 */

/* ================= PINS (ESP32-S3) ================= */
#define ENTRY_CS    10
#define EXIT_CS     9
#define ENTRY_RST   7       /* both RFID readers share one reset pin */
#define EXIT_RST    7
#define SPI_MOSI    11
#define SPI_MISO    13
#define SPI_SCK     12
#define SD_CS_PIN   6

#define ENTRY_SERVO 4
#define EXIT_SERVO  5
#define LED_GREEN   16
#define LED_RED     17
#define BUZZER_PIN  18

#define R1_RELAY    14
#define R2_RELAY    38
#define R1A_PIR     1       /* room 1, outside the door */
#define R1B_PIR     15      /* room 1, inside the door  */
#define R2A_PIR     41      /* room 2, outside the door */
#define R2B_PIR     47      /* room 2, inside the door  */
#define R1_LDR_CH   ADC_CHANNEL_1   /* GPIO2: room 1 light sensor AO */
#define R2_LDR_CH   ADC_CHANNEL_2   /* GPIO3: room 2 light sensor AO */

#define LCD_SDA_PIN 39
#define LCD_SCL_PIN 40
#define I2C_HALF_US 3       /* bit-banged I2C, about 150 kHz */

#define SPI_HOST_ID   SPI2_HOST
#define SD_MOUNT      "/sdcard"
#define CLI_UART      UART_NUM_0

/* 4x4 keypad (rows are outputs, columns are inputs with pull-ups) */
#define KP_R1 8
#define KP_R2 35
#define KP_R3 36
#define KP_R4 37
#define KP_C1 45
#define KP_C2 46
#define KP_C3 48
#define KP_C4 0     /* the A B C D column (GPIO0 is the BOOT pin) */

/* Emergency inputs: button to GND; smoke DO HIGH means alarm in the simulator. */
#define EMERGENCY_BUTTON_PIN 21
#define SMOKE_ALARM_PIN      42
#define BUTTON_ACTIVE_LEVEL  0
#define SMOKE_ACTIVE_LEVEL   1
#define EMERGENCY_POLL_MS    10
#define EMERGENCY_CLEAR_MS   1000
#define EMERGENCY_FLASH_MS   250
#define EMERGENCY_BUZZ_MS    500

/* Passage sensors: LOW while a beam is broken, HIGH while clear.
   GPIO19/20 are used as GPIO here, not native USB; keep the UART0 console. */
#define ENTRY_PASSAGE_PIN     19
#define EXIT_PASSAGE_PIN      20
#define PASSAGE_ACTIVE_LEVEL  0
#define PASSAGE_DEBOUNCE_MS   50
#define PASSAGE_TIMEOUT_MS    8000
#define CHECKLIST_SETTLE_MS   500

/* ================= SETTINGS ================= */
#define RTC_ADDR          0x68
#define LCD_COUNT         4
#define LCD_ENTRY         0   /* 0x27 */
#define LCD_EXIT          1   /* 0x26 */
#define LCD_ROOM1         2   /* 0x25 */
#define LCD_ROOM2         3   /* 0x24 */
#define LCD_MSG_MS        3000
#define LCD_BL            0x08
#define LCD_EN            0x04

#define TERM_ENTRANCE     0
#define TERM_ROOM1        1
#define TERM_ROOM2        2
#define TERM_COUNT        3
#define BUS_QUEUE_LEN     16

#define DOOR_OPEN_ANGLE   90
#define DOOR_CLOSED_ANGLE 0
#define DOOR_OPEN_MS      3000

#define DENIED_MS         500       /* access denied */
#define CLOSING_WARNING_MIN 10
#define CLOSING_BUZZ_MS     10000   /* short nonblocking warning */
#define CLOSING_BEEP_MS     500

#define SEQ_TIMEOUT_MS    6000      /* second PIR must fire within this */
#define STUCK_MS          10000     /* a PIR that stays HIGH this long is reported */
#define LUX_ON            100       /* occupied room: lights on below this */
#define LUX_OFF           150       /* lights off above this (gap stops flicker) */
#define LIGHT_READ_MS     250
#define LDR_VCC           3.3f      /* sensor module powered from 3V3 */
#define LDR_RL10          50.0f     /* must match the part's rl10 attribute */
#define LDR_GAMMA         0.7f      /* must match the part's gamma attribute */
#define HB_MS             5000
#define ROOM_OFFLINE_MS   15000

#define MAX_USERS         16
#define ADMIN_SESSION_MS  60000
#define LOCKOUT_MS        30000
#define MAX_PIN_FAILS     3
#define OVERRIDE_MS       600000
#define MENU_TIMEOUT_MS   30000

/* MFRC522 registers */
#define COMMAND_REG      0x01
#define COM_IRQ_REG      0x04
#define ERROR_REG        0x06
#define FIFO_DATA_REG    0x09
#define FIFO_LEVEL_REG   0x0A
#define BIT_FRAMING_REG  0x0D
#define COLL_REG         0x0E
#define MODE_REG         0x11
#define TX_CONTROL_REG   0x14
#define TX_ASK_REG       0x15
#define T_MODE_REG       0x2A
#define T_PRESCALER_REG  0x2B
#define T_RELOAD_H_REG   0x2C
#define T_RELOAD_L_REG   0x2D
#define VERSION_REG      0x37

/* Files on the SD card */
#define F_ACCESS   0
#define F_ROOMS    1
#define F_SYSTEM   2
#define F_USERS    3
#define F_CONFIG   4

/* Storage request kinds */
#define SQ_APPEND  0
#define SQ_TAIL    1
#define SQ_EXPORT  2
#define SQ_FIND    3
#define SQ_WRITE   4
#define SQ_LOGLCD  5

/* Room sequence states */
#define RS_IDLE     0
#define RS_WAIT_B   1   /* outside sensor fired first: possible entry */
#define RS_WAIT_A   2   /* inside sensor fired first: possible exit   */
#define RS_CLEARING 3

/* Keypad menu states */
#define M_IDLE        0
#define M_MAIN        1
#define M_INFO        2
#define M_WHO         3
#define M_LOG_WAIT    4
#define M_LOG         5
#define M_PIN         6
#define M_REG_ID      7
#define M_REG_SCAN    8
#define M_DEL         9
#define M_DEL_CONF    10
#define M_TIME_OPEN   11
#define M_TIME_CLOSE  12

/* Keypad menu items */
#define IT_OCC     0
#define IT_ROOMS   1
#define IT_WHO     2
#define IT_LOG     3
#define IT_SYS     4
#define IT_LOGIN   5
#define IT_REG     6
#define IT_DEL     7
#define IT_OPEN    8
#define IT_CLOSE   9
#define IT_OVR     10
#define IT_LOGOUT  11


#define SMARTGUARD_MAIN
#include "shared.h"

/* Export configuration values without a separate config.h. */
const int cfg_DEFAULT_OPEN_MIN = DEFAULT_OPEN_MIN;
const int cfg_DEFAULT_CLOSE_MIN = DEFAULT_CLOSE_MIN;
const int cfg_EMERGENCY_BUTTON_PIN = EMERGENCY_BUTTON_PIN;
const int cfg_SMOKE_ALARM_PIN = SMOKE_ALARM_PIN;
const int cfg_BUTTON_ACTIVE_LEVEL = BUTTON_ACTIVE_LEVEL;
const int cfg_SMOKE_ACTIVE_LEVEL = SMOKE_ACTIVE_LEVEL;
const int cfg_EMERGENCY_POLL_MS = EMERGENCY_POLL_MS;
const int cfg_EMERGENCY_CLEAR_MS = EMERGENCY_CLEAR_MS;
const int cfg_EMERGENCY_FLASH_MS = EMERGENCY_FLASH_MS;
const int cfg_EMERGENCY_BUZZ_MS = EMERGENCY_BUZZ_MS;
const int cfg_ENTRY_PASSAGE_PIN = ENTRY_PASSAGE_PIN;
const int cfg_EXIT_PASSAGE_PIN = EXIT_PASSAGE_PIN;
const int cfg_PASSAGE_ACTIVE_LEVEL = PASSAGE_ACTIVE_LEVEL;
const int cfg_PASSAGE_DEBOUNCE_MS = PASSAGE_DEBOUNCE_MS;
const int cfg_PASSAGE_TIMEOUT_MS = PASSAGE_TIMEOUT_MS;
const int cfg_CHECKLIST_SETTLE_MS = CHECKLIST_SETTLE_MS;
const int cfg_SIM_NO_BCC = SIM_NO_BCC;
const int cfg_RELAY_ACTIVE_HIGH = RELAY_ACTIVE_HIGH;
const char cfg_ADMIN_PIN[] = ADMIN_PIN;
const int cfg_SD_CS_PIN = SD_CS_PIN;
const int cfg_LCD_SDA_PIN = LCD_SDA_PIN;
const int cfg_LCD_SCL_PIN = LCD_SCL_PIN;
const int cfg_I2C_HALF_US = I2C_HALF_US;
const int cfg_SPI_HOST_ID = SPI_HOST_ID;
const char cfg_SD_MOUNT[] = SD_MOUNT;
const int cfg_CLI_UART = CLI_UART;
const int cfg_RTC_ADDR = RTC_ADDR;
const int cfg_LCD_MSG_MS = LCD_MSG_MS;
const int cfg_LCD_BL = LCD_BL;
const int cfg_LCD_EN = LCD_EN;
const int cfg_DOOR_OPEN_ANGLE = DOOR_OPEN_ANGLE;
const int cfg_DOOR_CLOSED_ANGLE = DOOR_CLOSED_ANGLE;
const int cfg_DOOR_OPEN_MS = DOOR_OPEN_MS;
const int cfg_DENIED_MS = DENIED_MS;
const int cfg_CLOSING_WARNING_MIN = CLOSING_WARNING_MIN;
const int cfg_CLOSING_BUZZ_MS = CLOSING_BUZZ_MS;
const int cfg_CLOSING_BEEP_MS = CLOSING_BEEP_MS;
const int cfg_SEQ_TIMEOUT_MS = SEQ_TIMEOUT_MS;
const int cfg_STUCK_MS = STUCK_MS;
const int cfg_LUX_ON = LUX_ON;
const int cfg_LUX_OFF = LUX_OFF;
const int cfg_LIGHT_READ_MS = LIGHT_READ_MS;
const float cfg_LDR_VCC = LDR_VCC;
const float cfg_LDR_RL10 = LDR_RL10;
const float cfg_LDR_GAMMA = LDR_GAMMA;
const int cfg_HB_MS = HB_MS;
const int cfg_ROOM_OFFLINE_MS = ROOM_OFFLINE_MS;
const int cfg_ADMIN_SESSION_MS = ADMIN_SESSION_MS;
const int cfg_LOCKOUT_MS = LOCKOUT_MS;
const int cfg_MAX_PIN_FAILS = MAX_PIN_FAILS;
const int cfg_OVERRIDE_MS = OVERRIDE_MS;
const int cfg_MENU_TIMEOUT_MS = MENU_TIMEOUT_MS;
const int cfg_COMMAND_REG = COMMAND_REG;
const int cfg_COM_IRQ_REG = COM_IRQ_REG;
const int cfg_ERROR_REG = ERROR_REG;
const int cfg_FIFO_DATA_REG = FIFO_DATA_REG;
const int cfg_FIFO_LEVEL_REG = FIFO_LEVEL_REG;
const int cfg_BIT_FRAMING_REG = BIT_FRAMING_REG;
const int cfg_COLL_REG = COLL_REG;
const int cfg_MODE_REG = MODE_REG;
const int cfg_TX_CONTROL_REG = TX_CONTROL_REG;
const int cfg_TX_ASK_REG = TX_ASK_REG;
const int cfg_T_MODE_REG = T_MODE_REG;
const int cfg_T_PRESCALER_REG = T_PRESCALER_REG;
const int cfg_T_RELOAD_H_REG = T_RELOAD_H_REG;
const int cfg_T_RELOAD_L_REG = T_RELOAD_L_REG;
const int cfg_VERSION_REG = VERSION_REG;

/* ================= GLOBALS ================= */
const char sd_directory[] = SD_MOUNT "/cocis";

door_t entry_door = { "ENTRANCE", LEDC_CHANNEL_0, ENTRY_SERVO, LED_GREEN, false, 0 };
door_t exit_door  = { "EXIT",     LEDC_CHANNEL_1, EXIT_SERVO,  LED_RED,   false, 0 };

reader_t entry_reader = {0};
reader_t exit_reader  = {0};

room_t room1;
room_t room2;

lcd_t lcds[LCD_COUNT];
const char *lcd_names[LCD_COUNT]  = { "ENTRANCE", "EXIT", "ROOM 1", "ROOM 2" };
const uint8_t lcd_addrs[LCD_COUNT] = { 0x27, 0x26, 0x25, 0x24 };

QueueHandle_t inbox[TERM_COUNT];
const char *term_names[TERM_COUNT] = { "ENTRANCE", "ROOM 1", "ROOM 2" };
const char *msg_names[6] = {
    "ENTER_REQ", "EXIT_NOTICE", "STATE", "FAULT", "ENTER_GRANTED", "ENTER_DENIED"
};

QueueHandle_t storage_q;
QueueHandle_t key_q;
SemaphoreHandle_t output_mutex;
SemaphoreHandle_t emergency_mutex;
SemaphoreHandle_t state_mutex;
SemaphoreHandle_t log_mutex;
_Atomic uint32_t log_failures = 0;
SemaphoreHandle_t lcd_mutex;
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t spi_mutex;
SemaphoreHandle_t clock_mutex;
SemaphoreHandle_t adc_mutex;
adc_oneshot_unit_handle_t adc1 = NULL;

const char *file_paths[5] = {
    SD_MOUNT "/cocis/access.csv", SD_MOUNT "/cocis/rooms.csv", SD_MOUNT "/cocis/system.log",
    SD_MOUNT "/cocis/users.csv",  SD_MOUNT "/cocis/config.txt"
};
sdmmc_card_t *sd_card = NULL;
const char *file_headers[3] = {
    "time,point,uid,name,result,detail,inside",
    "time,room,event,count,lights",
    "time,level,message"
};
bool sd_ok = false;

/* clock */
bool rtc_present = false;
bool rtc_ok = false;
time_t clock_epoch = 0;
time_t sw_base_epoch = 0;
uint32_t sw_base_ms = 0;

/* schedule + users (owned by the entrance terminal) */
int sched_open_min = DEFAULT_OPEN_MIN;
int sched_close_min = DEFAULT_CLOSE_MIN;
bool last_open = true;
bool exit_override = false;
uint32_t override_until_ms = 0;
user_t users[MAX_USERS];
int master_room[2] = {0, 0};
bool room_lights[2] = {false, false};
uint32_t room_seen_ms[2] = {0, 0};
int room_strikes[2] = {0, 0};
bool room_offline[2] = {false, false};

/* admin session */
bool admin_logged = false;
uint32_t admin_until_ms = 0;
int pin_fails = 0;
uint32_t lockout_until_ms = 0;

/* serial CLI */
char cli_buf[96];
int cli_len = 0;

/* keypad menu (all of this is touched only by the entrance terminal task,
   except lcdlog which the storage task fills before setting lcdlog_ready) */
int menu_state = M_IDLE;
bool menu_dirty = false;
int menu_cursor = 0;
int list_cursor = 0;
int menu_page = 0;
int info_kind = 0;
int menu_items[16];
int menu_item_n = 0;
char menu_buf[8];
int menu_len = 0;
char reg_id[8] = "";
uint32_t menu_last_key_ms = 0;
uint32_t menu_refresh_ms = 0;
int who_list[MAX_USERS];
int who_n = 0;
int del_list[MAX_USERS];
int del_n = 0;
int del_target = -1;
bool lcdlog_ready = false;
int lcdlog_n = 0;
char lcdlog[8][2][17];

/* buzzer */
volatile int pat_count = 0;
volatile uint32_t pat_on_ms = 0;
volatile uint32_t pat_off_ms = 0;
volatile uint32_t pat_start_ms = 0;
bool buzzer_is_on = false;

uint32_t last_periodic_ms = 0;


const char kp_map[4][4] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '*', '0', '#', 'D' }
};
const int kp_rows[4] = { KP_R1, KP_R2, KP_R3, KP_R4 };
const int kp_cols[4] = { KP_C1, KP_C2, KP_C3, KP_C4 };


/* ================= STARTUP ================= */
static void create_shared_resources(void)
{
    output_mutex = xSemaphoreCreateMutex();
    emergency_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateRecursiveMutex();
    log_mutex = xSemaphoreCreateMutex();
    lcd_mutex = xSemaphoreCreateMutex();
    i2c_mutex = xSemaphoreCreateMutex();
    spi_mutex = xSemaphoreCreateMutex();
    clock_mutex = xSemaphoreCreateMutex();
    adc_mutex = xSemaphoreCreateMutex();
    for (int i = 0; i < TERM_COUNT; i++) inbox[i] = xQueueCreate(BUS_QUEUE_LEN, sizeof(bus_msg_t));
    storage_q = xQueueCreate(24, sizeof(store_item_t));
    key_q = xQueueCreate(8, sizeof(char));
    ESP_ERROR_CHECK(output_mutex && emergency_mutex && state_mutex && log_mutex && lcd_mutex && i2c_mutex && spi_mutex && clock_mutex && adc_mutex ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(storage_q && key_q ? ESP_OK : ESP_ERR_NO_MEM);
    for (int i = 0; i < TERM_COUNT; i++) ESP_ERROR_CHECK(inbox[i] ? ESP_OK : ESP_ERR_NO_MEM);
}

static void init_inputs_and_displays(void)
{
    /* chip selects idle high until the SPI driver takes them over */
    pin_output(ENTRY_CS, 1);
    pin_output(EXIT_CS, 1);
    pin_output(SD_CS_PIN, 1);
    pin_output(ENTRY_RST, 1);
    pin_output(LED_GREEN, 0);
    pin_output(LED_RED, 0);

    keypad_init();

    /* I2C bus: LCDs + RTC */
    i2c_bus_init();
    lcd_init_all();
    clock_init();
}

static void init_pwm_outputs(void)
{
    /* servo + buzzer PWM */
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_timer_config_t btimer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&btimer));

    ledc_channel_config_t bch = {
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bch));

    door_init(&entry_door);
    door_init(&exit_door);
}

static void init_card_readers(void)
{
    /* RFID readers and SD card share one SPI bus (SPI2) */
    spi_bus_config_t bus = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &bus, SPI_DMA_CH_AUTO));

    /* both readers share one reset line (GPIO7) */
    pin_set(ENTRY_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    pin_set(ENTRY_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    setup_reader(&entry_reader, "ENTRY", &entry_door, false, ENTRY_CS);
    setup_reader(&exit_reader, "EXIT", &exit_door, true, EXIT_CS);
}

static void load_saved_settings(void)
{
    /* users: built-in defaults, then whatever the SD card holds */
    users_defaults();
    sd_ok = sd_mount();
    if (sd_ok) {
        printf("SD card: OK (logs in %s/cocis)\n", SD_MOUNT);
        sd_load_users();
        sd_load_config();
    } else {
        printf("SD card: NOT AVAILABLE, logs will print to the serial monitor\n");
        lcd_msg(LCD_ENTRY, "ERROR", "SD card fail");
    }

    if (!state_init()) {
        printf("ERROR: persistent state unavailable; changes and door grants disabled\n");
        lcd_msg(LCD_ENTRY, "STATE ERROR", "Check NVS flash");
    }
    passage_init();
    last_open = schedule_open();
    entrance_idle_refresh();
}

static void init_rooms(void)
{
    /* light sensors: ADC1, oneshot reads shared by both room terminals */
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &adc1));
    adc_oneshot_chan_cfg_t ccfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, R1_LDR_CH, &ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, R2_LDR_CH, &ccfg));

    room_init(&room1, "ROOM 1", TERM_ROOM1, LCD_ROOM1, R1_RELAY, R1A_PIR, R1B_PIR, R1_LDR_CH);
    room_init(&room2, "ROOM 2", TERM_ROOM2, LCD_ROOM2, R2_RELAY, R2A_PIR, R2B_PIR, R2_LDR_CH);
}

static void start_tasks(void)
{
    ESP_ERROR_CHECK(xTaskCreate(lcd_task, "lcd", 4096, NULL, 1, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(clock_task, "clock", 3072, NULL, 1, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(storage_task, "storage", 8192, NULL, 1, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(keypad_task, "keypad", 3072, NULL, 2, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(entrance_terminal, "T_ENTRANCE", 12288, NULL, 2, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(room_terminal, "T_ROOM1", 5120, &room1, 2, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(room_terminal, "T_ROOM2", 5120, &room2, 2, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

static void system_start(void)
{
    /* RTC fields hold Kampala wall-clock time. Use one neutral encoding for
       mktime/gmtime so host/SDK timezone settings cannot shift the schedule. */
    setenv("TZ", "UTC0", 1);
    tzset();
    /* UART0 is the Wokwi Serial Monitor: install the driver so commands can be read */
    uart_driver_install(CLI_UART, 1024, 0, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    printf("\n=== CoCIS SMART SYSTEM: 3 VIRTUAL TERMINALS (ESP-IDF) ===\n");
    printf("Firmware: verified passage + emergency inputs | built %s %s\n", __DATE__, __TIME__);
    printf("Entry: scan card, then press/release GPIO19 button; emergency: GPIO21 to GND.\n");

    create_shared_resources();
    init_inputs_and_displays();
    init_pwm_outputs();
    emergency_init();
    ESP_ERROR_CHECK(xTaskCreate(emergency_task, "emergency", 3072, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    init_card_readers();
    load_saved_settings();
    init_rooms();

    room_seen_ms[0] = room_seen_ms[1] = now_ms();
    last_periodic_ms = now_ms();

    log_event(F_SYSTEM, "INFO,system started (schedule %02d:%02d-%02d:%02d)",
              sched_open_min / 60, sched_open_min % 60,
              sched_close_min / 60, sched_close_min % 60);

    start_tasks();

    printf("Terminals running: ENTRANCE (master), ROOM 1, ROOM 2.\n");
    printf("Keypad: press * for the menu on the entrance screen, # = Enter.\n");
    printf("Serial: type a command in the box under the Serial Monitor. Try: help\n\n");
}

/* ================= ENTRY POINT (ESP-IDF) ================= */
#ifdef __cplusplus
extern "C" void app_main(void);   /* keeps the C name if the file is built as C++ */
#endif
void app_main(void)
{
    system_start();
}

/* ================= SECTION: helpers.c ================= */

/* ================= BASIC HELPERS ================= */
uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

uint8_t bcd2dec(uint8_t v)
{ return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
uint8_t dec2bcd(int v)
{ return (uint8_t)(((v / 10) << 4) | (v % 10)); }

/* ================= GPIO HELPERS (driver/gpio.h) ================= */
void delay_us(uint32_t us)
{ esp_rom_delay_us(us); }

void pin_output(int pin, int level)
{
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)pin, (uint32_t)level);
}

void pin_input(int pin, bool pullup)
{
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)pin, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
}

void pin_set(int pin, int level)
{ gpio_set_level((gpio_num_t)pin, (uint32_t)level); }
int  pin_get(int pin)
{ return gpio_get_level((gpio_num_t)pin); }


/* ================= SECTION: clock.c ================= */

/* ================= CLOCK (RTC DS1307, software fallback) ================= */
time_t make_epoch(int y, int mo, int d, int h, int mi, int s)
{
    struct tm t;
    memset(&t, 0, sizeof(t));
    t.tm_year = y - 1900;
    t.tm_mon = mo - 1;
    t.tm_mday = d;
    t.tm_hour = h;
    t.tm_min = mi;
    t.tm_sec = s;
    t.tm_isdst = 0;
    return mktime(&t);
}

static time_t compile_epoch(void)
{
    char mon[4] = "Jan";
    int d = 1, y = 2026, h = 0, mi = 0, s = 0;
    sscanf(__DATE__, "%3s %d %d", mon, &d, &y);
    sscanf(__TIME__, "%d:%d:%d", &h, &mi, &s);
    const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *p = strstr(months, mon);
    int mo = p ? (int)((p - months) / 3) + 1 : 1;
    return make_epoch(y, mo, d, h, mi, s);
}

static bool rtc_read(time_t *out, bool *halted)
{
    const uint8_t start_reg = 0;
    if (!i2c_write(RTC_ADDR, &start_reg, 1)) return false;

    uint8_t reg[7];
    if (!i2c_read(RTC_ADDR, reg, 7)) return false;

    *halted = (reg[0] & 0x80) != 0;
    int sec  = bcd2dec(reg[0] & 0x7F);
    int mi   = bcd2dec(reg[1] & 0x7F);
    int hour;
    if (reg[2] & 0x40) {
        int h12 = bcd2dec(reg[2] & 0x1F);
        if (h12 < 1 || h12 > 12) return false;
        hour = h12 % 12 + ((reg[2] & 0x20) ? 12 : 0);
    } else hour = bcd2dec(reg[2] & 0x3F);
    int day  = bcd2dec(reg[4] & 0x3F);
    int mon  = bcd2dec(reg[5] & 0x1F);
    int year = 2000 + bcd2dec(reg[6]);

    if (mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hour > 23 || mi > 59 || sec > 59) return false;

    *out = make_epoch(year, mon, day, hour, mi, sec);
    struct tm check;
    gmtime_r(out, &check);
    return check.tm_year == year - 1900 && check.tm_mon == mon - 1 && check.tm_mday == day;
}

static bool rtc_write(time_t t)
{
    struct tm tmv;
    gmtime_r(&t, &tmv);
    uint8_t buf[8];
    buf[0] = 0;                                   /* start at register 0 */
    buf[1] = (uint8_t)(dec2bcd(tmv.tm_sec) & 0x7F); /* bit 7 = 0 keeps the clock running */
    buf[2] = dec2bcd(tmv.tm_min);
    buf[3] = dec2bcd(tmv.tm_hour);
    buf[4] = dec2bcd(tmv.tm_wday + 1);
    buf[5] = dec2bcd(tmv.tm_mday);
    buf[6] = dec2bcd(tmv.tm_mon + 1);
    buf[7] = dec2bcd(tmv.tm_year % 100);
    return i2c_write(RTC_ADDR, buf, 8);
}

void clock_init(void)
{
    sw_base_epoch = compile_epoch();
    sw_base_ms = now_ms();

    rtc_present = i2c_write(RTC_ADDR, NULL, 0);

    time_t t = sw_base_epoch;
    if (rtc_present) {
        time_t r = 0;
        bool halted = false;
        bool ok = rtc_read(&r, &halted);
        bool valid = false;
        if (ok && !halted) {
            struct tm tmv;
            gmtime_r(&r, &tmv);
            valid = (tmv.tm_year + 1900) >= 2024;
        }
        if (valid) {
            t = r;
            rtc_ok = true;
            printf("CLOCK: DS1307 found, running\n");
        } else {
            printf("CLOCK: RTC not set, loading build time into it\n");
            rtc_ok = rtc_write(sw_base_epoch);
        }
    } else {
        printf("CLOCK: RTC not found, using software clock\n");
    }
    clock_epoch = t;
    sw_base_epoch = t;
    sw_base_ms = now_ms();
}

bool clock_set(time_t value)
{
    /* Called from the entrance task under state_mutex. */
    if (rtc_present) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        bool ok = rtc_write(value);
        xSemaphoreGive(i2c_mutex);
        if (!ok) return false;
    }
    sw_base_epoch = value;
    sw_base_ms = now_ms();
    xSemaphoreTake(clock_mutex, portMAX_DELAY);
    clock_epoch = value;
    xSemaphoreGive(clock_mutex);
    rtc_ok = rtc_present;
    schedule_check();
    return true;
}

void clock_task(void *arg)
{
    (void)arg;
    for (;;) {
        state_lock();
        if (rtc_present) {
            time_t t = 0;
            bool halted = false;
            xSemaphoreTake(i2c_mutex, portMAX_DELAY);
            bool ok = rtc_read(&t, &halted);
            xSemaphoreGive(i2c_mutex);
            if (ok && !halted) {
                xSemaphoreTake(clock_mutex, portMAX_DELAY);
                clock_epoch = t;
                xSemaphoreGive(clock_mutex);
                sw_base_epoch = t;
                sw_base_ms = now_ms();
                rtc_ok = true;
            } else {
                xSemaphoreTake(clock_mutex, portMAX_DELAY);
                clock_epoch = sw_base_epoch + (time_t)((now_ms() - sw_base_ms) / 1000);
                xSemaphoreGive(clock_mutex);
                rtc_ok = false;
            }
        } else {
            xSemaphoreTake(clock_mutex, portMAX_DELAY);
            clock_epoch = sw_base_epoch + (time_t)((now_ms() - sw_base_ms) / 1000);
            xSemaphoreGive(clock_mutex);
        }
        state_unlock();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void clock_get(struct tm *out)
{
    xSemaphoreTake(clock_mutex, portMAX_DELAY);
    time_t t = clock_epoch;
    xSemaphoreGive(clock_mutex);
    gmtime_r(&t, out);
}

int minutes_now(void)
{
    struct tm t;
    clock_get(&t);
    return t.tm_hour * 60 + t.tm_min;
}

void fmt_time(char *buf, size_t n)
{
    struct tm t;
    clock_get(&t);
    snprintf(buf, n, "%04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
}


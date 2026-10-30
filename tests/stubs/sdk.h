#ifndef TEST_SDK_H
#define TEST_SDK_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_NVS_NOT_FOUND 0x1102
#define ESP_ERROR_CHECK(x) do { int err_ = (x); if (err_ != ESP_OK) abort(); } while (0)
const char *esp_err_to_name(esp_err_t);
typedef unsigned TickType_t;
typedef unsigned UBaseType_t;
typedef int BaseType_t;
typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY 0xffffffffU
#define pdMS_TO_TICKS(x) (x)
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
int xSemaphoreTake(SemaphoreHandle_t,unsigned);
int xSemaphoreGive(SemaphoreHandle_t);
int xSemaphoreTakeRecursive(SemaphoreHandle_t,unsigned);
int xSemaphoreGiveRecursive(SemaphoreHandle_t);
QueueHandle_t xQueueCreate(unsigned,unsigned);
int xQueueReceive(QueueHandle_t,void*,unsigned);
int xQueueSend(QueueHandle_t,const void*,unsigned);
int xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,void*);
void vTaskDelay(unsigned);
typedef int gpio_num_t;
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_INPUT 2
#define GPIO_MODE_INPUT_OUTPUT_OD 3
#define GPIO_PULLUP_ONLY 1
#define GPIO_PULLDOWN_ONLY 2
#define GPIO_FLOATING 0
int gpio_reset_pin(int);int gpio_set_direction(int,int);int gpio_set_level(int,uint32_t);int gpio_set_pull_mode(int,int);int gpio_get_level(int);
typedef int ledc_channel_t;
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2
#define LEDC_TIMER_0 0
#define LEDC_TIMER_1 1
#define LEDC_LOW_SPEED_MODE 0
#define LEDC_TIMER_14_BIT 14
#define LEDC_TIMER_10_BIT 10
#define LEDC_AUTO_CLK 0
typedef struct {int speed_mode,duty_resolution,timer_num,freq_hz,clk_cfg;} ledc_timer_config_t;
typedef struct {int gpio_num,speed_mode,channel,timer_sel;uint32_t duty;int hpoint;} ledc_channel_config_t;
int ledc_timer_config(const ledc_timer_config_t*);int ledc_channel_config(const ledc_channel_config_t*);int ledc_set_duty(int,int,uint32_t);int ledc_update_duty(int,int);
typedef void *spi_device_handle_t;
#define SPI2_HOST 2
#define SPI_DMA_CH_AUTO 0
#define SPI_TRANS_USE_TXDATA 1
#define SPI_TRANS_USE_RXDATA 2
typedef struct {int mosi_io_num,miso_io_num,sclk_io_num,quadwp_io_num,quadhd_io_num,max_transfer_sz;} spi_bus_config_t;
typedef struct {int mode,clock_speed_hz,spics_io_num,queue_size;} spi_device_interface_config_t;
typedef struct {int flags,length;uint8_t tx_data[4],rx_data[4];} spi_transaction_t;
int spi_bus_initialize(int,const spi_bus_config_t*,int);int spi_bus_add_device(int,const spi_device_interface_config_t*,spi_device_handle_t*);int spi_device_polling_transmit(spi_device_handle_t,spi_transaction_t*);
typedef struct {int gpio_cs,host_id;} sdspi_device_config_t;
typedef struct {int slot,max_freq_khz;} sdmmc_host_t;
typedef struct {int unused;} sdmmc_card_t;
typedef struct {bool format_if_mount_failed;int max_files,allocation_unit_size;} esp_vfs_fat_sdmmc_mount_config_t;
#define SDSPI_HOST_DEFAULT() ((sdmmc_host_t){0})
#define SDSPI_DEVICE_CONFIG_DEFAULT() ((sdspi_device_config_t){0})
int esp_vfs_fat_sdspi_mount(const char*,const sdmmc_host_t*,const sdspi_device_config_t*,const esp_vfs_fat_sdmmc_mount_config_t*,sdmmc_card_t**);
#define UART_NUM_0 0
int uart_driver_install(int,int,int,int,void*,int);int uart_read_bytes(int,void*,unsigned,unsigned);
typedef int adc_channel_t;
typedef void *adc_oneshot_unit_handle_t;
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_UNIT_1 1
#define ADC_ATTEN_DB_12 12
#define ADC_BITWIDTH_12 12
typedef struct {int unit_id;} adc_oneshot_unit_init_cfg_t;
typedef struct {int atten,bitwidth;} adc_oneshot_chan_cfg_t;
int adc_oneshot_read(adc_oneshot_unit_handle_t,int,int*);int adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t*,adc_oneshot_unit_handle_t*);int adc_oneshot_config_channel(adc_oneshot_unit_handle_t,int,const adc_oneshot_chan_cfg_t*);
void esp_rom_delay_us(uint32_t);int64_t esp_timer_get_time(void);void esp_restart(void);
typedef unsigned nvs_handle_t;
#define NVS_READWRITE 1
int nvs_flash_init(void);int nvs_open(const char*,int,nvs_handle_t*);int nvs_get_blob(nvs_handle_t,const char*,void*,size_t*);int nvs_set_blob(nvs_handle_t,const char*,const void*,size_t);int nvs_commit(nvs_handle_t);
#endif

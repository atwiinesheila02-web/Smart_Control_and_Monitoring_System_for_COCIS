#include "sdk.h"
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <stdatomic.h>

bool mock_queue_full;
int mock_adc_raw=4095;
int mock_set_error, mock_commit_error, mock_read_error, mock_init_error;
int mock_passage_clear_error;
uint8_t mock_durable[4096], mock_staged[4096];
uint8_t mock_alarm_durable[128], mock_alarm_staged[128];
uint8_t mock_passage_durable[128], mock_passage_staged[128];
size_t mock_passage_durable_size, mock_passage_staged_size;
size_t mock_alarm_durable_size, mock_alarm_staged_size;
_Atomic int mock_gpio_input[64];
_Atomic uint32_t mock_gpio_output[64];
_Atomic uint32_t mock_ledc_duty[3];
size_t mock_durable_size, mock_staged_size;
uint32_t mock_time_ms = 1000;
int mock_commits;
const char *esp_err_to_name(int err) { return err == ESP_OK ? "ESP_OK" : "TEST_ERROR"; }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) {
    pthread_mutex_t *m=malloc(sizeof(*m));pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(m,&attr);pthread_mutexattr_destroy(&attr);return m;
}
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return xSemaphoreCreateRecursiveMutex(); }
int xSemaphoreTake(SemaphoreHandle_t m,unsigned wait) { (void)wait;assert(m);return pthread_mutex_lock(m)==0; }
int xSemaphoreGive(SemaphoreHandle_t m) {assert(m);return pthread_mutex_unlock(m)==0;}
int xSemaphoreTakeRecursive(SemaphoreHandle_t m,unsigned wait) {return xSemaphoreTake(m,wait);}
int xSemaphoreGiveRecursive(SemaphoreHandle_t m) {return xSemaphoreGive(m);}
QueueHandle_t xQueueCreate(unsigned count,unsigned size) {(void)count;(void)size;return (void*)1;}
int xQueueReceive(QueueHandle_t q,void *item,unsigned wait) {(void)q;(void)item;(void)wait;return pdFALSE;}
int xQueueSend(QueueHandle_t q,const void *item,unsigned wait) {(void)q;(void)item;(void)wait;return mock_queue_full?pdFALSE:pdTRUE;}
int xTaskCreate(void(*f)(void*),const char*n,unsigned stack,void*arg,unsigned pri,void*out) {(void)f;(void)n;(void)stack;(void)arg;(void)pri;(void)out;return pdPASS;}
void vTaskDelay(unsigned ticks) {(void)ticks;}
int gpio_reset_pin(int p){(void)p;return 0;}
int gpio_set_direction(int p,int mode){(void)p;(void)mode;return 0;}
int gpio_set_level(int p,uint32_t value){atomic_store(&mock_gpio_output[p],value);return 0;}
int gpio_set_pull_mode(int p,int mode){(void)p;(void)mode;return 0;}
int gpio_get_level(int p){return atomic_load(&mock_gpio_input[p]);}
int ledc_timer_config(const ledc_timer_config_t*c){(void)c;return 0;}
int ledc_channel_config(const ledc_channel_config_t*c){(void)c;return 0;}
int ledc_set_duty(int a,int b,uint32_t c){(void)a;atomic_store(&mock_ledc_duty[b],c);return 0;}
int ledc_update_duty(int a,int b){(void)a;(void)b;return 0;}
int spi_bus_initialize(int a,const spi_bus_config_t*b,int c){(void)a;(void)b;(void)c;return 0;}
int spi_bus_add_device(int a,const spi_device_interface_config_t*b,spi_device_handle_t*c){(void)a;(void)b;*c=(void*)1;return 0;}
int spi_device_polling_transmit(spi_device_handle_t a,spi_transaction_t*b){(void)a;memset(b->rx_data,0,4);return 0;}
int esp_vfs_fat_sdspi_mount(const char*a,const sdmmc_host_t*b,const sdspi_device_config_t*c,const esp_vfs_fat_sdmmc_mount_config_t*d,sdmmc_card_t**e){(void)a;(void)b;(void)c;(void)d;(void)e;return ESP_FAIL;}
int uart_driver_install(int a,int b,int c,int d,void*e,int f){(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;return 0;}
int uart_read_bytes(int a,void*b,unsigned c,unsigned d){(void)a;(void)b;(void)c;(void)d;return 0;}
int adc_oneshot_read(adc_oneshot_unit_handle_t a,int b,int*c){(void)a;(void)b;*c=mock_adc_raw;return 0;}
int adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t*a,adc_oneshot_unit_handle_t*b){(void)a;*b=(void*)1;return 0;}
int adc_oneshot_config_channel(adc_oneshot_unit_handle_t a,int b,const adc_oneshot_chan_cfg_t*c){(void)a;(void)b;(void)c;return 0;}
void esp_rom_delay_us(uint32_t us){(void)us;}
int64_t esp_timer_get_time(void){return (int64_t)mock_time_ms*1000;}
void esp_restart(void) {abort();}
int nvs_flash_init(void) {return mock_init_error;}
int nvs_open(const char*n,int mode,nvs_handle_t*out){(void)mode;*out=!strcmp(n,"smartguard_em")?2:(!strcmp(n,"smartguard_ps")?3:1);return 0;}
int nvs_get_blob(nvs_handle_t h,const char*k,void*out,size_t*size){
    (void)k;if(mock_read_error)return mock_read_error;
    if(h==3) {
        if(!mock_passage_durable_size)return ESP_ERR_NVS_NOT_FOUND;
        if(*size<mock_passage_durable_size)return ESP_FAIL;
        memcpy(out,mock_passage_durable,mock_passage_durable_size);*size=mock_passage_durable_size;return 0;
    }
    if(h==2) {
        if(!mock_alarm_durable_size)return ESP_ERR_NVS_NOT_FOUND;
        if(*size<mock_alarm_durable_size)return ESP_FAIL;
        memcpy(out,mock_alarm_durable,mock_alarm_durable_size);*size=mock_alarm_durable_size;return 0;
    }
    if(!mock_durable_size)return ESP_ERR_NVS_NOT_FOUND;
    if(*size<mock_durable_size)return ESP_FAIL;
    memcpy(out,mock_durable,mock_durable_size);*size=mock_durable_size;return 0;
}
int nvs_set_blob(nvs_handle_t h,const char*k,const void*data,size_t size){
    (void)k;if(mock_set_error)return mock_set_error;
    if(h==3) {
        uint32_t mask;memcpy(&mask,(const uint8_t*)data+8,sizeof(mask));
        if(!mask && mock_passage_clear_error)return ESP_FAIL;
        assert(size<=sizeof(mock_passage_staged));memcpy(mock_passage_staged,data,size);mock_passage_staged_size=size;return 0;
    }
    if(h==2) {
        assert(size<=sizeof(mock_alarm_staged));memcpy(mock_alarm_staged,data,size);mock_alarm_staged_size=size;return 0;
    }
    assert(size<=sizeof(mock_staged));memcpy(mock_staged,data,size);mock_staged_size=size;return 0;
}
int nvs_commit(nvs_handle_t h){
    if(mock_commit_error)return mock_commit_error;
    if(h==3) {memcpy(mock_passage_durable,mock_passage_staged,mock_passage_staged_size);mock_passage_durable_size=mock_passage_staged_size;mock_commits++;return 0;}
    if(h==2) {memcpy(mock_alarm_durable,mock_alarm_staged,mock_alarm_staged_size);mock_alarm_durable_size=mock_alarm_staged_size;mock_commits++;return 0;}
    memcpy(mock_durable,mock_staged,mock_staged_size);mock_durable_size=mock_staged_size;mock_commits++;return 0;
}

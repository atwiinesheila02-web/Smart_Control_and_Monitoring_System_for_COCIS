#include "shared.h"
#include "../main/access.c"
#include "../main/safety.c"
#include "../main/rooms.c"
extern bool mock_reader_simulate;
extern int mock_reader_versions[64],mock_reader_attach_count[64];
extern bool mock_reader_present[64];
extern uint8_t mock_reader_uid[4];
extern _Atomic int mock_gpio_input[64];
extern uint32_t mock_time_ms;
static int checks;
#define CHECK(x) do {checks++;if(!(x)){fprintf(stderr,"FAIL RFID line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void cycle(int pin) {
    atomic_store(&mock_gpio_input[pin],PASSAGE_ACTIVE_LEVEL);passage_sample();
    mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();
    atomic_store(&mock_gpio_input[pin],!PASSAGE_ACTIVE_LEVEL);passage_sample();
    mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();passage_poll();
}
int main(void) {
    setenv("TZ","UTC0",1);tzset();
    output_mutex=xSemaphoreCreateMutex();emergency_mutex=xSemaphoreCreateMutex();state_mutex=xSemaphoreCreateRecursiveMutex();
    log_mutex=xSemaphoreCreateMutex();lcd_mutex=xSemaphoreCreateMutex();i2c_mutex=xSemaphoreCreateMutex();
    spi_mutex=xSemaphoreCreateMutex();clock_mutex=xSemaphoreCreateMutex();adc_mutex=xSemaphoreCreateMutex();
    storage_q=(void*)1;key_q=(void*)1;
    users_defaults();sched_open_min=300;sched_close_min=1260;
    clock_epoch=make_epoch(2026,10,31,12,0,0);CHECK(state_init());
    atomic_store(&mock_gpio_input[EMERGENCY_BUTTON_PIN],!BUTTON_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],!SMOKE_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[ENTRY_PASSAGE_PIN],!PASSAGE_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[EXIT_PASSAGE_PIN],!PASSAGE_ACTIVE_LEVEL);
    emergency_init();passage_init();
    mock_reader_simulate=true;mock_reader_versions[10]=0x92;mock_reader_versions[9]=0;
    setup_reader(&entry_reader,"ENTRY",&entry_door,false,10);
    setup_reader(&exit_reader,"EXIT",&exit_door,true,9);
    CHECK(entry_reader.available&&!exit_reader.available);
    CHECK(mock_reader_attach_count[9]==1);
    mock_time_ms+=4999;poll_reader(&exit_reader);CHECK(!exit_reader.available);
    mock_time_ms++;poll_reader(&exit_reader);CHECK(!exit_reader.available&&mock_reader_attach_count[9]==1);
    mock_reader_versions[9]=0x92;mock_time_ms+=5000;poll_reader(&exit_reader);
    CHECK(exit_reader.available&&entry_reader.available&&mock_reader_attach_count[9]==1);
    memcpy(mock_reader_uid,users[0].uid,4);mock_reader_present[10]=true;
    poll_reader(&entry_reader);CHECK(passage_waiting(false)&&entry_door.open&&inside_count()==0);
    cycle(ENTRY_PASSAGE_PIN);CHECK(inside_count()==1&&users[0].inside);
    mock_reader_present[10]=false;mock_reader_present[9]=true;
    poll_reader(&exit_reader);CHECK(passage_waiting(true)&&exit_door.open&&inside_count()==1);
    CHECK(strncmp(lcds[LCD_EXIT].msg[0],"ACCESS APPROVED",15)==0);
    poll_reader(&exit_reader);CHECK(passage_waiting(true)&&inside_count()==1);
    cycle(EXIT_PASSAGE_PIN);CHECK(!users[0].inside&&inside_count()==0&&!exit_door.open);
    CHECK(strncmp(lcds[LCD_EXIT].msg[0],"PASSAGE VERIFIED",15)==0);
    fprintf(stderr,"PASS: %d RFID integration assertions (offline exit retry, SPI handle reuse, entry scan, exit scan and verified exit)\n",checks);
    return 0;
}

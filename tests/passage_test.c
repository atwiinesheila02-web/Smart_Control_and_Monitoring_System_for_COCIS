#include "shared.h"
#include <pthread.h>
#include <unistd.h>
#include "../main/access.c"
#include "../main/safety.c"
#include "../main/rooms.c"
extern uint32_t mock_time_ms;
extern int mock_set_error,mock_commit_error,mock_passage_clear_error;
extern _Atomic int mock_gpio_input[64];
extern uint8_t mock_passage_durable[128];
extern size_t mock_passage_durable_size;
static int checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL passage line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void command(const char *text){char b[128];snprintf(b,sizeof(b),"%s",text);state_lock();cli_exec(b);state_unlock();}
static void raw(bool exit,bool blocked){atomic_store(&mock_gpio_input[exit?EXIT_PASSAGE_PIN:ENTRY_PASSAGE_PIN],blocked?PASSAGE_ACTIVE_LEVEL:!PASSAGE_ACTIVE_LEVEL);passage_sample();}
static void stable(bool exit,bool blocked){raw(exit,blocked);mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();}
static void cycle(bool exit){stable(exit,true);stable(exit,false);}
static void heartbeat(void){room_seen_ms[0]=room_seen_ms[1]=mock_time_ms;room_offline[0]=room_offline[1]=false;}
static void recover_empty(void){
    raw(false,false);raw(true,false);mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();passage_poll();
    command("login 2026");CHECK(emergency_reconcile_empty());room1.count=room2.count=0;room1.lights_on=room2.lights_on=false;room_lights[0]=room_lights[1]=false;heartbeat();
}
static void approve(bool exit,int ui){handle_scan(exit,users[ui].uid);}
static void *sample_cycle_thread(void*unused){(void)unused;cycle(false);return NULL;}
int main(void){
    setenv("TZ","UTC0",1);tzset();
    output_mutex=xSemaphoreCreateMutex();emergency_mutex=xSemaphoreCreateMutex();state_mutex=xSemaphoreCreateRecursiveMutex();
    log_mutex=xSemaphoreCreateMutex();lcd_mutex=xSemaphoreCreateMutex();i2c_mutex=xSemaphoreCreateMutex();spi_mutex=xSemaphoreCreateMutex();clock_mutex=xSemaphoreCreateMutex();adc_mutex=xSemaphoreCreateMutex();
    storage_q=(void*)1;key_q=(void*)1;users_defaults();clock_epoch=make_epoch(2026,10,29,12,0,0);
    CHECK(state_init());
    atomic_store(&mock_gpio_input[EMERGENCY_BUTTON_PIN],!BUTTON_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],!SMOKE_ACTIVE_LEVEL);
    emergency_init();raw(false,false);raw(true,false);passage_init();
    room_init(&room1,"ROOM 1",TERM_ROOM1,LCD_ROOM1,14,1,15,ADC_CHANNEL_1);room_init(&room2,"ROOM 2",TERM_ROOM2,LCD_ROOM2,38,41,47,ADC_CHANNEL_2);heartbeat();
    CHECK(passage_healthy());CHECK(inside_count()==0);
    // A scan authorizes, but does not count, a person.
    approve(false,0);CHECK(passage_waiting(false)&&!users[0].inside&&inside_count()==0&&entry_door.open);
    uint32_t journal[5]={0};CHECK(mock_passage_durable_size==sizeof(journal));memcpy(journal,mock_passage_durable,sizeof(journal));CHECK(journal[2]==1);
    approve(false,1);CHECK(passage_waiting(false)&&!users[1].inside); // no overwrite
    CHECK(passage_user_pending(0));command("login 2026");command("deluser 11334400");CHECK(users[0].used);
    // Beam bounce shorter than the debounce interval cannot verify passage.
    raw(false,true);mock_time_ms+=PASSAGE_DEBOUNCE_MS-1;raw(false,false);mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();passage_poll();
    CHECK(!users[0].inside&&passage_waiting(false));
    mock_time_ms+=PASSAGE_TIMEOUT_MS;passage_poll();door_update(&entry_door);CHECK(!passage_waiting(false)&&!users[0].inside&&!entry_door.open&&!emergency_counts_uncertain());
    // Full stable blocked/clear cycle confirms exactly one entry.
    approve(false,0);stable(false,true);passage_poll();CHECK(!users[0].inside&&passage_waiting(false));
    stable(false,false);passage_poll();CHECK(users[0].inside&&inside_count()==1&&!passage_waiting(false)&&!entry_door.open);
    passage_poll();CHECK(inside_count()==1); // no double count
    // Exit reservation prevents a room entry from consuming the same building person.
    approve(true,0);CHECK(passage_reserved_exits()==1&&users[0].inside);
    CHECK(!record_room_motion(&room1,true));
    CHECK(!passage_begin(false,0)); // inside card cannot authorize an entry directly
    cycle(true);passage_poll();CHECK(!users[0].inside&&inside_count()==0&&checklist_status()==CHECKLIST_WAITING);
    room1.lights_on=false;room2.lights_on=false;room_lights[0]=room_lights[1]=false;heartbeat();mock_time_ms+=CHECKLIST_SETTLE_MS-1;checklist_poll();CHECK(checklist_status()==CHECKLIST_WAITING);mock_time_ms++;checklist_poll();CHECK(checklist_status()==CHECKLIST_CLEAR);
    CHECK(strstr(lcds[LCD_EXIT].msg[0],"BUILDING CLEAR")!=NULL);
    // A lit room or stale/offline room must not pass the checklist.
    room1.lights_on=true;checklist_run();CHECK(checklist_status()==CHECKLIST_REQUIRED);CHECK(strstr(checklist_summary(),"Lights"));
    room1.lights_on=false;room_seen_ms[1]=mock_time_ms-(uint32_t)ROOM_OFFLINE_MS-1;checklist_run();CHECK(checklist_status()==CHECKLIST_REQUIRED);CHECK(strstr(checklist_summary(),"offline"));
    heartbeat();room1.count=1;checklist_run();CHECK(checklist_status()==CHECKLIST_REQUIRED);CHECK(strstr(checklist_summary(),"mismatch"));room1.count=0;
    checklist_run();CHECK(checklist_status()==CHECKLIST_CLEAR);
    // A new authorization invalidates a clear-building claim immediately.
    approve(false,0);CHECK(checklist_status()==CHECKLIST_IDLE);command("reboot");CHECK(passage_waiting(false));
    // Capture the beam cycle even while normal state/SPI locks are held.
    state_lock();xSemaphoreTake(spi_mutex,portMAX_DELAY);pthread_t t;CHECK(!pthread_create(&t,NULL,sample_cycle_thread,NULL));pthread_join(t,NULL);
    CHECK(!users[0].inside);xSemaphoreGive(spi_mutex);state_unlock();passage_poll();CHECK(users[0].inside);
    // Two complete cycles in one authorization are ambiguous, not two people counted.
    approve(false,1);cycle(false);cycle(false);passage_poll();CHECK(!users[1].inside&&users[0].inside&&emergency_counts_uncertain());
    CHECK(!passage_waiting(false));checklist_run();CHECK(checklist_status()==CHECKLIST_REQUIRED);recover_empty();
    // Passage without an authorization window flags uncertainty rather than guessing identity.
    cycle(true);passage_poll();CHECK(emergency_counts_uncertain()&&inside_count()==0);recover_empty();
    // A beam already blocked cannot be used to approve a card.
    stable(false,true);approve(false,0);CHECK(!passage_waiting(false)&&!users[0].inside);stable(false,false);passage_poll();CHECK(emergency_counts_uncertain());recover_empty();
    // Schedule changes cancel a no-passage window without changing counts.
    approve(false,0);CHECK(passage_waiting(false));sched_close_min=11*60;CHECK(config_save());CHECK(!passage_waiting(false)&&!users[0].inside&&!emergency_counts_uncertain());
    sched_close_min=21*60;CHECK(config_save());
    // Timeout while obstructed preserves the open door and marks unknown occupancy.
    approve(false,0);stable(false,true);mock_time_ms+=PASSAGE_TIMEOUT_MS;passage_poll();door_update(&entry_door);
    CHECK(!passage_waiting(false)&&entry_door.open&&emergency_counts_uncertain()&&!users[0].inside);
    stable(false,false);passage_poll();door_update(&entry_door);CHECK(!entry_door.open);recover_empty();
    // Reboot with a durable, unresolved intent requires reconciliation; no replay/counting.
    approve(false,0);CHECK(passage_waiting(false));CHECK(state_init());passage_init();CHECK(emergency_counts_uncertain()&&!passage_waiting(false)&&!users[0].inside);
    CHECK(!passage_begin(false,0));recover_empty();CHECK(passage_healthy());passage_init();CHECK(!emergency_counts_uncertain());
    // Completed physical passage with failed state persistence cannot silently count a user.
    approve(false,0);cycle(false);mock_set_error=ESP_FAIL;passage_poll();CHECK(!users[0].inside&&!state_healthy()&&emergency_counts_uncertain());
    mock_set_error=0;CHECK(state_init());recover_empty();
    // A committed count plus failed intent clearing is not replayed on restart.
    approve(false,0);cycle(false);mock_passage_clear_error=1;passage_poll();
    CHECK(users[0].inside&&inside_count()==1&&!passage_healthy()&&emergency_counts_uncertain());
    CHECK(state_init());passage_init();CHECK(users[0].inside&&inside_count()==1&&emergency_counts_uncertain());
    mock_passage_clear_error=0;recover_empty();CHECK(inside_count()==0&&passage_healthy());
    // Intent-write failure does not open a door or change occupancy.
    door_close(&entry_door);mock_set_error=ESP_FAIL;approve(false,0);CHECK(!entry_door.open&&!passage_waiting(false)&&!users[0].inside&&emergency_counts_uncertain());
    mock_set_error=0;recover_empty();
    // Passage deadlines/debounce still work across the 32-bit millisecond wrap.
    mock_time_ms=UINT32_MAX-75U;heartbeat();approve(false,0);CHECK(passage_waiting(false));cycle(false);passage_poll();CHECK(users[0].inside);
    approve(true,0);cycle(true);passage_poll();CHECK(!users[0].inside&&inside_count()==0);heartbeat();
    mock_time_ms+=CHECKLIST_SETTLE_MS;checklist_poll();CHECK(checklist_status()==CHECKLIST_CLEAR);
    // Closing warning fires once, stays nonblocking and ends after ten seconds.
    sched_open_min=8*60;sched_close_min=18*60;last_open=true;
    clock_epoch=make_epoch(2026,10,30,17,49,0);schedule_check();
    pat_count=0;buzzer_update();CHECK(!buzzer_is_on);
    clock_epoch=make_epoch(2026,10,30,17,50,0);schedule_check();buzzer_update();CHECK(buzzer_is_on);
    uint32_t warning_start=now_ms();
    mock_time_ms+=500;schedule_check();buzzer_update();CHECK(!buzzer_is_on);
    mock_time_ms+=500;buzzer_update();CHECK(buzzer_is_on);
    mock_time_ms=warning_start+10000;schedule_check();buzzer_update();CHECK(!buzzer_is_on);
    clock_epoch=make_epoch(2026,10,30,17,59,0);schedule_check();buzzer_update();CHECK(!buzzer_is_on);
    clock_epoch=make_epoch(2026,10,30,18,0,0);schedule_check();CHECK(!schedule_open());
    // Next day and an overnight closing each get their own warning.
    clock_epoch=make_epoch(2026,10,31,17,50,0);schedule_check();buzzer_update();CHECK(buzzer_is_on);
    mock_time_ms+=10000;buzzer_update();
    sched_open_min=20*60;sched_close_min=5;
    clock_epoch=make_epoch(2026,10,31,23,55,0);schedule_check();buzzer_update();CHECK(buzzer_is_on);
    mock_time_ms+=10000;clock_epoch=make_epoch(2026,11,1,0,0,0);schedule_check();buzzer_update();CHECK(!buzzer_is_on);
    sched_open_min=8*60;sched_close_min=18*60;clock_epoch=make_epoch(2026,11,1,12,0,0);schedule_check();
    // Emergency cancels a pending normal passage, and last-person checks cannot claim clear.
    approve(false,0);atomic_store(&mock_gpio_input[EMERGENCY_BUTTON_PIN],!BUTTON_ACTIVE_LEVEL);atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],!SMOKE_ACTIVE_LEVEL);emergency_init();
    atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],SMOKE_ACTIVE_LEVEL);emergency_poll();passage_poll();
    CHECK(emergency_active()&&exit_door.open&&!passage_waiting(false)&&!users[0].inside);
    checklist_run();CHECK(checklist_status()==CHECKLIST_REQUIRED);CHECK(strstr(checklist_summary(),"unverified"));
    fprintf(stderr,"PASS: %d verified-passage/checklist assertions (debounce, no-pass timeout, confirmation, reservation, faults, recovery and emergency priority)\n",checks);
    return 0;
}

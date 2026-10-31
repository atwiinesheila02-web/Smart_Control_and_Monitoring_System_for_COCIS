#include "shared.h"
#include <pthread.h>
#include <unistd.h>
#include "../main/access.c"
#include "../main/safety.c"
#include "../main/rooms.c"
extern int mock_set_error, mock_commit_error, mock_read_error, mock_init_error, mock_commits;
extern uint32_t mock_time_ms;
extern size_t mock_alarm_durable_size;
extern uint8_t mock_alarm_durable[128];
extern _Atomic int mock_gpio_input[64];
extern _Atomic uint32_t mock_gpio_output[64],mock_ledc_duty[3];
static int checks;
#define CHECK(x) do {checks++;if(!(x)){fprintf(stderr,"FAIL emergency line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void command(const char *text) {char b[128];snprintf(b,sizeof(b),"%s",text);state_lock();cli_exec(b);state_unlock();}
static void set_inputs(bool button,bool smoke) {
    atomic_store(&mock_gpio_input[EMERGENCY_BUTTON_PIN],button?BUTTON_ACTIVE_LEVEL:!BUTTON_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],smoke?SMOKE_ACTIVE_LEVEL:!SMOKE_ACTIVE_LEVEL);
}
static void clear_inputs(void) {set_inputs(false,false);emergency_poll();mock_time_ms+=EMERGENCY_CLEAR_MS;emergency_poll();}
static void *poll_once(void *unused) {(void)unused;emergency_poll();return NULL;}
static void reset_ok(void) {clear_inputs();CHECK(emergency_reset());CHECK(!emergency_active());CHECK(exit_door.open);}
int main(void) {
    setenv("TZ","UTC0",1);tzset();
    output_mutex=xSemaphoreCreateMutex();emergency_mutex=xSemaphoreCreateMutex();
    state_mutex=xSemaphoreCreateRecursiveMutex();log_mutex=xSemaphoreCreateMutex();
    lcd_mutex=xSemaphoreCreateMutex();i2c_mutex=xSemaphoreCreateMutex();spi_mutex=xSemaphoreCreateMutex();
    clock_mutex=xSemaphoreCreateMutex();adc_mutex=xSemaphoreCreateMutex();storage_q=(void*)1;key_q=(void*)1;
    users_defaults();users[0].inside=true;clock_epoch=make_epoch(2026,10,29,22,0,0);
    CHECK(state_init());CHECK(!schedule_open());
    room_init(&room1,"ROOM 1",TERM_ROOM1,LCD_ROOM1,14,1,15,ADC_CHANNEL_1);
    room_init(&room2,"ROOM 2",TERM_ROOM2,LCD_ROOM2,38,41,47,ADC_CHANNEL_2);
    set_inputs(false,false);emergency_init();emergency_service();
    CHECK(!emergency_active()&&!emergency_counts_uncertain()&&!exit_door.open);
    // Smoke alone, after hours, with normal state and SPI locks unavailable.
    mock_time_ms=1000;set_inputs(false,true);int before=mock_commits;
    state_lock();xSemaphoreTake(spi_mutex,portMAX_DELAY);
    pthread_t t;CHECK(!pthread_create(&t,NULL,poll_once,NULL));pthread_join(t,NULL);
    CHECK(emergency_active()&&exit_door.open&&!entry_door.open);
    CHECK(mock_commits==before); // hardware path did not access NVS
    xSemaphoreGive(spi_mutex);state_unlock();
    CHECK(emergency_counts_uncertain());CHECK(inside_count()==1&&users[0].inside);
    CHECK(atomic_load(&mock_ledc_duty[LEDC_CHANNEL_2])==512);
    CHECK(atomic_load(&mock_gpio_output[exit_door.led_pin])==1);
    mock_time_ms+=250;emergency_poll();CHECK(atomic_load(&mock_gpio_output[exit_door.led_pin])==0);
    mock_time_ms+=250;emergency_poll();CHECK(atomic_load(&mock_ledc_duty[LEDC_CHANNEL_2])==0);
    // Door timeout, schedule expiry, denial patterns and explicit close cannot cancel evacuation.
    exit_door.close_at_us=1;door_update(&exit_door);CHECK(exit_door.open);
    door_close(&exit_door);CHECK(exit_door.open);
    schedule_check();CHECK(exit_door.open);
    buzzer_pattern(1,1,0);mock_time_ms=2000;emergency_poll();CHECK(atomic_load(&mock_ledc_duty[LEDC_CHANNEL_2])==512);
    door_open(&entry_door);CHECK(!entry_door.open);
    int recorded=inside_count();handle_scan(true,users[0].uid);CHECK(inside_count()==recorded&&users[0].inside);
    CHECK(record_room_motion(&room1,true));CHECK(master_room[0]==1);
    CHECK(record_room_motion(&room1,false));CHECK(master_room[0]==0);
    // Exit/room displays always show evacuation; entrance retains usable admin menu.
    char line1[17],line2[17];CHECK(emergency_display(LCD_EXIT,line1,line2));CHECK(strstr(line2,"NO RFID")!=NULL);
    CHECK(emergency_display(LCD_ROOM1,line1,line2));CHECK(strstr(line1,"EVACUATE")!=NULL);
    emergency_set_menu_visible(true);CHECK(!emergency_display(LCD_ENTRY,line1,line2));CHECK(emergency_display(LCD_EXIT,line1,line2));
    emergency_set_menu_visible(false);CHECK(emergency_display(LCD_ENTRY,line1,line2));
    // Audit activation even if alarm persistence fails.
    char dir[]="/tmp/smartguard-emergency-XXXXXX";CHECK(mkdtemp(dir));char path[256];snprintf(path,sizeof(path),"%s/system.log",dir);
    file_paths[F_SYSTEM]=path;sd_ok=true;mock_set_error=ESP_FAIL;emergency_service();
    CHECK(exit_door.open&&emergency_active());FILE *f=fopen(path,"r");CHECK(f);char text[2048]={0};fread(text,1,sizeof(text)-1,f);fclose(f);
    CHECK(strstr(text,"EMERGENCY,ACTIVE")&&strstr(text,"smoke=1"));
    mock_set_error=0;mock_time_ms+=1000;emergency_service();CHECK(mock_alarm_durable_size>0);
    // Releasing smoke cannot clear the latch; no PIN is needed to trigger it.
    set_inputs(false,false);emergency_poll();CHECK(emergency_active()&&exit_door.open);
    admin_logged=false;CHECK(!emergency_reset());CHECK(exit_door.open);
    command("login 2026");CHECK(admin_active());
    command("reboot");CHECK(emergency_active()&&exit_door.open);
    set_inputs(true,false);emergency_poll();CHECK(!emergency_reset());CHECK(exit_door.open);
    set_inputs(false,false);emergency_poll();mock_time_ms+=EMERGENCY_CLEAR_MS-1;
    CHECK(!emergency_reset());CHECK(exit_door.open);mock_time_ms++;
    // Failed reset persistence leaves the physical exit open.
    mock_commit_error=ESP_FAIL;CHECK(!emergency_reset());CHECK(emergency_active()&&exit_door.open);mock_commit_error=0;
    CHECK(emergency_reset());CHECK(!emergency_active()&&exit_door.open);
    CHECK(emergency_counts_uncertain()&&inside_count()==1&&users[0].inside);
    CHECK(atomic_load(&mock_ledc_duty[LEDC_CHANNEL_2])==0);
    emergency_service();
    // Post-alarm counts are explicitly unverified and not automatically zeroed.
    handle_scan(false,users[1].uid);CHECK(!users[1].inside);
    CHECK(record_room_motion(&room1,true));
    CHECK(record_room_motion(&room1,false));
    // Alarm reset must not trap remaining occupants, including after closing.
    master_room[0]=room1.count=1;CHECK(state_commit());
    CHECK(record_room_motion(&room1,false));CHECK(room1.count==0&&inside_count()==1);
    CHECK(emergency_counts_uncertain());schedule_check();door_update(&exit_door);
    CHECK(exit_door.open&&!emergency_active());
    CHECK(emergency_display(LCD_EXIT,line1,line2)&&strstr(line1,"BUILDING CLOSED"));
    CHECK(strstr(line2,"EXIT OPEN"));
    for(int id=0;id<LCD_COUNT;id++) {
        lcd_render_lines(id,line1,line2);
        CHECK(strstr(line1,"BUILDING CLOSED"));
    }
    clock_epoch=make_epoch(2026,10,29,12,0,0);
    CHECK(emergency_display(LCD_EXIT,line1,line2)&&strstr(line1,"EXIT OPEN"));
    clock_epoch=make_epoch(2026,10,29,22,0,0);
    admin_logged=false;CHECK(!emergency_reconcile_empty());CHECK(inside_count()==1);
    command("login 2026");CHECK(emergency_reconcile_empty());CHECK(inside_count()==0&&!emergency_counts_uncertain());emergency_service();
    // Button alone is sufficient; both sources are recorded when present.
    set_inputs(true,false);emergency_poll();CHECK(emergency_active()&&exit_door.open);
    emergency_service();uint32_t saved_alarm[5];memcpy(saved_alarm,mock_alarm_durable,sizeof(saved_alarm));
    CHECK(saved_alarm[4]==EM_SOURCE_BUTTON);
    set_inputs(true,true);emergency_poll();emergency_service();
    CHECK(!emergency_reconcile_empty());CHECK(emergency_active());
    f=fopen(path,"r");CHECK(f);memset(text,0,sizeof(text));fread(text,1,sizeof(text)-1,f);fclose(f);
    CHECK(strstr(text,"button=1 smoke=1")!=NULL);
    // Restart cannot clear a saved alarm even with both inputs now clear.
    set_inputs(false,false);emergency_init();CHECK(emergency_active()&&exit_door.open&&emergency_counts_uncertain());
    reset_ok();CHECK(emergency_counts_uncertain());emergency_service();
    // Reset status and uncertainty survive reboot independently of occupancy checkpoints.
    emergency_init();CHECK(!emergency_active()&&emergency_counts_uncertain());
    CHECK(emergency_reconcile_empty());emergency_service();emergency_init();CHECK(!emergency_counts_uncertain());
    // A live raw input protects the exit before the polling latch runs.
    set_inputs(false,true);door_close(&exit_door);CHECK(exit_door.open);CHECK(emergency_exit_required());emergency_poll();emergency_service();
    // State-saving failures must never close an emergency exit.
    mock_set_error=ESP_FAIL;CHECK(!state_commit());CHECK(exit_door.open);mock_set_error=0;CHECK(state_init());
    reset_ok();emergency_service();
    // A corrupted/unreadable alarm record opens the exit conservatively; no erase.
    size_t size=mock_alarm_durable_size;uint8_t saved[128];memcpy(saved,mock_alarm_durable,size);
    mock_alarm_durable[0]^=0xff;emergency_init();CHECK(emergency_active()&&exit_door.open);CHECK(mock_alarm_durable_size==size);
    memcpy(mock_alarm_durable,saved,size);emergency_init();CHECK(!emergency_active());
    mock_read_error=ESP_FAIL;emergency_init();CHECK(emergency_active()&&exit_door.open);mock_read_error=0;
    reset_ok();emergency_service();
    // Keypad reset uses the same authenticated, clear-input path.
    set_inputs(true,false);emergency_poll();clear_inputs();menu_select(IT_EM_RESET);CHECK(!emergency_active()&&exit_door.open);

    // Main menu changes without closing/reopening, preserving selected item.
    menu_state=M_MAIN;menu_cursor=0;menu_build_items();
    bool found=false;for(int i=0;i<menu_item_n;i++) if(menu_items[i]==IT_EMPTY) found=true;
    CHECK(found);menu_select(IT_EMPTY);CHECK(menu_state==M_EMPTY_CONF);
    menu_key('*');CHECK(menu_state==M_MAIN&&emergency_counts_uncertain());
    admin_logged=false;menu_select(IT_EMPTY);CHECK(menu_state==M_MAIN);
    command("login 2026");menu_select(IT_EMPTY);menu_key('#');
    CHECK(!emergency_counts_uncertain()&&inside_count()==0&&!exit_door.open);
    set_inputs(true,false);emergency_poll();menu_refresh_items();
    found=false;for(int i=0;i<menu_item_n;i++) if(menu_items[i]==IT_EM_RESET) found=true;
    CHECK(found);clear_inputs();CHECK(emergency_reset());menu_refresh_items();
    found=false;for(int i=0;i<menu_item_n;i++) if(menu_items[i]==IT_EM_RESET) found=true;
    CHECK(!found);admin_logged=false;menu_refresh_items();
    CHECK(menu_items[menu_item_n-1]==IT_LOGIN);
    // Events queued before reconciliation cannot reintroduce old movement.
    command("login 2026");
    atomic_store(&mock_gpio_input[room1.a.pin],1);room_capture_sample(0,&room1);
    atomic_store(&mock_gpio_input[room1.b.pin],1);room_capture_sample(0,&room1);
    CHECK(emergency_reconcile_empty());room_capture_process(&room1);CHECK(room1.count==0);
    // Overflow is visible and invalidates occupancy instead of guessing order.
    for(int i=0;i<ROOM_EVENTS+4;i++) {
        atomic_store(&mock_gpio_input[room1.a.pin],i%2);room_capture_sample(0,&room1);
    }
    room_capture_process(&room1);CHECK(emergency_counts_uncertain());
    unlink(path);rmdir(dir);
    fprintf(stderr,"PASS: %d emergency assertions (independent smoke/button triggers, guarded outputs, latching, persistence, reset and reconciliation)\n",checks);
    return 0;
}

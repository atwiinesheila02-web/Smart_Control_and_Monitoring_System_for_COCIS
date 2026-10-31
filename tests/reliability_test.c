#include "shared.h"
#include <pthread.h>
#include <unistd.h>
/* Include the real modules to exercise private command and log-publication paths. */
#include "../main/access.c"
#include "../main/safety.c"
#include "../main/rooms.c"
extern bool mock_queue_full;
extern _Atomic int mock_gpio_input[64];
extern _Atomic uint32_t mock_ledc_duty[3];
extern int mock_adc_raw;
extern uint32_t mock_time_ms;
extern int mock_set_error, mock_commit_error, mock_read_error, mock_init_error, mock_commits;
extern uint8_t mock_durable[4096];
extern size_t mock_durable_size;
static const uint8_t card_a[4]={0x11,0x33,0x44,0};
static const uint8_t card_b[4]={1,3,4,0};
static int checks;
#define CHECK(x) do {checks++;if(!(x)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void command(const char *text) {char b[128];snprintf(b,sizeof(b),"%s",text);state_lock();cli_exec(b);state_unlock();}
static void scan(bool exit,const uint8_t *uid) {
    state_lock();handle_scan(exit,uid);
    if (passage_waiting(exit)) {
        int pin=exit?EXIT_PASSAGE_PIN:ENTRY_PASSAGE_PIN;
        atomic_store(&mock_gpio_input[pin],PASSAGE_ACTIVE_LEVEL);passage_sample();
        mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();
        atomic_store(&mock_gpio_input[pin],!PASSAGE_ACTIVE_LEVEL);passage_sample();
        mock_time_ms+=PASSAGE_DEBOUNCE_MS;passage_sample();passage_poll();
    }
    state_unlock();
}
static void *move_people(void *arg) {
    room_t *room=arg;
    for(int i=0;i<100;i++) {
        if(!record_room_motion(room,true))return (void*)1;
        if(!record_room_motion(room,false))return (void*)1;
    }
    return NULL;
}
int main(void) {
    setenv("TZ", "UTC0", 1); tzset();
    output_mutex=xSemaphoreCreateMutex();emergency_mutex=xSemaphoreCreateMutex();
    state_mutex=xSemaphoreCreateRecursiveMutex();log_mutex=xSemaphoreCreateMutex();
    lcd_mutex=xSemaphoreCreateMutex();i2c_mutex=xSemaphoreCreateMutex();spi_mutex=xSemaphoreCreateMutex();
    clock_mutex=xSemaphoreCreateMutex();adc_mutex=xSemaphoreCreateMutex();
    storage_q=(void*)1;key_q=(void*)1;
    users_defaults();sched_open_min=300;sched_close_min=1260;
    entry_reader.available=exit_reader.available=true;
    clock_epoch=make_epoch(2026,10,29,12,0,0);
    CHECK(state_init());CHECK(state_healthy());CHECK(mock_durable_size>0);
    atomic_store(&mock_gpio_input[ENTRY_PASSAGE_PIN],!PASSAGE_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[EXIT_PASSAGE_PIN],!PASSAGE_ACTIVE_LEVEL);
    passage_init();
    room_init(&room1,"ROOM 1",TERM_ROOM1,LCD_ROOM1,14,1,15,ADC_CHANNEL_1);
    room_init(&room2,"ROOM 2",TERM_ROOM2,LCD_ROOM2,38,41,47,ADC_CHANNEL_2);
    int a=find_user(card_a),b=find_user(card_b);CHECK(a>=0&&b>=0);
    // Both doors, duplicate scans, unknown cards and room/corridor consistency.
    scan(false,card_a);CHECK(users[a].inside&&inside_count()==1&&!passage_waiting(false));
    scan(false,card_a);CHECK(inside_count()==1);
    scan(true,card_b);CHECK(inside_count()==1);
    uint8_t unknown[4]={9,9,9,9};scan(false,unknown);CHECK(inside_count()==1);
    CHECK(record_room_motion(&room1,true));CHECK(room1.count==1&&master_room[0]==1&&corridor_count()==0);
    CHECK(!record_room_motion(&room2,true));CHECK(room2.count==0);
    scan(true,card_a);CHECK(users[a].inside&&inside_count()==1&&state_healthy());
    CHECK(master_room[0]==1&&room1.count==1&&!exit_door.open&&!passage_waiting(true));
    CHECK(strstr(lcds[LCD_EXIT].msg[1],"Leave room first")!=NULL);
    CHECK(!passage_begin(true,a)); // direct callers cannot bypass the room-exit rule
    CHECK(state_init());CHECK(users[a].inside&&master_room[0]==1);
    // Inner then outer means room exit; building occupancy remains one.
    room1.seq=RS_IDLE;
    room_update_sequence(&room1,false,true,mock_time_ms);
    CHECK(master_room[0]==1&&inside_count()==1);
    room_update_sequence(&room1,true,false,mock_time_ms+100);
    CHECK(master_room[0]==0&&room1.count==0&&inside_count()==1&&corridor_count()==1);
    scan(true,card_a);CHECK(!users[a].inside&&inside_count()==0&&master_room[0]==0);
    room_seen_ms[0]=room_seen_ms[1]=mock_time_ms;
    mock_time_ms+=CHECKLIST_SETTLE_MS;checklist_poll();CHECK(checklist_status()==CHECKLIST_CLEAR);
    CHECK(!record_room_motion(&room1,false));CHECK(room1.count==0);
    // Unauthorized setting edits do not change the schedule.
    command("schedule set 06:00 20:00");CHECK(sched_open_min==300&&sched_close_min==1260);
    command("login 2026");CHECK(admin_logged);
    command("schedule set 06:00 20:00");CHECK(sched_open_min==360&&sched_close_min==1200);
    scan(false,card_a);scan(false,card_b);
    CHECK(record_room_motion(&room1,true));CHECK(record_room_motion(&room2,true));
    // Serial deletion must not remove someone still inside.
    command("deluser 11334400");CHECK(users[a].used&&users[a].inside);
    // Simulate reboot RAM loss and restore, then run the actual room initializers.
    users_defaults();master_room[0]=master_room[1]=0;sched_open_min=300;sched_close_min=1260;
    CHECK(state_init());CHECK(inside_count()==2&&master_room[0]==1&&master_room[1]==1);
    CHECK(sched_open_min==360&&sched_close_min==1200);
    room_init(&room1,"ROOM 1",TERM_ROOM1,LCD_ROOM1,14,1,15,ADC_CHANNEL_1);
    room_init(&room2,"ROOM 2",TERM_ROOM2,LCD_ROOM2,38,41,47,ADC_CHANNEL_2);
    CHECK(room1.count==1&&room2.count==1&&room1.lights_on&&room2.lights_on);
    // Enforce both doors after hours even if an old override flag is set.
    CHECK(record_room_motion(&room1,false));CHECK(record_room_motion(&room2,false));
    clock_epoch=make_epoch(2026,10,29,22,0,0);exit_override=true;
    scan(true,card_a);CHECK(users[a].inside);
    uint8_t card_c[4]={2,2,2,2};CHECK(add_user(card_c,"Third")>=0);CHECK(users_save());
    scan(false,card_c);CHECK(inside_count()==2);
    command("unlock");CHECK(!schedule_open());
    entry_door.open=exit_door.open=true;schedule_check();CHECK(!entry_door.open&&!exit_door.open);
    // Overnight schedule boundaries.
    sched_open_min=1200;sched_close_min=360;CHECK(config_save());
    CHECK(schedule_open());clock_epoch=make_epoch(2026,10,30,5,59,0);CHECK(schedule_open());
    clock_epoch=make_epoch(2026,10,30,6,0,0);CHECK(!schedule_open());
    sched_open_min=360;sched_close_min=1200;CHECK(config_save());clock_epoch=make_epoch(2026,10,30,12,0,0);
    // Concurrency: two real host threads use production count and checkpoint code.
    pthread_t t1,t2;CHECK(!pthread_create(&t1,NULL,move_people,&room1));CHECK(!pthread_create(&t2,NULL,move_people,&room2));
    void *r1,*r2;pthread_join(t1,&r1);pthread_join(t2,&r2);CHECK(!r1&&!r2);
    CHECK(master_room[0]==0&&master_room[1]==0&&room1.count==0&&room2.count==0&&corridor_count()==2);
    // Queue saturation cannot lose counts or append logs: neither uses the query queue.
    mock_queue_full=true;CHECK(record_room_motion(&room1,true));CHECK(record_room_motion(&room1,false));
    char dir[]="/tmp/smartguard-test-XXXXXX";CHECK(mkdtemp(dir)!=NULL);
    char path[256];snprintf(path,sizeof(path),"%s/access.csv",dir);file_paths[F_ACCESS]=path;sd_ok=true;
    uint32_t failures=atomic_load(&log_failures);log_event(F_ACCESS,"TEST,queue-full-log");
    FILE *f=fopen(path,"r");CHECK(f!=NULL);char text[1024]={0};fread(text,1,sizeof(text)-1,f);fclose(f);
    CHECK(strstr(text,"queue-full-log")!=NULL&&atomic_load(&log_failures)==failures);
    CHECK(!store_request(SQ_TAIL,F_ACCESS,10,NULL));mock_queue_full=false;
    // A missing SD card exposes the problem and retains a serial backup.
    file_paths[F_ACCESS]="/not-a-real-directory/access.csv";log_event(F_ACCESS,"TEST,sd-failure");
    CHECK(atomic_load(&log_failures)==failures+1);sd_ok=false;
    // Log display publication is a complete snapshot under its own mutex.
    char rows[8][2][17]={0};strcpy(rows[0][0],"12:00 IN OK");strcpy(rows[0][1],"Card A");
    publish_lcd_log(1,rows);CHECK(lcdlog_ready&&lcdlog_n==1&&!strcmp(lcdlog[0][1],"Card A"));
    // Failure during save rolls back RAM, does not increment the room and disables grants.
    int prior=master_room[0];mock_set_error=ESP_FAIL;
    entry_door.open=exit_door.open=true;CHECK(!record_room_motion(&room1,true));
    CHECK(master_room[0]==prior&&room1.count==prior&&!state_healthy());
    CHECK(!entry_door.open&&!exit_door.open);scan(true,card_a);CHECK(users[a].inside);
    mock_set_error=0;CHECK(state_init());CHECK(state_healthy());
    mock_commit_error=ESP_FAIL;sched_open_min=400;CHECK(!config_save());CHECK(sched_open_min==360&&!state_healthy());
    mock_commit_error=0;CHECK(state_init());CHECK(sched_open_min==360);
    // Invalid snapshots and failed NVS initialization are never silently erased.
    size_t bytes=mock_durable_size;uint8_t backup[4096];memcpy(backup,mock_durable,bytes);
    mock_durable[0]^=0xff;CHECK(!state_init());CHECK(mock_durable_size==bytes);memcpy(mock_durable,backup,bytes);
    mock_read_error=ESP_FAIL;CHECK(!state_init());CHECK(!state_healthy());mock_read_error=0;
    mock_init_error=ESP_FAIL;CHECK(!state_init());CHECK(mock_durable_size==bytes);mock_init_error=0;CHECK(state_init());
    // Deletion persists and does not resurrect the deleted default tag after reboot.
    scan(true,card_b);CHECK(!users[b].inside);command("login 2026");command("deluser 01030400");CHECK(find_user(card_b)<0);
    users_defaults();CHECK(find_user(card_b)>=0);CHECK(state_init());CHECK(find_user(card_b)<0);
    // Production PIR ordering, light hysteresis, stuck fault and stale telemetry.
    room1.seq=RS_IDLE;room1.seq_since_ms=0;room1.a.active=room1.b.active=false;
    room_update_sequence(&room1,true,false,100);CHECK(room1.seq==RS_WAIT_B);
    room_update_sequence(&room1,false,true,200);CHECK(room1.count==1&&master_room[0]==1&&room1.seq==RS_CLEARING);
    mock_adc_raw=4095;room_read_light(&room1);room_apply(&room1);CHECK(room1.dark&&room1.lights_on);
    float resistance=LDR_RL10*1000.0f*powf(10.0f,LDR_GAMMA)/powf(125.0f,LDR_GAMMA);
    int mid=(int)(4095.0f*resistance/(resistance+10000.0f));
    mock_adc_raw=mid;room_read_light(&room1);room_apply(&room1);CHECK(room1.lux>100&&room1.lux<150&&room1.dark);
    mock_adc_raw=0;room_read_light(&room1);room_apply(&room1);CHECK(!room1.dark&&!room1.lights_on);
    mock_adc_raw=mid;room_read_light(&room1);CHECK(!room1.dark);
    room_update_sequence(&room1,false,false,250);CHECK(room1.seq==RS_IDLE);
    room_update_sequence(&room1,false,true,300);CHECK(room1.seq==RS_WAIT_A);
    room_update_sequence(&room1,true,false,400);CHECK(room1.count==0&&master_room[0]==0);
    room1.a.active=true;mock_queue_full=true;room_update_sequence(&room1,false,false,10500);CHECK(room1.fault_sent);
    mock_queue_full=false;room1.a.active=false;room_update_sequence(&room1,false,false,10600);CHECK(!room1.fault_sent);
    room1.lights_on=true;room_lights[0]=true;
    bus_msg_t stale={.src=TERM_ROOM1,.type=MSG_ROOM_STATE,.value=999,.aux=0};master_handle(&stale);
    CHECK(room_lights[0]&&room_strikes[0]==0);
    // Expired registration sessions and admin lockouts cannot mutate settings/users.
    admin_logged=false;menu_state=M_REG_SCAN;scan(false,unknown);CHECK(find_user(unknown)<0&&menu_state==M_MAIN);
    CHECK(admin_login_attempt("wrong","test")==0);CHECK(admin_login_attempt("wrong","test")==0);
    CHECK(admin_login_attempt("wrong","test")==-1);CHECK(admin_login_attempt("2026","test")==-1);
    mock_time_ms+=31000;CHECK(admin_login_attempt("2026","test")==1);
    mock_time_ms+=61000;CHECK(!admin_active());
    // Clock command rejects impossible dates and immediately applies valid ones.
    command("login 2026");rtc_present=false;time_t before=clock_epoch;
    command("time set 2026-02-31 10:00:00");CHECK(clock_epoch==before);
    command("time set 2026-10-30 13:00:00");struct tm current;clock_get(&current);CHECK(current.tm_hour==13&&current.tm_mday==30);
    // Closing warning and closing-time alarm both pulse, once per occurrence.
    sched_open_min=300;sched_close_min=1260;last_open=true;
    have_warned_closing=false;buzzer_pattern(0,0,0);
    clock_epoch=make_epoch(2026,10,30,20,49,0);schedule_check();
    mock_time_ms+=CLOSING_BUZZ_MS;buzzer_update();CHECK(mock_ledc_duty[2]==0);
    clock_epoch=make_epoch(2026,10,30,20,50,0);schedule_check();buzzer_update();
    CHECK(mock_ledc_duty[2]==512);
    mock_time_ms+=CLOSING_BEEP_MS;buzzer_update();CHECK(mock_ledc_duty[2]==0);
    mock_time_ms+=CLOSING_BUZZ_MS;schedule_check();buzzer_update();CHECK(mock_ledc_duty[2]==0);
    clock_epoch=make_epoch(2026,10,30,21,0,0);schedule_check();buzzer_update();
    CHECK(!last_open&&mock_ledc_duty[2]==512);
    mock_time_ms+=CLOSING_BUZZ_MS;schedule_check();buzzer_update();CHECK(mock_ledc_duty[2]==0);
    // Test the production renderer, including expiry and competing idle updates.
    char screen1[17], screen2[17];
    menu_state=M_IDLE;
    clock_epoch=make_epoch(2026,10,30,20,50,0);
    for(int i=0;i<LCD_COUNT;i++) {
        lcds[i].msg_active=false;
        lcd_idle(i,"BUILDING CLOSED","");
        lcd_render_lines(i,screen1,screen2);
        CHECK(strncmp(screen1,"BUILDING CLOSING",15)==0);
        CHECK(strncmp(screen2,"IN 10 MINUTES",13)==0);
    }
    clock_epoch=make_epoch(2026,10,30,20,51,0);
    lcd_render_lines(LCD_ROOM1,screen1,screen2);CHECK(strncmp(screen2,"IN 9 MINUTES",12)==0);
    lcd_msg(LCD_ENTRY,"ACCESS APPROVED","Pass through");
    lcd_render_lines(LCD_ENTRY,screen1,screen2);CHECK(strncmp(screen1,"ACCESS APPROVED",15)==0);
    mock_time_ms+=LCD_MSG_MS+1;
    lcd_render_lines(LCD_ENTRY,screen1,screen2);CHECK(strncmp(screen1,"BUILDING CLOSING",15)==0);
    clock_epoch=make_epoch(2026,10,30,20,59,0);
    lcd_render_lines(LCD_EXIT,screen1,screen2);CHECK(strncmp(screen2,"IN 1 MINUTE",11)==0);
    clock_epoch=make_epoch(2026,10,30,21,0,0);
    for(int i=0;i<LCD_COUNT;i++) {
        lcd_msg(i,"BUILDING CLOSING","IN 1 MINUTE");
        lcd_render_lines(i,screen1,screen2);
        CHECK(strncmp(screen1,"BUILDING CLOSED",15)==0);
    }
    menu_state=M_MAIN;lcd_idle(LCD_ENTRY,"ADMIN MENU","");lcds[LCD_ENTRY].msg_active=false;
    lcd_render_lines(LCD_ENTRY,screen1,screen2);CHECK(strncmp(screen1,"ADMIN MENU",10)==0);
    menu_state=M_IDLE;
    // Overnight closing follows the same open-to-closed transition.
    sched_open_min=22*60;sched_close_min=2*60;
    clock_epoch=make_epoch(2026,10,31,1,49,0);schedule_check();
    clock_epoch=make_epoch(2026,10,31,1,50,0);schedule_check();buzzer_update();CHECK(mock_ledc_duty[2]==512);
    mock_time_ms+=CLOSING_BUZZ_MS;buzzer_update();CHECK(mock_ledc_duty[2]==0);
    clock_epoch=make_epoch(2026,10,31,2,0,0);schedule_check();buzzer_update();CHECK(mock_ledc_duty[2]==512);
    mock_time_ms+=CLOSING_BUZZ_MS;buzzer_update();
    // Emergency output keeps priority when the schedule closes.
    atomic_store(&mock_gpio_input[EMERGENCY_BUTTON_PIN],!BUTTON_ACTIVE_LEVEL);
    atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],!SMOKE_ACTIVE_LEVEL);
    emergency_init();
    clock_epoch=make_epoch(2026,11,1,1,49,0);schedule_check();
    atomic_store(&mock_gpio_input[SMOKE_ALARM_PIN],SMOKE_ACTIVE_LEVEL);emergency_poll();
    clock_epoch=make_epoch(2026,11,1,2,0,0);schedule_check();buzzer_update();
    CHECK(emergency_active()&&!last_open);
    lcd_render_lines(LCD_EXIT,screen1,screen2);CHECK(strncmp(screen1,"EMERGENCY EXIT",14)==0);
    CHECK(mock_ledc_duty[2]==((mock_time_ms/EMERGENCY_BUZZ_MS)%2==0?512U:0U));
    unlink(path);rmdir(dir);
    fprintf(stderr,"PASS: %d production-code reliability assertions; %d checkpoint commits; 400 concurrent room transitions\n",checks,mock_commits);
    return 0;
}

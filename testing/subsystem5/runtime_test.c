#define APP_H
#define APP_S5 1
#ifndef S5_HARDWARE_CONFIRMED
#define S5_HARDWARE_CONFIRMED 1
#endif
#include "../integration/controller_test_kernel.h"
#define TMO_POL 0
ER tk_dly_tsk(unsigned ms);
#include "../../app_program/subsystem5/subsystem5.h"
#include "../../app_program/subsystem2/motion.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
static jmp_buf finish;
static uint32_t clock_us,trigger_us,last_trigger;
static unsigned angle=90,loops,locks,scenario,stage,scan_angles;
static bool hw_started,pinging,stationary_wheels=true,fail_echo;
static ER snapshot_lock_error;
static void (*owner)(INT,void *);
uint32_t fake_read(uint32_t a) { (void)a;return clock_us; }
void fake_write(uint32_t a,uint32_t v) { (void)a;(void)v; }
ID tk_cre_mtx(const T_CMTX *m) { assert(m->mtxatr==TA_INHERIT);return 1; }
ER tk_loc_mtx(ID id,TMO timeout) { assert(id==1&&!locks);if(timeout==TMO_POL && snapshot_lock_error) { return snapshot_lock_error; }locks++;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id==1&&locks);locks--;return E_OK; }
ID app_start_task(FP f,PRI priority,UW core,const char *name)
{ (void)name;assert(priority==20&&core==TP_PRC1);owner=(void (*)(INT,void *))f;return 2; }
bool motion_get_feedback(motion_feedback_t *m)
{ assert(!locks);*m=(motion_feedback_t){.valid=true,.braked=stationary_wheels,.timestamp_us=clock_us,.state=MOTION_IDLE};return true; }
bool ultrasonic_sensor_init(void) { assert(!locks);hw_started=true;return true; }
bool scan_servo_init(void) { assert(!locks&&hw_started);return true; }
bool scan_servo_position(unsigned value)
{ assert(!locks);if(value!=90){assert(stationary_wheels);scan_angles++;}angle=value;return true; }
void scan_servo_disable(void) { }
bool ultrasonic_sensor_trigger(uint32_t now)
{ assert(!locks&&!pinging && (uint32_t)(now-last_trigger)>=70000U);trigger_us=last_trigger=now;pinging=true;return true; }
bool ultrasonic_sensor_poll(uint32_t now,uint16_t *mm,bool *valid)
{
    assert(!locks);if(!pinging || now-trigger_us<10000) return false;
    pinging=false;*valid=!fail_echo;*mm=fail_echo?0:(angle==90?250:2000);return true;
}
void ultrasonic_sensor_cancel(void) { assert(!locks);pinging=false; }
#include "../../app_program/subsystem5/subsystem5.c"
ER tk_dly_tsk(unsigned ms)
{
    s5_observation_t o;
    assert(!locks);clock_us+=ms*1000;loops++;
    assert(subsystem5_snapshot(&o));
    if(stage==0 && o.range_valid) {
        if(scenario==2) { fail_echo=true;stage=1; }
        else { if(scenario==3) { stationary_wheels=false; } assert(subsystem5_request_scan(42));assert(!subsystem5_request_scan(43));stage=1; }
    }
    if(scenario==0 && stage==1 && o.scan_done) {
        assert(o.scan_id==42 && o.profile.valid && scan_angles>=6);stage=2;
    }
    if(scenario==0 && stage==2 && o.range_valid) { longjmp(finish,1); }
    if(scenario==1 && stage==1 && angle!=90) { subsystem5_cancel_scan();stage=2; }
    if(scenario==1 && stage==2 && angle==90 && o.range_valid) { assert(!o.scan_done);longjmp(finish,1); }
    if(scenario==3 && stage==1 && o.scan_done) { assert(!o.profile.valid && scan_angles==0);longjmp(finish,1); }
    if(scenario==2 && stage==1 && o.no_echo) { assert(!o.range_valid && o.range_mm==0);longjmp(finish,1); }
    assert(loops<1500);return E_OK;
}
int main(void)
{
#if !S5_HARDWARE_CONFIRMED
    assert(subsystem5_init()==E_PAR);
    assert(subsystem5_start()==E_OBJ);
    assert(!hw_started);
    puts("PASS: unconfirmed hardware cannot initialise or start Buddy 5");
    return 0;
#endif
    for(scenario=0;scenario<4;scenario++) {
        mutex_id=task_id=0;requested=0;shared.cancel=shared.busy=false;
        clock_us=loops=locks=stage=scan_angles=last_trigger=0;
        angle=90;hw_started=pinging=fail_echo=false;stationary_wheels=true;
        assert(subsystem5_init()==E_OK && subsystem5_start()==E_OK);
        {
            s5_observation_t copy={.range_mm=123};
            assert(subsystem5_read_snapshot(&copy)==S5_SNAPSHOT_OK);
            snapshot_lock_error=E_TMOUT;
            assert(subsystem5_read_snapshot(&copy)==S5_SNAPSHOT_BUSY && !locks);
            assert(!subsystem5_snapshot(&copy));
            snapshot_lock_error=E_OBJ;
            assert(subsystem5_read_snapshot(&copy)==S5_SNAPSHOT_UNAVAILABLE);
            snapshot_lock_error=0;
            published[S5_PACKET_BYTES-1]|=0x80U;
            assert(subsystem5_read_snapshot(&copy)==S5_SNAPSHOT_INVALID && !locks);
            published[S5_PACKET_BYTES-1]&=0x7fU;
            assert(subsystem5_read_snapshot(&copy)==S5_SNAPSHOT_OK);
        }
        if(!setjmp(finish)) { owner(0,0); }
    }
    puts("PASS: production sensing task with fake clock/kernel/sensor; stop-gated scan, bounded mailbox, packed snapshots, cancellation/recentering, confirmed no-echo");
}

#include "subsystem5.h"
#include "obstacle_scan.h"
#include "scan_servo.h"
#include "ultrasonic_sensor.h"
#if APP_S5
#include <sys/sysdef.h>
#include "../subsystem2/motion.h"
static ID mutex_id,task_id;
static uint8_t published[S5_PACKET_BYTES],controller_status;
static uint16_t requested;
static uint8_t requested_angle; /* 0 = full scan, else side peek angle */
static struct { unsigned cancel:1,busy:1; } shared;
static uint32_t now_us(void) { return in_w(TIMER_TIMERAWL); }
#ifndef S5_SCAN_LOG
#define S5_SCAN_LOG 1 /* TEMP (28 Oct): print every full-scan reading on USB */
#endif
static void scan_log(const s5_scan_t *s,const s5_profile_t *p)
{
#if S5_SCAN_LOG
    tm_printf((UB *)"[S5SCAN] n=%u",(unsigned)s->count);
    for(unsigned i=0;i<s->count;i++) {
        unsigned a,mm;bool v;
        if(s5_sample_unpack(&s->sample[i],&a,&mm,&v)) { tm_printf((UB *)" %u:%u",a,mm); }
    }
    tm_printf((UB *)" | valid=%u width=%u L=%d R=%d fwd=%u lclear=%u rclear=%u\n",(unsigned)p->valid,
        (unsigned)p->width_mm,(int)p->left_edge_mm,(int)p->right_edge_mm,(unsigned)p->forward_mm,
        (unsigned)p->left_clear_mm,(unsigned)p->right_clear_mm);
#else
    (void)s;(void)p;
#endif
}
static void publish(const s5_observation_t *o)
{
    uint8_t packet[S5_PACKET_BYTES];
    if(s5_observation_pack(o,packet) && tk_loc_mtx(mutex_id,TMO_FEVR)==E_OK) {
        for(unsigned i=0;i<S5_PACKET_BYTES;i++) { published[i]=packet[i]; }
        tk_unl_mtx(mutex_id);
    }
}
static bool stationary(uint32_t now)
{
    motion_feedback_t m={0};
    return motion_get_feedback(&m) && m.valid && m.braked && m.fault==MOTION_FAULT_NONE &&
        (uint32_t)(now-m.timestamp_us)<=30000U;
}
static void task(INT code,void *unused)
{
    s5_observation_t obs={0}; s5_scan_t scan={0}; const s5_config_t config=s5_defaults();
    uint32_t positioned=0,last_ping=0,ping_at=0; unsigned angle=90,failures=0;
    bool initialized=false,scanning=false,pinging=false;
    uint16_t peek_id=0; /* nonzero while a side peek is running */
    (void)code;(void)unused;
    for(;;) {
        uint32_t now=now_us(); uint16_t req=0,mm=0; unsigned req_angle=0;
        bool cancel=false,valid=false,result=false;
        if(tk_loc_mtx(mutex_id,TMO_FEVR)==E_OK) {
            cancel=shared.cancel;shared.cancel=false;req=requested;requested=0;
            req_angle=requested_angle;requested_angle=0;
            tk_unl_mtx(mutex_id);
        }
        /* Buddy 2 starts first and owns PIO0 instructions 0..23. Wait for its
         * actual publication, not task creation, before enabling SM2. */
        if(!initialized) {
            motion_feedback_t m={0};
            if(motion_get_feedback(&m) && m.valid) {
                initialized=ultrasonic_sensor_init() && scan_servo_init();
                if(initialized) { positioned=now;last_ping=now; }
            }
            publish(&obs);tk_dly_tsk(10);continue;
        }
        if(cancel || (scanning && !stationary(now))) {
            ultrasonic_sensor_cancel();pinging=false;scanning=false;
            (void)scan_servo_position(90);angle=90;positioned=now;
            obs.scan_done=false;obs.profile=(s5_profile_t){0};peek_id=0;
            if(tk_loc_mtx(mutex_id,TMO_FEVR)==E_OK) { shared.busy=false;tk_unl_mtx(mutex_id); }
        }
        if(req && req_angle && !cancel) {
            /* Side peek: the last profile stays published (telemetry) until the
             * reading completes under the new id; moving = failed peek. */
            if(!stationary(now) || !s5_scan_begin_at(&scan,now,req_angle)) {
                obs.scan_id=req;obs.scan_done=true;obs.scan_us=now;obs.profile=(s5_profile_t){0};
            } else {
                ultrasonic_sensor_cancel();pinging=false;scanning=true;peek_id=req;
                angle=scan.angle;(void)scan_servo_position(angle);positioned=now;
            }
        } else if(req && !cancel) {
            obs.scan_id=req;obs.scan_done=false;obs.profile=(s5_profile_t){0};
            if(!stationary(now)) { obs.scan_done=true;obs.scan_us=now; }
            else {
                ultrasonic_sensor_cancel();pinging=false;s5_scan_begin(&scan,now);scanning=true;
                angle=scan.angle;(void)scan_servo_position(angle);positioned=now;
            }
        }
        if(pinging) {
            result=ultrasonic_sensor_poll(now,&mm,&valid);
            if(result) { pinging=false; }
        } else if((uint32_t)(now-positioned)>=250000U &&
                  (uint32_t)(now-last_ping)>=S5_PING_PERIOD_US) {
            last_ping=ping_at=now;pinging=ultrasonic_sensor_trigger(now);
            if(!pinging) { result=true;valid=false; }
        }
        if(result) {
            if(valid) { failures=0;obs.no_echo=false; }
            else { if(failures<3U) { failures++; } obs.no_echo=failures>=3U; }
            if(scanning) {
                unsigned old=scan.angle;
                s5_scan_accept(&scan,now,valid,mm);
                if(scan.done) {
                    obs.scan_done=true;obs.scan_us=now;
                    if(peek_id) {
                        obs.scan_id=peek_id;peek_id=0;
                        if(scan.failed || !s5_peek_merge(&obs.profile,&scan.sample[0])) { obs.profile=(s5_profile_t){0}; }
                    } else {
                        if(!scan.failed) { (void)s5_profile_build(scan.sample,scan.count,&config,&obs.profile); }
                        scan_log(&scan,&obs.profile);
                    }
                    scanning=false;angle=90;(void)scan_servo_position(angle);positioned=now;
                } else if(scan.angle!=old) {
                    angle=scan.angle;(void)scan_servo_position(angle);positioned=now;
                }
            } else if(angle==90U) {
                /* A single bad ping keeps the last good reading (still timestamped
                 * at its own ping, so it ages out after S5_RANGE_FRESH_US). Three
                 * in a row latch no_echo and clear it: then the controller faults. */
                if(valid) { obs.range_us=ping_at;obs.range_mm=mm;obs.range_valid=true; }
                else if(obs.no_echo) { obs.range_mm=0U;obs.range_valid=false; }
            }
        }
        if(scanning && (uint32_t)(now-scan.started_us)>=S5_SCAN_TIMEOUT_US) {
            ultrasonic_sensor_cancel();pinging=false;scanning=false;
            if(peek_id) { obs.scan_id=peek_id;peek_id=0; }
            obs.scan_done=true;obs.scan_us=now;obs.profile=(s5_profile_t){0};
            angle=90;(void)scan_servo_position(angle);positioned=now;
        }
        if(!scanning && tk_loc_mtx(mutex_id,TMO_FEVR)==E_OK) { shared.busy=false;tk_unl_mtx(mutex_id); }
        /* Front ranging is unavailable while the servo faces elsewhere. */
        if(scanning || (uint32_t)(now-positioned)<250000U) { obs.range_valid=false;obs.range_mm=0; }
        publish(&obs);tk_dly_tsk(10);
    }
}
ER subsystem5_init(void)
{
    T_CMTX m={0};s5_observation_t empty={0};
    if(mutex_id>0) { return E_OBJ; }
    if(!S5_HARDWARE_CONFIRMED) { return E_PAR; }
    m.mtxatr=TA_INHERIT;mutex_id=tk_cre_mtx(&m);
    if(mutex_id<=0) { return (ER)mutex_id; }
    (void)s5_observation_pack(&empty,published);return E_OK;
}
ER subsystem5_start(void)
{
    if(mutex_id<=0 || task_id>0) { return E_OBJ; }
    task_id=app_start_task((FP)task,S5_PRI_MAIN,TP_PRC1,"obstacle-scan");
    return task_id<0?(ER)task_id:E_OK;
}
s5_snapshot_result_t subsystem5_read_snapshot(s5_observation_t *out)
{
    uint8_t p[S5_PACKET_BYTES];
    ER er;
    if(!out || mutex_id<=0) { return S5_SNAPSHOT_UNAVAILABLE; }
    er=tk_loc_mtx(mutex_id,TMO_POL);
    if(er!=E_OK) { return er==E_TMOUT?S5_SNAPSHOT_BUSY:S5_SNAPSHOT_UNAVAILABLE; }
    for(unsigned i=0;i<S5_PACKET_BYTES;i++) { p[i]=published[i]; }
    tk_unl_mtx(mutex_id);
    return s5_observation_unpack(p,out)?S5_SNAPSHOT_OK:S5_SNAPSHOT_INVALID;
}
bool subsystem5_snapshot(s5_observation_t *out)
{ return subsystem5_read_snapshot(out)==S5_SNAPSHOT_OK; }
bool subsystem5_request_scan(uint16_t id)
{
    bool ok=false;
    if(!id || mutex_id<=0 || tk_loc_mtx(mutex_id,TMO_POL)!=E_OK) { return false; }
    if(!requested && !shared.busy && !shared.cancel) { requested=id;shared.busy=true;ok=true; }
    tk_unl_mtx(mutex_id);return ok;
}
bool subsystem5_request_peek(uint16_t id,unsigned angle)
{
    bool ok=false;
    if(!id || !s5_peek_angle_valid(angle) || mutex_id<=0 || tk_loc_mtx(mutex_id,TMO_POL)!=E_OK) { return false; }
    if(!requested && !shared.busy && !shared.cancel) {
        requested=id;requested_angle=(uint8_t)angle;shared.busy=true;ok=true;
    }
    tk_unl_mtx(mutex_id);return ok;
}
void subsystem5_cancel_scan(void)
{
    if(mutex_id>0 && tk_loc_mtx(mutex_id,TMO_FEVR)==E_OK) {
        shared.cancel=true;requested=0;requested_angle=0;tk_unl_mtx(mutex_id);
    }
}
void subsystem5_set_controller_status(unsigned state,unsigned fault,unsigned route)
{
    if(mutex_id>0 && tk_loc_mtx(mutex_id,TMO_POL)==E_OK) {
        controller_status=(uint8_t)((state&7U)|((fault&7U)<<3)|((route&3U)<<6));tk_unl_mtx(mutex_id);
    }
}
uint8_t subsystem5_controller_status(void)
{
    uint8_t value=0;
    if(mutex_id>0 && tk_loc_mtx(mutex_id,TMO_POL)==E_OK) { value=controller_status;tk_unl_mtx(mutex_id); }
    return value;
}
#else
ER subsystem5_init(void) { return E_OK; }
ER subsystem5_start(void) { return E_OK; }
bool subsystem5_snapshot(s5_observation_t *out) { (void)out;return false; }
s5_snapshot_result_t subsystem5_read_snapshot(s5_observation_t *out)
{ (void)out;return S5_SNAPSHOT_UNAVAILABLE; }
bool subsystem5_request_scan(uint16_t id) { (void)id;return false; }
bool subsystem5_request_peek(uint16_t id,unsigned angle) { (void)id;(void)angle;return false; }
void subsystem5_cancel_scan(void) { }
void subsystem5_set_controller_status(unsigned state,unsigned fault,unsigned route) { (void)state;(void)fault;(void)route; }
uint8_t subsystem5_controller_status(void) { return 0; }
#endif

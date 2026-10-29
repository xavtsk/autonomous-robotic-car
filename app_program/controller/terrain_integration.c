#include "terrain_integration.h"
#if APP_S4
#include "../subsystem4/subsystem4.h"
#include "../subsystem1/subsystem1.h"

#define TERRAIN_MAX_AGE_US 150000U
#define TERRAIN_RESET_WAIT_US 500000U
#define TERRAIN_EVENT_BUDGET 8U
typedef struct {
    terrain_result_t sample;
    uint32_t now, reset_us;
    uint16_t run, expected_run, humps, sent_humps, sent_height, sent_run;
    unsigned have:1, waiting:1, latched:1, run_owned:1, fault_sent:1, hump_sent:1, hump_invalidated:1;
    int sent_fault;
} terrain_control_t;
static terrain_control_t control;

static bool healthy(uint32_t now)
{
    return control.have && (uint32_t)(now-control.sample.timestamp_us)<=TERRAIN_MAX_AGE_US &&
        (control.sample.flags&TERRAIN_CALIBRATED) &&
        !(control.sample.flags&TERRAIN_FAULT) && !control.sample.impact &&
        control.sample.state!=TERRAIN_IMPACT;
}
void terrain_control_init(void) { control=(terrain_control_t){0}; }
bool terrain_control_ready(uint32_t now)
{ return !control.latched && !control.waiting && healthy(now); }
bool terrain_control_reset_ready(uint32_t now)
{ return !control.waiting && healthy(now); }
void terrain_control_clear_fault(void) { control.latched=false; }
bool terrain_control_begin(uint32_t now)
{
    uint16_t token;
    if(!terrain_control_ready(now) || !subsystem4_begin_run(&token)) { return false; }
    control.expected_run=token; control.reset_us=now;
    control.waiting=true; control.have=false; control.run_owned=true;
    control.hump_sent=false;
    control.hump_invalidated=buddy1_invalidate(BUDDY1_TOPIC_HUMP)==E_OK;
    return true;
}
void terrain_control_poll(s3_drive_t *drive,uint32_t now)
{
    terrain_result_t sample,event;
    uint16_t run,humps;
    bool running=drive->state>=S3_DRIVE_FOLLOW && drive->state<=S3_DRIVE_REACQUIRE;
    bool event_fault=false,lost=false;
    control.now=now; drive->external_hold=false;
    if(!running) { control.waiting=false; }
    if(subsystem4_read(&sample,&run,&humps)) {
        if(running && control.run_owned && run!=control.expected_run) { event_fault=true; }
        else {
            control.sample=sample; control.run=run; control.humps=humps; control.have=true;
            if(control.waiting && run==control.expected_run) { control.waiting=false; }
        }
    }
    /* Bound work per control cycle. Completed hump count comes from the
     * coherent cumulative snapshot, not this potentially lossy event queue. */
    for(unsigned i=0;i<TERRAIN_EVENT_BUDGET && subsystem4_event(&event,&lost);++i) {
        if(lost || event.impact || event.state==TERRAIN_IMPACT || (event.flags&TERRAIN_FAULT)) {
            event_fault=true;
        }
    }
    if(!running) { return; }
    if(control.waiting && (uint32_t)(now-control.reset_us)<=TERRAIN_RESET_WAIT_US && !event_fault) {
        drive->external_hold=true; return;
    }
    if(event_fault || control.waiting || !healthy(now)) {
        control.latched=true; control.waiting=false;
        s3_drive_terrain_fault(drive);
    }
}
void terrain_control_report(void)
{
    bool bad=control.latched || (!control.waiting &&
        (!control.have || (uint32_t)(control.now-control.sample.timestamp_us)>TERRAIN_MAX_AGE_US ||
         (control.sample.flags&TERRAIN_FAULT) || control.sample.impact));
    if(!control.fault_sent || control.sent_fault!=(int)bad) {
        ER er=buddy1_update_faults(BUDDY1_FAULT_IMU,bad?BUDDY1_FAULT_IMU:0U);
        if(er==E_OK || er==E_TMOUT) { control.fault_sent=true; control.sent_fault=(int)bad; }
    }
    /* A previous valid run peak survives transient sensor faults. It is not
     * relabelled as a current instantaneous measurement. */
    if(control.have && control.run_owned && control.run==control.expected_run &&
       (control.sample.flags&TERRAIN_PEAK_VALID) &&
       control.sample.peak_m>=0 && control.sample.peak_m<=0.5f) {
        uint16_t height=(uint16_t)(control.sample.peak_m*1000.0f+0.5f);
        if(!control.hump_sent || control.sent_run!=control.run ||
           control.sent_humps!=control.humps || control.sent_height!=height) {
            ER er=buddy1_report_hump(control.humps,height);
            if(er==E_OK || er==E_TMOUT) {
                control.hump_sent=true; control.hump_invalidated=false; control.sent_humps=control.humps;
                control.sent_height=height; control.sent_run=control.run;
            }
        }
    } else {
        if (!control.hump_invalidated)
            control.hump_invalidated=buddy1_invalidate(BUDDY1_TOPIC_HUMP)==E_OK;
        control.hump_sent=false;
    }
    /* S4 exports compass-derived turn RATE, not absolute heading. Never feed
     * degrees/second into Buddy 1's heading-degrees packet. */
}
#endif

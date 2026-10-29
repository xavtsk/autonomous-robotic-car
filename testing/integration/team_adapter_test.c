#include "../../app_program/controller/vehicle_controller.h"
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../app_program/subsystem2/motion_pack.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
ID mtx_buddy1_state=1;
static motion_status_t status;
static s3_integration_status_t runtime;
static s3_control_kind_t queued;
static bool queue_ok=true, status_ok=true, frozen;
static unsigned mask=BUDDY1_FAULT_WIFI, updates, invalidations, reports, states, barcodes, motion_reads;
static ER mirror_result=E_OK;
static BUDDY1_ROBOT_STATE last_state;
static int left_report,right_report;
bool s3_integration_submit(const s3_control_request_t *r)
{queued=r->kind;return queue_ok;}
bool s3_integration_status(s3_integration_status_t *out)
{*out=runtime;return true;}
bool motion_get_status(motion_status_t *out)
{motion_reads++;*out=status;return status_ok;}
ER buddy1_update_faults(UB changed,UB active)
{
    assert(!(changed & BUDDY1_FAULT_WIFI));updates++;
    if(mirror_result==E_OK||mirror_result==E_TMOUT)mask=(mask&~changed)|active;
    return mirror_result;
}
ER buddy1_report_robot_state(BUDDY1_ROBOT_STATE state)
{states++;last_state=state;return E_OK;}
ER buddy1_report_barcode(UB letter)
{assert(letter=='B');barcodes++;return E_OK;}
ER buddy1_report_motion(H left,H right)
{reports++;left_report=left;right_report=right;return E_OK;}
static unsigned poses; static int pose_cm=-1, pose_deg=-1;
ER buddy1_report_pose(UH cm,H deg)
{assert(cm<BUDDY1_POSE_DISTANCE_MOD&&deg>=0&&deg<360);poses++;pose_cm=cm;pose_deg=deg;return E_OK;}
ER buddy1_invalidate(UB type)
{assert(type==BUDDY1_TOPIC_MOTION);invalidations++;return E_OK;}
ER buddy1_configuration_claim(BUDDY1_BARCODE_MAP *map,BOOL *newly)
{*map=(BUDDY1_BARCODE_MAP){'A','B','C','D'};if(newly)*newly=!frozen;frozen=true;return E_OK;}
static unsigned reopens;
ER buddy1_configuration_reopen(void)
{frozen=false;reopens++;return E_OK;}
ER buddy1_apply_barcode_map(const BUDDY1_BARCODE_MAP *map)
{
    if(frozen)return E_OBJ;
    return map->forward=='A'&&map->left=='B'&&map->right=='C'&&map->uturn=='D'?E_OK:E_PAR;
}
bool s3_navigation_set_mapping(s3_navigation_t *nav,const s3_nav_action_t map[26])
{memcpy(nav->config.map,map,sizeof(nav->config.map));return true;}
static void cycles(unsigned count)
{for(unsigned i=0;i<count;i++)vehicle_controller_observe(&runtime);}
int main(void)
{
    assert(vehicle_controller_init()==E_OK&&vehicle_controller_start()==E_OK);
    assert(vehicle_controller_begin()==E_OK&&queued==S3_CONTROL_START);
    assert(vehicle_controller_request_stop()==E_OK&&queued==S3_CONTROL_STOP);
    queue_ok=false;assert(vehicle_controller_begin()==E_OBJ);queue_ok=true;
    runtime.state=S3_DRIVE_FOLLOW;status.b_valid=true;
    status.speed_left_mps=0.12f;status.speed_right_mps=-0.12f;
    cycles(20);assert(reports==1&&left_report==120&&right_report==-120);
    assert(motion_reads==20); /* one read/cycle, shared by faults/pose/speed */
    assert(last_state==BUDDY1_STATE_LINE_FOLLOWING&&mask==BUDDY1_FAULT_WIFI);
    status.fault_bits=MOTION_FAULT_BIT(MOTION_FAULT_STALL)|MOTION_FAULT_BIT(MOTION_FAULT_ENCODER);
    runtime.navigation.fault_mask=2;cycles(1);
    assert(mask==(BUDDY1_FAULT_WIFI|BUDDY1_FAULT_STALL|BUDDY1_FAULT_ENCODER|BUDDY1_FAULT_IR));
    unsigned before=updates;cycles(1);assert(updates==before);
    status_ok=false;runtime.navigation.fault_mask=0;cycles(1);
    assert(mask==(BUDDY1_FAULT_WIFI|BUDDY1_FAULT_STALL|BUDDY1_FAULT_ENCODER));
    status_ok=true;status.fault_bits=0;cycles(1);assert(mask==BUDDY1_FAULT_WIFI);
    runtime.navigation.events=S3_EVENT_BARCODE;runtime.navigation.barcode.letter='B';
    runtime.navigation.action=S3_NAV_LEFT;cycles(1);assert(barcodes==1);
    runtime.navigation.events=0;runtime.state=S3_DRIVE_MANOEUVRE;cycles(1);
    assert(last_state==BUDDY1_STATE_TURNING_LEFT);
    runtime.state=S3_DRIVE_FAULT;cycles(1);assert(last_state==BUDDY1_STATE_ERROR);
    status.speed_left_mps=0.884f;before=invalidations;cycles(20);
    assert(invalidations>before&&reports==1); /* Never clip into a valid 510. */
    status.b_valid=false;before=invalidations;cycles(20);assert(invalidations>before);
    runtime.navigation.fault_mask=1;mirror_result=E_OBJ;before=updates;cycles(2);assert(updates==before+2);
    mirror_result=E_TMOUT;cycles(1);before=updates;cycles(1);assert(updates==before);
    s3_nav_action_t map[26]={S3_NAV_STRAIGHT,S3_NAV_LEFT,S3_NAV_RIGHT,S3_NAV_UTURN};
    assert(vehicle_controller_apply_mapping(map));map[4]=S3_NAV_LEFT;assert(!vehicle_controller_apply_mapping(map));map[4]=S3_NAV_NONE;
    s3_navigation_t nav={0};assert(vehicle_controller_prepare_start(&nav)&&frozen);
    assert(nav.config.map[0]==S3_NAV_STRAIGHT&&nav.config.map[1]==S3_NAV_LEFT);
    assert(!vehicle_controller_apply_mapping(map));
    vehicle_controller_cancel_start();assert(!frozen&&reopens==1);
    assert(vehicle_controller_apply_mapping(map));
    /* Accepted START: the freeze belongs to the run; a stray cancel is a no-op. */
    assert(vehicle_controller_prepare_start(&nav)&&frozen);
    vehicle_controller_commit_start();vehicle_controller_cancel_start();
    assert(frozen&&reopens==1);
    /* Later refused restart: map was already frozen, so it is NOT reopened. */
    assert(vehicle_controller_prepare_start(&nav)&&frozen);
    vehicle_controller_cancel_start();assert(frozen&&reopens==1);
    assert(!vehicle_controller_apply_mapping(map));
    assert(vehicle_controller_submit_navigation(&runtime.navigation)==E_NOSPT);
    /* Route pose: relative to START, 5 Hz while running, final one at the end. */
    runtime=(s3_integration_status_t){0};status=(motion_status_t){0};status.b_valid=true;
    assert(vehicle_controller_init()==E_OK);poses=0;
    status.distance_m=1.00f;status.heading_deg=350.0f;cycles(5);assert(poses==0); /* not running */
    status.b_valid=false;runtime.state=S3_DRIVE_FOLLOW;cycles(3);assert(poses==0);  /* no reference yet */
    status.b_valid=true;cycles(1);assert(poses==1&&pose_cm==0&&pose_deg==0);      /* START */
    status.distance_m=1.25f;status.heading_deg=10.0f;cycles(19);assert(poses==1);
    cycles(1);assert(poses==2&&pose_cm==25&&pose_deg==20);                        /* heading wraps */
    runtime.state=S3_DRIVE_MANOEUVRE;status.distance_m=0.90f;status.heading_deg=260.0f;cycles(20);
    assert(poses==3&&pose_cm==4096-10&&pose_deg==270);                            /* reverse past START */
    status.distance_m=42.004f;cycles(20);assert(poses==4&&pose_cm==(4100%4096));  /* 41 m: wraps */
    status.b_valid=false;cycles(40);assert(poses==4);                              /* unusable: skipped */
    status.b_valid=true;runtime.state=S3_DRIVE_HALTED;cycles(1);assert(poses==5);  /* final pose */
    cycles(40);assert(poses==5);                                                   /* stopped: silent */
    status.distance_m=5.0f;status.heading_deg=90.0f;runtime.state=S3_DRIVE_FOLLOW;cycles(1);
    assert(poses==6&&pose_cm==0&&pose_deg==0);                                    /* new run, new START */
    puts("PASS: one-owner facade, explicit queued start/stop, full fault-bit ownership, unknown-status preservation, map freeze, telemetry and out-of-range invalidation, route pose relative to START (wrap, reverse, final, new run)");
}

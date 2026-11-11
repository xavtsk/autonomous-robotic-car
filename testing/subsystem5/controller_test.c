/* Real Buddy 2 controller/PID/estimator + real Buddy 3 core/bridge.
 * Hardware edges and PWM are mocked; this cannot prove physical behaviour. */
#include "motion_controller.h"
#include "motion_estimation.h"
#include "motion_lease.h"
#include "motor_control.h"
#include "s3_drive.h"
#include "s5_api.h"
#if APP_S4
#include "terrain_integration.h"
#include "../../app_program/subsystem4/subsystem4.h"
#include "../../app_program/subsystem1/subsystem1.h"
#endif
#include <assert.h>
#include <stdio.h>
static uint32_t now, lease_stamp, epoch = 1U;
#if APP_S4
static uint16_t imu_run;
static bool imu_impact;
bool subsystem4_read(terrain_result_t *out,uint16_t *run,uint16_t *humps)
{
    *out=(terrain_result_t){.timestamp_us=now,.flags=TERRAIN_CALIBRATED|TERRAIN_TILT_VALID,
        .state=TERRAIN_MOVING}; *run=imu_run; *humps=0; return true;
}
bool subsystem4_begin_run(uint16_t *run) { *run=++imu_run; return true; }
bool subsystem4_event(terrain_result_t *out,bool *lost)
{
    if(!imu_impact) { return false; }
    *out=(terrain_result_t){.timestamp_us=now,.impact=true}; *lost=false; imu_impact=false; return true;
}
ER buddy1_update_faults(UB changed,UB active) { (void)changed;(void)active;return E_OK; }
ER buddy1_report_hump(UH count,UH height) { (void)count;(void)height;return E_OK; }
ER buddy1_report_terrain(const BUDDY1_TERRAIN *t) { (void)t;return E_OK; }
ER buddy1_invalidate(UB topic) { (void)topic;return E_OK; }
#endif

static bool lease, reject_velocity, reject_turn, reject_reset;
static unsigned int polls, turns, velocities, stops;
static uint32_t report_losses;
static manoeuvre_t last_turn;
static int32_t counts[2];
static float duty[2];
int32_t wheel_encoder_count(wheel_id_t w) { return counts[w]; }
uint32_t wheel_encoder_bits(wheel_id_t w)
{
    return (uint32_t)counts[w] & WHEEL_COUNT_MASK;
}
int32_t wheel_encoder_delta(uint32_t now_bits, uint32_t then_bits)
{
    uint32_t delta = (now_bits - then_bits) & WHEEL_COUNT_MASK;
    return (delta & (1UL << (WHEEL_COUNT_BITS - 1U)))
               ? (int32_t)delta - (int32_t)(WHEEL_COUNT_MASK + 1U)
               : (int32_t)delta;
}
uint32_t wheel_encoder_glitches(wheel_id_t w)
{
    (void)w;
    return 0U;
}
void motor_control_set(wheel_id_t w, float value) { duty[w] = value; }
void motor_control_brake_all(void)
{
    duty[0] = 0.0f;
    duty[1] = 0.0f;
}
static motion_result_t velocity(float l, float r)
{
    velocities++;
    return reject_velocity ? MOTION_ERR_BUSY
                           : motion_controller_request_velocity(l, r, now);
}
static motion_result_t manoeuvre(manoeuvre_t m)
{
    int32_t left = 1, right = 1;
    turns++;
    last_turn = m;
    if (reject_turn)
    {
        return MOTION_ERR_BUSY;
    }
    if (m == MANOEUVRE_LEFT || m == MANOEUVRE_U_TURN)
    {
        left = -1;
    }
    if (m == MANOEUVRE_RIGHT)
    {
        right = -1;
    }
    return motion_controller_request_move(m == MANOEUVRE_STRAIGHT ? MOTION_DRIVE
                                                                  : MOTION_TURN,
                                          left, right, 0.02f, 0.10f, now);
}
static void brake(void)
{
    stops++;
    motion_controller_stop();
}
static motion_report_t reports[MOTION_REPORT_QUEUE_LEN];
static unsigned int report_count;
static void post(const motion_report_t *report)
{
    assert(report_count < MOTION_REPORT_QUEUE_LEN);
    reports[report_count++] = *report;
}
static bool receive(motion_report_t *report)
{
    if (report_count == 0U)
    {
        return false;
    }
    *report = reports[0];
    for (unsigned int i = 1U; i < report_count; i++)
    {
        reports[i - 1U] = reports[i];
    }
    report_count--;
    if (report->outcome == MOTION_OUTCOME_DONE)
    {
        polls++;
        motion_controller_ack_complete(report->command_id);
    }
    return true;
}
static bool reset(void)
{
    return !reject_reset && motion_controller_reset();
}
static void heartbeat(bool enabled)
{
    lease = enabled;
    if (enabled)
    {
        lease_stamp = now;
    }
}
static s3_drive_t drive;
static s5_observation_t obstacle;
static bool scanning_requested, range_bad, recovery_tape=true;
static unsigned scan_calls, peek_calls, peek_beside; /* peeks that still see the obstacle */
static unsigned step_moves, early_clear_count;
static unsigned peek_angle;
static int turn_sign=1, route_heading;
static unsigned route_turns;
static s5_snapshot_result_t snapshot_result;
static uint32_t sample_age_us;
s5_snapshot_result_t subsystem5_read_snapshot(s5_observation_t *out)
{
    if(snapshot_result==S5_SNAPSHOT_OK) { *out=obstacle;out->range_us-=sample_age_us; }
    return snapshot_result;
}
bool subsystem5_snapshot(s5_observation_t *out)
{ return subsystem5_read_snapshot(out)==S5_SNAPSHOT_OK; }
bool subsystem5_request_scan(uint16_t id) {
    assert(motion_controller_is_stopped());scan_calls++;scanning_requested=true;
    obstacle.scan_id=id;return true;
}
bool subsystem5_request_peek(uint16_t id,unsigned angle) {
    assert(motion_controller_is_stopped());peek_calls++;peek_angle=angle;
    obstacle.scan_id=id;return true;
}
void subsystem5_cancel_scan(void) { scanning_requested=false;peek_angle=0; }
void subsystem5_set_controller_status(unsigned s,unsigned f,unsigned r) { (void)s;(void)f;(void)r; }
uint8_t subsystem5_controller_status(void) { return 0; }
motion_result_t motion_request_drive(float metres,float speed) {
    if(drive.obstacle.leg==3U) {
        assert(drive.obstacle.pass!=S5_PASS_APPROACH);
        if(drive.obstacle.pass==S5_PASS_STEP) { step_moves++; }
    }
    return motion_controller_request_move(MOTION_DRIVE,1,1,metres,speed,now);
}
motion_result_t motion_request_turn(float degrees,float speed) {
    /* Route turns are gentle; the bounded tape search uses 20/40 degrees. */
    assert(degrees>=-45.0f && degrees<=45.0f);
    if(drive.obstacle.leg<7U) { assert(degrees==45.0f || degrees==-45.0f);route_turns++; }
    route_heading+=(int)degrees;
    turn_sign=degrees>0?1:-1;
    float distance=(degrees>0?degrees:-degrees)*0.13f*3.14159265f/360.0f;
    return motion_controller_request_move(MOTION_TURN,-turn_sign,turn_sign,distance,speed,now);
}

static void tick(uint16_t barcode, uint16_t left, uint16_t right, bool valid)
{
    unsigned int i;
    motion_feedback_t feedback;
    s3_sensor_frame_t frame = {0};
    bool early_clear=false;
    now += 10000U;
    obstacle.range_us=now;obstacle.range_valid=!range_bad;
    if(range_bad) { obstacle.range_mm=0;obstacle.no_echo=true; }
    if(peek_angle) { /* one side reading: obstacle still beside (200 mm) or clear */
        unsigned side=peek_beside?200U:2000U; if(peek_beside) { peek_beside--; }
        early_clear=side==2000U &&
            (motion_estimation_robot_distance()-drive.obstacle.pass_start_m)*1000.0<drive.obstacle.plan.clear_after_mm;
        if(early_clear) { early_clear_count++; }
        obstacle.scan_done=true;obstacle.scan_us=now;
        if(peek_angle<90U) { obstacle.profile.right_clear_mm=(uint16_t)side; } else { obstacle.profile.left_clear_mm=(uint16_t)side; }
        peek_angle=0;
    }
    if(scanning_requested) {
        obstacle.scan_done=true;obstacle.scan_us=now;
        obstacle.profile=(s5_profile_t){.closest_mm=250,.width_mm=100,
            .left_clear_mm=2000,.right_clear_mm=2000,.left_edge_mm=50,.right_edge_mm=-50,
            .front_mm=230,.forward_mm=270,.valid=true};
        obstacle.range_mm=2000;scanning_requested=false;
    }
    motion_estimation_update(0.01f);
    if (motion_lease_expired(lease, now, lease_stamp))
    {
        motion_controller_navigation_timeout();
        lease = false;
    }
    motion_controller_step(0.01f, now);
    feedback = (motion_feedback_t){motion_estimation_robot_distance(),
                                   now,
                                   epoch,
                                   motion_controller_state(),
                                   motion_controller_fault(),
                                   true,
                                   motion_controller_is_stopped(),
                                   motion_controller_command_id(),
                                   report_losses};
    frame.timestamp_us = now;
    for (i = 0; i < 3U; i++)
    {
        frame.sensor[i] = (s3_raw_sample_t){now, 0U, 0, true, valid};
    }
    frame.sensor[0].raw = barcode;
    frame.sensor[1].raw = left;
    frame.sensor[2].raw = right;
#if APP_S4
    terrain_control_poll(&drive,now);
#endif
    assert(s3_drive_process(&drive, &frame, &feedback));
    /* A BUSY snapshot may defer consuming this observation by one tick; it
     * still must never authorize clearance or the inward turn. */
    if(early_clear) { assert(drive.obstacle.pass!=S5_PASS_EXTRA && drive.obstacle.clear_count==0U); }
}
static void hold(unsigned int ticks, uint16_t barcode, uint16_t left,
                 uint16_t right)
{
    unsigned int i;
    for (i = 0; i < ticks; i++)
    {
        tick(barcode, left, right, true);
    }
}
static void init(bool calibrated)
{
    s3_navigation_config_t cfg = s3_navigation_defaults();
    const s3_drive_port_t port = {
        velocity, manoeuvre, brake, receive, motion_controller_command_id,
        reset,    heartbeat};
    s3_calibration_collector_t collector;
    unsigned int i;
    now = 0U; snapshot_result=S5_SNAPSHOT_OK; sample_age_us=0U;
#if APP_S4
    imu_run=0; imu_impact=false; terrain_control_init();
#endif
    obstacle=(s5_observation_t){.range_mm=2000,.range_valid=true};
    scan_calls=peek_calls=peek_beside=peek_angle=0;scanning_requested=range_bad=false;recovery_tape=true;
    step_moves=early_clear_count=0;
    lease = false;
    epoch = 1U;
    reject_velocity = false;
    reject_turn = false;
    reject_reset = false;
    turns = polls = velocities = stops = 0U;route_turns=0;route_heading=0;
    counts[0] = counts[1] = 0;
    duty[0] = duty[1] = 0;
    motion_estimation_init(0.001f, 0.13f);
    report_count = 0U;
    report_losses = 0U;
    motion_controller_init(0.30f, 0.0f, 0.0f, post);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(s3_drive_init(&drive, &cfg, &port));
    /* Known simulation footprint and floor limits; no physical claim. */
    drive.obstacle.config.half_width_mm=60;
    drive.obstacle.config.front_axle_mm=80;drive.obstacle.config.rear_axle_mm=120;
    drive.obstacle.config.sonar_axle_mm=50;drive.obstacle.config.sensor_axle_mm=0;
    drive.obstacle.config.margin_mm=20;
    drive.obstacle.config.left_space_mm=drive.obstacle.config.right_space_mm=1500;
    if (calibrated)
    {
        s3_calibration_begin(&collector);
        for (i = 0; i < 10U; i++)
        {
            assert(s3_calibration_add(&collector, IR_SURFACE_WHITE, 150U));
            assert(s3_calibration_add(&collector, IR_SURFACE_BLACK, 280U));
        }
        for (i = 0; i < 3U; i++)
        {
            assert(s3_navigation_apply_calibration(
                &drive.navigation, (s3_sensor_id_t)i, &collector));
        }
    }
    hold(12U, 150U, 202U, 202U);
}
static void start(void)
{
    assert(s3_drive_start(&drive, now));
#if APP_S4
    assert(terrain_control_begin(now));
#endif
    hold(5U, 150U, 202U, 202U);
    assert(drive.state == S3_DRIVE_FOLLOW &&
           motion_controller_state() == MOTION_VELOCITY);
}

static void simulate(void)
{
    motion_state_t state=motion_controller_state();
    if(state==MOTION_DRIVE) { counts[0]++;counts[1]++; }
    if(state==MOTION_TURN) { counts[0]-=turn_sign;counts[1]+=turn_sign; }
    bool tape=false;
    if(drive.obstacle_active && drive.obstacle.leg==5 && drive.obstacle.issued) {
        double travelled=motion_estimation_robot_distance()-drive.obstacle.leg_start_m;
        double goal=drive.obstacle.return_mm/1000.0;
        tape=recovery_tape && travelled>goal-0.07 && travelled<goal+0.07;
    }
    tick(150,tape?280:202,tape?150:202,true);
}
static void snapshot_contention(void)
{
    s5_observation_t cached;
    init(true);start();
    uint32_t stamp=now;
    snapshot_result=S5_SNAPSHOT_BUSY;
    simulate();
    assert(drive.state==S3_DRIVE_FOLLOW && drive.obstacle_snapshot_us==stamp);
    assert(s5_observation_unpack(drive.obstacle_snapshot,&cached) && cached.range_us==stamp);
    snapshot_result=S5_SNAPSHOT_OK;simulate();assert(drive.state==S3_DRIVE_FOLLOW);
    /* A new valid START sample covers contention on the very first cycle. */
    init(true);assert(s3_drive_start(&drive,now));
#if APP_S4
    assert(terrain_control_begin(now));
#endif
    snapshot_result=S5_SNAPSHOT_BUSY;simulate();assert(drive.state==S3_DRIVE_FOLLOW);
    /* Reusing a cache must never extend the source reading's lifetime. */
    init(true);start();snapshot_result=S5_SNAPSHOT_BUSY;
    for(unsigned i=0;i<25;i++) { simulate();assert(drive.state==S3_DRIVE_FOLLOW); }
    simulate();assert(drive.state==S3_DRIVE_FAULT && drive.obstacle.fault==S5_BAD_RANGE);
    init(true);start();sample_age_us=240000U;simulate();
    snapshot_result=S5_SNAPSHOT_BUSY;simulate();assert(drive.state==S3_DRIVE_FOLLOW);
    simulate();assert(drive.state==S3_DRIVE_FAULT);
    /* Explicit negative observations and read/codec failures never use old good data. */
    init(true);start();snapshot_result=S5_SNAPSHOT_BUSY;simulate();
    snapshot_result=S5_SNAPSHOT_OK;range_bad=true;simulate();assert(drive.state==S3_DRIVE_FAULT);
    init(true);start();snapshot_result=S5_SNAPSHOT_INVALID;simulate();assert(drive.state==S3_DRIVE_FAULT);
    init(true);start();snapshot_result=S5_SNAPSHOT_UNAVAILABLE;simulate();assert(drive.state==S3_DRIVE_FAULT);
    init(true);start();drive.obstacle_snapshot_valid=false;snapshot_result=S5_SNAPSHOT_BUSY;
    simulate();assert(drive.state==S3_DRIVE_FAULT);
    /* Active avoidance gets the same grace, without dropping a DONE event. */
    init(true);start();obstacle.range_mm=250;simulate();
    for(unsigned i=0;i<500 && !(drive.obstacle.issued && drive.obstacle.leg==0);i++) { simulate(); }
    assert(drive.obstacle.issued);snapshot_result=S5_SNAPSHOT_BUSY;simulate();
    assert(drive.obstacle_active && drive.state==S3_DRIVE_MANOEUVRE);
    s3_drive_stop(&drive);assert(!drive.obstacle_active && !drive.obstacle_snapshot_valid && !lease);
    init(true);start();obstacle.range_mm=250;simulate();
    for(unsigned i=0;i<500 && !(drive.obstacle.issued && drive.obstacle.leg==0);i++) { simulate(); }
    snapshot_result=S5_SNAPSHOT_BUSY;
    for(unsigned i=0;i<30 && drive.state!=S3_DRIVE_FAULT;i++) { simulate(); }
    assert(drive.state==S3_DRIVE_FAULT && !drive.obstacle_active);
    puts("PASS: transient BUSY read preserves fresh data; stale, missing, invalid and unavailable data still fault; active avoidance and STOP covered");
}
int main(void)
{
    snapshot_contention();
    init(true);start();obstacle.range_mm=250;peek_beside=2;
    tick(150,202,202,true);
    assert(drive.obstacle_active && drive.obstacle.state==S5_WAIT_STOP && scan_calls==0);
    /* Actual controller applies STOP and produces its matching cancellation. */
    for(unsigned i=0;i<5000 && drive.state!=S3_DRIVE_FAULT;i++) {
        /* Exercise scan and all seven movement handoffs under intermittent contention. */
        snapshot_result=(i%11U==0U)?S5_SNAPSHOT_BUSY:S5_SNAPSHOT_OK;
        simulate();if(!drive.obstacle_active) { break; }
    }
    snapshot_result=S5_SNAPSHOT_OK;
    assert(drive.state==S3_DRIVE_FOLLOW && !drive.obstacle_active && scan_calls==1);
    assert(drive.obstacle.recovery.confirmed && drive.navigation.mode==S3_MODE_TRACK);
    /* Two occupied peeks and any clear readings before the front gate cause
     * steps. Only two later fresh clear readings can confirm the rear. */
    assert(early_clear_count>0 && step_moves>=3U && peek_calls==step_moves+2U);
    assert(drive.obstacle.searched_mm>=step_moves*S5_PEEK_STEP_MM &&
           drive.obstacle.searched_mm<=step_moves*(S5_PEEK_STEP_MM+1U));
    assert(drive.obstacle.searched_mm>=drive.obstacle.plan.clear_after_mm);
    /* Four turns + out + measured steps + clearance + back. No blind approach. */
    assert(drive.obstacle.plan.forward_mm==0U);
    assert(polls==7U+step_moves && turns==0 && route_turns==4 && route_heading==0);
    s3_drive_stop(&drive);assert(!drive.obstacle_active && !lease);
    init(true);start();range_bad=true;simulate();assert(drive.state==S3_DRIVE_FAULT && !lease);
    init(true);start();obstacle.range_mm=250;simulate();s3_drive_stop(&drive);simulate();
    assert(drive.state==S3_DRIVE_OFF && !drive.obstacle_active && scan_calls==0);
    init(true);start();obstacle.range_mm=250;simulate();report_losses++;simulate();assert(drive.state==S3_DRIVE_FAULT && drive.obstacle.state==S5_FAULT && drive.obstacle.fault==S5_INTERRUPTED);
    init(true);start();obstacle.range_mm=250;simulate();
    for(unsigned i=0;i<500 && !(drive.obstacle.issued && drive.obstacle.leg==0);i++) { simulate(); }
    assert(drive.obstacle.issued);range_bad=true;simulate();assert(drive.state==S3_DRIVE_FAULT);
#if APP_S4
    init(true);start();obstacle.range_mm=250;simulate();
    for(unsigned i=0;i<500 && !(drive.obstacle.issued && drive.obstacle.leg==0);i++) { simulate(); }
    assert(drive.obstacle_active && drive.obstacle.issued);
    imu_impact=true;simulate();
    assert(drive.state==S3_DRIVE_FAULT && drive.reason==S3_DRIVE_TERRAIN_FAULT);
    assert(!drive.obstacle_active && drive.obstacle.fault==S5_INTERRUPTED && !lease);
    simulate();assert(duty[0]==0.0f && duty[1]==0.0f);
    for(unsigned i=0;i<100 && !motion_controller_is_stopped();i++) { simulate(); }
    assert(motion_controller_is_stopped());
    puts("PASS: IMU impact cancels an active S5 bypass through the sole controller and brakes Buddy 2");
#endif
    puts("PASS: real motion controller/PID + S5 + S3; brake acknowledgement, one report owner, bypass with side-peek depth search, real tape gate, STOP, stale range and report loss");
}

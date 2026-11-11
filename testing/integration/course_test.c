/* Unit checks for the course profile: the "tape between the sensors" line
 * detector mode and the s3_drive course options. Behaviour on a simulated
 * track (curves, crossings, turns) is covered by course_sim.c. */
#include "course_profile.h"
#include <assert.h>
#include <stdio.h>

static s3_line_observation_t step(s3_line_detector_t *d, uint32_t *t,
                                  uint16_t left, uint16_t right)
{
    *t += 10000U;
    return s3_line_update(d, *t, true, left, right);
}
static motion_result_t fake_drive(float m, float v) { (void)m; (void)v; return MOTION_OK; }
#if S3_STRAIGHT_TEST
static float requested_left,requested_right;
static unsigned requested_stops;
#endif
/* Referenced by the course profile when S3_SENSOR_TO_AXLE_MM > 0. */
motion_result_t motion_request_drive(float m, float v) { (void)m; (void)v; return MOTION_OK; }
static motion_result_t fake_velocity(float l, float r) {
#if S3_STRAIGHT_TEST
    requested_left=l;requested_right=r;
#else
    (void)l;(void)r;
#endif
    return MOTION_OK;
}
static motion_result_t fake_manoeuvre(manoeuvre_t m) { (void)m; return MOTION_OK; }
static void fake_stop(void) {
#if S3_STRAIGHT_TEST
    requested_left=requested_right=0;requested_stops++;
#endif
}
static bool fake_report(motion_report_t *r) { (void)r; return false; }
static uint8_t fake_id(void) { return 1U; }
static bool fake_reset(void) { return true; }
static void fake_heartbeat(bool on) { (void)on; }

#if S3_STRAIGHT_TEST
/* Exercise production navigation/drive with valid synthetic feedback, not a
 * physics simulation. Extreme IR contrast must not request differential speed. */
static s3_drive_t straight_drive;
static uint32_t straight_now;
static void straight_tick(uint16_t barcode,uint16_t left,uint16_t right,bool valid)
{
    s3_sensor_frame_t frame={0};motion_feedback_t feedback={0};
    straight_now+=10000U;frame.timestamp_us=straight_now;
    frame.sensor[S3_SENSOR_BARCODE]=(s3_raw_sample_t){straight_now,barcode,0,true,valid};
    frame.sensor[S3_SENSOR_LEFT]=(s3_raw_sample_t){straight_now,left,0,true,valid};
    frame.sensor[S3_SENSOR_RIGHT]=(s3_raw_sample_t){straight_now,right,0,true,valid};
    feedback.timestamp_us=straight_now;feedback.epoch=1;feedback.valid=true;
    feedback.braked=true;feedback.state=MOTION_IDLE;feedback.command_id=1;
    assert(s3_drive_process(&straight_drive,&frame,&feedback));
}
static void straight_hold(unsigned n,uint16_t barcode,uint16_t left,uint16_t right)
{ for(unsigned i=0;i<n;i++) { straight_tick(barcode,left,right,true); } }
static void straight_start(bool map_a)
{
    s3_navigation_config_t cfg=s3_navigation_defaults();
    s3_calibration_collector_t cal;
    const s3_drive_port_t port={fake_velocity,fake_manoeuvre,fake_stop,fake_report,
                                fake_id,fake_reset,fake_heartbeat};
    const s3_drive_options_t options=s3_course_options();
    s3_course_navigation(&cfg);cfg.acquisition.enabled_mask=S3_ALL_SENSORS;
    cfg.map[0]=map_a?S3_NAV_LEFT:S3_NAV_NONE;
    straight_now=0;requested_left=requested_right=0;requested_stops=0;
    assert(s3_drive_init(&straight_drive,&cfg,&port));
    assert(s3_drive_set_options(&straight_drive,&options));
    s3_calibration_begin(&cal);
    for(unsigned i=0;i<10;i++) {
        assert(s3_calibration_add(&cal,IR_SURFACE_WHITE,150));
        assert(s3_calibration_add(&cal,IR_SURFACE_BLACK,280));
    }
    for(unsigned i=0;i<S3_SENSOR_COUNT;i++) {
        assert(s3_navigation_apply_calibration(&straight_drive.navigation,(s3_sensor_id_t)i,&cal));
    }
    straight_hold(12,150,150,150);
    assert(s3_drive_start(&straight_drive,straight_now));
}
static void straight_scan_a(void)
{
    const unsigned symbols[3]={0x094U,0x109U,0x094U};
    straight_hold(40,150,150,150);
    for(unsigned c=0;c<3;c++) {
        for(unsigned i=0;i<9;i++) {
            unsigned ticks=(symbols[c]&(1U<<(8U-i)))?12:4;
            straight_hold(ticks,i%2?150:280,150,150);
        }
        if(c<2) { straight_hold(4,150,150,150); }
    }
    straight_hold(45,150,150,150);
}
static void test_straight_requests(void)
{
    const uint16_t readings[][2]={{150,150},{280,150},{150,280},{280,280},{150,150}};
    straight_start(false);
    for(unsigned c=0;c<sizeof(readings)/sizeof(readings[0]);c++) {
        for(unsigned i=0;i<20;i++) {
            straight_tick(150,readings[c][0],readings[c][1],true);
            assert(straight_drive.state==S3_DRIVE_FOLLOW);
            assert(requested_left==S3_COURSE_BASE_MPS && requested_right==S3_COURSE_BASE_MPS);
        }
    }
    /* An explicit STOP, an unmapped barcode and invalid sensors still stop. */
    s3_drive_stop(&straight_drive);
    assert(straight_drive.state==S3_DRIVE_OFF && requested_stops>0 && requested_left==0 && requested_right==0);
    straight_start(false);straight_scan_a();
    assert(straight_drive.state==S3_DRIVE_HALTED && straight_drive.reason==S3_DRIVE_UNMAPPED);
    assert(requested_left==0 && requested_right==0);
    straight_start(false);straight_tick(150,150,150,false);
    assert(straight_drive.state==S3_DRIVE_FAULT && straight_drive.reason==S3_DRIVE_SENSOR_FAULT);
    assert(requested_left==0 && requested_right==0);
    straight_start(false);straight_tick(150,150,150,true);
    s3_drive_terrain_fault(&straight_drive);
    assert(straight_drive.state==S3_DRIVE_FAULT && straight_drive.reason==S3_DRIVE_TERRAIN_FAULT);
    assert(requested_left==0 && requested_right==0);
    /* This profile does not erase mapped barcode commands. */
    straight_start(true);straight_scan_a();
    assert(straight_drive.state==S3_DRIVE_FOLLOW && straight_drive.pending==S3_NAV_LEFT);
    straight_hold(4,150,280,280);
    assert(straight_drive.state==S3_DRIVE_BRAKING && requested_left==0 && requested_right==0);
}
#endif

int main(void)
{
    s3_line_config_t cfg = s3_line_defaults();
    s3_line_detector_t det;
    s3_line_observation_t o;
    uint32_t t = 0U;

    /* Defaults are Buddy 3's original design. */
    assert(!cfg.between);

    /* Between mode: both white is the normal centred state, forever. */
    cfg.between = true;
    assert(s3_line_init(&det, &cfg));
    for (unsigned i = 0U; i < 500U; i++) /* 5 s of white */
    {
        o = step(&det, &t, 0U, 0U);
        assert(o.valid && o.present && !o.lost_event && o.error == 0 &&
               !o.touching && !o.junction_candidate);
    }
    /* One sensor on the tape: steer towards it (sign as in Buddy 3). */
    o = step(&det, &t, 1000U, 0U);
    assert(o.present && o.touching && o.error == -100);
    o = step(&det, &t, 0U, 1000U);
    assert(o.present && o.touching && o.error == 100);
    o = step(&det, &t, 150U, 0U); /* below the touching level */
    assert(!o.touching && o.error == -15);
    /* Both dark for 30 ms: a crossing (unchanged rule). */
    o = step(&det, &t, 900U, 900U);
    assert(o.junction_candidate && !o.junction_event);
    o = step(&det, &t, 900U, 900U);
    o = step(&det, &t, 900U, 900U);
    o = step(&det, &t, 900U, 900U);
    assert(o.junction_event);
    /* Invalid input still invalidates (sensor faults are not hidden). */
    t += 10000U;
    o = s3_line_update(&det, t, false, 0U, 0U);
    assert(!o.valid && !o.present);
    /* The touching level must be a per-sensor value in between mode. */
    cfg.signal_floor = 1500U;
    assert(!s3_line_init(&det, &cfg));

    /* Original mode is untouched: both white -> lost after 100 ms. */
    cfg = s3_line_defaults();
    assert(s3_line_init(&det, &cfg));
    t = 0U;
    for (unsigned i = 0U; i < 5U; i++) { (void)step(&det, &t, 500U, 500U); }
    {
        bool lost = false;
        for (unsigned i = 0U; i < 15U; i++)
        {
            o = step(&det, &t, 0U, 0U);
            lost = lost || o.lost_event;
        }
        assert(lost && !o.present);
    }

    /* Drive options: defaults reproduce the original controller. */
    {
        const s3_drive_options_t def = s3_drive_default_options();
        assert(!def.cross_straight && def.turn_advance_m == 0.0f &&
               def.pending_wait_us == 5000000U && def.turn_line_window_us == 0U &&
               def.search_deg == 0.0f &&
               def.drive == 0);
    }
    /* Course profile values. */
    {
        const s3_drive_options_t c = s3_course_options();
        s3_navigation_config_t nav = s3_navigation_defaults();
        s3_course_navigation(&nav);
        assert(nav.line.between == (S3_STRAIGHT_TEST || S3_LINE_BETWEEN != 0));
        assert(nav.steering.kp_mps == (S3_STRAIGHT_TEST ? 0.0f : S3_COURSE_STEERING_KP));
        /* Barcode cut-offs: barcode sensor only, line sensors unchanged. */
        assert(nav.calibration[S3_SENSOR_BARCODE].white_limit == S3_BARCODE_WHITE_LIMIT &&
               nav.calibration[S3_SENSOR_BARCODE].black_limit == S3_BARCODE_BLACK_LIMIT);
        assert(nav.calibration[S3_SENSOR_LEFT].white_limit == 400U &&
               nav.calibration[S3_SENSOR_LEFT].black_limit == 600U &&
               nav.calibration[S3_SENSOR_RIGHT].white_limit == 400U &&
               nav.calibration[S3_SENSOR_RIGHT].black_limit == 600U);
        {   /* and they are valid for the reflectance model */
            ir_reflectance_t chk;
            assert(ir_reflectance_init(&chk, &nav.calibration[S3_SENSOR_BARCODE]));
        }
        assert(nav.steering.base_mps == S3_COURSE_BASE_MPS);
        assert(s3_steering_config_valid(&nav.steering));
        {   /* full error: inner wheel stopped, outer at base + gain */
            const s3_line_observation_t full = {true, true, false, -100,
                                                false, false, false, true};
            const s3_steering_request_t r = s3_steering_compute(&nav.steering, &full);
#if S3_STRAIGHT_TEST
            assert(r.valid && r.left_mps == S3_COURSE_BASE_MPS && r.right_mps == S3_COURSE_BASE_MPS);
#else
            assert(r.valid && r.left_mps == 0.0f &&
                   r.right_mps > S3_COURSE_BASE_MPS + S3_COURSE_STEERING_KP - 0.001f);
#endif
        }
        assert(c.cross_straight && c.pending_wait_us == S3_COURSE_PENDING_WAIT_US &&
               c.turn_line_window_us == 300000U);
        /* After-turn search: on, below 45 deg (cannot reach the next arm). */
        assert(c.search_deg == S3_TURN_SEARCH_DEG && c.search_deg > 0.0f &&
               c.search_deg < 45.0f && c.search_mps == S3_TURN_SEARCH_MPS &&
               c.track_m == S3_TRACK_M);
#if S3_SENSOR_TO_AXLE_MM == 0
        assert(c.turn_advance_m == 0.0f && c.drive == 0);
#else
        assert(c.drive == motion_request_drive && c.advance_mps > 0.0f &&
               c.turn_advance_m > 0.10f * (float)S3_SENSOR_TO_AXLE_MM / 100.0f);
#endif
    }
    /* set_options validation. */
    {
        s3_drive_t drive;
        s3_navigation_config_t nav = s3_navigation_defaults();
        const s3_drive_port_t port = {fake_velocity, fake_manoeuvre, fake_stop,
                                      fake_report, fake_id, fake_reset,
                                      fake_heartbeat};
        s3_drive_options_t o2 = s3_course_options();
        nav.acquisition.enabled_mask = S3_ALL_SENSORS;
        assert(s3_drive_init(&drive, &nav, &port));
        assert(!drive.options.cross_straight); /* init installs the defaults */
        o2.turn_advance_m = 0.065f;
        o2.drive = 0;
        assert(!s3_drive_set_options(&drive, &o2)); /* advance needs a drive port */
        assert(!drive.options.cross_straight);      /* rejected: unchanged */
        o2.drive = fake_drive;
        assert(s3_drive_set_options(&drive, &o2));
        assert(drive.options.cross_straight && drive.options.turn_advance_m == 0.065f);
        o2.turn_advance_m = 0.5f; /* beyond any plausible sensor distance */
        assert(!s3_drive_set_options(&drive, &o2));
        o2.turn_advance_m = 0.065f;
        o2.search_deg = 70.0f;                         /* too wide */
        assert(!s3_drive_set_options(&drive, &o2));
        o2.search_deg = 35.0f; o2.search_mps = 0.0f;   /* sweep needs a speed */
        assert(!s3_drive_set_options(&drive, &o2));
        o2.search_mps = 0.05f; o2.turn_line_window_us = 0U; /* needs the check */
        assert(!s3_drive_set_options(&drive, &o2));
        o2.turn_line_window_us = 300000U;
        assert(s3_drive_set_options(&drive, &o2));
        o2.turn_advance_m = 0.065f;
        o2.pending_wait_us = 0U;
        assert(!s3_drive_set_options(&drive, &o2));
        /* A halt keeps this cycle's barcode event so telemetry still
         * reports the letter that caused it (e.g. an unmapped one). */
        drive.output.events = S3_EVENT_BARCODE | S3_EVENT_JUNCTION;
        s3_drive_stop(&drive);
        assert(drive.output.events == S3_EVENT_BARCODE);
        drive.state = S3_DRIVE_FOLLOW; /* not while running */
        o2.pending_wait_us = 20000000U;
        assert(!s3_drive_set_options(&drive, &o2));
    }
#if S3_STRAIGHT_TEST
    test_straight_requests();
    puts("PASS: straight-test equal wheel requests for white/extreme contrast/junctions; STOP, unmapped barcode and sensor/terrain faults preserved; mapped barcode commands retained");
#else
    puts("PASS: between-sensors detector (centred white, steering sign, touching, junction, no line lost), original mode unchanged, course options and validation");
#endif
    return 0;
}

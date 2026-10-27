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
static motion_result_t fake_velocity(float l, float r) { (void)l; (void)r; return MOTION_OK; }
static motion_result_t fake_manoeuvre(manoeuvre_t m) { (void)m; return MOTION_OK; }
static void fake_stop(void) {}
static bool fake_report(motion_report_t *r) { (void)r; return false; }
static uint8_t fake_id(void) { return 1U; }
static bool fake_reset(void) { return true; }
static void fake_heartbeat(bool on) { (void)on; }

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
               def.drive == 0);
    }
    /* Course profile values. */
    {
        const s3_drive_options_t c = s3_course_options();
        s3_navigation_config_t nav = s3_navigation_defaults();
        s3_course_navigation(&nav);
        assert(nav.line.between == (S3_LINE_BETWEEN != 0));
        assert(c.cross_straight && c.pending_wait_us == 20000000U &&
               c.turn_line_window_us == 500000U);
#if S3_SENSOR_TO_AXLE_MM == 0
        assert(c.turn_advance_m == 0.0f && c.drive == 0);
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
        o2.pending_wait_us = 0U;
        assert(!s3_drive_set_options(&drive, &o2));
        drive.state = S3_DRIVE_FOLLOW; /* not while running */
        o2.pending_wait_us = 20000000U;
        assert(!s3_drive_set_options(&drive, &o2));
    }
    puts("PASS: between-sensors detector (centred white, steering sign, touching, junction, no line lost), original mode unchanged, course options and validation");
    return 0;
}

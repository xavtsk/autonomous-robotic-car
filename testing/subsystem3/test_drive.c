/* Real Buddy 2 controller/PID/estimator + real Buddy 3 core/bridge.
 * Hardware edges and PWM are mocked; this cannot prove physical behaviour. */
#include <assert.h>
#include <stdio.h>
#include "s3_drive.h"
#include "motion_controller.h"
#include "motion_estimation.h"
#include "motion_lease.h"
#include "motor_control.h"
static uint32_t now, lease_stamp, epoch = 1U;
static bool lease, reject_velocity, reject_turn;
static unsigned int polls, turns, velocities, stops;
static manoeuvre_t last_turn;
static int32_t counts[2];
static float duty[2];
int32_t wheel_encoder_count(wheel_id_t w)
{
    return counts[w];
}
uint32_t wheel_encoder_glitches(wheel_id_t w)
{
    (void)w;
    return 0U;
}
void motor_control_set(wheel_id_t w, float value)
{
    duty[w] = value;
}
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
static motion_state_t poll(void)
{
    polls++;
    return motion_controller_poll();
}
static bool reset(void)
{
    return motion_controller_reset();
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
static void tick(uint16_t barcode, uint16_t left, uint16_t right, bool valid)
{
    unsigned int i;
    motion_feedback_t feedback;
    s3_sensor_frame_t frame = {0};
    now += 10000U;
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
                                   duty[0] == 0.0f && duty[1] == 0.0f};
    frame.timestamp_us = now;
    for (i = 0; i < 3U; i++)
    {
        frame.sensor[i] = (s3_raw_sample_t){now, 0U, 0, true, valid};
    }
    frame.sensor[0].raw = barcode;
    frame.sensor[1].raw = left;
    frame.sensor[2].raw = right;
    assert(s3_drive_process(&drive, &frame, &feedback));
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
    const s3_drive_port_t port = {velocity, manoeuvre, brake,
                                  poll,     reset,     heartbeat};
    s3_calibration_collector_t collector;
    unsigned int i;
    now = 0U;
    lease = false;
    epoch = 1U;
    reject_velocity = false;
    reject_turn = false;
    turns = polls = velocities = stops = 0U;
    counts[0] = counts[1] = 0;
    duty[0] = duty[1] = 0;
    motion_estimation_init(0.001f, 0.13f);
    motion_controller_init(0.30f, 0.0f, 0.0f);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(s3_drive_init(&drive, &cfg, &port));
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
    tick(150U, 202U, 202U, true);
}
static void start(void)
{
    assert(s3_drive_start(&drive, now));
    hold(5U, 150U, 202U, 202U);
    assert(drive.state == S3_DRIVE_FOLLOW &&
           motion_controller_state() == MOTION_VELOCITY);
}
static void scan_a(void)
{
    unsigned int c, i;
    const unsigned int symbols[3] = {0x094U, 0x109U, 0x094U};
    /* Slow synthetic bar widths; encoder progress prevents a motion stall. */
    unsigned int run, steps;
    for (run = 0; run < 40U; run++)
    {
        counts[0]++;
        counts[1]++;
        tick(150U, 202U, 202U, true);
    }
    for (c = 0; c < 3U; c++)
    {
        for (i = 0; i < 9U; i++)
        {
            steps = (symbols[c] & (1U << (8U - i))) ? 12U : 4U;
            for (run = 0; run < steps; run++)
            {
                counts[0]++;
                counts[1]++;
                tick((i % 2U) ? 150U : 280U, 202U, 202U, true);
            }
        }
        if (c < 2U)
        {
            for (run = 0; run < 4U; run++)
            {
                counts[0]++;
                counts[1]++;
                tick(150U, 202U, 202U, true);
            }
        }
    }
    for (run = 0; run < 45U; run++)
    {
        counts[0]++;
        counts[1]++;
        tick(150U, 202U, 202U, true);
    }
}
static void test_turn_cycle(void)
{
    unsigned int i;
    init(true);
    start();
    scan_a();
    hold(4U, 150U, 280U, 280U);
    assert(drive.state == S3_DRIVE_BRAKING && turns == 0U);
    tick(150U, 280U, 280U, true);
    assert(drive.state == S3_DRIVE_MANOEUVRE && turns == 1U &&
           last_turn == MANOEUVRE_LEFT);
    assert(drive.navigation.mode == S3_MODE_SUPPRESSED);
    for (i = 0; i < 30U && drive.state == S3_DRIVE_MANOEUVRE; i++)
    {
        counts[0] -= 2;
        counts[1] += 2;
        tick(150U, 150U, 150U, true);
    }
    assert(drive.state == S3_DRIVE_REACQUIRE && polls == 1U && turns == 1U);
    hold(8U, 150U, 202U, 202U);
    assert(drive.state == S3_DRIVE_FOLLOW && polls == 1U && velocities > 1U);
    assert(drive.pending == S3_NAV_NONE);
    s3_drive_stop(&drive);
    tick(150U, 202U, 202U, true);
    assert(drive.state == S3_DRIVE_OFF && !lease && duty[0] == 0.0f &&
           duty[1] == 0.0f);
}
static void test_faults_and_stops(void)
{
    unsigned int i;
    init(false);
    assert(!s3_drive_start(&drive, now));
    hold(20U, 150U, 202U, 202U);
    assert(velocities == 0U && turns == 0U && !lease);
    init(true);
    start();
    tick(150U, 202U, 202U, false);
    assert(drive.state == S3_DRIVE_FAULT && !lease && stops > 0U);
    tick(150U, 202U, 202U, true);
    assert(duty[0] == 0.0f && duty[1] == 0.0f);
    assert(!s3_drive_reset(&drive, now));
    hold(12U, 150U, 202U, 202U);
    assert(s3_drive_reset(&drive, now));
    assert(drive.state == S3_DRIVE_OFF);
    start();
    epoch++;
    tick(150U, 202U, 202U, true);
    assert(drive.state == S3_DRIVE_FAULT);
    init(true);
    start();
    hold(15U, 150U, 150U, 150U);
    assert(drive.state == S3_DRIVE_HALTED &&
           drive.reason == S3_DRIVE_LINE_LOST);
    hold(10U, 150U, 202U, 202U);
    assert(drive.state == S3_DRIVE_HALTED);
    init(true);
    start();
    hold(5U, 150U, 280U, 280U);
    assert(drive.state == S3_DRIVE_HALTED &&
           drive.reason == S3_DRIVE_NO_DIRECTION && turns == 0U);
    init(true);
    start();
    reject_velocity = true;
    tick(150U, 202U, 202U, true);
    assert(drive.state == S3_DRIVE_FAULT && drive.reason == S3_DRIVE_REJECTED);
    init(true);
    start();
    scan_a();
    reject_turn = true;
    hold(5U, 150U, 280U, 280U);
    assert(drive.state == S3_DRIVE_FAULT && polls == 0U);
    /* Independently running motion owner detects a stalled integration task,
     * including during a turn (which the old velocity timeout did not cover).
     */
    init(true);
    start();
    scan_a();
    hold(5U, 150U, 280U, 280U);
    assert(motion_controller_state() == MOTION_TURN);
    now += 40000U;
    assert(motion_lease_expired(lease, now, lease_stamp));
    motion_controller_navigation_timeout();
    motion_controller_step(0.01f, now);
    assert(motion_controller_state() == MOTION_FAULT);
    assert(duty[0] == 0.0f && duty[1] == 0.0f);
    tick(150U, 202U, 202U, true);
    assert(drive.state == S3_DRIVE_FAULT);
    for (i = 0; i < 30U; i++)
    {
        tick(150U, 202U, 202U, true);
    }
    assert(!s3_drive_reset(&drive,
                           now)); /* Snapshot still faulted; B2 reset first. */
    tick(150U, 202U, 202U, true);
    assert(s3_drive_reset(&drive, now));
    assert(!motion_lease_expired(true, 0x10U, 0xfffffff0U));
    assert(motion_lease_expired(true, 40000U, 0xfffffff0U));
}
static void test_mapping_and_expiry(void)
{
    const s3_nav_action_t actions[4] = {S3_NAV_LEFT, S3_NAV_RIGHT,
                                        S3_NAV_STRAIGHT, S3_NAV_UTURN};
    const manoeuvre_t expected[4] = {MANOEUVRE_LEFT, MANOEUVRE_RIGHT,
                                     MANOEUVRE_STRAIGHT, MANOEUVRE_U_TURN};
    unsigned int i, j;
    for (i = 0U; i < 4U; i++)
    {
        s3_nav_action_t map[26] = {0};
        init(true);
        map[0] = actions[i];
        assert(s3_navigation_set_mapping(&drive.navigation, map));
        start();
        scan_a();
        assert(drive.pending == actions[i]);
        hold(5U, 150U, 280U, 280U);
        assert(turns == 1U && last_turn == expected[i]);
        s3_drive_stop(&drive);
        tick(150U, 202U, 202U, true);
        assert(drive.state == S3_DRIVE_OFF && !lease && polls == 0U);
    }
    {
        s3_nav_action_t map[26] = {0};
        init(true);
        assert(s3_navigation_set_mapping(&drive.navigation, map));
        start();
        scan_a();
        assert(drive.state == S3_DRIVE_HALTED &&
               drive.reason == S3_DRIVE_UNMAPPED);
    }
    init(true);
    start();
    scan_a();
    assert(drive.pending == S3_NAV_LEFT);
    for (j = 0U; j < 510U; j++)
    {
        counts[0]++;
        counts[1]++;
        tick(150U, 202U, 202U, true);
    }
    assert(drive.state == S3_DRIVE_HALTED && drive.reason == S3_DRIVE_TIMEOUT);
    assert(drive.pending == S3_NAV_NONE && turns == 0U);
    /* A cancelled partial scan cannot become a command after explicit restart.
     */
    init(true);
    start();
    hold(40U, 150U, 202U, 202U);
    hold(4U, 280U, 202U, 202U);
    s3_drive_stop(&drive);
    tick(150U, 202U, 202U, true);
    assert(s3_drive_start(&drive, now));
    hold(20U, 150U, 202U, 202U);
    assert(drive.pending == S3_NAV_NONE && turns == 0U);
}
int main(void)
{
    test_turn_cycle();
    test_faults_and_stops();
    test_mapping_and_expiry();
    puts("PASS: real Buddy 2 controller/PID + Buddy 3 navigation bridge; "
         "mocked hardware");
    return 0;
}

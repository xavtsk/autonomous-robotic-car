/* Real Buddy 2 controller/PID/estimator + real Buddy 3 core/bridge.
 * Hardware edges and PWM are mocked; this cannot prove physical behaviour. */
#include "motion_controller.h"
#include "motion_estimation.h"
#include "motion_lease.h"
#include "motor_control.h"
#include "s3_drive.h"
#include <assert.h>
#include <stdio.h>
static uint32_t now, lease_stamp, epoch = 1U;
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
    const s3_drive_port_t port = {
        velocity, manoeuvre, brake, receive, motion_controller_command_id,
        reset,    heartbeat};
    s3_calibration_collector_t collector;
    unsigned int i;
    now = 0U;
    lease = false;
    epoch = 1U;
    reject_velocity = false;
    reject_turn = false;
    reject_reset = false;
    turns = polls = velocities = stops = 0U;
    counts[0] = counts[1] = 0;
    duty[0] = duty[1] = 0;
    motion_estimation_init(0.001f, 0.13f);
    report_count = 0U;
    report_losses = 0U;
    motion_controller_init(0.30f, 0.0f, 0.0f, post);
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
    hold(12U, 150U, 202U, 202U);
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
    hold(20U, 150U, 280U, 280U);
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
    hold(25U, 150U, 202U, 202U);
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
    assert(s3_drive_reset(&drive, now));
    assert(s3_drive_reset_result(&drive) == S3_RESET_PENDING);
    hold(30U, 150U, 202U, 202U);
    assert(s3_drive_reset_result(&drive) == S3_RESET_OK &&
           drive.state == S3_DRIVE_OFF);
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
    hold(25U, 150U, 280U, 280U);
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
    hold(25U, 150U, 280U, 280U);
    assert(drive.state == S3_DRIVE_FAULT && polls == 0U);
    /* Independently running motion owner detects a stalled integration task,
     * including during a turn (which the old velocity timeout did not cover).
     */
    init(true);
    start();
    scan_a();
    hold(25U, 150U, 280U, 280U);
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
    /* Exactly one RESET: Buddy 2 changes state now, but the controller keeps
     * its cached fault until the next published feedback. No second press. */
    assert(drive.feedback.fault != MOTION_FAULT_NONE);
    assert(s3_drive_reset(&drive, now));
    assert(motion_controller_state() == MOTION_IDLE);
    assert(drive.state == S3_DRIVE_FAULT &&
           s3_drive_reset_result(&drive) == S3_RESET_PENDING);
    tick(150U, 202U, 202U, true);
    assert(drive.state == S3_DRIVE_OFF &&
           s3_drive_reset_result(&drive) == S3_RESET_OK);
    assert(!lease && duty[0] == 0.0f && duty[1] == 0.0f);
    assert(!motion_lease_expired(true, 0x10U, 0xfffffff0U));
    assert(motion_lease_expired(true, 40000U, 0xfffffff0U));
}
/* Deliver a synthetic feedback snapshot without advancing the motion owner.
 * This specifically tests slow/stale publication and unhealthy replies; the
 * normal tick() above exercises the REAL Buddy 2 controller reset path. */
static void reset_observation(motion_feedback_t feedback)
{
    s3_sensor_frame_t frame = {0};
    now += 10000U;
    frame.timestamp_us = now;
    for (unsigned int i = 0; i < S3_SENSOR_COUNT; i++)
    {
        frame.sensor[i] = (s3_raw_sample_t){now, i ? 202U : 150U,
                                           0, true, true};
    }
    assert(s3_drive_process(&drive, &frame, &feedback));
}
static void test_reset_acknowledgement(void)
{
    motion_feedback_t feedback;
    unsigned int before;
    /* Stale, unbraked, still-faulted and invalid snapshots cannot acknowledge
     * a reset. Each attempt terminates within the bounded 500 ms wait. */
    for (unsigned int scenario = 0U; scenario < 4U; scenario++)
    {
        init(true);
        drive.state = S3_DRIVE_FAULT;
        assert(s3_drive_reset(&drive, now));
        assert(!s3_drive_reset(&drive, now)); /* no second reset in flight */
        assert(!s3_drive_start(&drive, now));
        feedback = drive.feedback;
        before = velocities;
        for (unsigned int i = 0U; i < S3_RESET_WAIT_US / 10000U; i++)
        {
            if (scenario != 0U)
            {
                feedback.timestamp_us = now + 10000U;
            }
            if (scenario == 1U)
            {
                feedback.braked = false;
            }
            else if (scenario == 2U)
            {
                feedback.state = MOTION_FAULT;
                feedback.fault = MOTION_FAULT_STALL;
            }
            else if (scenario == 3U)
            {
                feedback.valid = false;
            }
            reset_observation(feedback);
            assert(drive.state == S3_DRIVE_FAULT);
        }
        assert(s3_drive_reset_result(&drive) == S3_RESET_FAILED && !lease);
        assert(velocities == before && turns == 0U);
    }
    /* A port rejection stays faulted and is reported immediately. */
    init(true);
    drive.state = S3_DRIVE_FAULT;
    reject_reset = true;
    assert(!s3_drive_reset(&drive, now));
    assert(s3_drive_reset_result(&drive) == S3_RESET_FAILED);
    assert(drive.state == S3_DRIVE_FAULT && !lease);
    /* STOP wins over a reset awaiting new feedback; a later healthy snapshot
     * cannot finish the cancelled attempt or start either motor. */
    reject_reset = false;
    assert(s3_drive_reset(&drive, now));
    s3_drive_stop(&drive);
    assert(s3_drive_reset_result(&drive) == S3_RESET_FAILED);
    hold(30U, 150U, 202U, 202U);
    assert(drive.state == S3_DRIVE_FAULT && velocities == 0U && !lease);
    /* It is safe to explicitly request RESET again after cancellation. */
    assert(s3_drive_reset(&drive, now));
    tick(150U, 202U, 202U, true);
    assert(s3_drive_reset_result(&drive) == S3_RESET_OK &&
           drive.state == S3_DRIVE_OFF && velocities == 0U && !lease);
    /* Persistent sensor failure may not be cleared by a healthy motor reset. */
    init(true);
    start();
    tick(150U, 202U, 202U, false);
    assert(s3_drive_reset(&drive, now));
    before = velocities;
    for (unsigned int i = 0U; i < S3_RESET_WAIT_US / 10000U; i++)
    {
        tick(150U, 202U, 202U, false);
    }
    assert(s3_drive_reset_result(&drive) == S3_RESET_FAILED &&
           drive.state == S3_DRIVE_FAULT && velocities == before && !lease);
    assert(s3_drive_reset_result(0) == S3_RESET_FAILED);
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
        hold(25U, 150U, 280U, 280U);
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
    hold(20U, 150U, 202U, 202U);
    assert(s3_drive_start(&drive, now));
    hold(20U, 150U, 202U, 202U);
    assert(drive.pending == S3_NAV_NONE && turns == 0U);
}
static void enter_turn(void)
{
    init(true);
    start();
    scan_a();
    hold(25U, 150U, 280U, 280U);
    assert(drive.state == S3_DRIVE_MANOEUVRE);
}
static void test_new_motion_contract(void)
{
    unsigned int sent;
    motion_report_t report;
    /* Pending stop and observed wheel stillness are not immediate. A short
     * ambiguous reading must not fault merely because the brake is pending. */
    init(true);
    start();
    for (unsigned int i = 0U; i < 10U; i++)
    {
        counts[0]++;
        counts[1]++;
        tick(150U, 202U, 202U, true);
    }
    sent = velocities;
    tick(150U, 280U, 280U, true);
    assert(drive.waiting_stop && drive.command_id == 0U);
    hold(3U, 150U, 202U, 202U);
    assert(velocities == sent && drive.state == S3_DRIVE_FOLLOW);
    hold(15U, 150U, 202U, 202U);
    assert(velocities > sent && drive.state == S3_DRIVE_FOLLOW);
    assert(report_count == 0U && drive.cancel_id == 0U);

    enter_turn();
    report = (motion_report_t){(uint8_t)(drive.command_id - 1U),
                               MOTION_OUTCOME_DONE, MOTION_FAULT_NONE, false};
    post(&report);
    tick(150U, 280U, 280U, true);
    assert(drive.state == S3_DRIVE_MANOEUVRE); /* old ID cannot finish turn */
    report.command_id = drive.command_id;
    report.outcome = MOTION_OUTCOME_CANCELLED;
    post(&report);
    tick(150U, 280U, 280U, true);
    assert(drive.state == S3_DRIVE_FAULT &&
           drive.reason == S3_DRIVE_REPORT_FAULT);

    enter_turn();
    report = (motion_report_t){drive.command_id, MOTION_OUTCOME_DONE,
                               MOTION_FAULT_NONE, true};
    post(&report);
    tick(150U, 280U, 280U, true);
    assert(drive.state == S3_DRIVE_FAULT); /* report flags loss */

    enter_turn();
    report_losses++;
    tick(150U, 280U, 280U, true);
    assert(drive.state == S3_DRIVE_FAULT &&
           !lease); /* no later report needed */

    enter_turn();
    motion_controller_stop(); /* another caller cancelled our manoeuvre */
    tick(150U, 280U, 280U, true);
    assert(drive.state == S3_DRIVE_FAULT);

    /* Drain the full bounded queue even when inactive. */
    init(true);
    report = (motion_report_t){100U, MOTION_OUTCOME_CANCELLED,
                               MOTION_FAULT_NONE, false};
    for (unsigned int i = 0U; i < MOTION_REPORT_QUEUE_LEN; i++)
    {
        post(&report);
    }
    tick(150U, 202U, 202U, true);
    assert(report_count == 0U && drive.state == S3_DRIVE_OFF);
}
int main(void)
{
    test_turn_cycle();
    test_faults_and_stops();
    test_reset_acknowledgement();
    test_mapping_and_expiry();
    test_new_motion_contract();
    puts("PASS: real Buddy 2 controller/PID + Buddy 3 navigation bridge; "
         "mocked hardware");
    return 0;
}

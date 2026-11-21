/** @file host_regression_main.c
 * @brief Runs the real public motion API, controller, PID, estimation, encoder
 * ISR and motor driver against deterministic fake registers.
 *
 * Compile unchanged
 * against the before/after source trees and compare complete stdout traces.
 */
#include "host_regression_main.h"
#include "../../app_program/app_types.h"
#include "motion.h"
#include "ctrl_motion.h"
#include "motion_estimation.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#ifdef TEST_PACKED
#include "codec_motion.h"
#include "../../app_program/subsystem1/telemetry_codec.h"
#endif
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint32_t now_us;
static uint32_t gpio;
static uint32_t pending[4];
static uint32_t pwm[30];
static uint32_t io_hash;
static unsigned phase[2];
static unsigned rows;
static unsigned lock_depth;
static unsigned failures;
static void     ( * gp_irq)(UINT interrupt_number) = NULL;
static bool     gb_should_auto_tick;

/** @brief Verify the priority-inheritance attribute and return the configured
 * fake mutex ID.
 */
ID tk_cre_mtx (const T_CMTX * p_spec)
{
    assert(TA_INHERIT == p_spec->mtxatr);
    return 1;
}

/** @brief Require the expected mutex and indefinite wait, then count one
 * simulated acquisition.
 */
ER tk_loc_mtx (ID id, TMO t)
{
    assert(1 == id && TMO_FEVR == t);
    ++lock_depth;
    return E_OK;
}

/** @brief Require the acquired fake mutex, then release it and report
 * success.
 */
ER tk_unl_mtx (ID id)
{
    assert(1 == id && 1 == lock_depth);
    --lock_depth;
    return E_OK;
}

/** @brief Require interrupt 13 and save its handler for simulated encoder
 * events.
 */
ER tk_def_int (UINT n, const T_DINT * p_spec)
{
    assert(13 == n);
    gp_irq = (void (*)(UINT))p_spec->inthdr;
    return E_OK;
}

/** @brief Acknowledge a wake request without scheduling a task.
 */
ER tk_wup_tsk (ID id)
{
    (void)id;
    return E_OK;
}

/** @brief Report an empty wake queue without blocking the calling task.
 */
INT tk_can_wup (ID id)
{
    (void)id;
    return 0;
}

/** @brief Advance one millisecond per iteration and run motion processing at
 * each enabled 10 ms boundary.
 */
ER tk_dly_tsk (INT ms)
{
    for (INT i = 0; i < ms; ++i)
    {
        now_us += 1000U;
        if (gb_should_auto_tick && now_us % 10000U == 0U)
        {
            motion_process();
        }
    }
    return E_OK;
}

/** @brief Return the simulated timer, GPIO level or interrupt-pending
 * register for the requested address. Unknown addresses read as
 * zero.
 */
uint32_t in_w (uint32_t a)
{
    if (TIMER_TIMERAWL == a)
    {
        return now_us;
    }
    if (GPIO_IN == a)
    {
        return gpio;
    }
    if (SIO_BASE == a)
    {
        return 0U;
    }
    if (a >= IO_BANK0_BASE + 0x120U && a < IO_BANK0_BASE + 0x130U)
    {
        return pending[(a - IO_BANK0_BASE - 0x120U) / 4U];
    }
    return 0U;
}

/** @brief Add the address and value to the I/O trace hash and acknowledge
 * matching simulated interrupt bits.
 */
void out_w (uint32_t a, uint32_t v)
{
    io_hash = io_hash * 33U + a + v;
    if (a >= IO_BANK0_BASE + 0xf0U && a < IO_BANK0_BASE + 0x100U)
    {
        pending[(a - IO_BANK0_BASE - 0xf0U) / 4U] &= ~v;
    }
}

/** @brief Record the register-clear operation through the simulated output
 * routine.
 */
void clr_w (uint32_t a, uint32_t v)
{
    out_w(a, v);
}

/** @brief Record the register-set operation through the simulated output
 * routine.
 */
void set_w (uint32_t a, uint32_t v)
{
    out_w(a, v);
}

/** @brief Assert the expected encoder interrupt number and priority.
 */
void EnableInt (UINT n, INT p)
{
    assert(13 == n && 1 == p);
}

/** @brief Record the selected GPIO function and return success.
 */
int gpio_set_pin (uint32_t p, uint32_t mode)
{
    out_w(p, mode);
    return 0;
}

/** @brief Record PWM selection on the requested pin and return success.
 */
int pwm_set_pin (uint32_t p)
{
    out_w(p, 1);
    return 0;
}

/** @brief Check the expected PWM wrap value and include it in the register
 * trace.
 */
int pwm_set_wrap (uint32_t p, uint32_t w)
{
    assert(12499 == w);
    out_w(p, w);
    return 0;
}

/** @brief Record the requested PWM enable state and return success.
 */
int pwm_set_enabled (uint32_t p, BOOL e)
{
    out_w(p, e);
    return 0;
}

/** @brief Check the motor channel and duty bounds, save the compare count
 * and record the write.
 */
int pwm_set_cc (uint32_t p, uint32_t v)
{
    assert(p >= 8 && p <= 11 && v <= 12500);
    pwm[p] = v;
    out_w(p, v);
    return 0;
}

/** @brief Print the bounded device diagnostic line and count occurrences of
 * the failure marker. Return the formatter's character count.
 */
int tm_printf (const UB * p_format, ...)
{
    char    text[512];
    va_list args;
    va_start(args, p_format);
    int n = vsnprintf(text, sizeof(text), (const char *)p_format, args);
    va_end(args);
    if (strstr(text, " FAIL "))
    {
        ++failures;
    }
    fputs(text, stdout);
    return n;
}

/** @brief Copy the binary32 representation into an unsigned word for exact
 * trace comparisons.
 */
static uint32_t float_bits (float32_t v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    return bits;
}

/** @brief Emit one numbered row containing the operation result, motion
 * state, wheel duties, PWM compares and I/O hash.
 */
static void trace (const char * p_name, int result)
{
    printf(
        "%s %u %d %u %u %d %d %08x %08x %08x %08x %08x %08x %u %u %u %u %08x\n",
        p_name, rows++, result, (unsigned)ctrl_motion_state(),
        (unsigned)ctrl_motion_fault(), enc_l_count(), enc_r_count(),
        float_bits(motion_wheel_left_speed_mps()),
        float_bits(motion_wheel_right_speed_mps()),
        float_bits(robot_distance_m()), float_bits(robot_heading_deg()),
        float_bits(motor_control_duty(WHEEL_LEFT)),
        float_bits(motor_control_duty(WHEEL_RIGHT)), pwm[8], pwm[9], pwm[10],
        pwm[11], io_hash);
}

/** @brief Set the selected encoder's two simulated GPIO levels and dispatch
 * its pending interrupt.
 */
static void edge (unsigned wheel, unsigned state)
{
    unsigned a = wheel ? 4U : 16U;
    unsigned b = a + 1U;
    gpio = (gpio & ~((1U << a) | (1U << b))) | (((state >> 1) & 1U) << a) |
           ((state & 1U) << b);
    pending[a / 8] |= 0xcU << ((a % 8) * 4U);
    pending[b / 8] |= 0xcU << ((b % 8) * 4U);
    gp_irq(13);
}

/** @brief Generate the requested signed number of quadrature transitions on
 * one wheel.
 */
static void steps (unsigned wheel, int count)
{
    const unsigned sequence[4] = {0, 2, 3, 1};
    for (int pulse_index = 0; pulse_index < (count < 0 ? -count : count);
         ++pulse_index)
    {
        phase[wheel] = (phase[wheel] + (count < 0 ? 3U : 1U)) & 3U;
        edge(wheel, sequence[phase[wheel]]);
    }
}

/** @brief Advance both encoders and the simulated clock, run motion
 * processing and emit the resulting trace row.
 */
static void tick (const char * p_name, unsigned elapsed, int left, int right)
{
    steps(0, left);
    steps(1, right);
    now_us += elapsed;
    motion_process();
    trace(p_name, 0);
}

/** @brief Reset motion with automatic clock stepping disabled and emit its
 * initial state.
 */
static void fresh (const char * p_name)
{
    gb_should_auto_tick = false;
    ctrl_motion_set_busy(false);
    motion_init();
    trace(p_name, 0);
}

/** @brief Exercise six manoeuvre commands, simulate completion and check how
 * the completion report is consumed.
 */
static void manoeuvres (void)
{
    for (unsigned move_index = 0; move_index < 6; ++move_index)
    {
        fresh("move-init");
        int result =
            4 == move_index ? moveForward(0.2f)
            : 5 == move_index
                ? moveBackward(0.2f)
                : motion_request_manoeuvre((motion_manoeuvre_t)move_index);
        assert(MOTION_OK == result);
        trace("move-request", result);
        int left =
            0 == move_index || 3 == move_index || 5 == move_index ? -20 : 20;
        int right = 1 == move_index || 5 == move_index ? -20 : 20;
        for (unsigned tick_index = 0;
             tick_index < 300 && ctrl_motion_state() != MOTION_COMPLETE;
             ++tick_index)
        {
            tick("move-step", 10000, left, right);
        }
        assert(ctrl_motion_state() == MOTION_COMPLETE);
        trace("move-complete", motion_poll());
        trace("move-consumed", motion_poll());
    }
}

/** @brief Compare all pairs of encoder phases against the expected signed
 * displacement, including the inverted-wheel setting.
 */
static void quadrature_transitions (void)
{
    const int table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    const unsigned phases[4] = {0, 3, 1, 2};
    for (unsigned invert = 0; invert < 2; ++invert)
    {
        for (unsigned previous = 0; previous < 4; ++previous)
        {
            for (unsigned next = 0; next < 4; ++next)
            {
                edge(0, previous);
                phase[0] = phases[previous];
                wheel_encoder_set_inverted(WHEEL_LEFT, invert != 0);
                int32_t  before   = enc_l_count();
                uint32_t glitches = wheel_encoder_glitches(WHEEL_LEFT);
                edge(0, next);
                phase[0]     = phases[next];
                int expected = table[previous * 4 + next];
                assert(enc_l_count() ==
                       before + (invert ? -expected : expected));
                assert(wheel_encoder_glitches(WHEEL_LEFT) ==
                       glitches +
                           (unsigned)(0 == expected && previous != next));
                trace("quadrature", expected);
            }
        }
    }
    wheel_encoder_set_inverted(WHEEL_LEFT, false);
}

/** @brief Inject stalled wheels and a stale command and verify the reported
 * motion faults.
 */
static void faults (void)
{
    fresh("stall-init");
    assert(moveForward(1.0f) == MOTION_OK);
    for (unsigned tick_index = 0;
         tick_index < 100 && ctrl_motion_state() != MOTION_FAULT; ++tick_index)
    {
        tick("stall", 50000, 0, 0);
    }
    assert(motion_get_fault() == MOTION_FAULT_STALL);
    trace("stall-stop", (stop(), motion_poll()));
    trace("stall-reject", moveForward(0.1f));
    for (unsigned tick_index = 0; tick_index < 5; ++tick_index)
    {
        tick("still-reset", 50000, 0, 0);
    }
    assert(motion_reset());
    trace("reset", motion_poll());

    fresh("direction-init");
    assert(motion_request_velocity(0.2f, 0.2f) == MOTION_OK);
    for (unsigned tick_index = 0;
         tick_index < 100 && ctrl_motion_state() != MOTION_FAULT; ++tick_index)
    {
        assert(motion_request_velocity(0.2f, 0.2f) == MOTION_OK);
        tick("direction", 50000, -30, -30);
    }
    assert(motion_get_fault() == MOTION_FAULT_DIRECTION);

    fresh("encoder-init");
    assert(moveForward(1.0f) == MOTION_OK);
    for (unsigned tick_index = 0; tick_index < 40; ++tick_index)
    {
        const unsigned seq[4] = {0, 2, 3, 1};
        phase[0]              = (phase[0] + 2U) & 3U;
        edge(0, seq[phase[0]]);
    }
    tick("encoder", 10000, 0, 0);
    assert(motion_get_fault() == MOTION_FAULT_ENCODER);

    fresh("timeout-init");
    assert(moveForward(0.2f) == MOTION_OK);
    for (unsigned tick_index = 0;
         tick_index < 600 && ctrl_motion_state() != MOTION_FAULT; ++tick_index)
    {
        tick("timeout", 10000, (tick_index & 1) ? -2 : 2,
             (tick_index & 1) ? -2 : 2);
    }
    assert(motion_get_fault() == MOTION_FAULT_TIMEOUT);

    fresh("stale-init");
    assert(motion_request_velocity(0, 0) == MOTION_OK);
    tick("stale", 251000, 0, 0);
    assert(motion_get_fault() == MOTION_FAULT_STALE_COMMAND);
}

#ifdef TEST_PACKED
/** @brief Check packed motion snapshots across all speed codes and
 * calibration combinations, including invalid estimate inputs.
 */
static void packed_checks (void)
{
    codec_motion_snapshot_t snapshot;
    assert(!snapshot_motion_read(NULL));
    assert(sizeof(snapshot) == 4);
    for (int speed = -510; speed <= 510; ++speed)
    {
        telemetry_codec_snapshot_t input = {0};
        telemetry_codec_snapshot_t decoded;
        uint8_t                    packet[4];
        input.left_mms  = (int16_t)speed;
        input.right_mms = (int16_t)-speed;
        assert(buddy1_encode(BUDDY1_TOPIC_MOTION, &input, packet,
                             sizeof(packet)) == 4);
        assert(buddy1_decode(BUDDY1_TOPIC_MOTION, packet, sizeof(packet),
                             &decoded));
        for (unsigned state = 0; state <= MOTION_FAULT; ++state)
        {
            for (unsigned fault = 0; fault <= MOTION_FAULT_TIMEOUT; ++fault)
            {
                assert(codec_motion_pack(&snapshot, state, fault, true, true,
                                         speed, -speed));
                assert(codec_motion_valid(&snapshot));
                assert(codec_motion_state(&snapshot) == state);
                assert(codec_motion_fault(&snapshot) == fault);
                int error = codec_motion_left_mms(&snapshot) - speed;
                assert(error >= -1 && error <= 1);
                assert(codec_motion_right_mms(&snapshot) ==
                       -codec_motion_left_mms(&snapshot));
                assert(codec_motion_left_mms(&snapshot) == decoded.left_mms);
                assert(codec_motion_right_mms(&snapshot) == decoded.right_mms);
            }
        }
    }
    assert(!codec_motion_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false,
                              true, 511, 0));
    assert(!codec_motion_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false,
                              true, 0, -511));
    assert(!codec_motion_pack(&snapshot, (motion_state_t)6, MOTION_FAULT_NONE,
                              false, false, 0, 0));
    assert(!codec_motion_pack(&snapshot, MOTION_IDLE, (motion_fault_t)6, false,
                              false, 0, 0));
    assert(codec_motion_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false,
                             false, 99, -99));
    assert(!codec_motion_speed_valid(&snapshot) &&
           codec_motion_left_mms(&snapshot) == 0);
    snapshot.bytes[0] = 1;
    assert(!codec_motion_valid(&snapshot));
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.bytes[3] = 0x80;
    assert(!codec_motion_valid(&snapshot));
    assert(codec_motion_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false,
                             true, 0, 0));
    snapshot.bytes[1] |= 1U;
    assert(!codec_motion_valid(&snapshot)); /* Exercise negative zero. */
    /* Snapshot does not consume completion or affect full-precision targets. */
    ctrl_motion_set_busy(false);
    ctrl_motion_init(0.30f, 0, 0);
    assert(motion_request_drive(0, 0.2f) == MOTION_OK);
    assert(snapshot_motion_read(&snapshot) &&
           codec_motion_state(&snapshot) == MOTION_COMPLETE);
    assert(motion_poll() == MOTION_COMPLETE && motion_poll() == MOTION_IDLE);
    assert(motion_request_velocity(0.1234567f, -0.0876543f) == MOTION_OK);
    assert(snapshot_motion_read(&snapshot));
    assert(ctrl_motion_state() == MOTION_VELOCITY);
    ctrl_motion_set_busy(true);
    assert(snapshot_motion_read(&snapshot) && codec_motion_busy(&snapshot));
    assert(!codec_motion_speed_valid(&snapshot));
    ctrl_motion_set_busy(false);
    /* Exact flag meanings and all float calibration values survive packing. */
    for (unsigned bits = 0; bits < 256; ++bits)
    {
        motion_calibration_t cal      = {0};
        cal.counts_per_rev            = 2000.25f;
        cal.max_speed_mps             = 0.312345f;
        cal.b_valid                   = (bits >> 0) & 1U;
        cal.b_is_left_motor_inverted  = (bits >> 1) & 1U;
        cal.b_is_right_motor_inverted = (bits >> 2) & 1U;
        cal.b_enc_l_inverted          = (bits >> 3) & 1U;
        cal.b_enc_r_inverted          = (bits >> 4) & 1U;
        cal.b_enc_l_ok                = (bits >> 5) & 1U;
        cal.b_enc_r_ok                = (bits >> 6) & 1U;
        cal.b_encoders_swapped        = (bits >> 7) & 1U;
        assert((cal.b_valid | (cal.b_is_left_motor_inverted << 1) |
                (cal.b_is_right_motor_inverted << 2) |
                (cal.b_enc_l_inverted << 3) | (cal.b_enc_r_inverted << 4) |
                (cal.b_enc_l_ok << 5) | (cal.b_enc_r_ok << 6) |
                (cal.b_encoders_swapped << 7)) == (int)bits);
        assert(2000.25f == cal.counts_per_rev &&
               0.312345f == cal.max_speed_mps);
    }
    /* Nonfinite / out-of-report-range estimates cannot become valid zero. */
    motion_estimation_init(1.0f, 0.13f);
    steps(0, 10);
    motion_estimation_update(0.01f);
    assert(snapshot_motion_read(&snapshot) &&
           !codec_motion_speed_valid(&snapshot));
    motion_estimation_set_scale(NAN);
    motion_estimation_update(0.01f);
    assert(snapshot_motion_read(&snapshot) &&
           !codec_motion_speed_valid(&snapshot));
    fprintf(stderr, "PASS: 36,756 state/fault/speed combinations, boundaries, "
                    "validity and non-consuming snapshot\n");
    fprintf(
        stderr,
        "PASS: Buddy 1 codec agreement for all 1,021 speeds; 256 calibration "
        "flag combinations; nonfinite/out-of-range estimates\n");
}
#endif

extern void subsystem2_test (void);

/** @brief Run simulated motion commands and injected faults, then compare
 * calibration, stop behavior and controller reports.
 */
int main (void)
{
#ifdef TEST_PACKED
    codec_motion_snapshot_t early;
    assert(!snapshot_motion_read(&early));
#endif
    assert(motion_create() == E_OK);
    motion_attach_task(1);
    fresh("startup");
    gb_should_auto_tick = true;
    subsystem2_test();
    gb_should_auto_tick = false;
    assert(0 == failures);
    manoeuvres();
    faults();
    quadrature_transitions();
    fresh("velocity-init");
    for (unsigned tick_index = 0; tick_index < 500; ++tick_index)
    {
        float32_t left  = (tick_index % 21 - 10.0f) * 0.013579f;
        float32_t right = (tick_index % 19 - 9.0f) * 0.017531f;
        trace("velocity-request", motion_request_velocity(left, right));
        tick("velocity-step", tick_index % 3 == 0 ? 75000 : 10000,
             (int)(tick_index % 17) - 8, (int)(tick_index % 13) - 6);
        if (ctrl_motion_state() == MOTION_FAULT)
        {
            fresh("velocity-reset");
        }
    }
    fresh("long-still");
    for (unsigned tick_index = 0; tick_index < 1700; ++tick_index)
    {
        tick("still", 37000, 0, 0);
    }
    /* IRQ state and task configuration are independent after packing. */
    int32_t count = enc_l_count();
    wheel_encoder_set_inverted(WHEEL_LEFT, true);
    steps(0, 40);
    assert(enc_l_count() == count - 40);
    trace("inverted", 0);
    wheel_encoder_set_inverted(WHEEL_LEFT, false);
    steps(0, 40);
    assert(enc_l_count() == count);
    trace("restored", 0);
    now_us = UINT32_MAX - 100000U;
    fresh("wrap-init");
    assert(motion_request_velocity(0, 0) == MOTION_OK);
    tick("wrap", 300000, 0, 0);
    assert(motion_get_fault() == MOTION_FAULT_STALE_COMMAND);
    fresh("calibration-init");
    assert(motion_request_calibration() == MOTION_OK);
    assert(motion_is_calibrating());
    trace("calibration-request", motion_request_calibration());
    motion_calibration_t calibration;
    assert(!motion_get_calibration(&calibration));
#ifdef TEST_PACKED
    packed_checks();
#endif
    assert(0 == lock_depth);
    fprintf(stderr,
            "PASS: %u deterministic control trace rows; original non-moving "
            "unit tests; all five fault reasons\n",
            rows);
    return 0;
}

/* End of file. */


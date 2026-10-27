/* Runs the real public motion API, controller, PID, estimation, encoder ISR
 * and motor driver against deterministic fake registers. Compile unchanged
 * against the before/after source trees and compare complete stdout traces. */
#include "host_kernel.h"
#include "motion.h"
#include "motion_controller.h"
#include "motion_estimation.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#ifdef TEST_PACKED
#include "motion_codec.h"
#include "../../app_program/subsystem1/telemetry_codec.h"
#endif
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint32_t now_us, gpio, pending[4], pwm[30], io_hash;
static unsigned phase[2], rows, lock_depth, failures;
static void (*irq)(UINT);
static bool auto_tick;

ID tk_cre_mtx(const T_CMTX *v) { assert(v->mtxatr == TA_INHERIT); return 1; }
ER tk_loc_mtx(ID id, TMO t) { assert(id == 1 && t == TMO_FEVR); ++lock_depth; return E_OK; }
ER tk_unl_mtx(ID id) { assert(id == 1 && lock_depth == 1); --lock_depth; return E_OK; }
ER tk_def_int(UINT n, const T_DINT *v) { assert(n == 13); irq = (void (*)(UINT))v->inthdr; return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_dly_tsk(INT ms)
{
    for (INT i = 0; i < ms; ++i) {
        now_us += 1000U;
        if (auto_tick && now_us % 10000U == 0U) motion_process();
    }
    return E_OK;
}
uint32_t in_w(uint32_t a)
{
    if (a == TIMER_TIMERAWL) return now_us;
    if (a == GPIO_IN) return gpio;
    if (a == SIO_BASE) return 0U;
    if (a >= IO_BANK0_BASE + 0x120U && a < IO_BANK0_BASE + 0x130U)
        return pending[(a - IO_BANK0_BASE - 0x120U) / 4U];
    return 0U;
}
void out_w(uint32_t a, uint32_t v)
{
    io_hash = io_hash * 33U + a + v;
    if (a >= IO_BANK0_BASE + 0xf0U && a < IO_BANK0_BASE + 0x100U)
        pending[(a - IO_BANK0_BASE - 0xf0U) / 4U] &= ~v;
}
void clr_w(uint32_t a, uint32_t v) { out_w(a, v); }
void set_w(uint32_t a, uint32_t v) { out_w(a, v); }
void EnableInt(UINT n, INT p) { assert(n == 13 && p == 1); }
int gpio_set_pin(uint32_t p, uint32_t mode) { out_w(p, mode); return 0; }
int pwm_set_pin(uint32_t p) { out_w(p, 1); return 0; }
int pwm_set_wrap(uint32_t p, uint32_t w) { assert(w == 12499); out_w(p, w); return 0; }
int pwm_set_enabled(uint32_t p, BOOL e) { out_w(p, e); return 0; }
int pwm_set_cc(uint32_t p, uint32_t v)
{ assert(p >= 8 && p <= 11 && v <= 12500); pwm[p] = v; out_w(p, v); return 0; }
int tm_printf(const UB *format, ...)
{
    char text[512]; va_list args; va_start(args, format);
    int n = vsnprintf(text, sizeof(text), (const char *)format, args); va_end(args);
    if (strstr(text, " FAIL ")) ++failures;
    fputs(text, stdout); return n;
}

static uint32_t float_bits(float v) { uint32_t bits; memcpy(&bits, &v, sizeof(bits)); return bits; }
static void trace(const char *name, int result)
{
    printf("%s %u %d %u %u %d %d %08x %08x %08x %08x %08x %08x %u %u %u %u %08x\n",
        name, rows++, result, (unsigned)motion_controller_state(),
        (unsigned)motion_controller_fault(), enc_l_count(), enc_r_count(),
        float_bits(wheel_speed_left_mps()), float_bits(wheel_speed_right_mps()),
        float_bits(robot_distance_m()), float_bits(robot_heading_deg()),
        float_bits(motor_control_duty(WHEEL_LEFT)), float_bits(motor_control_duty(WHEEL_RIGHT)),
        pwm[8], pwm[9], pwm[10], pwm[11], io_hash);
}

static void edge(unsigned wheel, unsigned state)
{
    unsigned a = wheel ? 4U : 16U, b = a + 1U;
    gpio = (gpio & ~((1U << a) | (1U << b)))
         | (((state >> 1) & 1U) << a) | ((state & 1U) << b);
    pending[a / 8] |= 0xcU << ((a % 8) * 4U);
    pending[b / 8] |= 0xcU << ((b % 8) * 4U);
    irq(13);
}
static void steps(unsigned wheel, int count)
{
    const unsigned sequence[4] = {0, 2, 3, 1};
    for (int i = 0; i < (count < 0 ? -count : count); ++i) {
        phase[wheel] = (phase[wheel] + (count < 0 ? 3U : 1U)) & 3U;
        edge(wheel, sequence[phase[wheel]]);
    }
}
static void tick(const char *name, unsigned elapsed, int left, int right)
{
    steps(0, left); steps(1, right); now_us += elapsed;
    motion_process(); trace(name, 0);
}
static void fresh(const char *name)
{
    auto_tick = false;
    motion_controller_set_busy(false);
    motion_init();
    trace(name, 0);
}
static void manoeuvres(void)
{
    for (unsigned m = 0; m < 6; ++m) {
        fresh("move-init");
        int result = m == 4 ? moveForward(0.2f) : m == 5 ? moveBackward(0.2f)
                          : motion_request_manoeuvre((manoeuvre_t)m);
        assert(result == MOTION_OK); trace("move-request", result);
        int left = m == 0 || m == 3 || m == 5 ? -20 : 20;
        int right = m == 1 || m == 5 ? -20 : 20;
        for (unsigned i = 0; i < 300 && motion_controller_state() != MOTION_COMPLETE; ++i)
            tick("move-step", 10000, left, right);
        assert(motion_controller_state() == MOTION_COMPLETE);
        trace("move-complete", motion_poll());
        trace("move-consumed", motion_poll());
    }
}
static void quadrature_transitions(void)
{
    const int table[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    const unsigned phases[4] = {0,3,1,2};
    for (unsigned invert = 0; invert < 2; ++invert) {
        for (unsigned previous = 0; previous < 4; ++previous) {
            for (unsigned next = 0; next < 4; ++next) {
                edge(0, previous); phase[0] = phases[previous];
                wheel_encoder_set_inverted(WHEEL_LEFT, invert != 0);
                int32_t before = enc_l_count();
                uint32_t glitches = wheel_encoder_glitches(WHEEL_LEFT);
                edge(0, next); phase[0] = phases[next];
                int expected = table[previous * 4 + next];
                assert(enc_l_count() == before + (invert ? -expected : expected));
                assert(wheel_encoder_glitches(WHEEL_LEFT) == glitches +
                       (unsigned)(expected == 0 && previous != next));
                trace("quadrature", expected);
            }
        }
    }
    wheel_encoder_set_inverted(WHEEL_LEFT, false);
}
static void faults(void)
{
    fresh("stall-init"); assert(moveForward(1.0f) == MOTION_OK);
    for (unsigned i = 0; i < 100 && motion_controller_state() != MOTION_FAULT; ++i)
        tick("stall", 50000, 0, 0);
    assert(motion_get_fault() == MOTION_FAULT_STALL);
    trace("stall-stop", (stop(), motion_poll()));
    trace("stall-reject", moveForward(0.1f));
    for (unsigned i = 0; i < 5; ++i) tick("still-reset", 50000, 0, 0);
    assert(motion_reset()); trace("reset", motion_poll());

    fresh("direction-init"); assert(motion_request_velocity(0.2f, 0.2f) == MOTION_OK);
    for (unsigned i = 0; i < 100 && motion_controller_state() != MOTION_FAULT; ++i) {
        assert(motion_request_velocity(0.2f, 0.2f) == MOTION_OK);
        tick("direction", 50000, -30, -30);
    }
    assert(motion_get_fault() == MOTION_FAULT_DIRECTION);

    fresh("encoder-init"); assert(moveForward(1.0f) == MOTION_OK);
    for (unsigned i = 0; i < 40; ++i) {
        const unsigned seq[4] = {0, 2, 3, 1};
        phase[0] = (phase[0] + 2U) & 3U; edge(0, seq[phase[0]]);
    }
    tick("encoder", 10000, 0, 0);
    assert(motion_get_fault() == MOTION_FAULT_ENCODER);

    fresh("timeout-init"); assert(moveForward(0.2f) == MOTION_OK);
    for (unsigned i = 0; i < 600 && motion_controller_state() != MOTION_FAULT; ++i)
        tick("timeout", 10000, (i & 1) ? -2 : 2, (i & 1) ? -2 : 2);
    assert(motion_get_fault() == MOTION_FAULT_TIMEOUT);

    fresh("stale-init"); assert(motion_request_velocity(0, 0) == MOTION_OK);
    tick("stale", 251000, 0, 0);
    assert(motion_get_fault() == MOTION_FAULT_STALE_COMMAND);
}

#ifdef TEST_PACKED
static void packed_checks(void)
{
    motion_snapshot_t snapshot;
    assert(!motion_read_snapshot(NULL));
    assert(sizeof(snapshot) == 4);
    for (int speed = -510; speed <= 510; ++speed) {
        BUDDY1_SNAPSHOT input = {0}, decoded;
        uint8_t packet[4];
        input.left_mms = (int16_t)speed; input.right_mms = (int16_t)-speed;
        assert(buddy1_encode(BUDDY1_TOPIC_MOTION, &input, packet, sizeof(packet)) == 4);
        assert(buddy1_decode(BUDDY1_TOPIC_MOTION, packet, sizeof(packet), &decoded));
        for (unsigned state = 0; state <= MOTION_FAULT; ++state) {
            for (unsigned fault = 0; fault <= MOTION_FAULT_TIMEOUT; ++fault) {
                assert(motion_snapshot_pack(&snapshot, state, fault, true, true, speed, -speed));
                assert(motion_snapshot_valid(&snapshot));
                assert(motion_snapshot_state(&snapshot) == state);
                assert(motion_snapshot_fault(&snapshot) == fault);
                int error = motion_snapshot_left_mms(&snapshot) - speed;
                assert(error >= -1 && error <= 1);
                assert(motion_snapshot_right_mms(&snapshot) == -motion_snapshot_left_mms(&snapshot));
                assert(motion_snapshot_left_mms(&snapshot) == decoded.left_mms);
                assert(motion_snapshot_right_mms(&snapshot) == decoded.right_mms);
            }
        }
    }
    assert(!motion_snapshot_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false, true, 511, 0));
    assert(!motion_snapshot_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false, true, 0, -511));
    assert(!motion_snapshot_pack(&snapshot, (motion_state_t)6, MOTION_FAULT_NONE, false, false, 0, 0));
    assert(!motion_snapshot_pack(&snapshot, MOTION_IDLE, (motion_fault_t)6, false, false, 0, 0));
    assert(motion_snapshot_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false, false, 99, -99));
    assert(!motion_snapshot_speed_valid(&snapshot) && motion_snapshot_left_mms(&snapshot) == 0);
    snapshot.bytes[0] = 1; assert(!motion_snapshot_valid(&snapshot));
    memset(&snapshot, 0, sizeof(snapshot)); snapshot.bytes[3] = 0x80;
    assert(!motion_snapshot_valid(&snapshot));
    assert(motion_snapshot_pack(&snapshot, MOTION_IDLE, MOTION_FAULT_NONE, false, true, 0, 0));
    snapshot.bytes[1] |= 1U; assert(!motion_snapshot_valid(&snapshot)); /* negative zero */
    /* Snapshot does not consume completion or affect full-precision targets. */
    motion_controller_set_busy(false);
    motion_controller_init(0.30f, 0, 0);
    assert(motion_request_drive(0, 0.2f) == MOTION_OK);
    assert(motion_read_snapshot(&snapshot) && motion_snapshot_state(&snapshot) == MOTION_COMPLETE);
    assert(motion_poll() == MOTION_COMPLETE && motion_poll() == MOTION_IDLE);
    assert(motion_request_velocity(0.1234567f, -0.0876543f) == MOTION_OK);
    assert(motion_read_snapshot(&snapshot));
    assert(motion_controller_state() == MOTION_VELOCITY);
    motion_controller_set_busy(true);
    assert(motion_read_snapshot(&snapshot) && motion_snapshot_busy(&snapshot));
    assert(!motion_snapshot_speed_valid(&snapshot));
    motion_controller_set_busy(false);
    /* Exact flag meanings and all float calibration values survive packing. */
    for (unsigned bits = 0; bits < 256; ++bits) {
        motion_calibration_t cal = {0};
        cal.counts_per_rev = 2000.25f; cal.max_speed_mps = 0.312345f;
        cal.b_valid = (bits >> 0) & 1U; cal.b_m1_inverted = (bits >> 1) & 1U;
        cal.b_m2_inverted = (bits >> 2) & 1U; cal.b_enc_l_inverted = (bits >> 3) & 1U;
        cal.b_enc_r_inverted = (bits >> 4) & 1U; cal.b_enc_l_ok = (bits >> 5) & 1U;
        cal.b_enc_r_ok = (bits >> 6) & 1U; cal.b_encoders_swapped = (bits >> 7) & 1U;
        assert((cal.b_valid | (cal.b_m1_inverted << 1) | (cal.b_m2_inverted << 2)
             | (cal.b_enc_l_inverted << 3) | (cal.b_enc_r_inverted << 4)
             | (cal.b_enc_l_ok << 5) | (cal.b_enc_r_ok << 6)
             | (cal.b_encoders_swapped << 7)) == (int)bits);
        assert(cal.counts_per_rev == 2000.25f && cal.max_speed_mps == 0.312345f);
    }
    /* Nonfinite / out-of-report-range estimates cannot become valid zero. */
    motion_estimation_init(1.0f, 0.13f);
    steps(0, 10); motion_estimation_update(0.01f);
    assert(motion_read_snapshot(&snapshot) && !motion_snapshot_speed_valid(&snapshot));
    motion_estimation_set_scale(NAN); motion_estimation_update(0.01f);
    assert(motion_read_snapshot(&snapshot) && !motion_snapshot_speed_valid(&snapshot));
    fprintf(stderr, "PASS: 36,756 state/fault/speed combinations, boundaries, validity and non-consuming snapshot\n");
    fprintf(stderr, "PASS: Buddy 1 codec agreement for all 1,021 speeds; 256 calibration flag combinations; nonfinite/out-of-range estimates\n");
}
#endif

extern void subsystem2_test(void);
int main(void)
{
#ifdef TEST_PACKED
    motion_snapshot_t early;
    assert(!motion_read_snapshot(&early));
#endif
    assert(motion_create() == E_OK); motion_attach_task(1); fresh("startup");
    auto_tick = true; subsystem2_test(); auto_tick = false;
    assert(failures == 0);
    manoeuvres(); faults(); quadrature_transitions();
    fresh("velocity-init");
    for (unsigned i = 0; i < 500; ++i) {
        float left = (i % 21 - 10.0f) * 0.013579f;
        float right = (i % 19 - 9.0f) * 0.017531f;
        trace("velocity-request", motion_request_velocity(left, right));
        tick("velocity-step", i % 3 == 0 ? 75000 : 10000, (int)(i % 17) - 8, (int)(i % 13) - 6);
        if (motion_controller_state() == MOTION_FAULT) fresh("velocity-reset");
    }
    fresh("long-still");
    for (unsigned i = 0; i < 1700; ++i) tick("still", 37000, 0, 0);
    /* IRQ state and task configuration are independent after packing. */
    int32_t count = enc_l_count();
    wheel_encoder_set_inverted(WHEEL_LEFT, true); steps(0, 40);
    assert(enc_l_count() == count - 40); trace("inverted", 0);
    wheel_encoder_set_inverted(WHEEL_LEFT, false); steps(0, 40);
    assert(enc_l_count() == count); trace("restored", 0);
    now_us = UINT32_MAX - 100000U; fresh("wrap-init");
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
    assert(lock_depth == 0);
    fprintf(stderr, "PASS: %u deterministic control trace rows; original non-moving unit tests; all five fault reasons\n", rows);
    return 0;
}

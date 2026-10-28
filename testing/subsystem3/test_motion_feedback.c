#include "motion.h"
#include "motion_controller.h"
#include "motion_kernel_stub.h"
#include "motor_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t clock_us;
static int32_t count[2];
static float duty[2];
static bool locked;
static bool debug_lock_busy;
static unsigned int wakes;
static bool abort_on_delay;
ID tk_cre_mtx(const T_CMTX *c)
{
    (void)c;
    return 1;
}
ER tk_loc_mtx(ID id, int timeout)
{
    (void)id;
    if(debug_lock_busy){assert(timeout==TMO_POL);return -1;}
    assert(!locked);
    locked = true;
    return E_OK;
}
ER tk_unl_mtx(ID id)
{
    (void)id;
    assert(locked);
    locked = false;
    return E_OK;
}
ER tk_wup_tsk(ID id)
{
    (void)id;
    wakes++;
    return E_OK;
}
ER tk_dly_tsk(uint32_t ms)
{
    clock_us += ms * 1000U;
    if (abort_on_delay)
    {
        abort_on_delay = false;
        stop();
    }
    return E_OK;
}
ER tk_can_wup(ID id)
{
    (void)id;
    return E_OK;
}
static uint32_t watchdog_tick, reset_bits = RESETS_RESET_TIMER;
static unsigned int clock_starts;
uint32_t in_w(uint32_t address)
{
    if (address == TIMER_TIMERAWL)
    {
        return clock_us;
    }
    if (address == RESETS_RESET)
    {
        return reset_bits;
    }
    if (address == RESETS_RESET_DONE)
    {
        return RESETS_RESET_TIMER;
    }
    return watchdog_tick;
}
void out_w(uint32_t address, uint32_t value)
{
    (void)address;
    watchdog_tick = value | (1U << 10);
    clock_starts++;
}
void clr_w(uint32_t address, uint32_t value)
{
    assert(address == RESETS_RESET);
    reset_bits &= ~value;
}
static uint16_t queue[MOTION_REPORT_QUEUE_LEN];
static unsigned int queued;
ID tk_cre_mbf(const T_CMBF *config)
{
    assert(config->maxmsz == 2);
    queued = 0U;
    return 2;
}
ER tk_snd_mbf(ID id, const void *data, INT size, TMO timeout)
{
    (void)id;
    (void)timeout;
    assert(locked && size == 2);
    if (queued == MOTION_REPORT_QUEUE_LEN)
    {
        return -1;
    }
    memcpy(&queue[queued++], data, 2);
    return E_OK;
}
INT tk_rcv_mbf(ID id, void *data, TMO timeout)
{
    (void)id;
    (void)timeout;
    assert(!locked);
    if (queued == 0U)
    {
        return -1;
    }
    memcpy(data, &queue[0], 2);
    for (unsigned int i = 1U; i < queued; i++)
    {
        queue[i - 1U] = queue[i];
    }
    queued--;
    return 2;
}
static bool encoder_running = true;
void wheel_encoder_sample(void) {}
bool wheel_encoder_is_running(void) { return encoder_running; }
int32_t wheel_encoder_init(bool left, bool right)
{
    (void)left;
    (void)right;
    count[0] = count[1] = 0;
    return 0;
}
int32_t wheel_encoder_count(wheel_id_t wheel) { return count[wheel]; }
uint32_t wheel_encoder_bits(wheel_id_t w)
{
    return (uint32_t)count[w] & WHEEL_COUNT_MASK;
}
int32_t wheel_encoder_delta(uint32_t now_bits, uint32_t then_bits)
{
    uint32_t delta = (now_bits - then_bits) & WHEEL_COUNT_MASK;
    return (delta & (1UL << (WHEEL_COUNT_BITS - 1U)))
               ? (int32_t)delta - (int32_t)(WHEEL_COUNT_MASK + 1U)
               : (int32_t)delta;
}
uint32_t wheel_encoder_glitches(wheel_id_t wheel)
{
    (void)wheel;
    return 0U;
}
void wheel_encoder_set_inverted(wheel_id_t wheel, bool inverted)
{
    (void)wheel;
    (void)inverted;
}
void motor_control_init(bool left, bool right)
{
    (void)left;
    (void)right;
    duty[0] = duty[1] = 0.0f;
}
void motor_control_set(wheel_id_t wheel, float value) { duty[wheel] = value; }
float motor_control_duty(wheel_id_t wheel) { return duty[wheel]; }
void motor_control_brake_all(void) { duty[0] = duty[1] = 0.0f; }
static void step(void)
{
    clock_us += 10000U;
    motion_process();
}
static void test_reports_and_wrap(void)
{
    motion_feedback_t before, after;
    motion_report_t report;
    uint8_t latest;
    unsigned int original_clock_starts = clock_starts;
    assert(motion_create() == E_OK);
    motion_init();
    assert(clock_starts ==
           original_clock_starts); /* never reset running clock */
    for (unsigned int i = 0U; i < 12U; i++)
    {
        step();
    }
    /* Command IDs wrap 255 -> 1; every result is drained before ID reuse. */
    uint8_t expected_id = motion_command_id();
    for (unsigned int i = 1U; i <= 260U; i++)
    {
        assert(motion_request_drive(0.0f, 0.1f) == MOTION_OK);
        expected_id = expected_id == 255U ? 1U : (uint8_t)(expected_id + 1U);
        assert(motion_command_id() == expected_id);
        assert(motion_get_report(&report) && report.command_id == motion_command_id());
        assert(report.outcome == MOTION_OUTCOME_DONE);
        assert(!motion_get_report(&report));
    }
    /* Fill queue with zero-distance commands (no motor motion), then overflow.
     */
    for (unsigned int i = 0U; i <= MOTION_REPORT_QUEUE_LEN; i++)
    {
        assert(motion_request_drive(0.0f, 0.1f) == MOTION_OK);
        stop();
        step();
    }
    assert(motion_get_feedback(&after) && after.report_loss_count == 1U);
    assert(motion_report_peak() == MOTION_REPORT_QUEUE_LEN);
    /* A newer command must survive consumption of older DONE reports. */
    assert(motion_request_drive(1.0f, 0.1f) == MOTION_OK);
    latest = motion_command_id();
    for (unsigned int i = 0U; i < MOTION_REPORT_QUEUE_LEN; i++)
    {
        assert(motion_get_report(&report) && report.command_id != latest);
        assert(motion_controller_state() == MOTION_DRIVE);
    }
    assert(!motion_get_report(&report));
    stop();
    assert(motion_get_report(&report) && report.b_lost &&
           report.command_id == latest &&
           report.outcome == MOTION_OUTCOME_CANCELLED);
    assert(motion_get_feedback(&after) && after.report_loss_count == 1U);
    step();

    count[0] = count[1] = (1 << 19) - 2;
    step();
    assert(motion_get_feedback(&before));
    count[0] += 4;
    count[1] += 4;
    step();
    assert(motion_get_feedback(&after));
    assert(after.centre_m > before.centre_m &&
           after.centre_m - before.centre_m < 0.001);
    count[0] -= 4;
    count[1] -= 4;
    step();
    assert(motion_get_feedback(&after));
    assert(after.centre_m == before.centre_m); /* reverse wrap */
    encoder_running = false;
    assert(motion_get_feedback(&after) && !after.valid);
    encoder_running = true;
    /* Calibration completion (including abort) invalidates old epoch. */
    motion_init();
    step();
    assert(motion_get_feedback(&before));
    assert(motion_request_calibration() == MOTION_OK);
    abort_on_delay = true;
    motion_process();
    assert(motion_get_feedback(&after) && !after.valid &&
           after.epoch == before.epoch + 1U);
    step();
    assert(motion_get_feedback(&after) && after.valid && after.centre_m == 0.0);
}
int main(void)
{
    motion_debug_snapshot_t debug;
    assert(!motion_try_debug_snapshot(NULL));
    motion_feedback_t first, second;
    unsigned int i;
    assert(!motion_get_feedback(&first));
    assert(motion_create() == E_OK);
    motion_attach_task(1);
    motion_init();
    assert(motion_get_feedback(&first) && !first.valid);
    for (i = 0U; i < 12U; i++)
    {
        clock_us += 10000U;
        motion_process();
    }
    assert(clock_starts == 1U && reset_bits == 0U);
    assert(motion_get_feedback(&first) && first.valid && first.braked &&
           first.epoch == 1U);
    count[0] = 100;
    count[1] = 100;
    clock_us += 5000U;
    assert(motion_get_feedback(&second));
    {
        const float before_l=duty[0],before_r=duty[1];
        for(unsigned i=0;i<100;++i)assert(motion_try_debug_snapshot(&debug));
        assert(debug.timestamp_us==second.timestamp_us && debug.valid==second.valid);
        assert(duty[0]==before_l && duty[1]==before_r && !locked);
        debug_lock_busy=true;assert(!motion_try_debug_snapshot(&debug));debug_lock_busy=false;
    }
    assert(second.timestamp_us == first.timestamp_us &&
           second.centre_m == first.centre_m);
    clock_us += 5000U;
    motion_process();
    assert(motion_get_feedback(&second));
    assert(second.timestamp_us == clock_us && second.centre_m > first.centre_m);
    motion_navigation_heartbeat(true);
    assert(motion_request_calibration() == MOTION_ERR_BUSY);
    assert(motion_request_manoeuvre(MANOEUVRE_LEFT) == MOTION_OK);
    clock_us += 10000U;
    motion_process();
    assert(motion_get_feedback(&first) && first.state == MOTION_TURN);
    {
        const float before_l=duty[0],before_r=duty[1];
        for(unsigned j=0;j<100;++j)assert(motion_try_debug_snapshot(&debug));
        assert(debug.state==MOTION_TURN && debug.valid && debug.timestamp_us==first.timestamp_us);
        assert(debug.setpoint_mps[0]<=0.0f && debug.setpoint_mps[1]>=0.0f);
        assert(duty[0]==before_l && duty[1]==before_r && !locked);
        assert(motion_get_feedback(&second) && second.timestamp_us==first.timestamp_us && second.state==first.state);
    }
    clock_us += 40000U;
    motion_process();
    assert(motion_get_feedback(&first));
    assert(first.state == MOTION_FAULT &&
           first.fault == MOTION_FAULT_STALE_COMMAND);
    assert(duty[0] == 0.0f && duty[1] == 0.0f);
    motion_navigation_heartbeat(false);
    for (i = 0; i < 30U; i++)
    {
        clock_us += 10000U;
        motion_process();
    }
    assert(motion_reset());
    stop();
    assert(wakes > 0U);
    motion_init();
    assert(motion_get_feedback(&second) && !second.valid && second.epoch == 2U);
    clock_us += 10000U;
    motion_process();
    assert(motion_get_feedback(&second) && second.valid);
    assert(motion_request_calibration() == MOTION_OK);
    assert(motion_get_feedback(&second) && !second.valid);
    assert(!locked);
    test_reports_and_wrap();
    puts("PASS: production motion feedback timestamps, epochs, calibration "
         "exclusion and manoeuvre watchdog");
    return 0;
}

#include <assert.h>
#include <stdio.h>
#include "motion.h"
#include "motion_controller.h"
#include "motor_control.h"
#include "motion_kernel_stub.h"
static uint32_t clock_us;
static int32_t count[2];
static float duty[2];
static bool locked;
static unsigned int wakes;
ID tk_cre_mtx(const T_CMTX *c)
{
    (void)c;
    return 1;
}
ER tk_loc_mtx(ID id, int timeout)
{
    (void)id;
    (void)timeout;
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
    return E_OK;
}
ER tk_can_wup(ID id)
{
    (void)id;
    return E_OK;
}
uint32_t in_w(uint32_t address)
{
    (void)address;
    return clock_us;
}
int32_t wheel_encoder_init(bool left, bool right)
{
    (void)left;
    (void)right;
    count[0] = count[1] = 0;
    return 0;
}
int32_t wheel_encoder_count(wheel_id_t wheel)
{
    return count[wheel];
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
void motor_control_set(wheel_id_t wheel, float value)
{
    duty[wheel] = value;
}
float motor_control_duty(wheel_id_t wheel)
{
    return duty[wheel];
}
void motor_control_brake_all(void)
{
    duty[0] = duty[1] = 0.0f;
}
int main(void)
{
    motion_feedback_t first, second;
    unsigned int i;
    assert(!motion_get_feedback(&first));
    assert(motion_create() == E_OK);
    motion_attach_task(1);
    motion_init();
    assert(motion_get_feedback(&first) && !first.valid);
    clock_us += 10000U;
    motion_process();
    assert(motion_get_feedback(&first) && first.valid && first.braked &&
           first.epoch == 1U);
    count[0] = 100;
    count[1] = 100;
    clock_us += 5000U;
    assert(motion_get_feedback(&second));
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
    clock_us += 40000U;
    motion_process();
    assert(motion_get_feedback(&first));
    assert(first.state == MOTION_FAULT &&
           first.fault == MOTION_FAULT_STALE_COMMAND && first.braked);
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
    puts("PASS: production motion feedback timestamps, epochs, calibration "
         "exclusion and manoeuvre watchdog");
    return 0;
}

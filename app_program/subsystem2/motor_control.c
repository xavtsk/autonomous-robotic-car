/*
 * File: motor_control.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Motor actuation through the Robo Pico motor driver (PWM).
 */

#include "motor_control.h"
#include "../app.h"

/* Pins (Week 6 Design Review, section 2): Robo Pico motor channels */
#define M1A                     (8U)    /* left DC motor, motor channel 1  */
#define M1B                     (9U)
#define M2A                     (10U)   /* right DC motor, motor channel 2 */
#define M2B                     (11U)

/* PWM: clk_sys / (PWM_WRAP + 1) = 125 MHz / 12500 = 10 kHz
 * (Robo Pico motor driver maximum is 20 kHz) */
#define PWM_FREQ_HZ             (10000U)
#define PWM_WRAP                ((SYSCLK * 1000000U / PWM_FREQ_HZ) - 1U)
#define PWM_DIV_ONE             (1U << 4)       /* 8.4 fixed point: 1.0 */
#define PWM_SLICE_STRIDE        (0x14U)

typedef struct
{
    uint32_t pin_a;
    uint32_t pin_b;
    bool     b_inverted;
    float    duty;
} motor_t;

static motor_t g_motor[WHEEL_COUNT] =
{
    { M1A, M1B, false, 0.0f },
    { M2A, M2B, false, 0.0f }
};

static inline uint32_t pwm_slice(uint32_t pin)
{
    return (pin >> 1) & 0x7U;
}

static void motor_pin_init(uint32_t pin)
{
    (void)pwm_set_pin(pin);
    (void)pwm_set_wrap(pin, PWM_WRAP);
    out_w(PWM_BASE + (pwm_slice(pin) * PWM_SLICE_STRIDE) + PWM_CHx_DIV,
          PWM_DIV_ONE);
    (void)pwm_set_cc(pin, 0U);
    (void)pwm_set_enabled(pin, TRUE);
}

void motor_control_init(bool b_m1_inverted, bool b_m2_inverted)
{
    g_motor[WHEEL_LEFT].b_inverted  = b_m1_inverted;
    g_motor[WHEEL_RIGHT].b_inverted = b_m2_inverted;

    motor_pin_init(M1A);
    motor_pin_init(M1B);
    motor_pin_init(M2A);
    motor_pin_init(M2B);
    motor_control_brake_all();
}

void motor_control_set(wheel_id_t wheel, float duty)
{
    motor_t  *p_motor = &g_motor[wheel];
    float     out;
    float     mag;
    uint32_t  level;

    if (duty > 1.0f)
    {
        duty = 1.0f;
    }
    if (duty < -1.0f)
    {
        duty = -1.0f;
    }
    p_motor->duty = duty;

    out = p_motor->b_inverted ? -duty : duty;
    mag = (out >= 0.0f) ? out : -out;
    level = (uint32_t)(mag * (float)(PWM_WRAP + 1U));
    if (level > PWM_WRAP + 1U)
    {
        level = PWM_WRAP + 1U;
    }

    if (out > 0.0f)                 /* forward: A = PWM, B = low */
    {
        (void)pwm_set_cc(p_motor->pin_b, 0U);
        (void)pwm_set_cc(p_motor->pin_a, level);
    }
    else if (out < 0.0f)            /* backward: A = low, B = PWM */
    {
        (void)pwm_set_cc(p_motor->pin_a, 0U);
        (void)pwm_set_cc(p_motor->pin_b, level);
    }
    else                            /* brake: both low */
    {
        (void)pwm_set_cc(p_motor->pin_a, 0U);
        (void)pwm_set_cc(p_motor->pin_b, 0U);
    }
}

void motor_control_brake_all(void)
{
    motor_control_set(WHEEL_LEFT, 0.0f);
    motor_control_set(WHEEL_RIGHT, 0.0f);
}

float motor_control_duty(wheel_id_t wheel)
{
    return g_motor[wheel].duty;
}

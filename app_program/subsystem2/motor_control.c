/** @file motor_control.c
 * @brief Motor actuation through the Robo Pico motor driver (PWM).
 *
 * @par Ownership
 * Buddy 2 - Motion Control
 * The hardware masks fit in unsigned. Convert them before combining
 * register or packet bits.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 */

#include "motor_control.h"
#include "../app_types.h"
#include "../app.h"

/* The Robo Pico motor channels follow section 2 of the course report
 * AAI07_Week_6_Design_Review.pdf. */
#define M1A (8U) /* Channel 1 drives the left wheel. */
#define M1B (9U)
#define M2A (10U) /* Channel 2 drives the right wheel. */
#define M2B (11U)

/* The PWM frequency is 125 MHz divided by PWM_WRAP + 1, yielding 10 kHz. The
 * Robo Pico motor driver supports up to 20 kHz.
 */
#define PWM_FREQ_HZ (10000U)
#define PWM_WRAP ((SYSCLK * 1000000U / PWM_FREQ_HZ) - 1U)
#define PWM_DIV_ONE (1U << 4) /* Encode 1.0 with four fraction bits. */
#define PWM_SLICE_STRIDE (0x14U)

/* Only the motion task writes the packed output fields. */
typedef struct
{
    uint32_t  pin_a : 5; /* GPIO 0 ... 29                   */
    uint32_t  pin_b : 5;
    uint32_t  b_is_inverted : 1;
    float32_t duty; /* Keep full precision for the duty. */
} motor_t;

static motor_t g_motor[WHEEL_COUNT] = {{M1A, M1B, 0U, 0.0f},
                                       {M2A, M2B, 0U, 0.0f}};

static inline uint32_t pwm_slice (uint32_t pin);
static void            motor_pin_init (uint32_t pin);

/** @brief Store both polarity flags, release PWM from reset, initialize the
 * four motor outputs and brake both wheels.
 */
void motor_control_init (bool b_is_left_motor_inverted,
                         bool b_is_right_motor_inverted)
{
    g_motor[WHEEL_LEFT].b_is_inverted  = b_is_left_motor_inverted ? 1U : 0U;
    g_motor[WHEEL_RIGHT].b_is_inverted = b_is_right_motor_inverted ? 1U : 0U;

    /* The kernel releases PWM from reset only when USE_PTMR is set. */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while ((in_w(RESETS_RESET_DONE) & (unsigned)RESETS_RESET_PWM) == 0U)
    {
        /* Wait until the PWM peripheral leaves reset. */
    }

    motor_pin_init(M1A);
    motor_pin_init(M1B);
    motor_pin_init(M2A);
    motor_pin_init(M2B);
    motor_control_brake_all();
}

/** @brief For a valid wheel ID and finite duty, clamp to the range -1 to 1 and
 * retain the commanded value. Apply wheel polarity, clear the opposite
 * output before driving, and brake on zero.
 */
void motor_control_set (wheel_encoder_id_t wheel, float32_t duty)
{
    motor_t * p_motor = &g_motor[wheel];

    if (duty > 1.0f)
    {
        duty = 1.0f;
    }

    if (duty < -1.0f)
    {
        duty = -1.0f;
    }

    p_motor->duty = duty;

    const float32_t out = (p_motor->b_is_inverted != 0U) ? -duty : duty;
    const float32_t mag = (out >= 0.0f) ? out : -out;

    /* The configured period is representable in binary32. A clamped
     * finite duty produces 0..PWM_WRAP+1 ticks before truncation. */
    uint32_t level = (uint32_t)(mag * (float32_t)(PWM_WRAP + 1U));

    if (level > (PWM_WRAP + 1U))
    {
        level = PWM_WRAP + 1U;
    }

    if (out > 0.0f) /* For forward, drive A and clear B. */
    {
        (void)pwm_set_cc(p_motor->pin_b, 0U);
        (void)pwm_set_cc(p_motor->pin_a, level);
    }
    else if (out < 0.0f) /* For reverse, drive B and clear A. */
    {
        (void)pwm_set_cc(p_motor->pin_a, 0U);
        (void)pwm_set_cc(p_motor->pin_b, level);
    }
    else /* Brake by clearing both outputs. */
    {
        (void)pwm_set_cc(p_motor->pin_a, 0U);
        (void)pwm_set_cc(p_motor->pin_b, 0U);
    }
}

/** @brief Command zero duty on the left wheel followed by the right wheel.
 */
void motor_control_brake_all (void)
{
    motor_control_set(WHEEL_LEFT, 0.0f);
    motor_control_set(WHEEL_RIGHT, 0.0f);
}

/** @brief Read the last clamped, pre-inversion duty request for the supplied
 * valid wheel ID.
 */
float32_t motor_control_duty (wheel_encoder_id_t wheel)
{
    return g_motor[wheel].duty;
}

/** @brief Return the RP2040 PWM slice associated with a GPIO pin.
 */
static inline uint32_t pwm_slice (uint32_t pin)
{
    return (pin >> 1) & 0x7U;
}

/** @brief Select PWM for the pin, program its wrap and divider, set zero
 * compare and enable its slice.
 */
static void motor_pin_init (uint32_t pin)
{
    (void)pwm_set_pin(pin);
    (void)pwm_set_wrap(pin, PWM_WRAP);
    out_w((unsigned int)PWM_BASE + (pwm_slice(pin) * PWM_SLICE_STRIDE) +
              (unsigned int)PWM_CHx_DIV,
          PWM_DIV_ONE);
    (void)pwm_set_cc(pin, 0U);
    (void)pwm_set_enabled(pin, TRUE);
}

/* End of file. */


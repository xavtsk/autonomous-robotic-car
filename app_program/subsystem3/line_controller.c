/** @file line_controller.c
 * @brief Compute bounded differential wheel-speed proposals from line error.
 */
#include "line_controller.h"
#include "../app_types.h"

static bool      finite_number (float32_t operand);
static float32_t clamp_speed (float32_t speed, float32_t maximum);

/** @brief Return the fixed base speed, gain and maximum speed defaults.
 * @see line_controller.h for the values and units.
 */
line_controller_config_t line_controller_defaults (void)
{
    return (line_controller_config_t){0.10f, 0.06f, 0.20f};
}

/** @brief Accept only finite fields satisfying the steering speed bounds.
 * @param p_config Configuration to inspect; null returns false.
 * @return Whether the bounds documented in line_controller.h hold.
 */
bool line_controller_config_valid (const line_controller_config_t * p_config)
{
    return (((((((p_config != 0) && (finite_number(p_config->base_mps))) &&
                (finite_number(p_config->kp_mps))) &&
               (finite_number(p_config->maximum_mps))) &&
              (p_config->base_mps > 0.0f)) &&
             (p_config->kp_mps >= 0.0f)) &&
            (p_config->maximum_mps > 0.0f)) &&
           (p_config->base_mps <= p_config->maximum_mps);
}

/** @brief Form a bounded wheel proposal or return the zeroed invalid request.
 * @param p_config Steering configuration with speeds in metres per second.
 * @param p_line Current line observation, including normalized signed error.
 * @return The proposal defined by line_controller.h; inputs remain unchanged.
 */
line_controller_request_t
line_controller_compute (const line_controller_config_t *     p_config,
                         const line_detection_observation_t * p_line)
{
    line_controller_request_t out = {false};

    if (((((((!line_controller_config_valid(p_config)) || (0 == p_line)) ||
            (!p_line->b_is_valid)) ||
           (!p_line->b_is_line_present)) ||
          (p_line->b_has_junction_candidate)) ||
         (p_line->error < -100)) ||
        (p_line->error > 100))
    {
        return out;
    }

    /* The validated error in -100..100 converts exactly before division. */
    const float32_t correction =
        p_config->kp_mps * ((float32_t)p_line->error / 100.0f);
    /* clang-format off */
    out.left_mps   =
        clamp_speed(p_config->base_mps + correction, p_config->maximum_mps);
    out.right_mps  =
        clamp_speed(p_config->base_mps - correction, p_config->maximum_mps);
    out.b_is_valid = true;
    /* clang-format on */
    return out;
}

/** @brief Classify a value with the retained equality/subtraction checks.
 * @param operand Floating value to inspect.
 * @return True for finite values, including signed zero; false for NaN and
 * infinities under the configured floating-point compilation model.
 * @note The exact operations are retained for behavior preservation; their
 * BARR floating-equality review remains open.
 */
static bool finite_number (float32_t operand)
{
    return (operand == operand) && ((operand - operand) == 0.0f);
}

/** @brief Clamp a computed forward wheel speed to its configured interval.
 * @param speed Computed wheel speed in metres per second.
 * @param maximum Positive, finite speed ceiling in metres per second.
 * @return Zero below zero, maximum above maximum, otherwise speed unchanged.
 * @note Positive/negative infinities clamp at the corresponding endpoint;
 * a NaN passes through the comparisons unchanged.
 */
static float32_t clamp_speed (float32_t speed, float32_t maximum)
{
    if (speed < 0.0f)
    {
        return 0.0f;
    }

    return (speed > maximum) ? maximum : speed;
}

/* End of file. */


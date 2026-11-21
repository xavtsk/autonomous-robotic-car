/** @file line_controller.h
 * @brief Compute bounded differential wheel-speed proposals from line error.
 */
#ifndef BUDDY3_LINE_CONTROLLER_H
#define BUDDY3_LINE_CONTROLLER_H
#include "../app_types.h"
#include "line_detection.h"

typedef struct
{
    float32_t base_mps;
    float32_t kp_mps;
    float32_t maximum_mps;
} line_controller_config_t;

typedef struct
{
    bool      b_is_valid;
    float32_t left_mps;
    float32_t right_mps;
} line_controller_request_t;

/** @brief Return the default forward steering configuration.
 * @return Base speed 0.10 m/s, proportional gain 0.06 m/s at unit normalized
 * line error, and maximum wheel speed 0.20 m/s. No shared state is modified.
 */
line_controller_config_t line_controller_defaults (void);
/** @brief Validate the finite steering speeds and proportional gain.
 * @param p_config Configuration to inspect; null is rejected.
 * @return True only for finite fields, positive base/maximum speeds,
 * nonnegative gain, and base speed no greater than maximum speed.
 */
bool line_controller_config_valid (const line_controller_config_t * p_config);
/* Pure proposal only; no motion API calls. No reverse wheel commands. */
/** @brief Calculate a forward-only wheel-speed proposal from a line error.
 * @param p_config Valid steering configuration; invalid or null is rejected.
 * @param p_line Valid, present, non-junction line with error in [-100, 100].
 * Null or observations outside these conditions are rejected.
 * @return An invalid zero-speed request on rejection. Otherwise valid is true
 * and each wheel speed is independently clamped to [0, maximum_mps]. Positive
 * error adds correction to the left wheel and subtracts it from the right.
 * @note Reads inputs without mutation, locking or issuing a motion command.
 */
line_controller_request_t
line_controller_compute (const line_controller_config_t *     p_config,
                         const line_detection_observation_t * p_line);
#endif

/* End of file. */


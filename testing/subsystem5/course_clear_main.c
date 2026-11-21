/** @file course_clear_main.c
 * @brief Run the upstream course scenarios with the real S5 arbitration
 * enabled.
 *
 * Only the ultrasonic service is replaced with a fresh, clear-path reading.
 * Obstacle-present cases use s5_ctrl_main.c. No hardware evidence here.
 */
#include <assert.h>
#include "../../app_program/app_types.h"
#include "course_clear_main.h"
#include "../integration/course_sim_main.h"
#include "motion.h"

/** @brief Write a valid two-metre forward reading timestamped at the
 * simulated course time.
 */
bool s5_api_snapshot (s5_codec_observation_t * p_out)
{
    *p_out = (s5_codec_observation_t){.range_mm         = 2000U,
                                      .range_us         = course_sim_time_us(),
                                      .b_is_range_valid = true};
    return true;
}

/** @brief Return the snapshot status associated with the fixture's
 * clear-path reading.
 */
s5_api_snapshot_result_t s5_api_read_snapshot (s5_codec_observation_t * p_out)
{
    return s5_api_snapshot(p_out) ? S5_SNAPSHOT_OK : S5_SNAPSHOT_UNAVAILABLE;
}

/** @brief Fail the fixture assertion if clear-course travel requests an
 * obstacle scan.
 */
bool s5_api_request_scan (uint16_t scan_id)
{
    (void)scan_id;
    assert(!"Clear course must not request an obstacle scan");
    return false;
}

/** @brief Fail the fixture assertion if clear-course travel requests an
 * obstacle peek.
 */
bool s5_api_request_peek (uint16_t scan_id, unsigned angle)
{
    (void)scan_id;
    (void)angle;
    assert(!"Clear course must not request an obstacle peek");
    return false;
}

/** @brief Discard scan cancellation at the simulated ultrasonic boundary.
 */
void s5_api_cancel_scan (void)
{
}

/** @brief Accept the controller status fields without publishing telemetry.
 */
void s5_api_set_control_status (unsigned state, unsigned fault, unsigned route,
                                unsigned leg, unsigned pass, unsigned swapped)
{
    (void)state;
    (void)fault;
    (void)route;
    (void)leg;
    (void)pass;
    (void)swapped;
}

/** @brief Return a clear obstacle-control status word for this fixture.
 */
uint16_t s5_api_controller_status (void)
{
    return 0U;
}

/** @brief Return the fixed servo angle in degrees used by this fixture.
 */
uint8_t s5_api_servo_angle (void)
{
    return 0U;
}

/** @brief Fail the fixture assertion if clear-course travel requests a
 * detour turn.
 */
motion_result_t motion_request_turn (float32_t degrees, float32_t speed)
{
    (void)degrees;
    (void)speed;
    assert(!"Clear course must not request an obstacle detour turn");
    return MOTION_ERR_BUSY;
}

/* End of file. */


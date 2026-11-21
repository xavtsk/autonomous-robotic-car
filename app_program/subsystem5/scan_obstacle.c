/** @file scan_obstacle.c
 * @brief Track scan angles, sample acceptance and scan completion.
 */
#include "scan_obstacle.h"

/** @brief Reset all scan storage and seed the coarse scan's first angle.
 * @see scan_obstacle.h for pointer ownership and timestamp units.
 */
void scan_obstacle_begin (scan_obstacle_t * p_scan, uint32_t now)
{
    *p_scan            = (scan_obstacle_t){0};
    p_scan->angle      = 30U;
    p_scan->nearest    = 90U;
    p_scan->nearest_mm = S5_RANGE_MAX_MM;
    p_scan->started_us = now;
}

/** @brief Validate side-angle bounds and five-degree alignment.
 * @see scan_obstacle.h for the inclusive accepted ranges.
 */
bool scan_obstacle_peek_angle_valid (unsigned angle)
{
    return ((angle % 5U) == 0U) && (((angle >= 30U) && (angle <= 60U)) ||
                                    ((angle >= 120U) && (angle <= 150U)));
}

/** @brief Reject invalid peek requests before resetting caller state.
 * @see scan_obstacle.h for unchanged-on-failure and single-sample behavior.
 */
bool scan_obstacle_begin_at (scan_obstacle_t * p_scan, uint32_t now,
                             unsigned angle)
{
    if ((0 == p_scan) || (!scan_obstacle_peek_angle_valid(angle)))
    {
        return false;
    }

    scan_obstacle_begin(p_scan, now);
    /* Validation restricts the requested angle to 30..150 degrees. */
    p_scan->angle       = (uint8_t)angle;
    p_scan->b_is_single = 1U;
    return true;
}

/** @brief Apply timeout, retry, packing and scan progression in that order.
 * @details Reject a timed-out scan before using the echo. An unusable echo
 * retries the same angle and a third failure terminates the scan. A usable
 * echo resets the retry counter and can add at most one sample. A peek
 * finishes after that sample. A sweep moves from coarse angles to a fine
 * search around the closest coarse return. Only a shorter range replaces
 * the current candidate. The caller schedules echoes and moves the servo.
 * @see scan_obstacle.h for terminal state, failure effects and work limits.
 */
void scan_obstacle_accept (scan_obstacle_t * p_scan, uint32_t now,
                           bool b_is_valid, unsigned value_mm)
{
    if ((0 == p_scan) || (p_scan->b_is_done))
    {
        return;
    }

    if ((now - p_scan->started_us) >= S5_SCAN_TIMEOUT_US)
    {
        p_scan->b_has_failed = p_scan->b_is_done = true;
        return;
    }

    if (((!b_is_valid) || (value_mm < S5_RANGE_MIN_MM)) ||
        (value_mm > S5_RANGE_MAX_MM))
    {
        if (++p_scan->attempts < 3U)
        {
            return;
        }

        p_scan->b_has_failed = p_scan->b_is_done = true;
        return;
    }

    p_scan->attempts = 0;

    if ((p_scan->count >= S5_SAMPLES_MAX) ||
        (!profile_obstacle_sample_pack(p_scan->angle, value_mm, true,
                                       &p_scan->sample[p_scan->count])))
    {
        p_scan->b_has_failed = p_scan->b_is_done = true;
        return;
    }

    p_scan->count++;

    if (p_scan->b_is_single)
    {
        p_scan->b_is_done = true;
        return;
    }

    if ((!p_scan->b_is_fine) && (value_mm < p_scan->nearest_mm))
    {
        /* The accepted echo is 30..4000 mm and fits uint16_t. */
        p_scan->nearest_mm = (uint16_t)value_mm;
        p_scan->nearest    = p_scan->angle;
    }

    if (!p_scan->b_is_fine)
    {
        if (p_scan->angle < 150U)
        {
            p_scan->angle += 30U;
        }
        else
        {
            /* clang-format off */
            p_scan->b_is_fine = true;
            p_scan->angle     =
                p_scan->nearest > 45U ? p_scan->nearest - 15U : 30U;
            p_scan->fine_end  =
                p_scan->nearest < 135U ? p_scan->nearest + 15U : 150U;
            /* clang-format on */
        }
    }
    else if (p_scan->angle < p_scan->fine_end)
    {
        p_scan->angle += 5U;
    }
    else
    {
        p_scan->b_is_done = true;
    }
}

/* End of file. */


/** @file scan_obstacle.c
 * @brief Track scan angles, sample acceptance and scan completion.
 */
#include "scan_obstacle.h"

#define SWEEP_MIN_DEG 30U
#define SWEEP_MAX_DEG 150U
#define SWEEP_COARSE_STEP_DEG 30U
#define SWEEP_FINE_RADIUS_DEG 15U
#define SWEEP_FINE_STEP_DEG 5U
#define SWEEP_COARSE_COUNT                                                     \
    (((SWEEP_MAX_DEG - SWEEP_MIN_DEG) / SWEEP_COARSE_STEP_DEG) + 1U)
#define SWEEP_FINE_HALF_COUNT (SWEEP_FINE_RADIUS_DEG / SWEEP_FINE_STEP_DEG)

static void start_width_probes (scan_obstacle_t * p_scan, uint32_t now);
static void next_width_probe (scan_obstacle_t * p_scan, uint32_t now);

/** @brief Reset all scan storage and seed the coarse scan's first angle.
 * @see scan_obstacle.h for pointer ownership and timestamp units.
 */
void scan_obstacle_begin (scan_obstacle_t * p_scan, uint32_t now)
{
    *p_scan            = (scan_obstacle_t){0};
    p_scan->angle      = SWEEP_MIN_DEG;
    p_scan->nearest    = 90U;
    p_scan->nearest_mm = S5_RANGE_MAX_MM;
    p_scan->started_us = now;
}

/** @brief Derive the routing sample count from the frozen coarse minimum.
 * @see scan_obstacle.h for the initialized-state and ownership contract.
 */
unsigned scan_obstacle_profile_count (const scan_obstacle_t * p_scan)
{
    if (!p_scan->b_has_profile)
    {
        return 0U;
    }

    const unsigned nearest = p_scan->nearest;
    /* Only the two outermost coarse angles clip a side of the fine pass. */
    const bool b_is_clipped =
        (nearest == SWEEP_MIN_DEG) || (nearest == SWEEP_MAX_DEG);
    const unsigned fine_count =
        1U + ((b_is_clipped ? 1U : 2U) * SWEEP_FINE_HALF_COUNT);
    return SWEEP_COARSE_COUNT + fine_count;
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
 * the current candidate. Once those angles are complete, an optional probe
 * failure or deadline returns the available routing sweep. The caller
 * schedules echoes and moves the servo.
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
        p_scan->b_has_failed = !p_scan->b_has_profile;
        p_scan->b_is_done    = true;
        return;
    }

    if (((!b_is_valid) || (value_mm < S5_RANGE_MIN_MM)) ||
        (value_mm > S5_RANGE_MAX_MM))
    {
        if (p_scan->b_has_profile)
        {
            p_scan->b_is_done = true;
            return;
        }
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

    if (p_scan->b_has_profile)
    {
        next_width_probe(p_scan, now);
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
        if (p_scan->angle < SWEEP_MAX_DEG)
        {
            p_scan->angle += SWEEP_COARSE_STEP_DEG;
        }
        else
        {
            const unsigned nearest = p_scan->nearest;
            p_scan->b_is_fine      = true;
            p_scan->angle = (nearest > (SWEEP_MIN_DEG + SWEEP_FINE_RADIUS_DEG))
                                ? (nearest - SWEEP_FINE_RADIUS_DEG)
                                : SWEEP_MIN_DEG;
            p_scan->fine_end =
                (nearest < (SWEEP_MAX_DEG - SWEEP_FINE_RADIUS_DEG))
                    ? (nearest + SWEEP_FINE_RADIUS_DEG)
                    : SWEEP_MAX_DEG;
        }
    }
    else if (p_scan->angle < p_scan->fine_end)
    {
        p_scan->angle += SWEEP_FINE_STEP_DEG;
    }
    else
    {
        start_width_probes(p_scan, now);
    }
}

/** @brief Keep the original sweep and probe only a plausible front face.
 * @param[in,out] p_scan Scan after accepting its last fine reading.
 * @param now Current microsecond timestamp on the scan start clock.
 */
static void start_width_probes (scan_obstacle_t * p_scan, uint32_t now)
{
    const unsigned    centre_index = 2U; /* The third coarse angle is 90 deg. */
    unsigned          angle;
    unsigned          centre_mm;
    bool              b_has_echo;
    const s5_config_t cfg        = s5_config_defaults();
    const unsigned    profile_mm = cfg.profile_mm;

    p_scan->b_has_profile = true;
    if ((((profile_obstacle_sample_unpack(&p_scan->sample[centre_index], &angle,
                                          &centre_mm, &b_has_echo)) &&
          b_has_echo) &&
         (angle == 90U)) &&
        (centre_mm < profile_mm))
    {
        next_width_probe(p_scan, now);
    }
    else
    {
        p_scan->b_is_done = true;
    }
}

/** @brief Fill the existing gap between the coarse 30- and 60-degree side
 * bearings for face measurement. Skip positions covered by the fine scan.
 * Leave 1 s for final centring and publication before the detour deadline.
 * @param[in,out] p_scan Owned scan with the original routing sweep complete.
 * @param now Current microsecond timestamp on the scan start clock.
 */
static void next_width_probe (scan_obstacle_t * p_scan, uint32_t now)
{
    static const uint8_t angles[]   = {55U, 45U, 125U, 135U};
    const uint32_t       reserve_us = 1000000U;
    const unsigned       nearest    = p_scan->nearest;

    /* Both narrow fields are nonnegative and widen exactly to unsigned. */
    while (
        (((unsigned)p_scan->width_probe < (sizeof angles / sizeof angles[0])) &&
         ((unsigned)p_scan->count < S5_SAMPLES_MAX)) &&
        ((now - p_scan->started_us) < (S5_SCAN_TIMEOUT_US - reserve_us)))
    {
        const unsigned angle = angles[p_scan->width_probe];
        ++p_scan->width_probe;
        if (((angle + SWEEP_FINE_RADIUS_DEG) < nearest) ||
            (angle > (nearest + SWEEP_FINE_RADIUS_DEG)))
        {
            p_scan->angle = angle;
            return;
        }
    }

    p_scan->b_is_done = true;
}

/* End of file. */


/** @file hump_detection.c
 * @brief Track hump progress and height for the configured wheel geometry.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */
#include "hump_detection.h"
#include "../app_types.h"
#include <math.h>

static inline void record_level_sample (hump_detection_t * p_hump);

static hump_detection_reason_t right_discard (hump_detection_t *      p_hump,
                                              hump_detection_reason_t reason);
static hump_detection_reason_t
right_step (hump_detection_t * p_hump, float32_t pitch, float32_t roll,
            float32_t track_m, float32_t step_distance_m,
            hump_detection_sample_quality_t quality, uint32_t now);

/**
 * @brief Apply the complete reset contract in hump_detection.h.
 * @param[out] p_hump Non-NULL detector; the caller owns exclusive access.
 */
void hump_detection_reset (hump_detection_t * p_hump)
{
    *p_hump = (hump_detection_t){0};
}

/**
 * @brief Update the pitch-integrated crossing described in hump_detection.h.
 * @param[in,out] p_hump Non-NULL initialized, exclusively accessed detector.
 * @param pitch Nose-up pitch in degrees.
 * @param step_distance_m Signed distance increment in metres; invalid values
 * discard.
 * @param b_is_valid Caller quality gate.
 * @param now Wrapping microsecond timestamp used for the crossing timeout.
 */
void hump_detection_update (hump_detection_t * p_hump, float32_t pitch,
                            float32_t step_distance_m, bool b_is_valid,
                            uint32_t now)
{
    p_hump->b_has_completed_hump = false;

    if (((((((!b_is_valid) || (!isfinite(step_distance_m))) ||
            (!isfinite(pitch))) ||
           (step_distance_m < 0)) ||
          (step_distance_m > 0.05f)) ||
         (fabsf(pitch) > 40)) ||
        ((p_hump->b_is_hump_tracking) &&
         ((now - p_hump->started_us) > 10000000U)))
    {
        p_hump->b_is_hump_tracking = false;
        p_hump->height_m           = 0;
        p_hump->peak_m             = 0;
        p_hump->b_has_descended    = false;
        p_hump->level_samples      = 0;
        return;
    }

    if (((!p_hump->b_is_hump_tracking) && (pitch > 4.0f)) &&
        (step_distance_m > 0.0001f))
    {
        p_hump->b_is_hump_tracking = true;
        p_hump->started_us         = now;
        p_hump->height_m           = 0;
        p_hump->peak_m             = 0;
        p_hump->b_has_descended    = false;
        p_hump->level_samples      = 0;
    }

    if (!p_hump->b_is_hump_tracking)
    {
        return;
    }

    p_hump->height_m += step_distance_m * sinf(pitch * 0.01745329252f);

    if (p_hump->height_m > p_hump->peak_m)
    {
        p_hump->peak_m = p_hump->height_m;
    }

    if ((p_hump->peak_m > 0.50f) || (p_hump->height_m < -0.03f))
    {
        p_hump->b_is_hump_tracking = false;
        return;
    }

    if (pitch < -4.0f)
    {
        p_hump->b_has_descended = true;
    }

    p_hump->level_samples = ((p_hump->b_has_descended) && (fabsf(pitch) < 2.0f))
                                ? p_hump->level_samples + 1U
                                : 0U;

    if (p_hump->level_samples >= 5U)
    {
        const bool b_can_complete =
            (p_hump->peak_m >= 0.005f) && (fabsf(p_hump->height_m) < 0.03f);

        if ((b_can_complete) && ((!p_hump->b_is_peak_valid) ||
                                 (p_hump->peak_m > p_hump->highest_m)))
        {
            p_hump->highest_m = p_hump->peak_m;
        }

        if (b_can_complete)
        {
            p_hump->b_is_peak_valid      = true;
            p_hump->b_has_completed_hump = true;
        }

        p_hump->b_is_hump_tracking = false;
    }
}

/**
 * @brief Apply the right-wheel update contract in hump_detection.h.
 * @param[in,out] p_hump Non-NULL initialized, exclusively accessed detector.
 * @param pitch Nose-up pitch in degrees.
 * @param roll Right-side-up roll in degrees.
 * @param track_m Wheel-track width in metres.
 * @param step_distance_m Signed wheel-distance increment in metres.
 * @param quality Caller quality discriminator; no implicit enum validation.
 * @param now Wrapping microsecond timestamp.
 * @note The same step runs with debugging on or off; debug fields are derived
 * from the retained pre-step state and the completed step, without extra I/O.
 */
void hump_detection_update_right (hump_detection_t * p_hump, float32_t pitch,
                                  float32_t roll, float32_t track_m,
                                  float32_t step_distance_m,
                                  hump_detection_sample_quality_t quality,
                                  uint32_t                        now)
{
#if APP_IMU_DEBUG
    const bool      b_was_active = p_hump->b_is_hump_tracking;
    const bool      b_was_armed  = p_hump->b_is_armed;
    const float32_t prior_travel = p_hump->travel_m;
    /* The reason uses its existing wire code in the range 0..18. */
    p_hump->debug.reason   = (uint8_t)right_step(p_hump, pitch, roll, track_m,
                                                 step_distance_m, quality, now);
    p_hump->debug.travel_m = p_hump->b_is_hump_tracking
                                 ? p_hump->travel_m
                                 : (b_was_active ? prior_travel : 0);
    /* The seven debug flags occupy separate bits of the output byte. */
    p_hump->debug.flags =
        (uint8_t)((!b_was_armed && p_hump->b_is_armed ? HUMP_DEBUG_ARMED : 0) |
                  (!b_was_active && p_hump->b_is_hump_tracking
                       ? HUMP_DEBUG_STARTED
                       : 0) |
                  (b_was_active && !p_hump->b_is_hump_tracking &&
                           !p_hump->b_has_completed_hump
                       ? HUMP_DEBUG_DISCARDED
                       : 0) |
                  (p_hump->b_has_completed_hump ? HUMP_DEBUG_COMPLETED : 0) |
                  (p_hump->b_did_skip_acceleration ? HUMP_DEBUG_SKIPPED : 0) |
                  (p_hump->b_is_hump_tracking ? HUMP_DEBUG_ACTIVE : 0) |
                  (p_hump->b_is_armed ? HUMP_DEBUG_IS_ARMED : 0));
#else
    /* Update the retained detector and discard its debug reason. */
    (void)right_step(p_hump, pitch, roll, track_m, step_distance_m, quality,
                     now);
#endif
}

/**
 * @brief Disarm and clear right-wheel progress, retaining historical results.
 * @param[in,out] p_hump Non-NULL detector owned by the current update call.
 * @param reason Rejection reason passed through unchanged.
 * @return Stop reason. The caller already cleared b_has_completed_hump.
 * @note Retains highest_m, b_is_peak_valid, b_has_descended and started_us.
 */
static hump_detection_reason_t right_discard (hump_detection_t *      p_hump,
                                              hump_detection_reason_t reason)
{
    p_hump->b_is_hump_tracking      = false;
    p_hump->b_is_armed              = false;
    p_hump->height_m                = 0;
    p_hump->peak_m                  = 0;
    p_hump->travel_m                = 0;
    p_hump->level_samples           = 0;
    p_hump->b_did_skip_acceleration = false;
    return reason;
}

/**
 * @brief Apply one right-wheel state-machine step before debug publication.
 * @param[in,out] p_hump Non-NULL initialized detector; no concurrent access.
 * @param pitch Nose-up degrees, validated before geometric calculation.
 * @param roll Right-side-up degrees, validated before geometric calculation.
 * @param track_m Wheel-track width in metres, validated before use.
 * @param step_distance_m Forward distance increment in metres, validated before
 * use.
 * @param quality Quality discriminator with the public update's semantics.
 * @param now Wrapping microsecond timestamp used for the crossing timeout.
 * @return Diagnostic reason; NONE can mean progress or successful completion.
 * @note The step clears the previous completion flag. Rejection preserves
 * the highest completed peak. One active acceleration spike holds the
 * accepted height and travel for that sample.
 * @details Assume the left wheel stays on the level reference. The roll
 * component across the wheel track, projected by pitch, estimates the
 * right wheel lift. Five level samples arm the detector. A candidate needs
 * at least 5 mm of lift and forward movement to start, then at least 5 mm
 * of accumulated travel above the lift threshold and five level samples
 * to complete. Losing the geometry, odometry or ten-second window discards
 * the in-progress crossing.
 */
static hump_detection_reason_t
right_step (hump_detection_t * p_hump, float32_t pitch, float32_t roll,
            float32_t track_m, float32_t step_distance_m,
            hump_detection_sample_quality_t quality, uint32_t now)
{
    const float32_t start_m      = 0.005f;
    const float32_t level_m      = 0.003f;
    p_hump->b_has_completed_hump = false;

    if ((unsigned int)HUMP_SAMPLE_INVALID == quality)
    {
        /* The caller supplies the sensor or motion gate that rejected it. */
        return right_discard(p_hump, HUMP_REASON_NONE);
    }

    if ((!isfinite(step_distance_m)) || (step_distance_m > 0.05f))
    {
        return right_discard(p_hump, HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE);
    }

    if (step_distance_m < 0)
    {
        return right_discard(p_hump, HUMP_REASON_REVERSE_TRAVEL);
    }

    if (((((((!isfinite(pitch)) || (!isfinite(roll))) || (fabsf(pitch) > 40)) ||
           (fabsf(roll) >= 10)) ||
          (!isfinite(track_m))) ||
         (track_m <= 0)) ||
        (track_m > 0.5f))
    {
        return right_discard(p_hump, HUMP_REASON_INVALID_GEOMETRY);
    }

    if ((p_hump->b_is_hump_tracking) &&
        ((now - p_hump->started_us) > 10000000U))
    {
        return right_discard(p_hump, HUMP_REASON_CROSSING_TIMEOUT);
    }

    if ((unsigned int)HUMP_SAMPLE_ACCELERATION_SPIKE == quality)
    {
        if ((p_hump->b_is_hump_tracking) && (!p_hump->b_did_skip_acceleration))
        {
            /* Hold the last accepted height and travel through one sample. */
            p_hump->b_did_skip_acceleration = true;
            p_hump->level_samples           = 0;
            return HUMP_REASON_ACCELERATION_SPIKE;
        }

        return right_discard(p_hump, HUMP_REASON_ACCELERATION_SPIKE);
    }

    p_hump->b_did_skip_acceleration = false;
    const float32_t height =
        track_m * sinf(roll * 0.01745329252f) * cosf(pitch * 0.01745329252f);

    if (height < -level_m)
    {
        return right_discard(p_hump, HUMP_REASON_LEFT_WHEEL_RAISED);
    }

    if (!p_hump->b_is_hump_tracking)
    {
        if (fabsf(height) <= level_m)
        {
            record_level_sample(p_hump);
        }
        else
        {
            p_hump->level_samples = 0;
        }

        if (!p_hump->b_is_armed)
        {
            return HUMP_REASON_WAITING_FOR_LEVEL;
        }

        if (height < start_m)
        {
            return HUMP_REASON_BELOW_START_HEIGHT;
        }

        if (step_distance_m <= 0.0001f)
        {
            return HUMP_REASON_NO_FORWARD_TRAVEL;
        }

        p_hump->b_is_hump_tracking = true;
        p_hump->b_is_armed         = false;
        p_hump->started_us         = now;
        p_hump->peak_m             = 0;
        p_hump->travel_m           = 0;
        p_hump->level_samples      = 0;
    }

    p_hump->height_m = height < 0 ? 0 : height;

    if (height > p_hump->peak_m)
    {
        p_hump->peak_m = height;
    }
    /* A brief roll spike followed by flat travel must not qualify as a hump. */
    if (height >= start_m)
    {
        p_hump->travel_m += step_distance_m;
    }

    p_hump->level_samples =
        fabsf(height) <= level_m ? p_hump->level_samples + 1U : 0U;

    if (p_hump->level_samples >= 5U)
    {
        const bool b_can_complete =
            (p_hump->peak_m >= start_m) && (p_hump->travel_m >= 0.005f);

        if ((b_can_complete) && ((!p_hump->b_is_peak_valid) ||
                                 (p_hump->peak_m > p_hump->highest_m)))
        {
            p_hump->highest_m = p_hump->peak_m;
        }

        if (b_can_complete)
        {
            p_hump->b_is_peak_valid      = true;
            p_hump->b_has_completed_hump = true;
        }

        p_hump->b_is_hump_tracking = false;
        p_hump->height_m           = 0;
        p_hump->b_is_armed         = true;
        return p_hump->b_has_completed_hump
                   ? HUMP_REASON_NONE
                   : HUMP_REASON_INSUFFICIENT_CROSSING_TRAVEL;
    }

    return HUMP_REASON_NONE;
}

/** @brief Count a level reading and arm when the settled count reaches five. */
static inline void record_level_sample (hump_detection_t * p_hump)
{
    if (p_hump->level_samples < 5U)
    {
        ++p_hump->level_samples;
    }

    if (p_hump->level_samples >= 5U)
    {
        p_hump->b_is_armed = true;
    }
}

/* End of file. */


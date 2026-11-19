#include "hump_detection.h"
#include "../app_types.h"
#include <math.h>

void hump_reset (hump_detector_t * p_h)
{
    *p_h = (hump_detector_t){0};
}

void hump_update (hump_detector_t * p_h, float32_t pitch, float32_t value_ds,
                  bool valid, uint32_t now)
{
    p_h->completed = false;
    if (!valid || !isfinite(value_ds) || !isfinite(pitch) || value_ds < 0 ||
        value_ds > 0.05f || fabsf(pitch) > 40 ||
        (p_h->active && (uint32_t)(now - p_h->started_us) > 10000000U))
    {
        p_h->active = false;
        p_h->height_m = 0;
        p_h->peak_m = 0;
        p_h->descended = false;
        p_h->level_samples = 0;
        return;
    }
    if (!p_h->active && pitch > 4.0f && value_ds > 0.0001f)
    {
        p_h->active = true;
        p_h->started_us = now;
        p_h->height_m = 0;
        p_h->peak_m = 0;
        p_h->descended = false;
        p_h->level_samples = 0;
    }
    if (!p_h->active)
    {
        return;
    }
    p_h->height_m += value_ds * sinf(pitch * 0.01745329252f);
    if (p_h->height_m > p_h->peak_m)
    {
        p_h->peak_m = p_h->height_m;
    }
    if (p_h->peak_m > 0.50f || p_h->height_m < -0.03f)
    {
        p_h->active = false;
        return;
    }
    if (pitch < -4.0f)
    {
        p_h->descended = true;
    }
    p_h->level_samples =
        (p_h->descended && fabsf(pitch) < 2.0f) ? p_h->level_samples + 1 : 0;
    if (p_h->level_samples >= 5)
    {
        if (p_h->peak_m >= 0.005f && fabsf(p_h->height_m) < 0.03f)
        {
            if (!p_h->highest_valid || p_h->peak_m > p_h->highest_m)
            {
                p_h->highest_m = p_h->peak_m;
            }
            p_h->highest_valid = true;
            p_h->completed = true;
        }
        p_h->active = false;
    }
}

static hump_reason_t right_discard (hump_detector_t * p_h, hump_reason_t reason)
{
    p_h->active = false;
    p_h->armed = false;
    p_h->height_m = 0;
    p_h->peak_m = 0;
    p_h->travel_m = 0;
    p_h->level_samples = 0;
    p_h->skipped_acceleration = false;
    return reason;
}

static hump_reason_t right_step (hump_detector_t * p_h, float32_t pitch, float32_t roll,
                                 float32_t track_m, float32_t value_ds,
                                 hump_sample_quality_t quality, uint32_t now)
{
    const float32_t start_m = 0.005f, level_m = 0.003f;
    float32_t height;
    p_h->completed = false;
    if (quality == HUMP_SAMPLE_INVALID)
    {
        /* The caller supplies the sensor or motion gate that rejected it. */
        return right_discard(p_h, HUMP_REASON_NONE);
    }
    if (!isfinite(value_ds) || value_ds > 0.05f)
    {
        return right_discard(p_h, HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE);
    }
    if (value_ds < 0)
    {
        return right_discard(p_h, HUMP_REASON_REVERSE_TRAVEL);
    }
    if (!isfinite(pitch) || !isfinite(roll) || fabsf(pitch) > 40 ||
        fabsf(roll) >= 10 || !isfinite(track_m) || track_m <= 0 ||
        track_m > 0.5f)
    {
        return right_discard(p_h, HUMP_REASON_INVALID_GEOMETRY);
    }
    if (p_h->active && (uint32_t)(now - p_h->started_us) > 10000000U)
    {
        return right_discard(p_h, HUMP_REASON_CROSSING_TIMEOUT);
    }
    if (quality == HUMP_SAMPLE_ACCELERATION_SPIKE)
    {
        if (p_h->active && !p_h->skipped_acceleration)
        {
            /* Hold the last accepted height and travel through one sample. */
            p_h->skipped_acceleration = true;
            p_h->level_samples = 0;
            return HUMP_REASON_ACCELERATION_SPIKE;
        }
        return right_discard(p_h, HUMP_REASON_ACCELERATION_SPIKE);
    }
    p_h->skipped_acceleration = false;
    height =
        track_m * sinf(roll * 0.01745329252f) * cosf(pitch * 0.01745329252f);
    if (height < -level_m)
    {
        return right_discard(p_h, HUMP_REASON_LEFT_WHEEL_RAISED);
    }
    if (!p_h->active)
    {
        if (fabsf(height) <= level_m)
        {
            if (p_h->level_samples < 5)
            {
                ++p_h->level_samples;
            }
            if (p_h->level_samples >= 5)
            {
                p_h->armed = true;
            }
        }
        else
        {
            p_h->level_samples = 0;
        }
        if (!p_h->armed)
        {
            return HUMP_REASON_WAITING_FOR_LEVEL;
        }
        if (height < start_m)
        {
            return HUMP_REASON_BELOW_START_HEIGHT;
        }
        if (value_ds <= 0.0001f)
        {
            return HUMP_REASON_NO_FORWARD_TRAVEL;
        }
        p_h->active = true;
        p_h->armed = false;
        p_h->started_us = now;
        p_h->peak_m = 0;
        p_h->travel_m = 0;
        p_h->level_samples = 0;
    }
    p_h->height_m = height < 0 ? 0 : height;
    if (height > p_h->peak_m)
    {
        p_h->peak_m = height;
    }
    /* A brief roll spike followed by flat travel must not qualify as a hump. */
    if (height >= start_m)
    {
        p_h->travel_m += value_ds;
    }
    p_h->level_samples = fabsf(height) <= level_m ? p_h->level_samples + 1 : 0;
    if (p_h->level_samples >= 5)
    {
        if (p_h->peak_m >= start_m && p_h->travel_m >= 0.005f)
        {
            if (!p_h->highest_valid || p_h->peak_m > p_h->highest_m)
            {
                p_h->highest_m = p_h->peak_m;
            }
            p_h->highest_valid = true;
            p_h->completed = true;
        }
        p_h->active = false;
        p_h->height_m = 0;
        p_h->armed = true;
        return p_h->completed ? HUMP_REASON_NONE
                              : HUMP_REASON_INSUFFICIENT_CROSSING_TRAVEL;
    }
    return HUMP_REASON_NONE;
}

void hump_update_right_wheel (hump_detector_t * p_h, float32_t pitch, float32_t roll,
                              float32_t track_m, float32_t value_ds,
                              hump_sample_quality_t quality, uint32_t now)
{
#if APP_IMU_DEBUG
    const bool was_active = p_h->active, was_armed = p_h->armed;
    const float32_t prior_travel = p_h->travel_m;
    p_h->debug.reason =
        (uint8_t)right_step(p_h, pitch, roll, track_m, value_ds, quality, now);
    p_h->debug.travel_m =
        p_h->active ? p_h->travel_m : (was_active ? prior_travel : 0);
    p_h->debug.flags =
        (uint8_t)((!was_armed && p_h->armed ? HUMP_DEBUG_ARMED : 0) |
                  (!was_active && p_h->active ? HUMP_DEBUG_STARTED : 0) |
                  (was_active && !p_h->active && !p_h->completed
                       ? HUMP_DEBUG_DISCARDED
                       : 0) |
                  (p_h->completed ? HUMP_DEBUG_COMPLETED : 0) |
                  (p_h->skipped_acceleration ? HUMP_DEBUG_SKIPPED : 0) |
                  (p_h->active ? HUMP_DEBUG_ACTIVE : 0) |
                  (p_h->armed ? HUMP_DEBUG_IS_ARMED : 0));
#else
    (void)right_step(p_h, pitch, roll, track_m, value_ds, quality, now);
#endif
}

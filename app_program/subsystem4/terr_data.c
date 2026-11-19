/** @file terr_data.c
 * @brief Maintain terrain monitoring state and pack copied terrain
 * observations.
 */
#include "terr_data.h"
#include "../app_types.h"
#include "tilt_estimation.h"
#if APP_IMU_DEBUG
#include <float.h>
#endif
#include <math.h>
#include <string.h>

#define SAFETY_MAX_SAMPLE_GAP_US 50000U
#define SAFETY_LIFT_CONFIRM_US 40000U
#define SAFETY_RECOVERY_US 300000U

static void safety_update (terrain_monitor_t * p_t, const float32_t p_a[3],
                           float32_t norm, uint32_t now, bool continuous)
{
    bool near_gravity = norm >= 0.8f && norm <= 1.2f;
    bool tipped = norm >= 0.7f && norm <= 1.35f &&
                  p_a[2] <= 0.7071068f * norm; /* 45 deg */
    bool recovering = p_t->safety != TERRAIN_SAFETY_NONE;
    bool settled = norm >= 0.9f && norm <= 1.1f &&
                   p_a[2] >= 0.9781476f * norm; /* 12 deg */
    /* Use the raw calibrated vector rather than the filtered pitch/roll values.
     * Those values lag and are unavailable when the vehicle is upside down. */
    bool lift_tilt = near_gravity && p_a[2] <= 0.9063078f * norm; /* 25 deg */
    const float32_t horizontal_sq = p_a[0] * p_a[0] + p_a[1] * p_a[1];
    const float32_t upward_excess = p_a[2] - 1.0f;
    const bool gentle_pickup =
        p_a[2] >= 1.10f && horizontal_sq <= 0.15f * 0.15f;
    /* For a firm pickup, upward acceleration above 1 g must be at least
     * twice the horizontal acceleration. Compare squared magnitudes. */
    const bool firm_pickup = p_a[2] >= 1.30f && upward_excess * upward_excess >=
                                                    4.0f * horizontal_sq;
    const bool upward = gentle_pickup || firm_pickup;
#if APP_IMU_DEBUG
    p_t->safety_upward = upward;
    p_t->safety_lift_tilt = lift_tilt;
#endif
    if (!continuous)
    {
        p_t->safety_tracking = false;
    }
    if (tipped || norm < 0.3f)
    {
        if (tipped)
        {
            p_t->safety = TERRAIN_SAFETY_OVERTURNED;
        }
        else if (!recovering)
        {
            p_t->safety = TERRAIN_SAFETY_LIFTED;
        }
        p_t->safety_tracking = false;
    }
    else if (!(recovering ? settled : (lift_tilt || upward)))
    {
        p_t->safety_tracking = false;
    }
    else if (!p_t->safety_tracking)
    {
        p_t->safety_tracking = true;
        p_t->safety_since_us = now;
    }
    else if ((uint32_t)(now - p_t->safety_since_us) >=
             (recovering ? SAFETY_RECOVERY_US : SAFETY_LIFT_CONFIRM_US))
    {
        p_t->safety = recovering ? TERRAIN_SAFETY_NONE : TERRAIN_SAFETY_LIFTED;
        p_t->safety_tracking = false;
    }
}

void terrain_monitor_init (terrain_monitor_t * p_t)
{
    *p_t = (terrain_monitor_t){0};
    p_t->mag_scale[0] = p_t->mag_scale[1] = p_t->mag_scale[2] = 1;
}

void terrain_monitor_reset_run (terrain_monitor_t * p_t)
{
    hump_reset(&p_t->hump);
    p_t->seen = false;
    p_t->odometry_seen = false;
    p_t->heading_seen = false;
    p_t->stable_count = 0;
    p_t->safety_tracking = false;
    p_t->result = (terrain_result_t){.safety = p_t->safety};
}

bool terrain_monitor_set_geometry (terrain_monitor_t * p_t,
                                   terrain_hump_geometry_t geometry,
                                   float32_t track_m)
{
    if (!p_t ||
        (geometry != TERRAIN_HUMP_LONGITUDINAL &&
         geometry != TERRAIN_HUMP_RIGHT_WHEEL) ||
        (geometry == TERRAIN_HUMP_RIGHT_WHEEL &&
         (!isfinite(track_m) || track_m <= 0 || track_m > 0.5f)))
    {
        return false;
    }
    p_t->hump_geometry = geometry;
    p_t->wheel_track_m = track_m;
    terrain_monitor_reset_run(p_t);
    return true;
}

bool terrain_monitor_set_mag_cal (terrain_monitor_t * p_t,
                                  const float32_t p_offset[3],
                                  const float32_t p_scale[3])
{
    unsigned value_i;
    if (!p_t || !p_offset || !p_scale)
    {
        return false;
    }
    for (value_i = 0; value_i < 3; ++value_i)
    {
        if (!isfinite(p_offset[value_i]) || !isfinite(p_scale[value_i]) ||
            fabsf(p_offset[value_i]) > 5 || p_scale[value_i] < 0.1f ||
            p_scale[value_i] > 10)
        {
            return false;
        }
    }
    memcpy(p_t->mag_offset, p_offset, sizeof(p_t->mag_offset));
    memcpy(p_t->mag_scale, p_scale, sizeof(p_t->mag_scale));
    p_t->magnetic_calibrated = true;
    p_t->heading_seen = false;
    return true;
}

void terrain_monitor_process (terrain_monitor_t * p_t, const imu_sample_t * p_s,
                              const terrain_odometry_t * p_odo)
{
    float32_t value_a[3], value_m[3], pitch = 0, roll = 0, heading,
                                      value_ds = 0, speed = 0, value_dt = 0;
    /* BARR-P12, pending approval: this single primitive spelling preserves
     * Arm 14.3.1 executable bytes; see the recorded alias investigation. */
    float value_n;
    float32_t dynamic_g;
    uint32_t gap = p_t->seen ? p_s->timestamp_us - p_t->previous_us : 0;
    bool valid = imu_sample_is_valid(p_s), tilt_ok = false, movement_ok = false;
    /* Measure continuity from sample timestamps. A sensor overrun can still
     * yield a valid current sample. */
    bool continuity = p_t->seen && gap > 0 && gap <= 100000U;
    unsigned value_i;
    terrain_result_t value_r = {.timestamp_us = p_s->timestamp_us,
                                .safety = p_t->safety};
#if APP_IMU_DEBUG
    hump_reason_t odometry_reason = HUMP_REASON_ODOMETRY_UNAVAILABLE;
    p_t->hump.debug = (hump_debug_t){0};
#endif
    if (p_t->hump.highest_valid)
    {
        value_r.peak_m = p_t->hump.highest_m;
        value_r.flags |= TERRAIN_PEAK_VALID;
    }
    if (!valid || (p_t->seen && !continuity))
    {
        value_r.flags |= TERRAIN_FAULT;
        p_t->heading_seen = false;
        p_t->odometry_seen = false;
        p_t->safety_tracking = false;
        if (!p_t->calibration.ready)
        {
            imu_calibration_reset(&p_t->calibration);
        }
        if (p_t->hump_geometry == TERRAIN_HUMP_RIGHT_WHEEL)
        {
            hump_update_right_wheel(&p_t->hump, 0, 0, p_t->wheel_track_m, 0,
                                    HUMP_SAMPLE_INVALID, p_s->timestamp_us);
#if APP_IMU_DEBUG
            p_t->hump.debug.reason =
                valid ? HUMP_REASON_IMU_GAP : HUMP_REASON_INVALID_IMU;
#endif
        }
        else
        {
            hump_update(&p_t->hump, 0, 0, false, p_s->timestamp_us);
        }
        p_t->seen = false;
        p_t->result = value_r;
        return;
    }
    if (!p_t->calibration.ready)
    {
        imu_calibration_feed(&p_t->calibration, p_s,
                             p_odo && p_odo->stationary);
        p_t->previous_us = p_s->timestamp_us;
        p_t->seen = !p_t->calibration.ready;
#if APP_IMU_DEBUG
        p_t->hump.debug.reason = HUMP_REASON_CALIBRATION_PENDING;
#endif
        p_t->result = value_r;
        return;
    }
    value_r.flags |= TERRAIN_CALIBRATED;
    for (value_i = 0; value_i < 3; ++value_i)
    {
        value_a[value_i] =
            p_s->accel_g[value_i] - p_t->calibration.bias_g[value_i];
        p_t->filtered[value_i] =
            continuity ? p_t->filtered[value_i] +
                             0.25f * (value_a[value_i] - p_t->filtered[value_i])
                       : value_a[value_i];
        value_m[value_i] =
            (p_s->mag_gauss[value_i] - p_t->mag_offset[value_i]) *
            p_t->mag_scale[value_i];
    }
    value_n = sqrtf(value_a[0] * value_a[0] + value_a[1] * value_a[1] +
                    value_a[2] * value_a[2]);
    safety_update(p_t, value_a, value_n, p_s->timestamp_us,
                  continuity && gap <= SAFETY_MAX_SAMPLE_GAP_US);
    value_r.safety = p_t->safety;
    value_r.impact = isfinite(value_n) && (value_n > 2.0f || value_n < 0.3f);
    tilt_ok = value_n >= 0.9f && value_n <= 1.1f &&
              tilt_estimation(p_t->filtered, &pitch, &roll);
    if (tilt_ok)
    {
        value_r.pitch_deg = pitch;
        value_r.roll_deg = roll;
        value_r.flags |= TERRAIN_TILT_VALID;
    }
    if (tilt_ok && p_s->mag_valid && p_t->magnetic_calibrated &&
        tilt_heading(p_t->filtered, value_m, &heading))
    {
        uint32_t elapsed = p_s->timestamp_us - p_t->heading_us;
        if (p_t->heading_seen && elapsed >= 10000 && elapsed <= 200000)
        {
            float32_t rate =
                tilt_angle_delta(heading, p_t->heading) * 1000000.0f / elapsed;
            if (fabsf(rate) <= 360)
            {
                value_r.turn_dps = rate;
                value_r.flags |= TERRAIN_TURN_VALID;
            }
        }
        p_t->heading = heading;
        p_t->heading_us = p_s->timestamp_us;
        p_t->heading_seen = true;
    }
    else if (p_s->mag_valid || !tilt_ok ||
             (uint32_t)(p_s->timestamp_us - p_t->heading_us) > 100000)
    {
        p_t->heading_seen = false;
    }
    else if (p_t->heading_seen && (p_t->result.flags & TERRAIN_TURN_VALID))
    {
        value_r.turn_dps = p_t->result.turn_dps;
        value_r.flags |= TERRAIN_TURN_VALID;
    }
    if (p_odo && p_odo->valid && isfinite(p_odo->distance_m))
    {
#if APP_IMU_DEBUG
        odometry_reason = HUMP_REASON_ODOMETRY_BASELINE;
#endif
        if (p_t->odometry_seen && continuity && p_odo->epoch == p_t->epoch)
        {
            float64_t delta = p_odo->distance_m - p_t->previous_distance;
#if APP_IMU_DEBUG
            odometry_reason = HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE;
            if (isfinite(delta) && fabs(delta) <= FLT_MAX)
            {
                p_t->hump.debug.wheel_delta_m = (float32_t)delta;
                p_t->hump.debug.wheel_delta_valid = true;
            }
#endif
            value_dt = gap / 1000000.0f;
            if (fabs(delta) <= 0.05)
            {
                value_ds = (float32_t)delta;
                speed = value_ds / value_dt;
                movement_ok = fabsf(speed) <= 1.5f;
            }
        }
        p_t->previous_distance = p_odo->distance_m;
        p_t->epoch = p_odo->epoch;
        p_t->odometry_seen = true;
    }
    else
    {
        p_t->odometry_seen = false;
    }
    dynamic_g = value_a[0] - p_t->filtered[0];
    /* Acceleration alone cannot distinguish slope from sustained acceleration.
     * Reject obvious dynamics/roll/turns; known-height testing remains
     * required. */
    {
        bool hump_valid = tilt_ok && movement_ok && !value_r.impact &&
                          value_r.safety == TERRAIN_SAFETY_NONE &&
                          (!(value_r.flags & TERRAIN_TURN_VALID) ||
                           fabsf(value_r.turn_dps) < 15);
        bool forward_stable = fabsf(dynamic_g) < 0.08f;
        if (p_t->hump_geometry == TERRAIN_HUMP_RIGHT_WHEEL)
        {
            /* Lateral acceleration can imitate a raised right wheel. */
            hump_sample_quality_t quality = HUMP_SAMPLE_INVALID;
            if (hump_valid)
            {
                quality = forward_stable &&
                                  fabsf(value_a[1] - p_t->filtered[1]) < 0.08f
                              ? HUMP_SAMPLE_VALID
                              : HUMP_SAMPLE_ACCELERATION_SPIKE;
            }
            hump_update_right_wheel(&p_t->hump, pitch, roll, p_t->wheel_track_m,
                                    value_ds, quality, p_s->timestamp_us);
#if APP_IMU_DEBUG
            if (quality == HUMP_SAMPLE_INVALID)
            {
                if (value_r.impact || value_r.safety != TERRAIN_SAFETY_NONE)
                {
                    p_t->hump.debug.reason = HUMP_REASON_SAFETY_OR_IMPACT;
                }
                else if (!tilt_ok)
                {
                    p_t->hump.debug.reason = HUMP_REASON_INVALID_TILT;
                }
                else if (!movement_ok)
                {
                    p_t->hump.debug.reason = (uint8_t)odometry_reason;
                }
                else
                {
                    p_t->hump.debug.reason = HUMP_REASON_TURN_RATE;
                }
            }
#endif
        }
        else
        {
            hump_update(&p_t->hump, pitch, value_ds,
                        hump_valid && forward_stable && fabsf(roll) < 10,
                        p_s->timestamp_us);
        }
    }
    if (p_t->hump.highest_valid)
    {
        value_r.peak_m = p_t->hump.highest_m;
        value_r.flags |= TERRAIN_PEAK_VALID;
    }
    value_r.hump_completed = p_t->hump.completed;
    /* Live climb: height so far on the hump being crossed (not yet a peak). */
    value_r.hump_active = p_t->hump.active;
    if (value_r.hump_active)
    {
        value_r.hump_height_m =
            p_t->hump.height_m < 0
                ? 0.0f
                : (p_t->hump.height_m > 0.5f ? 0.5f : p_t->hump.height_m);
    }
    if (value_r.impact)
    {
        value_r.state = TERRAIN_IMPACT;
        p_t->stable_count = 0;
    }
    else if (tilt_ok && movement_ok)
    {
        terrain_state_t candidate =
            motion_classify(pitch, speed, dynamic_g, value_r.turn_dps,
                            (value_r.flags & TERRAIN_TURN_VALID) != 0, false);
        if (candidate != p_t->candidate)
        {
            p_t->candidate = candidate;
            p_t->stable_count = 1;
        }
        else if (p_t->stable_count < 3)
        {
            ++p_t->stable_count;
        }
        value_r.state = p_t->stable_count >= 3 ? candidate : p_t->result.state;
        if (value_r.state == TERRAIN_IMPACT)
        {
            value_r.state = TERRAIN_UNKNOWN;
        }
    }
    else
    {
        p_t->stable_count = 0;
        value_r.state = TERRAIN_UNKNOWN;
    }
    p_t->previous_us = p_s->timestamp_us;
    p_t->seen = true;
    p_t->result = value_r;
}

static void put_bits (uint8_t * p_out, unsigned * p_bit, uint32_t value,
                      unsigned width)
{
    unsigned value_i;
    for (value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        if (value & (1UL << value_i))
        {
            p_out[*p_bit / 8] |= (uint8_t)(1U << (*p_bit % 8));
        }
    }
}

void terrain_pack (const terrain_result_t * p_r,
                   uint8_t p_out[TERRAIN_PACKET_SIZE])
{
    unsigned bit = 0;
    uint8_t flags = p_r->flags;
    int32_t pitch = 0, roll = 0, rate = 0;
    uint32_t height = 0, climb = 0;
    bool active = p_r->hump_active && isfinite(p_r->hump_height_m) &&
                  p_r->hump_height_m >= 0 && p_r->hump_height_m <= 0.5f;
    memset(p_out, 0, TERRAIN_PACKET_SIZE);
    if (!isfinite(p_r->pitch_deg) || !isfinite(p_r->roll_deg) ||
        fabsf(p_r->pitch_deg) > 90 || fabsf(p_r->roll_deg) > 90)
    {
        flags &= (uint8_t)~TERRAIN_TILT_VALID;
    }
    if (!isfinite(p_r->turn_dps) || fabsf(p_r->turn_dps) > 360)
    {
        flags &= (uint8_t)~TERRAIN_TURN_VALID;
    }
    if (!isfinite(p_r->peak_m) || p_r->peak_m < 0 || p_r->peak_m > 0.5f)
    {
        flags &= (uint8_t)~TERRAIN_PEAK_VALID;
    }
    if (flags & TERRAIN_TILT_VALID)
    {
        pitch = (int32_t)lroundf(p_r->pitch_deg * 10);
        roll = (int32_t)lroundf(p_r->roll_deg * 10);
    }
    if (flags & TERRAIN_TURN_VALID)
    {
        rate = (int32_t)lroundf(p_r->turn_dps);
    }
    if (flags & TERRAIN_PEAK_VALID)
    {
        height = (uint32_t)lroundf(p_r->peak_m * 1000);
    }
    if (active)
    {
        climb = (uint32_t)lroundf(p_r->hump_height_m * 1000);
    }
    put_bits(p_out, &bit, p_r->timestamp_us, 32);
    put_bits(p_out, &bit, (uint32_t)p_r->state, 3);
    put_bits(p_out, &bit, flags, 5);
    put_bits(p_out, &bit, (uint32_t)pitch, 11);
    put_bits(p_out, &bit, (uint32_t)roll, 11);
    put_bits(p_out, &bit, (uint32_t)rate, 10);
    put_bits(p_out, &bit, height, 9);
    put_bits(p_out, &bit, p_r->hump_completed, 1);
    put_bits(p_out, &bit, p_r->impact, 1);
    put_bits(p_out, &bit, active, 1);
    put_bits(p_out, &bit, climb, 9);
    put_bits(p_out, &bit, (uint32_t)p_r->safety, 2);
}

static uint32_t get_bits (const uint8_t * p_p, unsigned * p_bit, unsigned width)
{
    uint32_t value_v = 0;
    unsigned value_i;
    for (value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        value_v |= (uint32_t)((p_p[*p_bit / 8] >> (*p_bit % 8)) & 1U)
                   << value_i;
    }
    return value_v;
}

static int32_t signed_value (uint32_t value_v, unsigned width)
{
    return (value_v & (1U << (width - 1U)))
               ? (int32_t)value_v - (int32_t)(1U << width)
               : (int32_t)value_v;
}

void terrain_unpack (const uint8_t p_in[TERRAIN_PACKET_SIZE],
                     terrain_result_t * p_r)
{
    unsigned bit = 0;
    *p_r = (terrain_result_t){0};
    p_r->timestamp_us = get_bits(p_in, &bit, 32);
    p_r->state = (terrain_state_t)get_bits(p_in, &bit, 3);
    p_r->flags = (uint8_t)get_bits(p_in, &bit, 5);
    p_r->pitch_deg = signed_value(get_bits(p_in, &bit, 11), 11) / 10.0f;
    p_r->roll_deg = signed_value(get_bits(p_in, &bit, 11), 11) / 10.0f;
    p_r->turn_dps = (float32_t)signed_value(get_bits(p_in, &bit, 10), 10);
    p_r->peak_m = get_bits(p_in, &bit, 9) / 1000.0f;
    p_r->hump_completed = get_bits(p_in, &bit, 1) != 0;
    p_r->impact = get_bits(p_in, &bit, 1) != 0;
    p_r->hump_active = get_bits(p_in, &bit, 1) != 0;
    p_r->hump_height_m = get_bits(p_in, &bit, 9) / 1000.0f;
    p_r->safety = (terrain_safety_t)get_bits(p_in, &bit, 2);
}

/* End of file. */


/** @file terr_data.c
 * @brief Maintain terrain monitoring state and pack copied terrain
 * observations.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected constants and mask results are nonnegative.
 * Cast them in the existing unsigned arithmetic or comparison domain.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "terr_data.h"
#include "../app_types.h"
#include "tilt.h"
#if APP_IMU_DEBUG
#include <float.h>
#endif
#include <math.h>
#include <string.h>

#define SAFETY_MAX_SAMPLE_GAP_US 50000U
#define SAFETY_LIFT_CONFIRM_US 40000U
#define SAFETY_RECOVERY_US 300000U

static void safety_update (terr_data_monitor_t * p_monitor,
                           const float32_t p_accel_g[3], float32_t norm,
                           uint32_t now, bool b_is_continuous);

static void     put_bits (uint8_t * p_out, unsigned * p_bit, uint32_t value,
                          unsigned width);
static uint32_t get_bits (const uint8_t * p_packet, unsigned * p_bit,
                          unsigned width);
static int32_t  signed_value (uint32_t encoded, unsigned width);

/** @brief Initialize the supplied terrain monitor and level calibration before
 * its first sample.
 */
void terr_data_init (terr_data_monitor_t * p_monitor)
{
    *p_monitor                  = (terr_data_monitor_t){0};
    p_monitor->mag_scale[0]     = p_monitor->mag_scale[1] =
        p_monitor->mag_scale[2] = 1;
}

/** @brief Clear run-specific hump, motion and event tracking while retaining
 * calibration and the current safety state.
 */
void terr_data_reset_run (terr_data_monitor_t * p_monitor)
{
    hump_detection_reset(&p_monitor->hump);
    /* clang-format off */
    p_monitor->b_has_imu_sample       = false;
    p_monitor->b_has_terrain_odometry = false;
    p_monitor->b_has_seen_heading     = false;
    p_monitor->stable_count           = 0;
    p_monitor->b_is_safety_tracking   = false;
    p_monitor->result                 =
        (terr_data_result_t){.safety = p_monitor->safety};
    /* clang-format on */
}

/** @brief Select longitudinal or right-wheel hump geometry. The right-wheel
 * mode needs a finite track width greater than zero and no more than
 * 0.5 m. Accepted geometry resets the run history.
 */
bool terr_data_set_geometry (terr_data_monitor_t *     p_monitor,
                             terr_data_hump_geometry_t geometry,
                             float32_t                 track_m)
{
    if (((0 == p_monitor) ||
         ((geometry != (unsigned int)TERRAIN_HUMP_LONGITUDINAL) &&
          (geometry != (unsigned int)TERRAIN_HUMP_RIGHT_WHEEL))) ||
        (((unsigned int)TERRAIN_HUMP_RIGHT_WHEEL == geometry) &&
         (((!isfinite(track_m)) || (track_m <= 0)) || (track_m > 0.5f))))
    {
        return false;
    }

    p_monitor->hump_geometry = geometry;
    p_monitor->wheel_track_m = track_m;
    terr_data_reset_run(p_monitor);
    return true;
}

/** @brief Validate all three finite magnetometer offsets and scales before
 * copying them. Reject offset magnitudes above 5 or scales outside 0.1
 * to 10. Acceptance resets heading history.
 */
bool terr_data_set_mag_cal (terr_data_monitor_t * p_monitor,
                            const float32_t       p_offset[3],
                            const float32_t       p_scale[3])
{

    if (((0 == p_monitor) || (!p_offset)) || (!p_scale))
    {
        return false;
    }

    unsigned axis_index;
    for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
         ++axis_index)
    {
        if (((((!isfinite(p_offset[axis_index])) ||
               (!isfinite(p_scale[axis_index]))) ||
              (fabsf(p_offset[axis_index]) > 5)) ||
             (p_scale[axis_index] < 0.1f)) ||
            (p_scale[axis_index] > 10))
        {
            return false;
        }
    }

    memcpy(p_monitor->mag_offset, p_offset, sizeof(p_monitor->mag_offset));
    memcpy(p_monitor->mag_scale, p_scale, sizeof(p_monitor->mag_scale));
    p_monitor->b_is_magnetic_calibrated = true;
    p_monitor->b_has_seen_heading       = false;
    return true;
}

/** @brief Update terrain classification from one IMU sample and optional
 * wheel odometry owned by the calling task.
 * @details The sample timestamps establish continuity for calibration and
 * tilt. Magnetic heading requires the calibrated field. Use only continuous
 * odometry from the current epoch to estimate wheel travel for a hump.
 * Stale or invalid observations follow the existing safety and hump reset
 * paths. The caller serializes access to the monitor and its published result.
 */
void terr_data_process (terr_data_monitor_t *        p_monitor,
                        const imu_sensor_sample_t *  p_sample,
                        const terr_data_odometry_t * p_odometry)
{
    float32_t accel_vec[3];
    float32_t mag_gauss[3];
    float32_t pitch = 0;
    float32_t roll  = 0;
    float32_t heading;
    float32_t travel_m = 0;
    float32_t speed    = 0;
    float32_t sample_s = 0;
    /* WARNING: BARR-P12 records the Arm 14.3.1 instruction and stack changes
     * caused by replacing this spelling with float32_t. Approval is pending. */
    float     norm_g;
    float32_t dynamic_g;

    const uint32_t gap =
        (p_monitor->b_has_imu_sample
             ? (p_sample->timestamp_us - p_monitor->previous_us)
             : 0U);

    bool b_is_sample_valid = imu_sensor_sample_is_valid(p_sample);
    bool b_is_tilt_valid   = false;
    bool b_is_motion_valid = false;
    /* Measure continuity from sample timestamps. A sensor overrun can still
     * yield a valid current sample. */
    const bool b_has_continuity =
        ((p_monitor->b_has_imu_sample) && (gap > 0U)) && (gap <= 100000U);
    unsigned           axis_index;
    terr_data_result_t reading = {.timestamp_us = p_sample->timestamp_us,
                                  .safety       = p_monitor->safety};
#if APP_IMU_DEBUG
    hump_detection_reason_t odometry_reason = HUMP_REASON_ODOMETRY_UNAVAILABLE;
    p_monitor->hump.debug                   = (hump_detection_debug_t){0};
#endif
    if (p_monitor->hump.b_is_peak_valid)
    {
        reading.peak_m = p_monitor->hump.highest_m;
        reading.flags |= TERRAIN_PEAK_VALID;
    }

    if ((!b_is_sample_valid) ||
        ((p_monitor->b_has_imu_sample) && (!b_has_continuity)))
    {
        reading.flags                    |= TERRAIN_FAULT;
        p_monitor->b_has_seen_heading     = false;
        p_monitor->b_has_terrain_odometry = false;
        p_monitor->b_is_safety_tracking   = false;

        if (!p_monitor->calibration.b_is_calibrated)
        {
            imu_calibration_reset(&p_monitor->calibration);
        }

        if ((unsigned int)TERRAIN_HUMP_RIGHT_WHEEL == p_monitor->hump_geometry)
        {
            hump_detection_update_right(
                &p_monitor->hump, 0, 0, p_monitor->wheel_track_m, 0,
                HUMP_SAMPLE_INVALID, p_sample->timestamp_us);
#if APP_IMU_DEBUG
            p_monitor->hump.debug.reason = b_is_sample_valid
                                               ? HUMP_REASON_IMU_GAP
                                               : HUMP_REASON_INVALID_IMU;
#endif
        }
        else
        {
            hump_detection_update(&p_monitor->hump, 0, 0, false,
                                  p_sample->timestamp_us);
        }

        p_monitor->b_has_imu_sample = false;
        p_monitor->result           = reading;
        return;
    }

    if (!p_monitor->calibration.b_is_calibrated)
    {
        imu_calibration_feed(&p_monitor->calibration, p_sample,
                             p_odometry && (p_odometry->b_is_stationary));
        p_monitor->previous_us      = p_sample->timestamp_us;
        p_monitor->b_has_imu_sample = !p_monitor->calibration.b_is_calibrated;
#if APP_IMU_DEBUG
        p_monitor->hump.debug.reason = HUMP_REASON_CALIBRATION_PENDING;
#endif
        p_monitor->result = reading;
        return;
    }

    reading.flags |= TERRAIN_CALIBRATED;

    for (axis_index = 0; axis_index < (unsigned int)IMU_AXIS_COUNT;
         ++axis_index)
    {
        /* clang-format off */
        accel_vec[axis_index]           =
            p_sample->accel_g[axis_index] -
            p_monitor->calibration.bias_g[axis_index];
        p_monitor->filtered[axis_index] =
            b_has_continuity ? p_monitor->filtered[axis_index] +
                                   (0.25f * (accel_vec[axis_index] -
                                             p_monitor->filtered[axis_index]))
                             : accel_vec[axis_index];
        mag_gauss[axis_index]           =
            (p_sample->mag_gauss[axis_index] -
             p_monitor->mag_offset[axis_index]) *
            p_monitor->mag_scale[axis_index];
        /* clang-format on */
    }

    norm_g =
        sqrtf((accel_vec[0] * accel_vec[0]) + (accel_vec[1] * accel_vec[1]) +
              (accel_vec[2] * accel_vec[2]));
    safety_update(p_monitor, accel_vec, norm_g, p_sample->timestamp_us,
                  b_has_continuity && (gap <= SAFETY_MAX_SAMPLE_GAP_US));
    /* clang-format off */
    reading.safety       = p_monitor->safety;
    reading.b_has_impact =
        (isfinite(norm_g)) && ((norm_g > 2.0f) || (norm_g < 0.3f));
    b_is_tilt_valid      = ((norm_g >= 0.9f) && (norm_g <= 1.1f)) &&
                      (tilt_estimation(p_monitor->filtered, &pitch, &roll));
    /* clang-format on */

    if (b_is_tilt_valid)
    {
        reading.pitch_deg = pitch;
        reading.roll_deg  = roll;
        reading.flags    |= TERRAIN_TILT_VALID;
    }

    if (((b_is_tilt_valid && (p_sample->b_is_mag_valid)) &&
         (p_monitor->b_is_magnetic_calibrated)) &&
        (tilt_heading(p_monitor->filtered, mag_gauss, &heading)))
    {
        const uint32_t elapsed = p_sample->timestamp_us - p_monitor->heading_us;

        if (((p_monitor->b_has_seen_heading) && (elapsed >= 10000U)) &&
            (elapsed <= 200000U))
        {
            const float32_t rate =
                tilt_angle_delta(heading, p_monitor->heading) * 1000000.0f /
                elapsed;

            if (fabsf(rate) <= 360)
            {
                reading.turn_dps = rate;
                reading.flags   |= TERRAIN_TURN_VALID;
            }
        }

        p_monitor->heading            = heading;
        p_monitor->heading_us         = p_sample->timestamp_us;
        p_monitor->b_has_seen_heading = true;
    }
    else if (((p_sample->b_is_mag_valid) || (!b_is_tilt_valid)) ||
             ((p_sample->timestamp_us - p_monitor->heading_us) > 100000U))
    {
        p_monitor->b_has_seen_heading = false;
    }
    else if ((p_monitor->b_has_seen_heading) &&
             (p_monitor->result.flags & TERRAIN_TURN_VALID))
    {
        reading.turn_dps = p_monitor->result.turn_dps;
        reading.flags   |= TERRAIN_TURN_VALID;
    }
    else
    {
        /* No prior valid turn rate is available for reuse. */
    }

    if ((p_odometry && (p_odometry->b_is_valid)) &&
        (isfinite(p_odometry->distance_m)))
    {
#if APP_IMU_DEBUG
        odometry_reason = HUMP_REASON_ODOMETRY_BASELINE;
#endif
        if (((p_monitor->b_has_terrain_odometry) && b_has_continuity) &&
            (p_odometry->epoch == p_monitor->epoch))
        {
            float64_t delta =
                p_odometry->distance_m - p_monitor->previous_distance;
#if APP_IMU_DEBUG
            odometry_reason = HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE;
            if (isfinite(delta) && fabs(delta) <= FLT_MAX)
            {
                p_monitor->hump.debug.wheel_delta_m          = (float32_t)delta;
                p_monitor->hump.debug.b_is_wheel_delta_valid = true;
            }
#endif
            sample_s = gap / 1000000.0f;

            if (fabs(delta) <= 0.05)
            {
                travel_m          = (float32_t)delta;
                speed             = travel_m / sample_s;
                b_is_motion_valid = fabsf(speed) <= 1.5f;
            }
        }

        p_monitor->previous_distance      = p_odometry->distance_m;
        p_monitor->epoch                  = p_odometry->epoch;
        p_monitor->b_has_terrain_odometry = true;
    }
    else
    {
        p_monitor->b_has_terrain_odometry = false;
    }

    dynamic_g = accel_vec[0] - p_monitor->filtered[0];
    /* Acceleration alone cannot distinguish slope from sustained acceleration.
     * Reject obvious dynamics/roll/turns; known-height testing remains
     * required. */
    {
        const bool b_is_hump_valid =
            (((b_is_tilt_valid && b_is_motion_valid) &&
              (!reading.b_has_impact)) &&
             ((unsigned int)TERRAIN_SAFETY_NONE == reading.safety)) &&
            ((!(reading.flags & TERRAIN_TURN_VALID)) ||
             (fabsf(reading.turn_dps) < 15));
        const bool b_is_forward_stable = fabsf(dynamic_g) < 0.08f;

        if ((unsigned int)TERRAIN_HUMP_RIGHT_WHEEL == p_monitor->hump_geometry)
        {
            /* Lateral acceleration can imitate a raised right wheel. */
            hump_detection_sample_quality_t quality = HUMP_SAMPLE_INVALID;

            if (b_is_hump_valid)
            {
                quality = b_is_forward_stable &&
                                  (fabsf(accel_vec[1] -
                                         p_monitor->filtered[1]) < 0.08f)
                              ? HUMP_SAMPLE_VALID
                              : HUMP_SAMPLE_ACCELERATION_SPIKE;
            }

            hump_detection_update_right(&p_monitor->hump, pitch, roll,
                                        p_monitor->wheel_track_m, travel_m,
                                        quality, p_sample->timestamp_us);
#if APP_IMU_DEBUG
            if ((unsigned)HUMP_SAMPLE_INVALID == quality)
            {
                if (reading.b_has_impact ||
                    reading.safety != (unsigned)TERRAIN_SAFETY_NONE)
                {
                    p_monitor->hump.debug.reason = HUMP_REASON_SAFETY_OR_IMPACT;
                }
                else if (!b_is_tilt_valid)
                {
                    p_monitor->hump.debug.reason = HUMP_REASON_INVALID_TILT;
                }
                else if (!b_is_motion_valid)
                {
                    /* The three odometry reasons use codes 4..6. */
                    p_monitor->hump.debug.reason = (uint8_t)odometry_reason;
                }
                else
                {
                    p_monitor->hump.debug.reason = HUMP_REASON_TURN_RATE;
                }
            }
#endif
        }
        else
        {
            hump_detection_update(&p_monitor->hump, pitch, travel_m,
                                  (b_is_hump_valid && b_is_forward_stable) &&
                                      (fabsf(roll) < 10),
                                  p_sample->timestamp_us);
        }
    }

    if (p_monitor->hump.b_is_peak_valid)
    {
        reading.peak_m = p_monitor->hump.highest_m;
        reading.flags |= TERRAIN_PEAK_VALID;
    }

    reading.b_is_hump_complete = p_monitor->hump.b_has_completed_hump;
    /* Live climb: height so far on the hump being crossed (not yet a peak). */
    reading.b_is_hump_active = p_monitor->hump.b_is_hump_tracking;

    if (reading.b_is_hump_active)
    {
        reading.hump_height_m =
            p_monitor->hump.height_m < 0
                ? 0.0f
                : (p_monitor->hump.height_m > 0.5f ? 0.5f
                                                   : p_monitor->hump.height_m);
    }

    if (reading.b_has_impact)
    {
        reading.state           = TERRAIN_IMPACT;
        p_monitor->stable_count = 0;
    }
    else if (b_is_tilt_valid && b_is_motion_valid)
    {
        const events_motion_state_t candidate = events_motion_classify(
            pitch, speed, dynamic_g, reading.turn_dps,
            (reading.flags & TERRAIN_TURN_VALID) != 0U, false);

        if (candidate != p_monitor->candidate)
        {
            p_monitor->candidate    = candidate;
            p_monitor->stable_count = 1;
        }
        else if (p_monitor->stable_count < 3U)
        {
            ++p_monitor->stable_count;
        }
        else
        {
            /* The stability counter has reached its cap. */
        }

        reading.state =
            p_monitor->stable_count >= 3U ? candidate : p_monitor->result.state;

        if ((unsigned int)TERRAIN_IMPACT == reading.state)
        {
            reading.state = TERRAIN_UNKNOWN;
        }
    }
    else
    {
        p_monitor->stable_count = 0;
        reading.state           = TERRAIN_UNKNOWN;
    }

    p_monitor->previous_us      = p_sample->timestamp_us;
    p_monitor->b_has_imu_sample = true;
    p_monitor->result           = reading;
}

/** @brief Quantize the terrain result into the fixed packet. Clear unavailable
 * or out-of-range measurements' validity as defined by the format. The
 * caller supplies the full writable packet.
 */
void terr_data_pack (const terr_data_result_t * p_result,
                     uint8_t                    p_out[TERRAIN_PACKET_SIZE])
{
    unsigned   bit         = 0;
    uint8_t    flags       = p_result->flags;
    int32_t    pitch       = 0;
    int32_t    roll        = 0;
    int32_t    rate        = 0;
    uint32_t   height      = 0;
    uint32_t   climb       = 0;
    const bool b_is_active = (((p_result->b_is_hump_active) &&
                               (isfinite(p_result->hump_height_m))) &&
                              (p_result->hump_height_m >= 0)) &&
                             (p_result->hump_height_m <= 0.5f);
    memset(p_out, 0, TERRAIN_PACKET_SIZE);

    /* Complement each validity bit in unsigned arithmetic. Intersecting
     * it with the original byte keeps the result in 0..255. */
    if ((((!isfinite(p_result->pitch_deg)) ||
          (!isfinite(p_result->roll_deg))) ||
         (fabsf(p_result->pitch_deg) > 90)) ||
        (fabsf(p_result->roll_deg) > 90))
    {
        flags = (uint8_t)((unsigned)flags & ~(unsigned)TERRAIN_TILT_VALID);
    }

    if ((!isfinite(p_result->turn_dps)) || (fabsf(p_result->turn_dps) > 360))
    {
        flags = (uint8_t)((unsigned)flags & ~(unsigned)TERRAIN_TURN_VALID);
    }

    if (((!isfinite(p_result->peak_m)) || (p_result->peak_m < 0)) ||
        (p_result->peak_m > 0.5f))
    {
        flags = (uint8_t)((unsigned)flags & ~(unsigned)TERRAIN_PEAK_VALID);
    }

    if (flags & TERRAIN_TILT_VALID)
    {
        /* The validity gate bounds the rounded values to -900..900 tenths. */
        pitch = (int32_t)lroundf(p_result->pitch_deg * 10);
        roll  = (int32_t)lroundf(p_result->roll_deg * 10);
    }

    if (flags & TERRAIN_TURN_VALID)
    {
        /* Finite turn rates are within -360..360 before rounding. */
        rate = (int32_t)lroundf(p_result->turn_dps);
    }

    if (flags & TERRAIN_PEAK_VALID)
    {
        /* The validity gate bounds the rounded peak to 0..500 mm. */
        height = (uint32_t)lroundf(p_result->peak_m * 1000);
    }

    if (b_is_active)
    {
        /* The active-hump predicate bounds current climb to 0..500 mm. */
        climb = (uint32_t)lroundf(p_result->hump_height_m * 1000);
    }

    /* The state and safety conversions supply their raw unsigned codes.
     * The writer emits the low three and two bits, respectively. */
    put_bits(p_out, &bit, p_result->timestamp_us, 32);
    put_bits(p_out, &bit, (uint32_t)p_result->state, 3);
    put_bits(p_out, &bit, flags, 5);
    /* Unsigned conversion keeps the low bits of each signed value for
     * the explicit two's-complement packet fields. */
    put_bits(p_out, &bit, (uint32_t)pitch, 11);
    put_bits(p_out, &bit, (uint32_t)roll, 11);
    put_bits(p_out, &bit, (uint32_t)rate, 10);
    put_bits(p_out, &bit, height, 9);
    put_bits(p_out, &bit, p_result->b_is_hump_complete, 1);
    put_bits(p_out, &bit, p_result->b_has_impact, 1);
    put_bits(p_out, &bit, b_is_active, 1);
    put_bits(p_out, &bit, climb, 9);
    put_bits(p_out, &bit, (uint32_t)p_result->safety, 2);
}

/** @brief Expand one fixed-size packed terrain result, returning tilt in
 * degrees, turn rate in degrees per second and hump heights in metres
 * through caller storage.
 */
void terr_data_unpack (const uint8_t        p_in[TERRAIN_PACKET_SIZE],
                       terr_data_result_t * p_result)
{
    unsigned bit = 0;
    /* Retain all raw state, validity and safety codes from fields of three,
     * five and two bits. The signed rate is -512..511 and converts exactly
     * to float32_t. */
    /* clang-format off */
    *p_result                    = (terr_data_result_t){0};
    p_result->timestamp_us       = get_bits(p_in, &bit, 32);
    p_result->state              =
        (events_motion_state_t)get_bits(p_in, &bit, 3);
    p_result->flags              = (uint8_t)get_bits(p_in, &bit, 5);
    p_result->pitch_deg          =
        signed_value(get_bits(p_in, &bit, 11), 11) / 10.0f;
    p_result->roll_deg           =
        signed_value(get_bits(p_in, &bit, 11), 11) / 10.0f;
    p_result->turn_dps           =
        (float32_t)signed_value(get_bits(p_in, &bit, 10), 10);
    p_result->peak_m             = get_bits(p_in, &bit, 9) / 1000.0f;
    p_result->b_is_hump_complete = get_bits(p_in, &bit, 1) != 0U;
    p_result->b_has_impact       = get_bits(p_in, &bit, 1) != 0U;
    p_result->b_is_hump_active   = get_bits(p_in, &bit, 1) != 0U;
    p_result->hump_height_m      = get_bits(p_in, &bit, 9) / 1000.0f;
    p_result->safety             = (terr_data_safety_t)get_bits(p_in, &bit, 2);
    /* clang-format on */
}

/** @brief Use acceleration and elapsed microseconds to track and debounce
 * transitions between normal, lifted and overturned states.
 */
static void safety_update (terr_data_monitor_t * p_monitor,
                           const float32_t p_accel_g[3], float32_t norm,
                           uint32_t now, bool b_is_continuous)
{
    const bool b_is_near_gravity = (norm >= 0.8f) && (norm <= 1.2f);
    const bool b_is_tipped =
        ((norm >= 0.7f) && (norm <= 1.35f)) &&
        (p_accel_g[2] <=
         (0.7071068f * norm)); /* Tip at 45 degrees from upright. */
    const bool b_is_recovering =
        p_monitor->safety != (unsigned int)TERRAIN_SAFETY_NONE;
    const bool b_is_settled =
        ((norm >= 0.9f) && (norm <= 1.1f)) &&
        (p_accel_g[2] >=
         (0.9781476f * norm)); /* Settle within 12 degrees of upright. */
    /* Use the raw calibrated vector rather than the filtered pitch/roll values.
     * Those values lag and are unavailable when the vehicle is upside down. */
    bool b_is_lift_tilt =
        b_is_near_gravity &&
        (p_accel_g[2] <=
         (0.9063078f * norm)); /* The lift tilt gate starts at 25 degrees. */
    const float32_t horizontal_sq =
        (p_accel_g[0] * p_accel_g[0]) + (p_accel_g[1] * p_accel_g[1]);
    const float32_t upward_excess = p_accel_g[2] - 1.0f;
    const bool      b_is_gentle_pickup =
        (p_accel_g[2] >= 1.10f) && (horizontal_sq <= (0.15f * 0.15f));
    /* For a firm pickup, upward acceleration above 1 g must be at least
     * twice the horizontal acceleration. Compare squared magnitudes. */
    const bool b_is_firm_pickup =
        (p_accel_g[2] >= 1.30f) &&
        ((upward_excess * upward_excess) >= (4.0f * horizontal_sq));
    const bool b_is_upward = b_is_gentle_pickup || b_is_firm_pickup;
#if APP_IMU_DEBUG
    p_monitor->b_is_moving_upward = b_is_upward;
    p_monitor->b_has_lift_tilt    = b_is_lift_tilt;
#endif
    if (!b_is_continuous)
    {
        p_monitor->b_is_safety_tracking = false;
    }

    if (b_is_tipped || (norm < 0.3f))
    {
        if (b_is_tipped)
        {
            p_monitor->safety = TERRAIN_SAFETY_OVERTURNED;
        }
        else if (!b_is_recovering)
        {
            p_monitor->safety = TERRAIN_SAFETY_LIFTED;
        }
        else
        {
            /* Keep the current safety state. */
        }

        p_monitor->b_is_safety_tracking = false;
    }
    else if (!(b_is_recovering ? b_is_settled
                               : (b_is_lift_tilt || b_is_upward)))
    {
        p_monitor->b_is_safety_tracking = false;
    }
    else if (!p_monitor->b_is_safety_tracking)
    {
        p_monitor->b_is_safety_tracking = true;
        p_monitor->safety_since_us      = now;
    }
    else if ((now - p_monitor->safety_since_us) >=
             (b_is_recovering ? SAFETY_RECOVERY_US : SAFETY_LIFT_CONFIRM_US))
    {
        /* clang-format off */
        p_monitor->safety               =
            b_is_recovering ? TERRAIN_SAFETY_NONE : TERRAIN_SAFETY_LIFTED;
        p_monitor->b_is_safety_tracking = false;
        /* clang-format on */
    }
    else
    {
        /* Keep timing this safety transition. */
    }
}

/** @brief Append the low width bits of a value in little-endian bit order and
 * advance the caller's bit position.
 */
static void put_bits (uint8_t * p_out, unsigned * p_bit, uint32_t value,
                      unsigned width)
{
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        if (value & (1UL << bit_offset))
        {
            p_out[*p_bit / 8U] =
                (uint8_t)((unsigned)p_out[*p_bit / 8U] | (1U << (*p_bit % 8U)));
        }

        ++bit_offset;
        ++*p_bit;
    }
}

/** @brief Read an unsigned field from the supplied packet and advance the
 * caller's bit position by its width.
 */
static uint32_t get_bits (const uint8_t * p_packet, unsigned * p_bit,
                          unsigned width)
{
    uint32_t field      = 0;
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        field |= (uint32_t)((p_packet[*p_bit / 8U] >> (*p_bit % 8U)) & 1U)
                 << bit_offset;
        ++bit_offset;
        ++*p_bit;
    }

    return field;
}

/** @brief Decode a bounded two's-complement field of the specified width as a
 * signed integer.
 */
static int32_t signed_value (uint32_t encoded, unsigned width)
{
    return (encoded & (1U << (width - 1U)))
               ? (int32_t)encoded - (int32_t)(1U << width)
               : (int32_t)encoded;
}

/* End of file. */


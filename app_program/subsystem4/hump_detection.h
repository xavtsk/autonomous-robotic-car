/** @file hump_detection.h
 * @brief Track hump progress and height for the configured wheel geometry.
 */
#ifndef BUDDY4_HUMP_DETECTION_H
#define BUDDY4_HUMP_DETECTION_H
#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#if APP_IMU_DEBUG
typedef enum
{
    HUMP_DEBUG_ARMED     = 1U << 0,
    HUMP_DEBUG_STARTED   = 1U << 1,
    HUMP_DEBUG_DISCARDED = 1U << 2,
    HUMP_DEBUG_COMPLETED = 1U << 3,
    HUMP_DEBUG_SKIPPED   = 1U << 4,
    HUMP_DEBUG_ACTIVE    = 1U << 5,
    HUMP_DEBUG_IS_ARMED  = 1U << 6
} hump_detection_debug_flag_t;

typedef struct
{
    float32_t wheel_delta_m;
    float32_t travel_m;
    float32_t candidate_peak_m;
    bool      b_is_wheel_delta_valid;
    uint8_t   flags;
    uint8_t   reason;
} hump_detection_debug_t;
#endif
/* Values carried by version 2 of the IMU debug stream. */
typedef enum
{
    HUMP_REASON_NONE                         = 0,
    HUMP_REASON_INVALID_IMU                  = 1,
    HUMP_REASON_IMU_GAP                      = 2,
    HUMP_REASON_INVALID_TILT                 = 3,
    HUMP_REASON_ODOMETRY_UNAVAILABLE         = 4,
    HUMP_REASON_ODOMETRY_BASELINE            = 5,
    HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE   = 6,
    HUMP_REASON_SAFETY_OR_IMPACT             = 7,
    HUMP_REASON_TURN_RATE                    = 8,
    HUMP_REASON_ACCELERATION_SPIKE           = 9,
    HUMP_REASON_REVERSE_TRAVEL               = 10,
    HUMP_REASON_INVALID_GEOMETRY             = 11,
    HUMP_REASON_CROSSING_TIMEOUT             = 12,
    HUMP_REASON_LEFT_WHEEL_RAISED            = 13,
    HUMP_REASON_WAITING_FOR_LEVEL            = 14,
    HUMP_REASON_BELOW_START_HEIGHT           = 15,
    HUMP_REASON_NO_FORWARD_TRAVEL            = 16,
    HUMP_REASON_INSUFFICIENT_CROSSING_TRAVEL = 17,
    HUMP_REASON_CALIBRATION_PENDING          = 18
} hump_detection_reason_t;

typedef struct
{
    float32_t height_m;
    float32_t peak_m;
    float32_t highest_m;
    float32_t travel_m;
    uint32_t  started_us;
    unsigned  level_samples;
    bool      b_is_hump_tracking;
    bool      b_has_descended;
    bool      b_is_peak_valid;
    bool      b_has_completed_hump;
    bool      b_is_armed;
    bool      b_did_skip_acceleration;
#if APP_IMU_DEBUG
    hump_detection_debug_t debug;
#endif
} hump_detection_t;

typedef enum
{
    HUMP_SAMPLE_INVALID,
    HUMP_SAMPLE_VALID,
    HUMP_SAMPLE_ACCELERATION_SPIKE
} hump_detection_sample_quality_t;

/**
 * @brief Clear all crossing, historical-peak and optional debug state.
 * @param[out] p_hump Non-NULL caller-owned detector with exclusive write
 * access.
 * @note Use this reset or equivalent zero initialization before first use.
 * This module does not acquire locks or retain pointers; the caller serializes
 * resets, updates and reads of each detector.
 */
void hump_detection_reset (hump_detection_t * p_hump);
/* Caller supplies signed distance delta, pitch positive nose-up. Only complete
 * ascent/descent cycles on valid, low-dynamic, forward travel become run peaks.
 */
/**
 * @brief Advance the pitch-integrated ascent/descent detector by one sample.
 * @param[in,out] p_hump Non-NULL initialized detector with exclusive access.
 * @param pitch Nose-up pitch in degrees; finite with magnitude at most 40.
 * @param step_distance_m Forward travel since the last sample, finite in
 * 0..0.05 m.
 * @param b_is_valid Caller measurement-quality gate; false discards this
 * crossing.
 * @param now Wrapping microsecond clock; active crossings expire after 10 s.
 * @note Clears completed on every call. Invalid measurements clear active
 * progress while retaining the highest completed peak. Acceptance needs an
 * ascent, descent and five level samples; callers read completion from p_hump.
 * This update does not populate right-wheel diagnostic fields.
 */
void hump_detection_update (hump_detection_t * p_hump, float32_t pitch,
                            float32_t step_distance_m, bool b_is_valid,
                            uint32_t now);
/* Right wheel above a level left wheel: height = track*sin(roll)*cos(pitch).
 * Positive roll raises the right side. Require forward travel and a complete
 * level -> raised -> level excursion; do not integrate pitch in this mode. */
/**
 * @brief Advance the right-wheel geometry detector and optional debug record.
 * @param[in,out] p_hump Non-NULL initialized detector with exclusive access.
 * @param pitch Nose-up pitch in degrees; finite with magnitude at most 40.
 * @param roll Right-side-up roll in degrees; finite with magnitude below 10.
 * @param track_m Wheel-track width, finite and greater than 0 up to 0.5 m.
 * @param step_distance_m Forward wheel travel since the last sample, 0..0.05 m.
 * @param quality Caller quality gate; one acceleration spike may be held during
 * an active crossing. INVALID discards progress; other values use the valid
 * path except the explicitly handled ACCELERATION_SPIKE value.
 * @param now Wrapping microsecond clock; active crossings expire after 10 s.
 * @note Clears completed each call and retains the highest completed peak on
 * rejection. Level samples arm/rearm the detector; completion also requires
 * forward travel while raised. Read results from p_hump. With APP_IMU_DEBUG,
 * reason, travel and transition flags are updated after the same state-machine
 * step.
 */
void hump_detection_update_right (hump_detection_t * p_hump, float32_t pitch,
                                  float32_t roll, float32_t track_m,
                                  float32_t step_distance_m,
                                  hump_detection_sample_quality_t quality,
                                  uint32_t                        now);
#endif

/* End of file. */


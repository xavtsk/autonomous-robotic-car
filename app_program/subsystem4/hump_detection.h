#ifndef BUDDY4_HUMP_DETECTION_H
#define BUDDY4_HUMP_DETECTION_H
#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#if APP_IMU_DEBUG
enum
{
    HUMP_DEBUG_ARMED = 1U << 0,
    HUMP_DEBUG_STARTED = 1U << 1,
    HUMP_DEBUG_DISCARDED = 1U << 2,
    HUMP_DEBUG_COMPLETED = 1U << 3,
    HUMP_DEBUG_SKIPPED = 1U << 4,
    HUMP_DEBUG_ACTIVE = 1U << 5,
    HUMP_DEBUG_IS_ARMED = 1U << 6
};

typedef struct
{
    float32_t wheel_delta_m, travel_m;
    bool wheel_delta_valid;
    uint8_t flags, reason;
} hump_debug_t;
#endif
/* Values carried by version 2 of the IMU debug stream. */
typedef enum
{
    HUMP_REASON_NONE = 0,
    HUMP_REASON_INVALID_IMU = 1,
    HUMP_REASON_IMU_GAP = 2,
    HUMP_REASON_INVALID_TILT = 3,
    HUMP_REASON_ODOMETRY_UNAVAILABLE = 4,
    HUMP_REASON_ODOMETRY_BASELINE = 5,
    HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE = 6,
    HUMP_REASON_SAFETY_OR_IMPACT = 7,
    HUMP_REASON_TURN_RATE = 8,
    HUMP_REASON_ACCELERATION_SPIKE = 9,
    HUMP_REASON_REVERSE_TRAVEL = 10,
    HUMP_REASON_INVALID_GEOMETRY = 11,
    HUMP_REASON_CROSSING_TIMEOUT = 12,
    HUMP_REASON_LEFT_WHEEL_RAISED = 13,
    HUMP_REASON_WAITING_FOR_LEVEL = 14,
    HUMP_REASON_BELOW_START_HEIGHT = 15,
    HUMP_REASON_NO_FORWARD_TRAVEL = 16,
    HUMP_REASON_INSUFFICIENT_CROSSING_TRAVEL = 17,
    HUMP_REASON_CALIBRATION_PENDING = 18
} hump_reason_t;

typedef struct
{
    float32_t height_m, peak_m, highest_m, travel_m;
    uint32_t started_us;
    unsigned level_samples;
    bool active, descended, highest_valid, completed, armed,
        skipped_acceleration;
#if APP_IMU_DEBUG
    hump_debug_t debug;
#endif
} hump_detector_t;

typedef enum
{
    HUMP_SAMPLE_INVALID,
    HUMP_SAMPLE_VALID,
    HUMP_SAMPLE_ACCELERATION_SPIKE
} hump_sample_quality_t;

void hump_reset (hump_detector_t * p_h);
/* Caller supplies signed distance delta, pitch positive nose-up. Only complete
 * ascent/descent cycles on valid, low-dynamic, forward travel become run peaks.
 */
void hump_update (hump_detector_t * p_h, float32_t pitch, float32_t distance_m,
                  bool valid, uint32_t now);
/* Right wheel above a level left wheel: height = track*sin(roll)*cos(pitch).
 * Positive roll raises the right side. Require forward travel and a complete
 * level -> raised -> level excursion; do not integrate pitch in this mode. */
void hump_update_right_wheel (hump_detector_t * p_h, float32_t pitch, float32_t roll,
                              float32_t track_m, float32_t distance_m,
                              hump_sample_quality_t quality, uint32_t now);
#endif

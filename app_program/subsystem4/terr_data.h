#ifndef BUDDY4_TERRAIN_MONITOR_H
#define BUDDY4_TERRAIN_MONITOR_H
#include "imu_calibration.h"
#include "hump_detection.h"
#include "events_motion.h"
#define TERRAIN_PACKET_SIZE 12U
/* Flags describe validity, not proof of calibrated physical accuracy. */
#define TERRAIN_TILT_VALID 1U
#define TERRAIN_TURN_VALID 2U
#define TERRAIN_PEAK_VALID 4U
#define TERRAIN_CALIBRATED 8U
#define TERRAIN_FAULT 16U

typedef enum
{
    TERRAIN_HUMP_LONGITUDINAL = 0,
    TERRAIN_HUMP_RIGHT_WHEEL
} terrain_hump_geometry_t;

typedef enum
{
    TERRAIN_SAFETY_NONE = 0,
    TERRAIN_SAFETY_LIFTED,
    TERRAIN_SAFETY_OVERTURNED
} terrain_safety_t;

typedef struct
{
    uint32_t timestamp_us;
    float pitch_deg, roll_deg, turn_dps, peak_m;
    float hump_height_m; /* current height on a hump in progress (0..0.5 m) */
    terrain_state_t state;
    terrain_safety_t safety;
    uint8_t flags;
    bool impact, hump_completed, hump_active;
} terrain_result_t;

typedef struct
{
    double distance_m;
    uint32_t epoch;
    bool valid, stationary;
} terrain_odometry_t;

typedef struct
{
    imu_calibration_t calibration;
    hump_detector_t hump;
    terrain_result_t result;
    double previous_distance;
    float filtered[3], heading, mag_offset[3], mag_scale[3];
    uint32_t previous_us, heading_us, epoch, safety_since_us;
    unsigned stable_count;
    terrain_state_t candidate;
    terrain_hump_geometry_t hump_geometry;
    terrain_safety_t safety;
    float wheel_track_m;
    bool seen, odometry_seen, heading_seen, magnetic_calibrated,
        safety_tracking;
#if APP_IMU_DEBUG
    bool safety_upward, safety_lift_tilt;
#endif
} terrain_monitor_t;

void terrain_monitor_init (terrain_monitor_t * p_t);
/* Portable default is longitudinal. Right-wheel mode assumes the left wheel
 * stays on level ground; track is the wheel-centre spacing in metres.
 * Selecting a geometry clears run statistics and retains IMU calibration. */
bool terrain_monitor_set_geometry (terrain_monitor_t * p_t,
                                   terrain_hump_geometry_t geometry,
                                   float track_m);
/* New run clears peaks/history but retains calibration and an active safety
 * trip. Owner task only. */
void terrain_monitor_reset_run (terrain_monitor_t * p_t);
/* Vehicle-axis hard-iron offsets and diagonal soft-iron scale. Must come from
 * measured calibration; calling this asserts calibration was performed. */
bool terrain_monitor_set_mag_cal (terrain_monitor_t * p_t,
                                  const float p_offset[3],
                                  const float p_scale[3]);
void terrain_monitor_process (terrain_monitor_t * p_t, const imu_sample_t * p_s,
                              const terrain_odometry_t * p_odo);
/* Explicit bit packing, never C struct memcpy. See DATA_FORMAT.md. */
void terrain_pack (const terrain_result_t * p_r,
                   uint8_t p_out[TERRAIN_PACKET_SIZE]);
void terrain_unpack (const uint8_t p_in[TERRAIN_PACKET_SIZE],
                     terrain_result_t * p_r);
#endif

/** @file terr_data.h
 * @brief Maintain terrain monitoring state and pack copied terrain
 * observations.
 */
#ifndef BUDDY4_TERRAIN_MONITOR_H
#define BUDDY4_TERRAIN_MONITOR_H
#include "../app_types.h"
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
} terr_data_hump_geometry_t;

typedef enum
{
    TERRAIN_SAFETY_NONE = 0,
    TERRAIN_SAFETY_LIFTED,
    TERRAIN_SAFETY_OVERTURNED
} terr_data_safety_t;

typedef struct
{
    uint32_t  timestamp_us;
    float32_t pitch_deg;
    float32_t roll_deg;
    float32_t turn_dps;
    float32_t peak_m;
    float32_t hump_height_m; /* Report current climb between 0 and 0.5 m. */
    events_motion_state_t state;
    terr_data_safety_t    safety;
    uint8_t               flags;
    bool                  b_has_impact;
    bool                  b_is_hump_complete;
    bool                  b_is_hump_active;
} terr_data_result_t;

typedef struct
{
    float64_t distance_m;
    uint32_t  epoch;
    bool      b_is_valid;
    bool      b_is_stationary;
} terr_data_odometry_t;

typedef struct
{
    imu_calibration_t         calibration;
    hump_detection_t          hump;
    terr_data_result_t        result;
    float64_t                 previous_distance;
    float32_t                 filtered[3];
    float32_t                 heading;
    float32_t                 mag_offset[3];
    float32_t                 mag_scale[3];
    uint32_t                  previous_us;
    uint32_t                  heading_us;
    uint32_t                  epoch;
    uint32_t                  safety_since_us;
    unsigned                  stable_count;
    events_motion_state_t     candidate;
    terr_data_hump_geometry_t hump_geometry;
    terr_data_safety_t        safety;
    float32_t                 wheel_track_m;
    bool                      b_has_imu_sample;
    bool                      b_has_terrain_odometry;
    bool                      b_has_seen_heading;
    bool                      b_is_magnetic_calibrated;
    bool                      b_is_safety_tracking;
#if APP_IMU_DEBUG
    bool b_is_moving_upward;
    bool b_has_lift_tilt;
#endif
} terr_data_monitor_t;

void terr_data_init (terr_data_monitor_t * p_monitor);
/* Portable default is longitudinal. Right-wheel mode assumes the left wheel
 * stays on level ground; track is the wheel-centre spacing in metres.
 * Selecting a geometry clears run statistics and retains IMU calibration. */
bool terr_data_set_geometry (terr_data_monitor_t *     p_monitor,
                             terr_data_hump_geometry_t geometry,
                             float32_t                 track_m);
/* New run clears peaks/history but retains calibration and an active safety
 * trip. Owner task only. */
void terr_data_reset_run (terr_data_monitor_t * p_monitor);
/* Vehicle-axis hard-iron offsets and diagonal soft-iron scale. Must come from
 * measured calibration; calling this asserts calibration was performed. */
bool terr_data_set_mag_cal (terr_data_monitor_t * p_monitor,
                            const float32_t       p_offset[3],
                            const float32_t       p_scale[3]);
void terr_data_process (terr_data_monitor_t *        p_monitor,
                        const imu_sensor_sample_t *  p_sample,
                        const terr_data_odometry_t * p_odometry);
/* Explicit bit packing, never C struct memcpy. See DATA_FORMAT.md. */
void terr_data_pack (const terr_data_result_t * p_result,
                     uint8_t                    p_out[TERRAIN_PACKET_SIZE]);
void terr_data_unpack (const uint8_t        p_in[TERRAIN_PACKET_SIZE],
                       terr_data_result_t * p_result);
#endif

/* End of file. */


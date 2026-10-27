#ifndef BUDDY4_TERRAIN_MONITOR_H
#define BUDDY4_TERRAIN_MONITOR_H
#include "imu_calibration.h"
#include "hump_detection.h"
#include "motion_events.h"
#define TERRAIN_PACKET_SIZE 11U
/* Flags describe validity, not proof of calibrated physical accuracy. */
#define TERRAIN_TILT_VALID 1U
#define TERRAIN_TURN_VALID 2U
#define TERRAIN_PEAK_VALID 4U
#define TERRAIN_CALIBRATED 8U
#define TERRAIN_FAULT 16U
typedef struct {
    uint32_t timestamp_us;
    float pitch_deg, roll_deg, turn_dps, peak_m;
    terrain_state_t state;
    uint8_t flags;
    bool impact, hump_completed;
} terrain_result_t;
typedef struct {
    double distance_m;
    uint32_t epoch;
    bool valid, stationary;
} terrain_odometry_t;
typedef struct {
    imu_calibration_t calibration;
    hump_detector_t hump;
    terrain_result_t result;
    double previous_distance;
    float filtered[3], heading, mag_offset[3], mag_scale[3];
    uint32_t previous_us, heading_us, epoch;
    unsigned stable_count;
    terrain_state_t candidate;
    bool seen, odometry_seen, heading_seen, magnetic_calibrated;
} terrain_monitor_t;
void terrain_monitor_init(terrain_monitor_t *t);
/* New run clears peaks/history but retains calibration. Owner task only. */
void terrain_monitor_reset_run(terrain_monitor_t *t);
/* Vehicle-axis hard-iron offsets and diagonal soft-iron scale. Must come from
 * measured calibration; calling this asserts calibration was performed. */
bool terrain_monitor_set_magnetic_calibration(terrain_monitor_t *t,const float offset[3],const float scale[3]);
void terrain_monitor_process(terrain_monitor_t *t,const imu_sample_t *s,const terrain_odometry_t *odo);
/* Explicit bit packing, never C struct memcpy. See DATA_FORMAT.md. */
void terrain_pack(const terrain_result_t *r,uint8_t out[TERRAIN_PACKET_SIZE]);
void terrain_unpack(const uint8_t in[TERRAIN_PACKET_SIZE], terrain_result_t *r);
#endif

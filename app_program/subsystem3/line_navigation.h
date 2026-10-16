/* Buddy 3 integration boundary. Pure C; no motor, MQTT or RTOS dependencies.
 * Context is single-owner: the future controller must pass mode/config changes
 * through the sampling owner, never modify this object concurrently. */
#ifndef BUDDY3_LINE_NAVIGATION_H
#define BUDDY3_LINE_NAVIGATION_H
#include "ir_acquisition.h"
#include "ir_calibration.h"
#include "line_controller.h"
#include "barcode_decoder.h"
typedef enum { S3_NAV_NONE=0,S3_NAV_LEFT,S3_NAV_RIGHT,S3_NAV_STRAIGHT,S3_NAV_UTURN,
               S3_NAV_ACTION_COUNT } s3_nav_action_t;
typedef enum { S3_MODE_TRACK=0,S3_MODE_SUPPRESSED } s3_processing_mode_t;
typedef enum {
    S3_EVENT_BARCODE=1U, S3_EVENT_JUNCTION=2U, S3_EVENT_LINE_LOST=4U,
    S3_EVENT_LINE_REACQUIRED=8U, S3_EVENT_SENSOR_FAULT=16U
} s3_event_flag_t;
typedef struct {
    s3_acquisition_config_t acquisition;
    ir_calibration_t calibration[S3_SENSOR_COUNT];
    s3_line_config_t line;
    s3_steering_config_t steering;
    s3_barcode_config_t barcode;
    s3_nav_action_t map[26];
    uint32_t max_sample_age_us;
} s3_navigation_config_t;
typedef struct {
    bool valid;
    uint16_t darkness;
    ir_surface_t surface;
} s3_processed_sensor_t;
typedef struct {
    uint32_t timestamp_us, sample_gap_us;
    uint32_t events; /* Per-call edge events: consumer must process every call. */
    uint8_t fault_mask;
    s3_processed_sensor_t sensor[S3_SENSOR_COUNT];
    s3_line_observation_t line;
    s3_steering_request_t steering;
    s3_barcode_status_t barcode_status;
    s3_barcode_result_t barcode;
    s3_nav_action_t action;
} s3_navigation_output_t;
typedef struct {
    s3_navigation_config_t config;
    ir_reflectance_t sensor[S3_SENSOR_COUNT];
    s3_line_detector_t line;
    s3_barcode_decoder_t barcode;
    uint32_t last_us, sensor_last_us[S3_SENSOR_COUNT];
    uint8_t seen_sensors, faults;
    s3_processing_mode_t mode;
    bool configured, seen_frame;
} s3_navigation_t;
/* GP26 barcode enabled only. Left GP27/right GP28 require explicit enablement.
 * References and algorithm settings are development defaults, not validation. */
s3_navigation_config_t s3_navigation_defaults(void);
bool s3_navigation_init(s3_navigation_t *context, const s3_navigation_config_t *config);
bool s3_navigation_set_mode(s3_navigation_t *context, s3_processing_mode_t mode);
bool s3_navigation_apply_calibration(s3_navigation_t *context, s3_sensor_id_t id,
                                     const s3_calibration_collector_t *collector);
bool s3_navigation_process(s3_navigation_t *context, const s3_sensor_frame_t *frame,
                            s3_navigation_output_t *output);
const char *s3_surface_name(ir_surface_t surface);
const char *s3_action_name(s3_nav_action_t action);
#endif

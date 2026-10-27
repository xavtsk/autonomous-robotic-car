/* Buddy 3 integration boundary. Pure C; no motor, MQTT or RTOS dependencies.
 * Context is single-owner: the future controller must pass mode/config changes
 * through the sampling owner, never modify this object concurrently. */
#ifndef BUDDY3_LINE_NAVIGATION_H
#define BUDDY3_LINE_NAVIGATION_H
#include "ir_acquisition.h"
#include "ir_calibration.h"
#include "line_controller.h"
#include "barcode_decoder.h"
typedef enum
{
    S3_NAV_NONE = 0,
    S3_NAV_LEFT,
    S3_NAV_RIGHT,
    S3_NAV_STRAIGHT,
    S3_NAV_UTURN,
    S3_NAV_ACTION_COUNT
} s3_nav_action_t;
typedef enum
{
    S3_MODE_TRACK = 0,
    S3_MODE_SUPPRESSED
} s3_processing_mode_t;
typedef enum
{
    S3_RUN_INACTIVE = 0,
    S3_RUN_ACTIVE,
    S3_RUN_FAULT_LATCHED
} s3_run_state_t;
typedef enum
{
    S3_READY_CHANNELS = 1U,
    S3_READY_CALIBRATION = 2U,
    S3_READY_SENSORS = 4U,
    S3_READY_ODOMETRY = 8U,
    S3_READY_FAULT = 16U
} s3_readiness_flag_t;
typedef enum
{
    S3_FAULT_SENSOR = 1U,
    S3_FAULT_TIMING = 2U,
    S3_FAULT_ODOMETRY = 4U,
    S3_FAULT_ODOMETRY_EPOCH = 8U
} s3_fault_flag_t;
/* Signed centre displacement, not accumulated absolute wheel travel. The
 * producer changes epoch whenever its distance counter is reset. Timestamp uses
 * the same wrapping microsecond clock as the sensor frame. */
typedef struct
{
    double centre_m;
    uint32_t timestamp_us, epoch;
    bool valid;
} s3_odometry_sample_t;
typedef struct
{
    double start_m, end_m;
    uint32_t epoch;
    bool start_valid, end_valid;
} s3_capture_distance_t;
typedef enum
{
    S3_EVENT_BARCODE = 1U,
    S3_EVENT_JUNCTION = 2U,
    S3_EVENT_LINE_LOST = 4U,
    S3_EVENT_LINE_REACQUIRED = 8U,
    S3_EVENT_SENSOR_FAULT = 16U
} s3_event_flag_t;
typedef struct
{
    s3_acquisition_config_t acquisition;
    ir_calibration_t calibration[S3_SENSOR_COUNT];
    s3_line_config_t line;
    s3_steering_config_t steering;
    s3_barcode_config_t barcode;
    s3_nav_action_t map[26];
    uint32_t max_sample_age_us;
    uint32_t rearm_stable_us, fault_clear_us;
    double rearm_distance_m;
} s3_navigation_config_t;
typedef struct
{
    bool valid;
    uint16_t darkness;
    ir_surface_t surface;
} s3_processed_sensor_t;
typedef struct
{
    uint32_t timestamp_us, sample_gap_us;
    /* Per-call edge events: consumer must process every call. */
    uint32_t events;
    uint8_t fault_mask;
    s3_processed_sensor_t sensor[S3_SENSOR_COUNT];
    s3_line_observation_t line;
    /* Actionable only when active and healthy. */
    s3_steering_request_t steering;
    s3_steering_request_t steering_preview; /* Never a motor request. */
    s3_run_state_t lifecycle;
    uint32_t readiness_blockers, latched_faults;
    uint8_t accepted_calibration;
    bool stop_requested, odometry_available, rearming, diagnostic;
    s3_capture_distance_t capture_distance;
    s3_barcode_status_t barcode_status;
    s3_barcode_result_t barcode;
    s3_nav_action_t action;
} s3_navigation_output_t;
typedef struct
{
    s3_navigation_config_t config;
    ir_reflectance_t sensor[S3_SENSOR_COUNT];
    s3_line_detector_t line;
    s3_barcode_decoder_t barcode;
    uint32_t last_us, sensor_last_us[S3_SENSOR_COUNT];
    uint16_t seen_sensors : 3;
    uint16_t mode : 1;
    uint16_t configured : 1;
    uint16_t seen_frame : 1;
    uint16_t lifecycle : 2;
    uint16_t accepted_calibration : 3;
    uint16_t valid_sensors : 3;
    uint8_t current_faults : 4;
    uint8_t latched_faults : 4;
    s3_odometry_sample_t odometry;
    uint8_t odometry_available : 1;
    uint8_t seen_odometry : 1;
    uint8_t healthy_tracking : 1;
    uint32_t healthy_since_us;
    uint8_t rearming : 1;
    uint8_t rearm_anchor_valid : 1;
    uint8_t rearm_stable : 1;
    double rearm_anchor_m;
    uint32_t rearm_epoch, rearm_since_us;
    s3_capture_distance_t capture_distance;
} s3_navigation_t;
/* Barcode input only enabled (pins: S3_ADC_CHANNEL_* in ir_acquisition.h).
 * Left/right require explicit enablement.
 * References and algorithm settings are development defaults, not validation.
 */
s3_navigation_config_t s3_navigation_defaults(void);
bool s3_navigation_init(s3_navigation_t *context,
                        const s3_navigation_config_t *config);
bool s3_navigation_set_mode(s3_navigation_t *context,
                            s3_processing_mode_t mode);
bool s3_navigation_apply_calibration(
    s3_navigation_t *context, s3_sensor_id_t id,
    const s3_calibration_collector_t *collector);
/* All control calls and processing must run through the same owner. now_us must
 * share the frame clock. Readiness is evaluated again at start/reset time. */
uint32_t s3_navigation_readiness(const s3_navigation_t *context,
                                 uint32_t now_us);
bool s3_navigation_start(s3_navigation_t *context, uint32_t now_us);
bool s3_navigation_end_run(s3_navigation_t *context);
bool s3_navigation_reset_fault(s3_navigation_t *context, uint32_t now_us);
bool s3_navigation_set_mapping(s3_navigation_t *context,
                               const s3_nav_action_t map[26]);
/* NULL odometry is permitted for diagnostics; it faults an active run. Each
 * call replaces the latest input, so NULL never silently reuses a previous
 * distance. */
bool s3_navigation_process(s3_navigation_t *context,
                           const s3_sensor_frame_t *frame,
                           const s3_odometry_sample_t *odometry,
                           s3_navigation_output_t *output);
const char *s3_surface_name(ir_surface_t surface);
const char *s3_action_name(s3_nav_action_t action);
#endif

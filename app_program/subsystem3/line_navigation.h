/** @file line_navigation.h
 * @brief Define single-owner navigation state and copied sensor, odometry and
 * result interfaces.
 */
/* Buddy 3 integration boundary. Pure C; no motor, MQTT or RTOS dependencies.
 * Context is single-owner: the future controller must pass mode/config changes
 * through the sampling owner, never modify this object concurrently. */
#ifndef BUDDY3_LINE_NAVIGATION_H
#define BUDDY3_LINE_NAVIGATION_H
#include "../app_types.h"
#include "ir_acquisition.h"
#include "ir_calibration.h"
#include "line_controller.h"
#include "bc_decoder.h"

typedef enum
{
    S3_NAV_NONE = 0,
    S3_NAV_LEFT,
    S3_NAV_RIGHT,
    S3_NAV_STRAIGHT,
    S3_NAV_UTURN,
    S3_NAV_ACTION_COUNT
} line_navigation_action_t;

typedef enum
{
    S3_MODE_TRACK = 0,
    S3_MODE_SUPPRESSED
} line_navigation_mode_t;

typedef enum
{
    S3_RUN_INACTIVE = 0,
    S3_RUN_ACTIVE,
    S3_RUN_FAULT_LATCHED
} line_navigation_run_state_t;

typedef enum
{
    S3_READY_CHANNELS    = 1U,
    S3_READY_CALIBRATION = 2U,
    S3_READY_SENSORS     = 4U,
    S3_READY_ODOMETRY    = 8U,
    S3_READY_FAULT       = 16U
} line_navigation_blocker_t;

typedef enum
{
    S3_FAULT_SENSOR         = 1U,
    S3_FAULT_TIMING         = 2U,
    S3_FAULT_ODOMETRY       = 4U,
    S3_FAULT_ODOMETRY_EPOCH = 8U
} line_navigation_fault_flag_t;

/* Signed centre displacement, not accumulated absolute wheel travel. The
 * producer changes epoch whenever its distance counter is reset. Timestamp uses
 * the same wrapping microsecond clock as the sensor frame. */
typedef struct
{
    float64_t centre_m;
    uint32_t  timestamp_us;
    uint32_t  epoch;
    bool      b_is_valid;
} line_navigation_odometry_t;

typedef struct
{
    float64_t start_m;
    float64_t end_m;
    uint32_t  epoch;
    bool      b_is_start_valid;
    bool      b_is_end_valid;
} line_navigation_distance_t;

typedef enum
{
    S3_EVENT_BARCODE         = 1U,
    S3_EVENT_JUNCTION        = 2U,
    S3_EVENT_LINE_LOST       = 4U,
    S3_EVENT_LINE_REACQUIRED = 8U,
    S3_EVENT_SENSOR_FAULT    = 16U
} line_navigation_event_flag_t;

typedef struct
{
    ir_acquisition_config_t      acquisition;
    ir_reflectance_calibration_t calibration[S3_SENSOR_COUNT];
    line_detection_config_t      line;
    line_controller_config_t     steering;
    bc_decoder_config_t          barcode;
    line_navigation_action_t     map[S3_BARCODE_LETTERS];
    uint32_t                     max_sample_age_us;
    uint32_t                     rearm_stable_us;
    uint32_t                     fault_clear_us;
    float64_t                    rearm_distance_m;
} line_navigation_config_t;

typedef struct
{
    bool                     b_is_valid;
    uint16_t                 darkness;
    ir_reflectance_surface_t surface;
} line_navigation_sensor_t;

typedef struct
{
    uint32_t timestamp_us;
    uint32_t sample_gap_us;
    /* Per-call edge events: consumer must process every call. */
    uint32_t                     events;
    uint8_t                      fault_mask;
    line_navigation_sensor_t     sensor[S3_SENSOR_COUNT];
    line_detection_observation_t line;
    /* Actionable only when active and healthy. */
    line_controller_request_t   steering;
    line_controller_request_t   steering_preview; /* Never a motor request. */
    line_navigation_run_state_t lifecycle;
    uint32_t                    readiness_blockers;
    uint32_t                    latched_faults;
    uint8_t                     accepted_calibration;
    bool                        b_is_stop_requested;
    bool                        b_has_odometry;
    bool                        b_is_rearming;
    bool                        b_is_diagnostic;
    line_navigation_distance_t  capture_distance;
    bc_decoder_status_t         barcode_status;
    bc_decoder_result_t         barcode;
    line_navigation_action_t    action;
} line_navigation_output_t;

typedef struct
{
    line_navigation_config_t config;
    ir_reflectance_t         sensor[S3_SENSOR_COUNT];
    line_detection_t         line;
    bc_decoder_t             barcode;

    bc_decoder_record_t
             barcode_record; /* Track the separate recording decoder. */
    uint32_t last_us;
    uint32_t sensor_last_us[S3_SENSOR_COUNT];
    uint16_t seen_sensors : 3;
    uint16_t mode : 1;
    uint16_t b_is_configured : 1;
    uint16_t b_has_seen_frame : 1;
    uint16_t lifecycle : 2;
    uint16_t accepted_calibration : 3;
    uint16_t valid_sensors : 3;
    uint8_t  current_faults : 4;
    uint8_t  latched_faults : 4;

    line_navigation_odometry_t odometry;
    uint8_t                    b_has_odometry : 1;
    uint8_t                    b_has_seen_odometry : 1;
    uint8_t                    b_is_healthy_tracking : 1;
    uint32_t                   healthy_since_us;
    uint8_t                    b_is_rearming : 1;
    uint8_t                    b_is_rearm_anchor_valid : 1;
    uint8_t                    b_is_rearm_stable : 1;
    uint8_t   barcode_held : 3; /* Hold a live failure while recording. */
    float64_t rearm_anchor_m;
    uint32_t  rearm_epoch;
    uint32_t  rearm_since_us;
    line_navigation_distance_t capture_distance;
} line_navigation_t;

/* Barcode input only enabled (pins: S3_ADC_CHANNEL_* in ir_acquisition.h).
 * Left/right require explicit enablement.
 * References and algorithm settings are development defaults, not validation.
 */
line_navigation_config_t line_navigation_defaults (void);

bool line_navigation_init (line_navigation_t *              p_context,
                           const line_navigation_config_t * p_config);
bool line_navigation_set_mode (line_navigation_t *    p_context,
                               line_navigation_mode_t mode);
bool line_navigation_set_calibration (
    line_navigation_t * p_context, ir_acquisition_sensor_id_t value_id,
    const ir_calibration_collector_t * p_collector);
/* All control calls and processing must run through the same owner. now_us must
 * share the frame clock. Readiness is evaluated again at start/reset time. */
uint32_t line_navigation_readiness (const line_navigation_t * p_context,
                                    uint32_t                  now_us);
bool     line_navigation_start (line_navigation_t * p_context, uint32_t now_us);
bool     line_navigation_end_run (line_navigation_t * p_context);
bool     line_navigation_reset_fault (line_navigation_t * p_context,
                                      uint32_t            now_us);
bool     line_navigation_set_mapping (
        line_navigation_t *            p_context,
        const line_navigation_action_t p_map[S3_BARCODE_LETTERS]);
/* NULL odometry is permitted for diagnostics; it faults an active run. Each
 * call replaces the latest input, so NULL never silently reuses a previous
 * distance. */
bool line_navigation_process (line_navigation_t *                p_context,
                              const ir_acquisition_frame_t *     p_frame,
                              const line_navigation_odometry_t * p_odometry,
                              line_navigation_output_t *         p_output);

const char * line_navigation_surface_name (ir_reflectance_surface_t surface);
const char * line_navigation_action_name (line_navigation_action_t action);
#endif

/* End of file. */


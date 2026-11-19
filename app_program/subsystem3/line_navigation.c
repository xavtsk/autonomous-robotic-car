#include "line_navigation.h"
#include "../app_types.h"
#include <math.h>

s3_navigation_config_t s3_navigation_defaults (void)
{
    s3_navigation_config_t cfg = {0};
    unsigned int value_i;
    cfg.acquisition = (s3_acquisition_config_t){
        {S3_ADC_CHANNEL_BARCODE, S3_ADC_CHANNEL_LEFT, S3_ADC_CHANNEL_RIGHT},
        1U};
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        cfg.calibration[value_i] = (ir_calibration_t){150U, 280U, 400U, 600U};
    }
    cfg.line = s3_line_defaults();
    cfg.steering = s3_steering_defaults();
    cfg.barcode = s3_barcode_defaults();
    cfg.map[0] = S3_NAV_LEFT;
    cfg.map[1] = S3_NAV_RIGHT;
    cfg.map[2] = S3_NAV_STRAIGHT;
    cfg.map[3] = S3_NAV_UTURN;
    cfg.max_sample_age_us = 30000U;
    cfg.rearm_stable_us = 30000U;
    cfg.fault_clear_us = 100000U;
    cfg.rearm_distance_m = 0.020;
    return cfg;
}

bool s3_navigation_init (s3_navigation_t * p_context,
                         const s3_navigation_config_t * p_config)
{
    unsigned int value_i;
    if (p_context == 0)
    {
        return false;
    }
    *p_context = (s3_navigation_t){0};
    if ((p_config == 0) ||
        !s3_acquisition_config_valid(&p_config->acquisition) ||
        !s3_steering_config_valid(&p_config->steering) ||
        !s3_line_init(&p_context->line, &p_config->line) ||
        !s3_barcode_init(&p_context->barcode, &p_config->barcode) ||
        (p_config->max_sample_age_us == 0U) ||
        (p_config->max_sample_age_us >= 0x80000000U) ||
        (p_config->rearm_stable_us == 0U) ||
        (p_config->rearm_stable_us >= 0x80000000U) ||
        (p_config->fault_clear_us == 0U) ||
        (p_config->fault_clear_us >= 0x80000000U) ||
        !isfinite(p_config->rearm_distance_m) ||
        (p_config->rearm_distance_m <= 0.0))
    {
        return false;
    }
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        if (!ir_reflectance_init(&p_context->sensor[value_i],
                                 &p_config->calibration[value_i]))
        {
            return false;
        }
    }
    for (value_i = 0U; value_i < 26U; value_i++)
    {
        if ((unsigned int)p_config->map[value_i] >= S3_NAV_ACTION_COUNT)
        {
            return false;
        }
    }
    p_context->config = *p_config;
    p_context->mode = S3_MODE_TRACK;
    p_context->configured = true;
    return true;
}

/* Both barcode readers restart together so one barcode never gives two letters.
 */
static void reset_barcode (s3_navigation_t * p_context)
{
    s3_barcode_reset(&p_context->barcode);
    s3_barcode_record_reset(&p_context->barcode_record);
    p_context->barcode_held = 0U;
}

static void begin_rearm (s3_navigation_t * p_context)
{
    p_context->rearming = true;
    p_context->rearm_stable = false;
    p_context->rearm_anchor_valid = p_context->odometry_available;
    p_context->rearm_anchor_m = p_context->odometry.centre_m;
    p_context->rearm_epoch = p_context->odometry.epoch;
    reset_barcode(p_context);
}

bool s3_navigation_set_mode (s3_navigation_t * p_context,
                             s3_processing_mode_t mode)
{
    if ((p_context == 0) || !p_context->configured ||
        ((mode != S3_MODE_TRACK) && (mode != S3_MODE_SUPPRESSED)))
    {
        return false;
    }
    if (p_context->mode != mode)
    {
        s3_line_reset(&p_context->line);
        reset_barcode(p_context);
        p_context->capture_distance = (s3_capture_distance_t){0};
        if (p_context->lifecycle == S3_RUN_ACTIVE)
        {
            begin_rearm(p_context);
        }
        p_context->mode = mode;
    }
    return true;
}

bool s3_navigation_apply_calibration (
    s3_navigation_t * p_context, s3_sensor_id_t value_id,
    const s3_calibration_collector_t * p_collector)
{
    ir_calibration_t candidate;
    if ((p_context == 0) || !p_context->configured ||
        (p_context->lifecycle != S3_RUN_INACTIVE) ||
        ((unsigned int)value_id >= S3_SENSOR_COUNT))
    {
        return false;
    }
    candidate = p_context->config.calibration[value_id];
    if (!s3_calibration_finish(p_collector, &candidate))
    {
        return false;
    }
    p_context->config.calibration[value_id] = candidate;
    p_context->accepted_calibration |= (uint8_t)S3_SENSOR_BIT(value_id);
    p_context->valid_sensors &= (uint8_t)~S3_SENSOR_BIT(value_id);
    (void)ir_reflectance_init(&p_context->sensor[value_id], &candidate);
    s3_line_reset(&p_context->line);
    reset_barcode(p_context);
    return true;
}

bool s3_navigation_set_mapping (s3_navigation_t * p_context,
                                const s3_nav_action_t p_map[26])
{
    unsigned int value_i;
    if ((p_context == 0) || !p_context->configured ||
        (p_context->lifecycle != S3_RUN_INACTIVE) || (p_map == 0))
    {
        return false;
    }
    for (value_i = 0U; value_i < 26U; value_i++)
    {
        if ((unsigned int)p_map[value_i] >= S3_NAV_ACTION_COUNT)
        {
            return false;
        }
    }
    for (value_i = 0U; value_i < 26U; value_i++)
    {
        p_context->config.map[value_i] = p_map[value_i];
    }
    return true;
}

uint32_t s3_navigation_readiness (const s3_navigation_t * p_context,
                                  uint32_t now_us)
{
    uint32_t blockers = 0U;
    unsigned int value_i;
    if ((p_context == 0) || !p_context->configured)
    {
        return S3_READY_CHANNELS | S3_READY_CALIBRATION | S3_READY_SENSORS |
               S3_READY_ODOMETRY;
    }
    if (p_context->config.acquisition.enabled_mask != S3_ALL_SENSORS)
    {
        blockers |= S3_READY_CHANNELS;
    }
    if (p_context->accepted_calibration != S3_ALL_SENSORS)
    {
        blockers |= S3_READY_CALIBRATION;
    }
    if (!p_context->seen_frame ||
        (p_context->valid_sensors != S3_ALL_SENSORS) ||
        ((uint32_t)(now_us - p_context->last_us) >
         p_context->config.max_sample_age_us))
    {
        blockers |= S3_READY_SENSORS;
    }
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        if ((uint32_t)(now_us - p_context->sensor_last_us[value_i]) >
            p_context->config.max_sample_age_us)
        {
            blockers |= S3_READY_SENSORS;
        }
    }
    if (!p_context->odometry_available ||
        ((uint32_t)(now_us - p_context->odometry.timestamp_us) >
         p_context->config.max_sample_age_us))
    {
        blockers |= S3_READY_ODOMETRY;
    }
    if (p_context->lifecycle == S3_RUN_FAULT_LATCHED)
    {
        blockers |= S3_READY_FAULT;
    }
    return blockers;
}

bool s3_navigation_start (s3_navigation_t * p_context, uint32_t now_us)
{
    if ((p_context == 0) || !p_context->configured ||
        (p_context->lifecycle != S3_RUN_INACTIVE) ||
        (s3_navigation_readiness(p_context, now_us) != 0U))
    {
        return false;
    }
    p_context->lifecycle = S3_RUN_ACTIVE;
    s3_line_reset(&p_context->line);
    reset_barcode(p_context);
    p_context->capture_distance = (s3_capture_distance_t){0};
    p_context->rearming = false;
    return true;
}

bool s3_navigation_end_run (s3_navigation_t * p_context)
{
    if ((p_context == 0) || !p_context->configured ||
        (p_context->lifecycle == S3_RUN_FAULT_LATCHED))
    {
        return false;
    }
    p_context->lifecycle = S3_RUN_INACTIVE;
    p_context->rearming = false;
    s3_line_reset(&p_context->line);
    reset_barcode(p_context);
    p_context->capture_distance = (s3_capture_distance_t){0};
    return true;
}

bool s3_navigation_reset_fault (s3_navigation_t * p_context, uint32_t now_us)
{
    if ((p_context == 0) || !p_context->configured ||
        (p_context->lifecycle != S3_RUN_FAULT_LATCHED) ||
        ((s3_navigation_readiness(p_context, now_us) &
          ~(uint32_t)S3_READY_FAULT) != 0U) ||
        (p_context->current_faults != 0U) || !p_context->healthy_tracking ||
        ((uint32_t)(p_context->last_us - p_context->healthy_since_us) <
         p_context->config.fault_clear_us))
    {
        return false;
    }
    p_context->lifecycle = S3_RUN_INACTIVE;
    p_context->latched_faults = 0U;
    return s3_navigation_end_run(p_context);
}

/* Rearming requires continuous clear observations AND net displacement. */
static bool rearm_ready (s3_navigation_t * p_context,
                         const s3_sensor_frame_t * p_frame,
                         const s3_navigation_output_t * p_output)
{
    const s3_processed_sensor_t * p_barcode =
        &p_output->sensor[S3_SENSOR_BARCODE];
    float64_t distance;
    bool clear = p_output->line.valid && p_output->line.present &&
                 !p_output->line.junction_candidate && p_barcode->valid &&
                 (p_barcode->surface == IR_SURFACE_WHITE) &&
                 p_context->odometry_available;
    if (!clear)
    {
        p_context->rearm_stable = false;
        return false;
    }
    if (!p_context->rearm_anchor_valid)
    {
        p_context->rearm_anchor_m = p_context->odometry.centre_m;
        p_context->rearm_epoch = p_context->odometry.epoch;
        p_context->rearm_anchor_valid = true;
    }
    if (!p_context->rearm_stable)
    {
        p_context->rearm_since_us = p_frame->timestamp_us;
        p_context->rearm_stable = true;
    }
    distance = p_context->odometry.centre_m - p_context->rearm_anchor_m;
    if (distance < 0.0)
    {
        distance = -distance;
    }
    if ((p_context->odometry.epoch != p_context->rearm_epoch) ||
        ((uint32_t)(p_frame->timestamp_us - p_context->rearm_since_us) <
         p_context->config.rearm_stable_us) ||
        (distance < p_context->config.rearm_distance_m))
    {
        return false;
    }
    p_context->rearming = false;
    /* Decoder was reset at capture completion: seed a new quiet zone now.
     */
    return true;
}

static void process_barcode (s3_navigation_t * p_context,
                             const s3_sensor_frame_t * p_frame,
                             s3_navigation_output_t * p_output)
{
    const s3_processed_sensor_t * p_barcode =
        &p_output->sensor[S3_SENSOR_BARCODE];
    bool was_collecting, active = p_context->lifecycle == S3_RUN_ACTIVE;
    if ((p_context->mode != S3_MODE_TRACK) ||
        ((p_context->config.acquisition.enabled_mask &
          S3_SENSOR_BIT(S3_SENSOR_BARCODE)) == 0U))
    {
        return;
    }
    if (active && p_context->rearming &&
        !rearm_ready(p_context, p_frame, p_output))
    {
        return;
    }
    was_collecting = p_context->barcode.collecting ||
                     s3_barcode_record_active(&p_context->barcode_record);
    {
        const uint32_t value_t =
            p_frame->sensor[S3_SENSOR_BARCODE].timestamp_us;
        s3_barcode_result_t live_result, rec_result;
        s3_barcode_status_t live = s3_barcode_update(
            &p_context->barcode, value_t,
            p_barcode->valid && (p_barcode->surface != IR_SURFACE_UNKNOWN),
            p_barcode->surface == IR_SURFACE_BLACK, &live_result);
        s3_barcode_status_t rec = s3_barcode_record_update(
            &p_context->barcode_record, &p_context->config.barcode, value_t,
            p_barcode->valid, p_barcode->darkness, &rec_result);
        p_output->barcode_status = S3_BARCODE_NONE;
        if (live == S3_BARCODE_DECODED)
        { /* First reader to decode wins; the other restarts. */
            p_output->barcode_status = live;
            p_output->barcode = live_result;
            s3_barcode_record_reset(&p_context->barcode_record);
            p_context->barcode_held = 0U;
        }
        else if (rec == S3_BARCODE_DECODED)
        {
            p_output->barcode_status = rec;
            p_output->barcode = rec_result;
            s3_barcode_reset(&p_context->barcode);
            p_context->barcode_held = 0U;
        }
        else
        {
            if (live != S3_BARCODE_NONE)
            { /* e.g. a faint bar never crossed the fixed limits: let the
               * recorder, still reading this barcode, have its say first. */
                if (s3_barcode_record_active(&p_context->barcode_record))
                {
                    p_context->barcode_held = (uint8_t)live;
                }
                else
                {
                    p_output->barcode_status = live;
                }
            }
            if ((rec != S3_BARCODE_NONE) && !p_context->barcode.collecting &&
                !p_context->barcode.pending)
            { /* Recorder failed and the live decoder is idle: report. */
                p_output->barcode_status =
                    (p_context->barcode_held != 0U)
                        ? (s3_barcode_status_t)p_context->barcode_held
                        : rec;
                p_context->barcode_held = 0U;
            }
            else if ((p_context->barcode_held != 0U) &&
                     !s3_barcode_record_active(&p_context->barcode_record))
            {
                p_output->barcode_status =
                    (s3_barcode_status_t)p_context->barcode_held;
                p_context->barcode_held = 0U;
            }
        }
    }
    if (!was_collecting &&
        (p_context->barcode.collecting ||
         s3_barcode_record_active(&p_context->barcode_record)))
    {
        p_context->capture_distance = (s3_capture_distance_t){0};
        if (p_context->odometry_available)
        {
            p_context->capture_distance.start_valid = true;
            p_context->capture_distance.start_m = p_context->odometry.centre_m;
            p_context->capture_distance.epoch = p_context->odometry.epoch;
        }
    }
    if (was_collecting && (p_output->barcode_status != S3_BARCODE_NONE))
    {
        if (p_context->odometry_available)
        {
            p_context->capture_distance.end_valid = true;
            p_context->capture_distance.end_m = p_context->odometry.centre_m;
            if (p_context->capture_distance.start_valid &&
                (p_context->capture_distance.epoch !=
                 p_context->odometry.epoch))
            {
                p_context->capture_distance.start_valid = false;
            }
            p_context->capture_distance.epoch = p_context->odometry.epoch;
        }
        if (active)
        {
            begin_rearm(p_context);
        }
    }
}

static bool process_sensors (s3_navigation_t * p_context,
                             const s3_sensor_frame_t * p_frame,
                             s3_navigation_output_t * p_output,
                             bool frame_time_bad)
{
    unsigned int value_i;
    bool sample_time_bad = frame_time_bad;
    p_context->valid_sensors = 0U;
    for (value_i = 0U; value_i < S3_SENSOR_COUNT; value_i++)
    {
        const s3_raw_sample_t * p_raw = &p_frame->sensor[value_i];
        uint8_t bit = (uint8_t)S3_SENSOR_BIT(value_i);
        bool enabled = (p_context->config.acquisition.enabled_mask & bit) != 0U;
        bool timing_bad =
            ((uint32_t)(p_frame->timestamp_us - p_raw->timestamp_us) >
             p_context->config.max_sample_age_us);
        if ((p_context->seen_sensors & bit) != 0U)
        {
            uint32_t gap =
                p_raw->timestamp_us - p_context->sensor_last_us[value_i];
            timing_bad = timing_bad || (gap == 0U) ||
                         (gap > p_context->config.max_sample_age_us);
        }
        if (enabled && timing_bad)
        {
            sample_time_bad = true;
        }
        if (enabled && p_raw->enabled && p_raw->valid && (p_raw->error == 0) &&
            !timing_bad && !frame_time_bad &&
            ir_reflectance_update(&p_context->sensor[value_i], p_raw->raw,
                                  &p_output->sensor[value_i].darkness))
        {
            p_output->sensor[value_i].valid = true;
            p_context->valid_sensors |= bit;
            p_output->sensor[value_i].surface =
                p_context->sensor[value_i].surface;
            p_context->seen_sensors |= bit;
            p_context->sensor_last_us[value_i] = p_raw->timestamp_us;
        }
        else
        {
            ir_reflectance_reset(&p_context->sensor[value_i]);
            p_context->seen_sensors &= (uint8_t)~bit;
            if (enabled)
            {
                p_output->fault_mask |= bit;
            }
        }
    }
    return sample_time_bad;
}

static void process_health (s3_navigation_t * p_context,
                            const s3_sensor_frame_t * p_frame,
                            const s3_odometry_sample_t * p_odometry,
                            s3_navigation_output_t * p_output,
                            bool sample_time_bad)
{
    bool epoch_changed;
    p_context->odometry_available =
        (p_odometry != 0) && p_odometry->valid &&
        isfinite(p_odometry->centre_m) &&
        ((uint32_t)(p_frame->timestamp_us - p_odometry->timestamp_us) <=
         p_context->config.max_sample_age_us);
    epoch_changed = p_context->odometry_available && p_context->seen_odometry &&
                    (p_odometry->epoch != p_context->odometry.epoch);
    if (p_context->odometry_available)
    {
        p_context->odometry = *p_odometry;
        p_context->seen_odometry = true;
    }
    p_context->current_faults = 0U;
    if (p_output->fault_mask != 0U)
    {
        p_context->current_faults |= S3_FAULT_SENSOR;
    }
    if (sample_time_bad)
    {
        p_context->current_faults |= S3_FAULT_TIMING;
    }
    if (!p_context->odometry_available)
    {
        p_context->current_faults |= S3_FAULT_ODOMETRY;
    }
    if (epoch_changed)
    {
        p_context->current_faults |= S3_FAULT_ODOMETRY_EPOCH;
    }
    if ((p_context->current_faults == 0U) &&
        (p_context->valid_sensors == S3_ALL_SENSORS))
    {
        if (!p_context->healthy_tracking)
        {
            p_context->healthy_since_us = p_frame->timestamp_us;
            p_context->healthy_tracking = true;
        }
    }
    else
    {
        p_context->healthy_tracking = false;
    }
    if ((p_context->lifecycle == S3_RUN_ACTIVE) &&
        (p_context->current_faults != 0U))
    {
        p_context->lifecycle = S3_RUN_FAULT_LATCHED;
        p_context->latched_faults = p_context->current_faults;
        p_output->events |= S3_EVENT_SENSOR_FAULT;
        reset_barcode(p_context);
        p_context->capture_distance = (s3_capture_distance_t){0};
    }
    else if (p_context->lifecycle == S3_RUN_FAULT_LATCHED)
    {
        p_context->latched_faults |= p_context->current_faults;
    }
}

bool s3_navigation_process (s3_navigation_t * p_context,
                            const s3_sensor_frame_t * p_frame,
                            const s3_odometry_sample_t * p_odometry,
                            s3_navigation_output_t * p_output)
{
    bool frame_time_bad, line_valid, sample_time_bad;
    if (p_output == 0)
    {
        return false;
    }
    *p_output = (s3_navigation_output_t){0};
    if ((p_context == 0) || !p_context->configured || (p_frame == 0))
    {
        return false;
    }
    p_output->timestamp_us = p_frame->timestamp_us;
    p_output->sample_gap_us = p_context->seen_frame
                                  ? (p_frame->timestamp_us - p_context->last_us)
                                  : 0U;
    frame_time_bad =
        p_context->seen_frame &&
        ((p_output->sample_gap_us == 0U) ||
         (p_output->sample_gap_us > p_context->config.max_sample_age_us));
    p_context->last_us = p_frame->timestamp_us;
    p_context->seen_frame = true;
    sample_time_bad =
        process_sensors(p_context, p_frame, p_output, frame_time_bad);
    process_health(p_context, p_frame, p_odometry, p_output, sample_time_bad);
    line_valid = p_output->sensor[S3_SENSOR_LEFT].valid &&
                 p_output->sensor[S3_SENSOR_RIGHT].valid;
    p_output->line =
        s3_line_update(&p_context->line, p_frame->timestamp_us, line_valid,
                       p_output->sensor[S3_SENSOR_LEFT].darkness,
                       p_output->sensor[S3_SENSOR_RIGHT].darkness);
    p_output->steering_preview =
        s3_steering_compute(&p_context->config.steering, &p_output->line);
    process_barcode(p_context, p_frame, p_output);
    if ((p_context->lifecycle == S3_RUN_ACTIVE) &&
        (p_context->mode == S3_MODE_TRACK))
    {
        p_output->steering = p_output->steering_preview;
        if (p_output->line.lost_event)
        {
            p_output->events |= S3_EVENT_LINE_LOST;
        }
        if (p_output->line.reacquired_event)
        {
            p_output->events |= S3_EVENT_LINE_REACQUIRED;
        }
        if (p_output->line.junction_event)
        {
            p_output->events |= S3_EVENT_JUNCTION;
        }
        if (p_output->barcode_status == S3_BARCODE_DECODED)
        {
            p_output->events |= S3_EVENT_BARCODE;
            p_output->action =
                p_context->config
                    .map[(unsigned int)(p_output->barcode.letter - 'A')];
        }
    }
    else
    {
        p_output->line.lost_event = false;
        p_output->line.reacquired_event = false;
        p_output->line.junction_event = false;
    }
    p_output->lifecycle = p_context->lifecycle;
    p_output->readiness_blockers =
        s3_navigation_readiness(p_context, p_frame->timestamp_us);
    p_output->accepted_calibration = p_context->accepted_calibration;
    p_output->latched_faults = p_context->latched_faults;
    p_output->stop_requested = p_context->lifecycle == S3_RUN_FAULT_LATCHED;
    p_output->odometry_available = p_context->odometry_available;
    p_output->rearming = p_context->rearming;
    p_output->diagnostic = (p_context->lifecycle != S3_RUN_ACTIVE) ||
                           (p_context->mode != S3_MODE_TRACK);
    p_output->capture_distance = p_context->capture_distance;
    return true;
}

const char * s3_surface_name (ir_surface_t surface)
{
    switch (surface)
    {
    case IR_SURFACE_WHITE:
        return "WHITE";
    case IR_SURFACE_BLACK:
        return "BLACK";
    default:
        return "UNKNOWN";
    }
}

const char * s3_action_name (s3_nav_action_t action)
{
    switch (action)
    {
    case S3_NAV_LEFT:
        return "LEFT";
    case S3_NAV_RIGHT:
        return "RIGHT";
    case S3_NAV_STRAIGHT:
        return "STRAIGHT";
    case S3_NAV_UTURN:
        return "U-TURN";
    default:
        return "NONE";
    }
}

#include "line_navigation.h"
#include <math.h>
s3_navigation_config_t s3_navigation_defaults(void)
{
    s3_navigation_config_t cfg = {0};
    unsigned int i;
    cfg.acquisition = (s3_acquisition_config_t){
        {S3_ADC_CHANNEL_BARCODE, S3_ADC_CHANNEL_LEFT, S3_ADC_CHANNEL_RIGHT}, 1U};
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        cfg.calibration[i] = (ir_calibration_t){150U, 280U, 400U, 600U};
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
bool s3_navigation_init(s3_navigation_t *context,
                        const s3_navigation_config_t *config)
{
    unsigned int i;
    if (context == 0)
    {
        return false;
    }
    *context = (s3_navigation_t){0};
    if ((config == 0) || !s3_acquisition_config_valid(&config->acquisition) ||
        !s3_steering_config_valid(&config->steering) ||
        !s3_line_init(&context->line, &config->line) ||
        !s3_barcode_init(&context->barcode, &config->barcode) ||
        (config->max_sample_age_us == 0U) ||
        (config->max_sample_age_us >= 0x80000000U) ||
        (config->rearm_stable_us == 0U) ||
        (config->rearm_stable_us >= 0x80000000U) ||
        (config->fault_clear_us == 0U) ||
        (config->fault_clear_us >= 0x80000000U) ||
        !isfinite(config->rearm_distance_m) ||
        (config->rearm_distance_m <= 0.0))
    {
        return false;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        if (!ir_reflectance_init(&context->sensor[i], &config->calibration[i]))
        {
            return false;
        }
    }
    for (i = 0U; i < 26U; i++)
    {
        if ((unsigned int)config->map[i] >= S3_NAV_ACTION_COUNT)
        {
            return false;
        }
    }
    context->config = *config;
    context->mode = S3_MODE_TRACK;
    context->configured = true;
    return true;
}
static void begin_rearm(s3_navigation_t *context)
{
    context->rearming = true;
    context->rearm_stable = false;
    context->rearm_anchor_valid = context->odometry_available;
    context->rearm_anchor_m = context->odometry.centre_m;
    context->rearm_epoch = context->odometry.epoch;
    s3_barcode_reset(&context->barcode);
}
bool s3_navigation_set_mode(s3_navigation_t *context, s3_processing_mode_t mode)
{
    if ((context == 0) || !context->configured ||
        ((mode != S3_MODE_TRACK) && (mode != S3_MODE_SUPPRESSED)))
    {
        return false;
    }
    if (context->mode != mode)
    {
        s3_line_reset(&context->line);
        s3_barcode_reset(&context->barcode);
        context->capture_distance = (s3_capture_distance_t){0};
        if (context->lifecycle == S3_RUN_ACTIVE)
        {
            begin_rearm(context);
        }
        context->mode = mode;
    }
    return true;
}
bool s3_navigation_apply_calibration(
    s3_navigation_t *context, s3_sensor_id_t id,
    const s3_calibration_collector_t *collector)
{
    ir_calibration_t candidate;
    if ((context == 0) || !context->configured ||
        (context->lifecycle != S3_RUN_INACTIVE) ||
        ((unsigned int)id >= S3_SENSOR_COUNT))
    {
        return false;
    }
    candidate = context->config.calibration[id];
    if (!s3_calibration_finish(collector, &candidate))
    {
        return false;
    }
    context->config.calibration[id] = candidate;
    context->accepted_calibration |= (uint8_t)S3_SENSOR_BIT(id);
    context->valid_sensors &= (uint8_t)~S3_SENSOR_BIT(id);
    (void)ir_reflectance_init(&context->sensor[id], &candidate);
    s3_line_reset(&context->line);
    s3_barcode_reset(&context->barcode);
    return true;
}
bool s3_navigation_set_mapping(s3_navigation_t *context,
                               const s3_nav_action_t map[26])
{
    unsigned int i;
    if ((context == 0) || !context->configured ||
        (context->lifecycle != S3_RUN_INACTIVE) || (map == 0))
    {
        return false;
    }
    for (i = 0U; i < 26U; i++)
    {
        if ((unsigned int)map[i] >= S3_NAV_ACTION_COUNT)
        {
            return false;
        }
    }
    for (i = 0U; i < 26U; i++)
    {
        context->config.map[i] = map[i];
    }
    return true;
}
uint32_t s3_navigation_readiness(const s3_navigation_t *context,
                                 uint32_t now_us)
{
    uint32_t blockers = 0U;
    unsigned int i;
    if ((context == 0) || !context->configured)
    {
        return S3_READY_CHANNELS | S3_READY_CALIBRATION | S3_READY_SENSORS |
               S3_READY_ODOMETRY;
    }
    if (context->config.acquisition.enabled_mask != S3_ALL_SENSORS)
    {
        blockers |= S3_READY_CHANNELS;
    }
    if (context->accepted_calibration != S3_ALL_SENSORS)
    {
        blockers |= S3_READY_CALIBRATION;
    }
    if (!context->seen_frame || (context->valid_sensors != S3_ALL_SENSORS) ||
        ((uint32_t)(now_us - context->last_us) >
         context->config.max_sample_age_us))
    {
        blockers |= S3_READY_SENSORS;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        if ((uint32_t)(now_us - context->sensor_last_us[i]) >
            context->config.max_sample_age_us)
        {
            blockers |= S3_READY_SENSORS;
        }
    }
    if (!context->odometry_available ||
        ((uint32_t)(now_us - context->odometry.timestamp_us) >
         context->config.max_sample_age_us))
    {
        blockers |= S3_READY_ODOMETRY;
    }
    if (context->lifecycle == S3_RUN_FAULT_LATCHED)
    {
        blockers |= S3_READY_FAULT;
    }
    return blockers;
}
bool s3_navigation_start(s3_navigation_t *context, uint32_t now_us)
{
    if ((context == 0) || !context->configured ||
        (context->lifecycle != S3_RUN_INACTIVE) ||
        (s3_navigation_readiness(context, now_us) != 0U))
    {
        return false;
    }
    context->lifecycle = S3_RUN_ACTIVE;
    s3_line_reset(&context->line);
    s3_barcode_reset(&context->barcode);
    context->capture_distance = (s3_capture_distance_t){0};
    context->rearming = false;
    return true;
}
bool s3_navigation_end_run(s3_navigation_t *context)
{
    if ((context == 0) || !context->configured ||
        (context->lifecycle == S3_RUN_FAULT_LATCHED))
    {
        return false;
    }
    context->lifecycle = S3_RUN_INACTIVE;
    context->rearming = false;
    s3_line_reset(&context->line);
    s3_barcode_reset(&context->barcode);
    context->capture_distance = (s3_capture_distance_t){0};
    return true;
}
bool s3_navigation_reset_fault(s3_navigation_t *context, uint32_t now_us)
{
    if ((context == 0) || !context->configured ||
        (context->lifecycle != S3_RUN_FAULT_LATCHED) ||
        ((s3_navigation_readiness(context, now_us) &
          ~(uint32_t)S3_READY_FAULT) != 0U) ||
        (context->current_faults != 0U) || !context->healthy_tracking ||
        ((uint32_t)(context->last_us - context->healthy_since_us) <
         context->config.fault_clear_us))
    {
        return false;
    }
    context->lifecycle = S3_RUN_INACTIVE;
    context->latched_faults = 0U;
    return s3_navigation_end_run(context);
}
/* Rearming requires continuous clear observations AND net displacement. */
static bool rearm_ready(s3_navigation_t *context,
                        const s3_sensor_frame_t *frame,
                        const s3_navigation_output_t *output)
{
    const s3_processed_sensor_t *barcode = &output->sensor[S3_SENSOR_BARCODE];
    double distance;
    bool clear = output->line.valid && output->line.present &&
                 !output->line.junction_candidate && barcode->valid &&
                 (barcode->surface == IR_SURFACE_WHITE) &&
                 context->odometry_available;
    if (!clear)
    {
        context->rearm_stable = false;
        return false;
    }
    if (!context->rearm_anchor_valid)
    {
        context->rearm_anchor_m = context->odometry.centre_m;
        context->rearm_epoch = context->odometry.epoch;
        context->rearm_anchor_valid = true;
    }
    if (!context->rearm_stable)
    {
        context->rearm_since_us = frame->timestamp_us;
        context->rearm_stable = true;
    }
    distance = context->odometry.centre_m - context->rearm_anchor_m;
    if (distance < 0.0)
    {
        distance = -distance;
    }
    if ((context->odometry.epoch != context->rearm_epoch) ||
        ((uint32_t)(frame->timestamp_us - context->rearm_since_us) <
         context->config.rearm_stable_us) ||
        (distance < context->config.rearm_distance_m))
    {
        return false;
    }
    context->rearming = false;
    /* Decoder was reset at capture completion: seed a new quiet zone now.
     */
    return true;
}
static void process_barcode(s3_navigation_t *context,
                            const s3_sensor_frame_t *frame,
                            s3_navigation_output_t *output)
{
    const s3_processed_sensor_t *barcode = &output->sensor[S3_SENSOR_BARCODE];
    bool was_collecting, active = context->lifecycle == S3_RUN_ACTIVE;
    if ((context->mode != S3_MODE_TRACK) ||
        ((context->config.acquisition.enabled_mask &
          S3_SENSOR_BIT(S3_SENSOR_BARCODE)) == 0U))
    {
        return;
    }
    if (active && context->rearming && !rearm_ready(context, frame, output))
    {
        return;
    }
    was_collecting = context->barcode.collecting;
    output->barcode_status = s3_barcode_update(
        &context->barcode, frame->sensor[S3_SENSOR_BARCODE].timestamp_us,
        barcode->valid && (barcode->surface != IR_SURFACE_UNKNOWN),
        barcode->surface == IR_SURFACE_BLACK, &output->barcode);
    if (!was_collecting && context->barcode.collecting)
    {
        context->capture_distance = (s3_capture_distance_t){0};
        if (context->odometry_available)
        {
            context->capture_distance.start_valid = true;
            context->capture_distance.start_m = context->odometry.centre_m;
            context->capture_distance.epoch = context->odometry.epoch;
        }
    }
    if (was_collecting && (output->barcode_status != S3_BARCODE_NONE))
    {
        if (context->odometry_available)
        {
            context->capture_distance.end_valid = true;
            context->capture_distance.end_m = context->odometry.centre_m;
            if (context->capture_distance.start_valid &&
                (context->capture_distance.epoch != context->odometry.epoch))
            {
                context->capture_distance.start_valid = false;
            }
            context->capture_distance.epoch = context->odometry.epoch;
        }
        if (active)
        {
            begin_rearm(context);
        }
    }
}
static bool process_sensors(s3_navigation_t *context,
                            const s3_sensor_frame_t *frame,
                            s3_navigation_output_t *output, bool frame_time_bad)
{
    unsigned int i;
    bool sample_time_bad = frame_time_bad;
    context->valid_sensors = 0U;
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        const s3_raw_sample_t *raw = &frame->sensor[i];
        uint8_t bit = (uint8_t)S3_SENSOR_BIT(i);
        bool enabled = (context->config.acquisition.enabled_mask & bit) != 0U;
        bool timing_bad = ((uint32_t)(frame->timestamp_us - raw->timestamp_us) >
                           context->config.max_sample_age_us);
        if ((context->seen_sensors & bit) != 0U)
        {
            uint32_t gap = raw->timestamp_us - context->sensor_last_us[i];
            timing_bad = timing_bad || (gap == 0U) ||
                         (gap > context->config.max_sample_age_us);
        }
        if (enabled && timing_bad)
        {
            sample_time_bad = true;
        }
        if (enabled && raw->enabled && raw->valid && (raw->error == 0) &&
            !timing_bad && !frame_time_bad &&
            ir_reflectance_update(&context->sensor[i], raw->raw,
                                  &output->sensor[i].darkness))
        {
            output->sensor[i].valid = true;
            context->valid_sensors |= bit;
            output->sensor[i].surface = context->sensor[i].surface;
            context->seen_sensors |= bit;
            context->sensor_last_us[i] = raw->timestamp_us;
        }
        else
        {
            ir_reflectance_reset(&context->sensor[i]);
            context->seen_sensors &= (uint8_t)~bit;
            if (enabled)
            {
                output->fault_mask |= bit;
            }
        }
    }
    return sample_time_bad;
}
static void process_health(s3_navigation_t *context,
                           const s3_sensor_frame_t *frame,
                           const s3_odometry_sample_t *odometry,
                           s3_navigation_output_t *output, bool sample_time_bad)
{
    bool epoch_changed;
    context->odometry_available =
        (odometry != 0) && odometry->valid && isfinite(odometry->centre_m) &&
        ((uint32_t)(frame->timestamp_us - odometry->timestamp_us) <=
         context->config.max_sample_age_us);
    epoch_changed = context->odometry_available && context->seen_odometry &&
                    (odometry->epoch != context->odometry.epoch);
    if (context->odometry_available)
    {
        context->odometry = *odometry;
        context->seen_odometry = true;
    }
    context->current_faults = 0U;
    if (output->fault_mask != 0U)
    {
        context->current_faults |= S3_FAULT_SENSOR;
    }
    if (sample_time_bad)
    {
        context->current_faults |= S3_FAULT_TIMING;
    }
    if (!context->odometry_available)
    {
        context->current_faults |= S3_FAULT_ODOMETRY;
    }
    if (epoch_changed)
    {
        context->current_faults |= S3_FAULT_ODOMETRY_EPOCH;
    }
    if ((context->current_faults == 0U) &&
        (context->valid_sensors == S3_ALL_SENSORS))
    {
        if (!context->healthy_tracking)
        {
            context->healthy_since_us = frame->timestamp_us;
            context->healthy_tracking = true;
        }
    }
    else
    {
        context->healthy_tracking = false;
    }
    if ((context->lifecycle == S3_RUN_ACTIVE) &&
        (context->current_faults != 0U))
    {
        context->lifecycle = S3_RUN_FAULT_LATCHED;
        context->latched_faults = context->current_faults;
        output->events |= S3_EVENT_SENSOR_FAULT;
        s3_barcode_reset(&context->barcode);
        context->capture_distance = (s3_capture_distance_t){0};
    }
    else if (context->lifecycle == S3_RUN_FAULT_LATCHED)
    {
        context->latched_faults |= context->current_faults;
    }
}
bool s3_navigation_process(s3_navigation_t *context,
                           const s3_sensor_frame_t *frame,
                           const s3_odometry_sample_t *odometry,
                           s3_navigation_output_t *output)
{
    bool frame_time_bad, line_valid, sample_time_bad;
    if (output == 0)
    {
        return false;
    }
    *output = (s3_navigation_output_t){0};
    if ((context == 0) || !context->configured || (frame == 0))
    {
        return false;
    }
    output->timestamp_us = frame->timestamp_us;
    output->sample_gap_us =
        context->seen_frame ? (frame->timestamp_us - context->last_us) : 0U;
    frame_time_bad =
        context->seen_frame &&
        ((output->sample_gap_us == 0U) ||
         (output->sample_gap_us > context->config.max_sample_age_us));
    context->last_us = frame->timestamp_us;
    context->seen_frame = true;
    sample_time_bad = process_sensors(context, frame, output, frame_time_bad);
    process_health(context, frame, odometry, output, sample_time_bad);
    line_valid = output->sensor[S3_SENSOR_LEFT].valid &&
                 output->sensor[S3_SENSOR_RIGHT].valid;
    output->line =
        s3_line_update(&context->line, frame->timestamp_us, line_valid,
                       output->sensor[S3_SENSOR_LEFT].darkness,
                       output->sensor[S3_SENSOR_RIGHT].darkness);
    output->steering_preview =
        s3_steering_compute(&context->config.steering, &output->line);
    process_barcode(context, frame, output);
    if ((context->lifecycle == S3_RUN_ACTIVE) &&
        (context->mode == S3_MODE_TRACK))
    {
        output->steering = output->steering_preview;
        if (output->line.lost_event)
        {
            output->events |= S3_EVENT_LINE_LOST;
        }
        if (output->line.reacquired_event)
        {
            output->events |= S3_EVENT_LINE_REACQUIRED;
        }
        if (output->line.junction_event)
        {
            output->events |= S3_EVENT_JUNCTION;
        }
        if (output->barcode_status == S3_BARCODE_DECODED)
        {
            output->events |= S3_EVENT_BARCODE;
            output->action =
                context->config
                    .map[(unsigned int)(output->barcode.letter - 'A')];
        }
    }
    else
    {
        output->line.lost_event = false;
        output->line.reacquired_event = false;
        output->line.junction_event = false;
    }
    output->lifecycle = context->lifecycle;
    output->readiness_blockers =
        s3_navigation_readiness(context, frame->timestamp_us);
    output->accepted_calibration = context->accepted_calibration;
    output->latched_faults = context->latched_faults;
    output->stop_requested = context->lifecycle == S3_RUN_FAULT_LATCHED;
    output->odometry_available = context->odometry_available;
    output->rearming = context->rearming;
    output->diagnostic = (context->lifecycle != S3_RUN_ACTIVE) ||
                         (context->mode != S3_MODE_TRACK);
    output->capture_distance = context->capture_distance;
    return true;
}
const char *s3_surface_name(ir_surface_t surface)
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
const char *s3_action_name(s3_nav_action_t action)
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

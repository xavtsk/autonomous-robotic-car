/** @file line_navigation.c
 * @brief Combine reflectance, line, barcode and odometry observations into
 * navigation results.
 * @details The caller serializes updates to the navigation state. The
 * module publishes observations and steering proposals for the drive owner.
 * @note The target's int represents byte and half-word operands, and
 * int64_t represents uint32_t values. Sensor and event masks and validated
 * enum bounds use unsigned arithmetic.
 */
#include "line_navigation.h"
#include "../app_types.h"
#include <math.h>

static void reset_barcode (line_navigation_t * p_context);
static void begin_rearm (line_navigation_t * p_context);
static bool rearm_ready (line_navigation_t *              p_context,
                         const ir_acquisition_frame_t *   p_frame,
                         const line_navigation_output_t * p_output);
static void process_barcode (line_navigation_t *            p_context,
                             const ir_acquisition_frame_t * p_frame,
                             line_navigation_output_t *     p_output);
static bool process_sensors (line_navigation_t *            p_context,
                             const ir_acquisition_frame_t * p_frame,
                             line_navigation_output_t *     p_output,
                             bool b_is_frame_time_invalid);
static void process_health (line_navigation_t *                p_context,
                            const ir_acquisition_frame_t *     p_frame,
                            const line_navigation_odometry_t * p_odometry,
                            line_navigation_output_t *         p_output,
                            bool b_is_sample_time_invalid);

/** @brief Return development acquisition, calibration, steering, barcode and
 * fault-timing defaults. The caller applies the course configuration
 * before use.
 */
line_navigation_config_t line_navigation_defaults (void)
{
    line_navigation_config_t cfg = {0};
    cfg.acquisition              = (ir_acquisition_config_t){
        {S3_ADC_CHANNEL_BARCODE, S3_ADC_CHANNEL_LEFT, S3_ADC_CHANNEL_RIGHT},
        1U};

    unsigned int sensor_index;
    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        cfg.calibration[sensor_index] =
            (ir_reflectance_calibration_t){150U, 280U, 400U, 600U};
    }

    cfg.line              = line_detection_defaults();
    cfg.steering          = line_controller_defaults();
    cfg.barcode           = bc_decoder_defaults();
    cfg.map[0]            = S3_NAV_LEFT;
    cfg.map[1]            = S3_NAV_RIGHT;
    cfg.map[2]            = S3_NAV_STRAIGHT;
    cfg.map[3]            = S3_NAV_UTURN;
    cfg.max_sample_age_us = 30000U;
    cfg.rearm_stable_us   = 30000U;
    cfg.fault_clear_us    = 100000U;
    cfg.rearm_distance_m  = 0.020;
    return cfg;
}

/** @brief Clear the supplied context and validate the constituent sensor,
 * line, barcode, mapping and timing settings. Set configured and enter
 * track mode only after all checks succeed.
 */
bool line_navigation_init (line_navigation_t *              p_context,
                           const line_navigation_config_t * p_config)
{

    if (0 == p_context)
    {
        return false;
    }

    *p_context = (line_navigation_t){0};

    if (((((((((((((0 == p_config) ||
                   (!ir_acquisition_config_valid(&p_config->acquisition))) ||
                  (!line_controller_config_valid(&p_config->steering))) ||
                 (!line_detection_init(&p_context->line, &p_config->line))) ||
                (!bc_decoder_init(&p_context->barcode, &p_config->barcode))) ||
               (0U == p_config->max_sample_age_us)) ||
              (p_config->max_sample_age_us >= 0x80000000U)) ||
             (0U == p_config->rearm_stable_us)) ||
            (p_config->rearm_stable_us >= 0x80000000U)) ||
           (0U == p_config->fault_clear_us)) ||
          (p_config->fault_clear_us >= 0x80000000U)) ||
         (!isfinite(p_config->rearm_distance_m))) ||
        (p_config->rearm_distance_m <= 0.0))
    {
        return false;
    }

    unsigned int index;
    for (index = 0U; index < (unsigned int)S3_SENSOR_COUNT; index++)
    {
        if (!ir_reflectance_init(&p_context->sensor[index],
                                 &p_config->calibration[index]))
        {
            return false;
        }
    }

    for (index = 0U; index < S3_BARCODE_LETTERS; index++)
    {
        if ((unsigned int)p_config->map[index] >=
            (unsigned int)S3_NAV_ACTION_COUNT)
        {
            return false;
        }
    }

    p_context->config          = *p_config;
    p_context->mode            = S3_MODE_TRACK;
    p_context->b_is_configured = true;
    return true;
}

/** @brief Accept track or suppressed mode on a configured context. A mode
 * change clears line and capture history and starts rearming during an
 * active run.
 */
bool line_navigation_set_mode (line_navigation_t *    p_context,
                               line_navigation_mode_t mode)
{
    if (((0 == p_context) || (!p_context->b_is_configured)) ||
        ((mode != (unsigned int)S3_MODE_TRACK) &&
         (mode != (unsigned int)S3_MODE_SUPPRESSED)))
    {
        return false;
    }

    if (p_context->mode != mode)
    {
        line_detection_reset(&p_context->line);
        reset_barcode(p_context);
        p_context->capture_distance = (line_navigation_distance_t){0};

        if (S3_RUN_ACTIVE == (int)p_context->lifecycle)
        {
            begin_rearm(p_context);
        }

        p_context->mode = mode;
    }

    return true;
}

/** @brief Apply validated collector measurements for one sensor while
 * navigation is inactive. Report success only for a calibration
 * accepted by the sensor.
 */
bool line_navigation_set_calibration (
    line_navigation_t * p_context, ir_acquisition_sensor_id_t value_id,
    const ir_calibration_collector_t * p_collector)
{

    if ((((0 == p_context) || (!p_context->b_is_configured)) ||
         ((int)p_context->lifecycle != S3_RUN_INACTIVE)) ||
        ((unsigned int)value_id >= (unsigned int)S3_SENSOR_COUNT))
    {
        return false;
    }

    ir_reflectance_calibration_t candidate;
    candidate = p_context->config.calibration[value_id];

    if (!ir_calibration_finish(p_collector, &candidate))
    {
        return false;
    }

    /* The validated ID selects one of three sensor bits. Operate on
     * unsigned masks and retain only the original byte and selected bit. */
    /* clang-format off */
    p_context->config.calibration[value_id] = candidate;
    p_context->accepted_calibration         =
        (uint8_t)((unsigned)p_context->accepted_calibration |
                  S3_SENSOR_BIT(value_id));
    p_context->valid_sensors                =
        (uint8_t)((unsigned)p_context->valid_sensors &
                                         ~S3_SENSOR_BIT(value_id));
    /* clang-format on */
    (void)ir_reflectance_init(&p_context->sensor[value_id], &candidate);
    line_detection_reset(&p_context->line);
    reset_barcode(p_context);
    return true;
}

/** @brief Validate all 26 navigation actions before copying the map into a
 * configured, inactive context. A rejected map leaves the current
 * mapping in place.
 */
bool line_navigation_set_mapping (
    line_navigation_t *            p_context,
    const line_navigation_action_t p_map[S3_BARCODE_LETTERS])
{

    if ((((0 == p_context) || (!p_context->b_is_configured)) ||
         ((int)p_context->lifecycle != S3_RUN_INACTIVE)) ||
        (0 == p_map))
    {
        return false;
    }

    unsigned int letter_index;
    for (letter_index = 0U; letter_index < S3_BARCODE_LETTERS; letter_index++)
    {
        if ((unsigned int)p_map[letter_index] >=
            (unsigned int)S3_NAV_ACTION_COUNT)
        {
            return false;
        }
    }

    for (letter_index = 0U; letter_index < S3_BARCODE_LETTERS; letter_index++)
    {
        p_context->config.map[letter_index] = p_map[letter_index];
    }

    return true;
}

/** @brief Return the current blocker bit mask for channels, calibration,
 * sensor or odometry freshness and latched faults using the wrapping
 * microsecond clock.
 */
uint32_t line_navigation_readiness (const line_navigation_t * p_context,
                                    uint32_t                  now_us)
{
    uint32_t blockers = 0U;

    if ((0 == p_context) || (!p_context->b_is_configured))
    {
        return (unsigned)S3_READY_CHANNELS | (unsigned)S3_READY_CALIBRATION |
               (unsigned)S3_READY_SENSORS | (unsigned)S3_READY_ODOMETRY;
    }
    /* The blocker enumerators are positive masks in 1..16. Convert them
     * to unsigned before merging them into the readiness word. */
    if (p_context->config.acquisition.enabled_mask != S3_ALL_SENSORS)
    {
        blockers |= (unsigned)S3_READY_CHANNELS;
    }

    if (p_context->accepted_calibration != S3_ALL_SENSORS)
    {
        blockers |= (unsigned)S3_READY_CALIBRATION;
    }

    if (((!p_context->b_has_seen_frame) ||
         (p_context->valid_sensors != S3_ALL_SENSORS)) ||
        ((now_us - p_context->last_us) > p_context->config.max_sample_age_us))
    {
        blockers |= (unsigned)S3_READY_SENSORS;
    }

    unsigned int sensor_index;
    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        if ((now_us - p_context->sensor_last_us[sensor_index]) >
            p_context->config.max_sample_age_us)
        {
            blockers |= (unsigned)S3_READY_SENSORS;
        }
    }

    if ((!p_context->b_has_odometry) ||
        ((now_us - p_context->odometry.timestamp_us) >
         p_context->config.max_sample_age_us))
    {
        blockers |= (unsigned)S3_READY_ODOMETRY;
    }

    if (S3_RUN_FAULT_LATCHED == (int)p_context->lifecycle)
    {
        blockers |= (unsigned)S3_READY_FAULT;
    }

    return blockers;
}

/** @brief Start a configured, inactive context with no readiness blockers. On
 * acceptance reset line, barcode and capture history for the new run.
 */
bool line_navigation_start (line_navigation_t * p_context, uint32_t now_us)
{
    if ((((0 == p_context) || (!p_context->b_is_configured)) ||
         ((int)p_context->lifecycle != S3_RUN_INACTIVE)) ||
        (line_navigation_readiness(p_context, now_us) != 0U))
    {
        return false;
    }

    p_context->lifecycle = S3_RUN_ACTIVE;
    line_detection_reset(&p_context->line);
    reset_barcode(p_context);
    p_context->capture_distance = (line_navigation_distance_t){0};
    p_context->b_is_rearming    = false;
    return true;
}

/** @brief End the run and clear line and barcode history. A missing,
 * unconfigured or fault-latched context returns false.
 */
bool line_navigation_end_run (line_navigation_t * p_context)
{
    if (((0 == p_context) || (!p_context->b_is_configured)) ||
        (S3_RUN_FAULT_LATCHED == (int)p_context->lifecycle))
    {
        return false;
    }

    p_context->lifecycle     = S3_RUN_INACTIVE;
    p_context->b_is_rearming = false;
    line_detection_reset(&p_context->line);
    reset_barcode(p_context);
    p_context->capture_distance = (line_navigation_distance_t){0};
    return true;
}

/** @brief Clear a latched fault only after other readiness checks pass and the
 * healthy interval reaches the configured duration. Return to the
 * inactive state on success.
 */
bool line_navigation_reset_fault (line_navigation_t * p_context,
                                  uint32_t            now_us)
{
    if (((((((0 == p_context) || (!p_context->b_is_configured)) ||
            ((int)p_context->lifecycle != S3_RUN_FAULT_LATCHED)) ||
           ((line_navigation_readiness(p_context, now_us) &
             ~(uint32_t)S3_READY_FAULT) != 0U)) ||
          (p_context->current_faults != 0U)) ||
         (!p_context->b_is_healthy_tracking)) ||
        ((p_context->last_us - p_context->healthy_since_us) <
         p_context->config.fault_clear_us))
    {
        return false;
    }

    p_context->lifecycle      = S3_RUN_INACTIVE;
    p_context->latched_faults = 0U;
    return line_navigation_end_run(p_context);
}

/** @brief Process a frame and optional odometry on the navigation owner.
 * @param[in,out] p_context Initialized, exclusively owned navigation state.
 * @param[in] p_frame Borrowed IR frame with a wrapping microsecond timestamp.
 * @param[in] p_odometry Optional borrowed odometry for the current update.
 * @param[out] p_output Writable observation. A nonnull output is cleared
 * before validation.
 * @details Record the frame timestamp, validate samples and update health
 * before interpreting line and barcode data. Always compute the steering
 * preview from the current line result. Only active track mode promotes the
 * preview to steering and emits track events and a mapped barcode action.
 * Publish the resulting readiness, calibration, fault and capture state.
 * @return False for missing or unconfigured inputs. True means the frame
 * was processed and can still indicate faults or invalid samples.
 */
bool line_navigation_process (line_navigation_t *                p_context,
                              const ir_acquisition_frame_t *     p_frame,
                              const line_navigation_odometry_t * p_odometry,
                              line_navigation_output_t *         p_output)
{

    if (0 == p_output)
    {
        return false;
    }

    *p_output = (line_navigation_output_t){0};

    if (((0 == p_context) || (!p_context->b_is_configured)) || (0 == p_frame))
    {
        return false;
    }

    p_output->timestamp_us  = p_frame->timestamp_us;
    p_output->sample_gap_us = p_context->b_has_seen_frame
                                  ? (p_frame->timestamp_us - p_context->last_us)
                                  : 0U;
    const bool b_is_frame_time_invalid =
        (p_context->b_has_seen_frame) &&
        ((0U == p_output->sample_gap_us) ||
         (p_output->sample_gap_us > p_context->config.max_sample_age_us));
    p_context->last_us          = p_frame->timestamp_us;
    p_context->b_has_seen_frame = true;
    const bool b_is_sample_time_invalid =
        process_sensors(p_context, p_frame, p_output, b_is_frame_time_invalid);
    process_health(p_context, p_frame, p_odometry, p_output,
                   b_is_sample_time_invalid);
    const bool b_is_line_valid =
        (p_output->sensor[S3_SENSOR_LEFT].b_is_valid) &&
        (p_output->sensor[S3_SENSOR_RIGHT].b_is_valid);
    /* clang-format off */
    p_output->line             = line_detection_update(
        &p_context->line, p_frame->timestamp_us, b_is_line_valid,
        p_output->sensor[S3_SENSOR_LEFT].darkness,
        p_output->sensor[S3_SENSOR_RIGHT].darkness);
    p_output->steering_preview =
        line_controller_compute(&p_context->config.steering, &p_output->line);
    /* clang-format on */
    process_barcode(p_context, p_frame, p_output);

    if ((S3_RUN_ACTIVE == (int)p_context->lifecycle) &&
        (S3_MODE_TRACK == (int)p_context->mode))
    {
        p_output->steering = p_output->steering_preview;
        /* All emitted event enumerators are masks in 1..16. Convert them
         * before merging them into the unsigned event word. */
        if (p_output->line.b_has_lost_event)
        {
            p_output->events |= (unsigned)S3_EVENT_LINE_LOST;
        }

        if (p_output->line.b_has_reacquired_event)
        {
            p_output->events |= (unsigned)S3_EVENT_LINE_REACQUIRED;
        }

        if (p_output->line.b_has_junction_event)
        {
            p_output->events |= (unsigned)S3_EVENT_JUNCTION;
        }

        if ((unsigned int)S3_BARCODE_DECODED == p_output->barcode_status)
        {
            p_output->events |= (unsigned)S3_EVENT_BARCODE;
            /* A decoded letter is A..Z. Its byte value fits int, and
             * subtracting A gives an unsigned index in 0..25. */
            const unsigned map_index =
                (unsigned)((int)p_output->barcode.letter - 'A');
            p_output->action = p_context->config.map[map_index];
        }
    }
    else
    {
        p_output->line.b_has_lost_event       = false;
        p_output->line.b_has_reacquired_event = false;
        p_output->line.b_has_junction_event   = false;
    }

    /* clang-format off */
    p_output->lifecycle            = p_context->lifecycle;
    p_output->readiness_blockers   =
        line_navigation_readiness(p_context, p_frame->timestamp_us);
    p_output->accepted_calibration = p_context->accepted_calibration;
    p_output->latched_faults       = p_context->latched_faults;
    /* clang-format on */

    /* clang-format off */
    p_output->b_is_stop_requested =
        S3_RUN_FAULT_LATCHED == (int)p_context->lifecycle;
    p_output->b_has_odometry      = p_context->b_has_odometry;
    p_output->b_is_rearming       = p_context->b_is_rearming;
    p_output->b_is_diagnostic     =
        ((int)p_context->lifecycle != S3_RUN_ACTIVE) ||
                                ((int)p_context->mode != S3_MODE_TRACK);
    p_output->capture_distance    = p_context->capture_distance;
    /* clang-format on */
    return true;
}

/** @brief Return the diagnostic display name for a reflectance surface value.
 */
const char * line_navigation_surface_name (ir_reflectance_surface_t surface)
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

/** @brief Return the diagnostic display name for a navigation action value.
 */
const char * line_navigation_action_name (line_navigation_action_t action)
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

/* Both barcode readers restart together so one barcode never gives two letters.
 */
/** @brief Reset both readers and clear a held barcode result in the owner's
 * context.
 */
static void reset_barcode (line_navigation_t * p_context)
{
    bc_decoder_reset(&p_context->barcode);
    bc_decoder_record_reset(&p_context->barcode_record);
    p_context->barcode_held = 0U;
}

/** @brief Start the stable-clear and travel checks, snapshot the odometry
 * anchor and reset the readers before allowing another barcode.
 */
static void begin_rearm (line_navigation_t * p_context)
{
    p_context->b_is_rearming           = true;
    p_context->b_is_rearm_stable       = false;
    p_context->b_is_rearm_anchor_valid = p_context->b_has_odometry;
    p_context->rearm_anchor_m          = p_context->odometry.centre_m;
    p_context->rearm_epoch             = p_context->odometry.epoch;
    reset_barcode(p_context);
}

/* Rearming requires continuous clear observations AND net displacement. */
/** @brief Track continuous clear observations and net forward displacement in
 * the current odometry epoch. Return true when both rearm requirements
 * hold.
 */
static bool rearm_ready (line_navigation_t *              p_context,
                         const ir_acquisition_frame_t *   p_frame,
                         const line_navigation_output_t * p_output)
{
    const line_navigation_sensor_t * p_barcode =
        &p_output->sensor[S3_SENSOR_BARCODE];
    const bool b_is_clear =
        (((((p_output->line.b_is_valid) &&
            (p_output->line.b_is_line_present)) &&
           (!p_output->line.b_has_junction_candidate)) &&
          (p_barcode->b_is_valid)) &&
         ((unsigned int)IR_SURFACE_WHITE == p_barcode->surface)) &&
        (p_context->b_has_odometry);

    if (!b_is_clear)
    {
        p_context->b_is_rearm_stable = false;
        return false;
    }

    if (!p_context->b_is_rearm_anchor_valid)
    {
        p_context->rearm_anchor_m          = p_context->odometry.centre_m;
        p_context->rearm_epoch             = p_context->odometry.epoch;
        p_context->b_is_rearm_anchor_valid = true;
    }

    if (!p_context->b_is_rearm_stable)
    {
        p_context->rearm_since_us    = p_frame->timestamp_us;
        p_context->b_is_rearm_stable = true;
    }

    float64_t distance =
        p_context->odometry.centre_m - p_context->rearm_anchor_m;

    if (distance < 0.0)
    {
        distance = -distance;
    }

    if (((p_context->odometry.epoch != p_context->rearm_epoch) ||
         ((p_frame->timestamp_us - p_context->rearm_since_us) <
          p_context->config.rearm_stable_us)) ||
        (distance < p_context->config.rearm_distance_m))
    {
        return false;
    }

    p_context->b_is_rearming = false;
    /* Decoder was reset at capture completion: seed a new quiet zone now.
     */
    return true;
}

/** @brief Advance barcode tracking from the current frame and odometry,
 * release eligible held results and report the letter, status and
 * mapped action through the navigation output.
 * @details TRACK mode and an enabled barcode sensor are prerequisites.
 * Live decoding takes precedence if both readers succeed on this frame.
 * Hold live failures while the recorder is still active. Measure capture
 * travel from the first collecting frame. Odometry measured before an epoch
 * change cannot be paired with an endpoint from the new epoch.
 */
static void process_barcode (line_navigation_t *            p_context,
                             const ir_acquisition_frame_t * p_frame,
                             line_navigation_output_t *     p_output)
{
    const line_navigation_sensor_t * p_barcode =
        &p_output->sensor[S3_SENSOR_BARCODE];
    const bool b_is_active = S3_RUN_ACTIVE == (int)p_context->lifecycle;

    if (((int)p_context->mode != S3_MODE_TRACK) ||
        ((p_context->config.acquisition.enabled_mask &
          S3_SENSOR_BIT(S3_SENSOR_BARCODE)) == 0U))
    {
        return;
    }

    if ((b_is_active && (p_context->b_is_rearming)) &&
        (!rearm_ready(p_context, p_frame, p_output)))
    {
        return;
    }

    const bool b_was_collecting =
        (p_context->barcode.b_is_collecting) ||
        (bc_decoder_record_active(&p_context->barcode_record));
    {
        const uint32_t timestamp_us =
            p_frame->sensor[S3_SENSOR_BARCODE].timestamp_us;
        bc_decoder_result_t       live_result;
        const bc_decoder_status_t live = bc_decoder_update(
            &p_context->barcode, timestamp_us,
            (p_barcode->b_is_valid) &&
                (p_barcode->surface != (unsigned int)IR_SURFACE_UNKNOWN),
            (unsigned int)IR_SURFACE_BLACK == p_barcode->surface, &live_result);
        bc_decoder_result_t       rec_result;
        const bc_decoder_status_t rec = bc_decoder_record_update(
            &p_context->barcode_record, &p_context->config.barcode,
            timestamp_us, p_barcode->b_is_valid, p_barcode->darkness,
            &rec_result);
        p_output->barcode_status = S3_BARCODE_NONE;

        if ((unsigned int)S3_BARCODE_DECODED == live)
        {
            /* First reader to decode wins; the other restarts. */
            p_output->barcode_status = live;
            p_output->barcode        = live_result;
            bc_decoder_record_reset(&p_context->barcode_record);
            p_context->barcode_held = 0U;
        }
        else if ((unsigned int)S3_BARCODE_DECODED == rec)
        {
            p_output->barcode_status = rec;
            p_output->barcode        = rec_result;
            bc_decoder_reset(&p_context->barcode);
            p_context->barcode_held = 0U;
        }
        else
        {
            /* A faint bar may leave the live result pending while the
             * recording decoder still has a scan to complete. */
            if ((live != (unsigned int)S3_BARCODE_NONE) &&
                (bc_decoder_record_active(&p_context->barcode_record)))
            {
                /* A non-NONE live status fits in the byte-sized latch. */
                p_context->barcode_held = (uint8_t)live;
            }
            else if (live != (unsigned int)S3_BARCODE_NONE)
            {
                p_output->barcode_status = live;
            }
            else
            {
                /* There is no live result to publish. */
            }

            if (((rec != (unsigned int)S3_BARCODE_NONE) &&
                 (!p_context->barcode.b_is_collecting)) &&
                (!p_context->barcode.b_is_pending))
            {
                /* Recorder failed and the live decoder is idle: report. */
                /* clang-format off */
                p_output->barcode_status =
                    (p_context->barcode_held != 0U)
                        ? (bc_decoder_status_t)p_context->barcode_held
                        : rec;
                p_context->barcode_held  = 0U;
                /* clang-format on */
            }
            else if ((p_context->barcode_held != 0U) &&
                     (!bc_decoder_record_active(&p_context->barcode_record)))
            {
                /* clang-format off */
                p_output->barcode_status =
                    (bc_decoder_status_t)p_context->barcode_held;
                p_context->barcode_held  = 0U;
                /* clang-format on */
            }
            else
            {
                /* There is no held barcode result ready for release. */
            }
        }
    }

    if ((!b_was_collecting) &&
        ((p_context->barcode.b_is_collecting) ||
         (bc_decoder_record_active(&p_context->barcode_record))))
    {
        p_context->capture_distance = (line_navigation_distance_t){0};

        if (p_context->b_has_odometry)
        {
            /* clang-format off */
            p_context->capture_distance.b_is_start_valid = true;
            p_context->capture_distance.start_m          =
                p_context->odometry.centre_m;
            p_context->capture_distance.epoch            =
                p_context->odometry.epoch;
            /* clang-format on */
        }
    }

    if (b_was_collecting &&
        (p_output->barcode_status != (unsigned int)S3_BARCODE_NONE))
    {
        if (p_context->b_has_odometry)
        {
            /* clang-format off */
            p_context->capture_distance.b_is_end_valid = true;
            p_context->capture_distance.end_m          =
                p_context->odometry.centre_m;
            /* clang-format on */

            if ((p_context->capture_distance.b_is_start_valid) &&
                (p_context->capture_distance.epoch !=
                 p_context->odometry.epoch))
            {
                p_context->capture_distance.b_is_start_valid = false;
            }

            p_context->capture_distance.epoch = p_context->odometry.epoch;
        }

        if (b_is_active)
        {
            begin_rearm(p_context);
        }
    }
}

/** @brief Process enabled ADC channels in the supplied frame and update each
 * sensor's validity, normalized reflectance and last valid reading
 * time.
 */
static bool process_sensors (line_navigation_t *            p_context,
                             const ir_acquisition_frame_t * p_frame,
                             line_navigation_output_t *     p_output,
                             bool b_is_frame_time_invalid)
{
    bool b_is_sample_time_invalid = b_is_frame_time_invalid;
    p_context->valid_sensors      = 0U;

    unsigned int sensor_index;
    for (sensor_index = 0U; sensor_index < (unsigned int)S3_SENSOR_COUNT;
         sensor_index++)
    {
        /* Indices zero through two produce one bit that fits a byte.
         * Setting or clearing it under an unsigned mask still fits a byte. */
        const ir_acquisition_sample_t * p_raw = &p_frame->sensor[sensor_index];
        const uint8_t bit = (uint8_t)S3_SENSOR_BIT(sensor_index);
        const bool    b_is_enabled =
            ((unsigned int)((unsigned)
                                p_context->config.acquisition.enabled_mask &
                            (unsigned)bit)) != 0U;
        bool b_is_timing_invalid =
            ((p_frame->timestamp_us - p_raw->timestamp_us) >
             p_context->config.max_sample_age_us);

        if (((unsigned int)((unsigned)p_context->seen_sensors &
                            (unsigned)bit)) != 0U)
        {
            const uint32_t gap =
                p_raw->timestamp_us - p_context->sensor_last_us[sensor_index];
            b_is_timing_invalid = (b_is_timing_invalid || (0U == gap)) ||
                                  (gap > p_context->config.max_sample_age_us);
        }

        if (b_is_enabled && b_is_timing_invalid)
        {
            b_is_sample_time_invalid = true;
        }

        if ((((((b_is_enabled && (p_raw->b_is_enabled)) &&
                (p_raw->b_is_valid)) &&
               (0 == p_raw->error)) &&
              (!b_is_timing_invalid)) &&
             (!b_is_frame_time_invalid)) &&
            (ir_reflectance_update(&p_context->sensor[sensor_index], p_raw->raw,
                                   &p_output->sensor[sensor_index].darkness)))
        {
            /* clang-format off */
            p_output->sensor[sensor_index].b_is_valid = true;
            p_context->valid_sensors                  =
                (uint8_t)((unsigned)p_context->valid_sensors | (unsigned)bit);
            p_output->sensor[sensor_index].surface    =
                p_context->sensor[sensor_index].surface;
            p_context->seen_sensors                   =
                (uint8_t)((unsigned)p_context->seen_sensors | (unsigned)bit);
            p_context->sensor_last_us[sensor_index]   = p_raw->timestamp_us;
            /* clang-format on */
        }
        else
        {
            ir_reflectance_reset(&p_context->sensor[sensor_index]);
            p_context->seen_sensors =
                (uint8_t)((unsigned)p_context->seen_sensors & ~(unsigned)bit);

            if (b_is_enabled)
            {
                p_output->fault_mask =
                    (uint8_t)((unsigned)p_output->fault_mask | (unsigned)bit);
            }
        }
    }

    return b_is_sample_time_invalid;
}

/** @brief Update the current sensor and odometry fault bits, healthy interval
 * and lifecycle fault latch from this frame.
 * @details Accept optional odometry only when its distance is finite and
 * its timestamp is recent relative to the frame. A change from the last
 * accepted epoch is itself a fault. Start the healthy interval when all
 * channels are valid and no current fault remains. During an active run,
 * a fault latches the lifecycle and clears capture and decoder progress.
 * Once latched, later frames can add fault bits until the caller resets.
 */
static void process_health (line_navigation_t *                p_context,
                            const ir_acquisition_frame_t *     p_frame,
                            const line_navigation_odometry_t * p_odometry,
                            line_navigation_output_t *         p_output,
                            bool b_is_sample_time_invalid)
{
    p_context->b_has_odometry =
        (((p_odometry != 0) && (p_odometry->b_is_valid)) &&
         (isfinite(p_odometry->centre_m))) &&
        ((p_frame->timestamp_us - p_odometry->timestamp_us) <=
         p_context->config.max_sample_age_us);
    const bool b_has_epoch_changed =
        ((p_context->b_has_odometry) && (p_context->b_has_seen_odometry)) &&
        (p_odometry->epoch != p_context->odometry.epoch);

    if (p_context->b_has_odometry)
    {
        p_context->odometry            = *p_odometry;
        p_context->b_has_seen_odometry = true;
    }
    /* Fault enumerators are masks in 1..8. Widen each to unsigned before
     * adding it to the current-fault word. */
    p_context->current_faults = 0U;

    if (p_output->fault_mask != 0U)
    {
        p_context->current_faults |= (unsigned)S3_FAULT_SENSOR;
    }

    if (b_is_sample_time_invalid)
    {
        p_context->current_faults |= (unsigned)S3_FAULT_TIMING;
    }

    if (!p_context->b_has_odometry)
    {
        p_context->current_faults |= (unsigned)S3_FAULT_ODOMETRY;
    }

    if (b_has_epoch_changed)
    {
        p_context->current_faults |= (unsigned)S3_FAULT_ODOMETRY_EPOCH;
    }

    if ((0U == p_context->current_faults) &&
        (S3_ALL_SENSORS == p_context->valid_sensors))
    {
        if (!p_context->b_is_healthy_tracking)
        {
            p_context->healthy_since_us      = p_frame->timestamp_us;
            p_context->b_is_healthy_tracking = true;
        }
    }
    else
    {
        p_context->b_is_healthy_tracking = false;
    }

    if ((S3_RUN_ACTIVE == (int)p_context->lifecycle) &&
        (p_context->current_faults != 0U))
    {
        p_context->lifecycle      = S3_RUN_FAULT_LATCHED;
        p_context->latched_faults = p_context->current_faults;
        p_output->events         |= (unsigned)S3_EVENT_SENSOR_FAULT;
        reset_barcode(p_context);
        p_context->capture_distance = (line_navigation_distance_t){0};
    }
    else if (S3_RUN_FAULT_LATCHED == (int)p_context->lifecycle)
    {
        p_context->latched_faults =
            (uint8_t)((unsigned)p_context->latched_faults |
                      (unsigned)p_context->current_faults);
    }
    else
    {
        /* No latched fault update is needed in this lifecycle state. */
    }
}

/* End of file. */


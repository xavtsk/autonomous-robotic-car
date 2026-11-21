/** @file line_detection.c
 * @brief Track line error, loss, reacquisition and junction persistence.
 */
#include "line_detection.h"

static bool persisted (line_detection_persistence_t * p_timer,
                       bool b_is_condition_met, uint32_t now,
                       uint32_t duration);

/** @brief Return the line detector's default darkness thresholds, persistence
 * durations in microseconds and line geometry selection.
 */
line_detection_config_t line_detection_defaults (void)
{
    return (line_detection_config_t){200U,   600U,    100000U, 30000U,
                                     30000U, 100000U, 30000U,  false};
}

/** @brief Clear detector history and latches while retaining the current
 * configuration and configured flag. A null pointer is ignored.
 */
void line_detection_reset (line_detection_t * p_detector)
{
    if (p_detector != 0)
    {
        line_detection_config_t cfg              = p_detector->config;
        const bool              b_was_configured = p_detector->b_is_configured;
        *p_detector                              = (line_detection_t){0};
        p_detector->config                       = cfg;
        p_detector->b_is_configured              = b_was_configured;
    }
}

/** @brief Clear the supplied detector, validate the thresholds and timing
 * settings, then store an accepted configuration. Invalid settings
 * leave the detector unconfigured.
 */
bool line_detection_init (line_detection_t *              p_detector,
                          const line_detection_config_t * p_config)
{
    if (0 == p_detector)
    {
        return false;
    }

    *p_detector = (line_detection_t){0};

    if ((((((((((((((((0 == p_config) || (0U == p_config->signal_floor)) ||
                     (p_config->signal_floor > 2000U)) ||
                    (0U == p_config->junction_dark)) ||
                   (p_config->junction_dark > 1000U)) ||
                  (0U == p_config->lost_us)) ||
                 (0U == p_config->reacquire_us)) ||
                (0U == p_config->junction_us)) ||
               (0U == p_config->rearm_us)) ||
              (0U == p_config->max_gap_us)) ||
             (p_config->max_gap_us >= 0x80000000U)) ||
            (p_config->lost_us >= 0x80000000U)) ||
           (p_config->reacquire_us >= 0x80000000U)) ||
          (p_config->junction_us >= 0x80000000U)) ||
         (p_config->rearm_us >= 0x80000000U)) ||
        ((p_config->b_is_tape_between) && (p_config->signal_floor > 1000U)))
    {
        return false;
    }

    p_detector->config          = *p_config;
    p_detector->b_is_configured = true;
    return true;
}

/** @brief Evaluate left and right darkness in the range 0 to 1000 at the given
 * microsecond timestamp. Invalid input or a bad sample gap clears
 * history. Return line error, presence and timed junction or loss
 * events according to configured geometry.
 */
line_detection_observation_t
line_detection_update (line_detection_t * p_detector, uint32_t timestamp_us,
                       bool b_are_inputs_valid, uint16_t left_dark,
                       uint16_t right_dark)
{
    line_detection_observation_t out = {false};

    if ((0 == p_detector) || (!p_detector->b_is_configured))
    {
        return out;
    }

    const bool b_is_gap_invalid = (p_detector->b_has_seen_sample) &&
                                  (((timestamp_us - p_detector->last_us) >
                                    p_detector->config.max_gap_us) ||
                                   (timestamp_us == p_detector->last_us));

    if ((((!b_are_inputs_valid) || (left_dark > 1000U)) ||
         (right_dark > 1000U)) ||
        b_is_gap_invalid)
    {
        line_detection_reset(p_detector);
        return out;
    }

    /* Both validated darkness inputs fit int32_t. Their signed difference
     * is in -1000..1000, so division by ten fits the retained int16_t. */
    /* clang-format off */
    p_detector->last_us           = timestamp_us;
    p_detector->b_has_seen_sample = true;
    out.b_is_valid                = true;
    out.error                     =
        (int16_t)(((int32_t)right_dark - (int32_t)left_dark) / 10);
    /* clang-format on */
    const bool b_are_both_black =
        (left_dark >= p_detector->config.junction_dark) &&
        (right_dark >= p_detector->config.junction_dark);

    bool b_has_signal;
    if (p_detector->config.b_is_tape_between)
    {
        /* The tape lies between the sensors: both white = centred. There is
         * no "line lost" while following; a sensor touching the tape is
         * steered away from by the unchanged error term below. */
        /* clang-format off */
        out.b_is_touching_tape        =
            (left_dark >= p_detector->config.signal_floor) ||
            (right_dark >= p_detector->config.signal_floor);
        b_has_signal                  = true;
        p_detector->b_is_line_present = true;
        /* clang-format on */
    }
    else
    {
        b_has_signal = ((uint32_t)left_dark + right_dark) >=
                       p_detector->config.signal_floor;

        if ((persisted(&p_detector->lost, !b_has_signal, timestamp_us,
                       p_detector->config.lost_us)) &&
            (!p_detector->b_was_loss_reported))
        {
            p_detector->b_is_line_present   = false;
            out.b_has_lost_event            = true;
            p_detector->b_was_loss_reported = true;
        }

        if ((persisted(&p_detector->acquired, b_has_signal, timestamp_us,
                       p_detector->config.reacquire_us)) &&
            (!p_detector->b_is_line_present))
        {
            p_detector->b_is_line_present   = true;
            p_detector->b_was_loss_reported = false;
            out.b_has_reacquired_event      = true;
        }
    }

    if ((persisted(&p_detector->junction, b_are_both_black, timestamp_us,
                   p_detector->config.junction_us)) &&
        (!p_detector->b_is_junction_latched))
    {
        p_detector->b_is_junction_latched = true;
        out.b_has_junction_event          = true;
    }

    if (persisted(&p_detector->clear, !b_are_both_black, timestamp_us,
                  p_detector->config.rearm_us))
    {
        p_detector->b_is_junction_latched = false;
    }

    /* clang-format off */
    out.b_is_line_present        =
        (p_detector->b_is_line_present) && b_has_signal;
    out.b_has_junction_candidate =
        b_are_both_black || (p_detector->b_is_junction_latched);
    /* clang-format on */
    return out;
}

/** @brief Measure continuous assertion in wrapping microseconds. A false
 * condition clears tracking. A newly true condition starts the
 * interval at now.
 */
static bool persisted (line_detection_persistence_t * p_timer,
                       bool b_is_condition_met, uint32_t now, uint32_t duration)
{
    if (!b_is_condition_met)
    {
        p_timer->b_is_active = false;
        return false;
    }

    if (!p_timer->b_is_active)
    {
        p_timer->b_is_active = true;
        p_timer->since_us    = now;
    }

    return (now - p_timer->since_us) >= duration;
}

/* End of file. */


/** @file line_detection.c
 * @brief Track line error, loss, reacquisition and junction persistence.
 */
#include "line_detection.h"

s3_line_config_t s3_line_defaults (void)
{
    return (s3_line_config_t){200U,   600U,    100000U, 30000U,
                              30000U, 100000U, 30000U,  false};
}

void s3_line_reset (s3_line_detector_t * p_detector)
{
    if (p_detector != 0)
    {
        s3_line_config_t cfg = p_detector->config;
        bool configured = p_detector->configured;
        *p_detector = (s3_line_detector_t){0};
        p_detector->config = cfg;
        p_detector->configured = configured;
    }
}

bool s3_line_init (s3_line_detector_t * p_detector,
                   const s3_line_config_t * p_config)
{
    if (p_detector == 0)
    {
        return false;
    }
    *p_detector = (s3_line_detector_t){0};
    if ((p_config == 0) || (p_config->signal_floor == 0U) ||
        (p_config->signal_floor > 2000U) || (p_config->junction_dark == 0U) ||
        (p_config->junction_dark > 1000U) || (p_config->lost_us == 0U) ||
        (p_config->reacquire_us == 0U) || (p_config->junction_us == 0U) ||
        (p_config->rearm_us == 0U) || (p_config->max_gap_us == 0U) ||
        (p_config->max_gap_us >= 0x80000000U) ||
        (p_config->lost_us >= 0x80000000U) ||
        (p_config->reacquire_us >= 0x80000000U) ||
        (p_config->junction_us >= 0x80000000U) ||
        (p_config->rearm_us >= 0x80000000U) ||
        (p_config->between && (p_config->signal_floor > 1000U)))
    {
        return false;
    }
    p_detector->config = *p_config;
    p_detector->configured = true;
    return true;
}

static bool persisted (s3_persistence_t * p_timer, bool condition, uint32_t now,
                       uint32_t duration)
{
    if (!condition)
    {
        p_timer->active = false;
        return false;
    }
    if (!p_timer->active)
    {
        p_timer->active = true;
        p_timer->since_us = now;
    }
    return (uint32_t)(now - p_timer->since_us) >= duration;
}

s3_line_observation_t s3_line_update (s3_line_detector_t * p_detector,
                                      uint32_t timestamp_us, bool inputs_valid,
                                      uint16_t left_dark, uint16_t right_dark)
{
    s3_line_observation_t out = {0};
    bool signal, both_black, gap_bad;
    if ((p_detector == 0) || !p_detector->configured)
    {
        return out;
    }
    gap_bad = p_detector->seen_sample &&
              (((uint32_t)(timestamp_us - p_detector->last_us) >
                p_detector->config.max_gap_us) ||
               (timestamp_us == p_detector->last_us));
    if (!inputs_valid || (left_dark > 1000U) || (right_dark > 1000U) || gap_bad)
    {
        s3_line_reset(p_detector);
        return out;
    }
    p_detector->last_us = timestamp_us;
    p_detector->seen_sample = true;
    out.valid = true;
    out.error = (int16_t)(((int32_t)right_dark - (int32_t)left_dark) / 10);
    both_black = (left_dark >= p_detector->config.junction_dark) &&
                 (right_dark >= p_detector->config.junction_dark);
    if (p_detector->config.between)
    {
        /* The tape lies between the sensors: both white = centred. There is
         * no "line lost" while following; a sensor touching the tape is
         * steered away from by the unchanged error term below. */
        out.touching = (left_dark >= p_detector->config.signal_floor) ||
                       (right_dark >= p_detector->config.signal_floor);
        signal = true;
        p_detector->present = true;
    }
    else
    {
        signal = ((uint32_t)left_dark + right_dark) >=
                 p_detector->config.signal_floor;
        if (persisted(&p_detector->lost, !signal, timestamp_us,
                      p_detector->config.lost_us) &&
            !p_detector->loss_reported)
        {
            p_detector->present = false;
            out.lost_event = true;
            p_detector->loss_reported = true;
        }
        if (persisted(&p_detector->acquired, signal, timestamp_us,
                      p_detector->config.reacquire_us) &&
            !p_detector->present)
        {
            p_detector->present = true;
            p_detector->loss_reported = false;
            out.reacquired_event = true;
        }
    }
    if (persisted(&p_detector->junction, both_black, timestamp_us,
                  p_detector->config.junction_us) &&
        !p_detector->junction_latched)
    {
        p_detector->junction_latched = true;
        out.junction_event = true;
    }
    if (persisted(&p_detector->clear, !both_black, timestamp_us,
                  p_detector->config.rearm_us))
    {
        p_detector->junction_latched = false;
    }
    out.present = p_detector->present && signal;
    out.junction_candidate = both_black || p_detector->junction_latched;
    return out;
}

/* End of file. */


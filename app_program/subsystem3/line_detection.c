#include "line_detection.h"
s3_line_config_t s3_line_defaults(void)
{
    return (s3_line_config_t){200U,   600U,    100000U, 30000U,
                              30000U, 100000U, 30000U};
}
void s3_line_reset(s3_line_detector_t *detector)
{
    if (detector != 0)
    {
        s3_line_config_t cfg = detector->config;
        bool configured = detector->configured;
        *detector = (s3_line_detector_t){0};
        detector->config = cfg;
        detector->configured = configured;
    }
}
bool s3_line_init(s3_line_detector_t *detector, const s3_line_config_t *config)
{
    if (detector == 0)
    {
        return false;
    }
    *detector = (s3_line_detector_t){0};
    if ((config == 0) || (config->signal_floor == 0U) ||
        (config->signal_floor > 2000U) || (config->junction_dark == 0U) ||
        (config->junction_dark > 1000U) || (config->lost_us == 0U) ||
        (config->reacquire_us == 0U) || (config->junction_us == 0U) ||
        (config->rearm_us == 0U) || (config->max_gap_us == 0U) ||
        (config->max_gap_us >= 0x80000000U) ||
        (config->lost_us >= 0x80000000U) ||
        (config->reacquire_us >= 0x80000000U) ||
        (config->junction_us >= 0x80000000U) ||
        (config->rearm_us >= 0x80000000U))
    {
        return false;
    }
    detector->config = *config;
    detector->configured = true;
    return true;
}
static bool persisted(s3_persistence_t *timer, bool condition, uint32_t now,
                      uint32_t duration)
{
    if (!condition)
    {
        timer->active = false;
        return false;
    }
    if (!timer->active)
    {
        timer->active = true;
        timer->since_us = now;
    }
    return (uint32_t)(now - timer->since_us) >= duration;
}
s3_line_observation_t s3_line_update(s3_line_detector_t *detector,
                                     uint32_t timestamp_us, bool inputs_valid,
                                     uint16_t left_dark, uint16_t right_dark)
{
    s3_line_observation_t out = {0};
    bool signal, both_black, gap_bad;
    if ((detector == 0) || !detector->configured)
    {
        return out;
    }
    gap_bad = detector->seen_sample &&
              (((uint32_t)(timestamp_us - detector->last_us) >
                detector->config.max_gap_us) ||
               (timestamp_us == detector->last_us));
    if (!inputs_valid || (left_dark > 1000U) || (right_dark > 1000U) || gap_bad)
    {
        s3_line_reset(detector);
        return out;
    }
    detector->last_us = timestamp_us;
    detector->seen_sample = true;
    out.valid = true;
    out.error = (int16_t)(((int32_t)right_dark - (int32_t)left_dark) / 10);
    signal =
        ((uint32_t)left_dark + right_dark) >= detector->config.signal_floor;
    both_black = (left_dark >= detector->config.junction_dark) &&
                 (right_dark >= detector->config.junction_dark);
    if (persisted(&detector->lost, !signal, timestamp_us,
                  detector->config.lost_us) &&
        !detector->loss_reported)
    {
        detector->present = false;
        out.lost_event = true;
        detector->loss_reported = true;
    }
    if (persisted(&detector->acquired, signal, timestamp_us,
                  detector->config.reacquire_us) &&
        !detector->present)
    {
        detector->present = true;
        detector->loss_reported = false;
        out.reacquired_event = true;
    }
    if (persisted(&detector->junction, both_black, timestamp_us,
                  detector->config.junction_us) &&
        !detector->junction_latched)
    {
        detector->junction_latched = true;
        out.junction_event = true;
    }
    if (persisted(&detector->clear, !both_black, timestamp_us,
                  detector->config.rearm_us))
    {
        detector->junction_latched = false;
    }
    out.present = detector->present && signal;
    out.junction_candidate = both_black || detector->junction_latched;
    return out;
}

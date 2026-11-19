#include "line_controller.h"

static bool finite_number (float value_x)
{
    return (value_x == value_x) && ((value_x - value_x) == 0.0f);
}

s3_steering_config_t s3_steering_defaults (void)
{
    return (s3_steering_config_t){0.10f, 0.06f, 0.20f};
}

bool s3_steering_config_valid (const s3_steering_config_t * p_config)
{
    return (p_config != 0) && finite_number(p_config->base_mps) &&
           finite_number(p_config->kp_mps) &&
           finite_number(p_config->maximum_mps) &&
           (p_config->base_mps > 0.0f) && (p_config->kp_mps >= 0.0f) &&
           (p_config->maximum_mps > 0.0f) &&
           (p_config->base_mps <= p_config->maximum_mps);
}

static float clamp_speed (float speed, float maximum)
{
    if (speed < 0.0f)
    {
        return 0.0f;
    }
    return (speed > maximum) ? maximum : speed;
}

s3_steering_request_t
s3_steering_compute (const s3_steering_config_t * p_config,
                     const s3_line_observation_t * p_line)
{
    s3_steering_request_t out = {0};
    float correction;
    if (!s3_steering_config_valid(p_config) || (p_line == 0) ||
        !p_line->valid || !p_line->present || p_line->junction_candidate ||
        (p_line->error < -100) || (p_line->error > 100))
    {
        return out;
    }
    correction = p_config->kp_mps * ((float)p_line->error / 100.0f);
    out.left_mps =
        clamp_speed(p_config->base_mps + correction, p_config->maximum_mps);
    out.right_mps =
        clamp_speed(p_config->base_mps - correction, p_config->maximum_mps);
    out.valid = true;
    return out;
}

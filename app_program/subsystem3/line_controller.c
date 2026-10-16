#include "line_controller.h"
static bool finite_number(float x) { return (x == x) && ((x-x) == 0.0f); }
s3_steering_config_t s3_steering_defaults(void)
{
    return (s3_steering_config_t){0.10f, 0.06f, 0.20f};
}
bool s3_steering_config_valid(const s3_steering_config_t *config)
{
    return (config != 0) && finite_number(config->base_mps) &&
        finite_number(config->kp_mps) && finite_number(config->maximum_mps) &&
        (config->base_mps > 0.0f) && (config->kp_mps >= 0.0f) &&
        (config->maximum_mps > 0.0f) && (config->base_mps <= config->maximum_mps);
}
static float clamp_speed(float speed, float maximum)
{
    if (speed < 0.0f) { return 0.0f; }
    return (speed > maximum) ? maximum : speed;
}
s3_steering_request_t s3_steering_compute(const s3_steering_config_t *config,
                                         const s3_line_observation_t *line)
{
    s3_steering_request_t out = {0};
    float correction;
    if (!s3_steering_config_valid(config) || (line == 0) || !line->valid ||
        !line->present || line->junction_candidate || (line->error < -100) || (line->error > 100)) {
        return out;
    }
    correction = config->kp_mps * ((float)line->error / 100.0f);
    out.left_mps = clamp_speed(config->base_mps + correction, config->maximum_mps);
    out.right_mps = clamp_speed(config->base_mps - correction, config->maximum_mps);
    out.valid = true;
    return out;
}

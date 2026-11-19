/*
 * File: estimate_motion.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Encoder-based speed and distance estimation.
 */

#include "estimate_motion.h"
#include "../app_types.h"

#define SPEED_FILTER (0.4f) /* 0..1, 1 = no smoothing */
#define RAD_TO_DEG (57.2957795f)

/* Speeds stay float: control precision (DATA_FORMAT.md, internal fields) */
typedef struct
{
    uint32_t last_count : WHEEL_COUNT_BITS;
    volatile float32_t speed_mps;
    volatile float32_t accel_mps2;
} wheel_estimate_t;

static wheel_estimate_t g_wheel[WHEEL_COUNT];
static volatile float32_t g_m_per_count = 0.0f;
static float32_t g_track_width_m = 0.0f;

void motion_estimation_init (float32_t m_per_count, float32_t track_width_m)
{
    uint32_t value_i;

    g_m_per_count = m_per_count;
    g_track_width_m = track_width_m;
    for (value_i = 0U; value_i < (uint32_t)WHEEL_COUNT; value_i++)
    {
        g_wheel[value_i].last_count = wheel_encoder_bits((wheel_id_t)value_i);
        g_wheel[value_i].speed_mps = 0.0f;
        g_wheel[value_i].accel_mps2 = 0.0f;
    }
}

void motion_estimation_set_scale (float32_t m_per_count)
{
    g_m_per_count = m_per_count;
}

float32_t motion_estimation_scale (void)
{
    return g_m_per_count;
}

void motion_estimation_update (float32_t dt_s)
{
    uint32_t value_i;

    for (value_i = 0U; value_i < (uint32_t)WHEEL_COUNT; value_i++)
    {
        wheel_estimate_t * p_w = &g_wheel[value_i];
        uint32_t count = wheel_encoder_bits((wheel_id_t)value_i);
        float32_t raw = (float32_t)wheel_encoder_delta(count, p_w->last_count) *
                    g_m_per_count / dt_s;
        float32_t prev = p_w->speed_mps;

        p_w->last_count = count;
        p_w->speed_mps = prev + (SPEED_FILTER * (raw - prev));
        p_w->accel_mps2 = (p_w->speed_mps - prev) / dt_s;
    }
}

float32_t motion_estimation_wheel_speed (wheel_id_t wheel)
{
    return g_wheel[wheel].speed_mps;
}

float32_t motion_estimation_wheel_accel (wheel_id_t wheel)
{
    return g_wheel[wheel].accel_mps2;
}

float32_t motion_estimation_wheel_dist (wheel_id_t wheel)
{
    return (float32_t)wheel_encoder_count(wheel) * g_m_per_count;
}

float32_t motion_estimation_robot_speed (void)
{
    return 0.5f *
           (g_wheel[WHEEL_LEFT].speed_mps + g_wheel[WHEEL_RIGHT].speed_mps);
}

float32_t motion_estimation_robot_dist (void)
{
    return 0.5f * (motion_estimation_wheel_dist(WHEEL_LEFT) +
                   motion_estimation_wheel_dist(WHEEL_RIGHT));
}

float32_t motion_estimation_heading_deg (void)
{
    if (g_track_width_m <= 0.0f)
    {
        return 0.0f;
    }

    return (motion_estimation_wheel_dist(WHEEL_RIGHT) -
            motion_estimation_wheel_dist(WHEEL_LEFT)) /
           g_track_width_m * RAD_TO_DEG;
}

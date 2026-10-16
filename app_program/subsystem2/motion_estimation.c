/*
 * File: motion_estimation.c
 * Owner: Buddy 2 - Motion Control
 * Purpose: Encoder-based speed and distance estimation.
 */

#include "motion_estimation.h"

#define SPEED_FILTER            (0.4f)          /* 0..1, 1 = no smoothing */
#define RAD_TO_DEG              (57.2957795f)

typedef struct
{
    int32_t         last_count;
    volatile float  speed_mps;
    volatile float  accel_mps2;
} wheel_estimate_t;

static wheel_estimate_t g_wheel[WHEEL_COUNT];
static volatile float   g_m_per_count = 0.0f;
static float            g_track_width_m = 0.0f;

void motion_estimation_init(float m_per_count, float track_width_m)
{
    uint32_t i;

    g_m_per_count = m_per_count;
    g_track_width_m = track_width_m;
    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        g_wheel[i].last_count = wheel_encoder_count((wheel_id_t)i);
        g_wheel[i].speed_mps  = 0.0f;
        g_wheel[i].accel_mps2 = 0.0f;
    }
}

void motion_estimation_set_scale(float m_per_count)
{
    g_m_per_count = m_per_count;
}

float motion_estimation_scale(void)
{
    return g_m_per_count;
}

void motion_estimation_update(float dt_s)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)WHEEL_COUNT; i++)
    {
        wheel_estimate_t *p_w = &g_wheel[i];
        int32_t count = wheel_encoder_count((wheel_id_t)i);
        float   raw = (float)(count - p_w->last_count) * g_m_per_count / dt_s;
        float   prev = p_w->speed_mps;

        p_w->last_count = count;
        p_w->speed_mps  = prev + (SPEED_FILTER * (raw - prev));
        p_w->accel_mps2 = (p_w->speed_mps - prev) / dt_s;
    }
}

float motion_estimation_wheel_speed(wheel_id_t wheel)
{
    return g_wheel[wheel].speed_mps;
}

float motion_estimation_wheel_accel(wheel_id_t wheel)
{
    return g_wheel[wheel].accel_mps2;
}

float motion_estimation_wheel_distance(wheel_id_t wheel)
{
    return (float)wheel_encoder_count(wheel) * g_m_per_count;
}

float motion_estimation_robot_speed(void)
{
    return 0.5f * (g_wheel[WHEEL_LEFT].speed_mps
                   + g_wheel[WHEEL_RIGHT].speed_mps);
}

float motion_estimation_robot_distance(void)
{
    return 0.5f * (motion_estimation_wheel_distance(WHEEL_LEFT)
                   + motion_estimation_wheel_distance(WHEEL_RIGHT));
}

float motion_estimation_robot_heading_deg(void)
{
    if (g_track_width_m <= 0.0f)
    {
        return 0.0f;
    }

    return (motion_estimation_wheel_distance(WHEEL_RIGHT)
            - motion_estimation_wheel_distance(WHEEL_LEFT))
           / g_track_width_m * RAD_TO_DEG;
}

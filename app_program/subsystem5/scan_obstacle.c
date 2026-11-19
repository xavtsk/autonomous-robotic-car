#include "scan_obstacle.h"

void s5_scan_begin (s5_scan_t * p_s, uint32_t now)
{
    *p_s = (s5_scan_t){0};
    p_s->angle = 30U;
    p_s->nearest = 90U;
    p_s->nearest_mm = S5_RANGE_MAX_MM;
    p_s->started_us = now;
}

bool s5_peek_angle_valid (unsigned angle)
{
    return angle % 5U == 0U &&
           ((angle >= 30U && angle <= 60U) || (angle >= 120U && angle <= 150U));
}

bool s5_scan_begin_at (s5_scan_t * p_s, uint32_t now, unsigned angle)
{
    if (!p_s || !s5_peek_angle_valid(angle))
    {
        return false;
    }
    s5_scan_begin(p_s, now);
    p_s->angle = (uint8_t)angle;
    p_s->single = 1U;
    return true;
}

void s5_scan_accept (s5_scan_t * p_s, uint32_t now, bool valid,
                     unsigned value_mm)
{
    if (!p_s || p_s->done)
    {
        return;
    }
    if ((uint32_t)(now - p_s->started_us) >= S5_SCAN_TIMEOUT_US)
    {
        p_s->failed = p_s->done = true;
        return;
    }
    if (!valid || value_mm < S5_RANGE_MIN_MM || value_mm > S5_RANGE_MAX_MM)
    {
        if (++p_s->attempts < 3U)
        {
            return;
        }
        p_s->failed = p_s->done = true;
        return;
    }
    p_s->attempts = 0;
    if (p_s->count >= S5_SAMPLES_MAX ||
        !s5_sample_pack(p_s->angle, value_mm, true, &p_s->sample[p_s->count]))
    {
        p_s->failed = p_s->done = true;
        return;
    }
    p_s->count++;
    if (p_s->single)
    {
        p_s->done = true;
        return;
    }
    if (!p_s->fine && value_mm < p_s->nearest_mm)
    {
        p_s->nearest_mm = (uint16_t)value_mm;
        p_s->nearest = p_s->angle;
    }
    if (!p_s->fine)
    {
        if (p_s->angle < 150U)
        {
            p_s->angle += 30U;
        }
        else
        {
            p_s->fine = true;
            p_s->angle = p_s->nearest > 45U ? p_s->nearest - 15U : 30U;
            p_s->fine_end = p_s->nearest < 135U ? p_s->nearest + 15U : 150U;
        }
    }
    else if (p_s->angle < p_s->fine_end)
    {
        p_s->angle += 5U;
    }
    else
    {
        p_s->done = true;
    }
}

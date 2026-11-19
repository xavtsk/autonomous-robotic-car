#include "profile_obstacle.h"
/* sin/cos of angles -60..60 in 5-degree steps, scaled by 1000.
 * x positive LEFT, y forward, servo 90 straight, 150 left. */
static const int16_t g_sx[25] = {
    -866, -819, -766, -707, -643, -574, -500, -423, -342, -259, -174, -87, 0,
    87,   174,  259,  342,  423,  500,  574,  643,  707,  766,  819,  866};
static const int16_t g_cy[25] = {500, 574, 643, 707,  766, 819, 866, 906, 940,
                                 966, 985, 996, 1000, 996, 985, 966, 940, 906,
                                 866, 819, 766, 707,  643, 574, 500};

bool s5_sample_pack (unsigned angle, unsigned value_mm, bool valid,
                     s5_sample_t * p_out)
{
    uint32_t word;
    if (!p_out || angle < 30U || angle > 150U || (angle - 30U) % 5U ||
        (valid && (value_mm < S5_RANGE_MIN_MM || value_mm > S5_RANGE_MAX_MM)))
    {
        return false;
    }
    word = (valid ? value_mm : 0U) | (((angle - 30U) / 5U) << 12) |
           ((uint32_t)valid << 17);
    for (unsigned value_i = 0; value_i < 3; value_i++)
    {
        p_out->bytes[value_i] = (uint8_t)(word >> (8U * value_i));
    }
    return true;
}

bool s5_sample_unpack (const s5_sample_t * p_s, unsigned * p_angle,
                       unsigned * p_mm, bool * p_valid)
{
    uint32_t value_w;
    if (!p_s || !p_angle || !p_mm || !p_valid)
    {
        return false;
    }
    value_w = (uint32_t)p_s->bytes[0] | ((uint32_t)p_s->bytes[1] << 8) |
              ((uint32_t)p_s->bytes[2] << 16);
    if ((value_w >> 18) || ((value_w >> 12) & 31U) > 24U)
    {
        return false;
    }
    *p_angle = 30U + 5U * ((value_w >> 12) & 31U);
    *p_mm = value_w & 4095U;
    *p_valid = ((value_w >> 17) & 1U) != 0;
    return *p_valid ? (*p_mm >= S5_RANGE_MIN_MM && *p_mm <= S5_RANGE_MAX_MM)
                    : *p_mm == 0U;
}

bool s5_profile_build (const s5_sample_t * p_samples, unsigned count,
                       const s5_config_t * p_cfg, s5_profile_t * p_out)
{
    s5_profile_t value_p = {0};
    bool found = false, left = false, right = false, centre = false;
    unsigned far = 0;
    if (!p_out)
    {
        return false;
    }
    *p_out = value_p;
    if (!p_samples || !p_cfg || count < 5U || count > S5_SAMPLES_MAX)
    {
        return false;
    }
    value_p.closest_mm = value_p.left_clear_mm = value_p.right_clear_mm =
        S5_RANGE_MAX_MM;
    value_p.left_edge_mm = -4000;
    value_p.right_edge_mm = 4000;
    for (unsigned value_i = 0; value_i < count; value_i++)
    {
        unsigned angle, value_mm, index;
        bool valid;
        if (!s5_sample_unpack(&p_samples[value_i], &angle, &value_mm, &valid) ||
            !valid)
        {
            return false;
        }
        index = (angle - 30U) / 5U;
        /* Side clearance only from >=45 deg off-centre: at 60/120 the ~30 deg
         * beam still hits a close obstacle and made both sides look blocked. */
        if (angle >= 135U)
        {
            left = true;
            if (value_mm < value_p.left_clear_mm)
            {
                value_p.left_clear_mm = (uint16_t)value_mm;
            }
        }
        if (angle <= 45U)
        {
            right = true;
            if (value_mm < value_p.right_clear_mm)
            {
                value_p.right_clear_mm = (uint16_t)value_mm;
            }
        }
        if (angle == 90U)
        {
            centre = true;
        }
        /* Obstacle shape only from 45..135 deg; 30/150 are side-clearance
         * readings (track surroundings there made the obstacle look 391 mm
         * wide). */
        if (value_mm < p_cfg->profile_mm && angle >= 45U && angle <= 135U)
        {
            /* Beam-width correction: the HC-SR04 reports the nearest return
             * anywhere in its ~30 deg cone, so a return at servo angle t may
             * come from up to 15 deg (3 table steps) nearer the centre. */
            unsigned value_c = index > 12U
                                   ? (index - 3U > 12U ? index - 3U : 12U)
                                   : (index + 3U < 12U ? index + 3U : 12U);
            int value_x = (int)value_mm * g_sx[value_c] / 1000,
                value_y = (int)value_mm * g_cy[value_c] / 1000;
            found = true;
            if (value_mm > far)
            {
                far = value_mm;
            }
            if (value_mm < value_p.closest_mm)
            {
                value_p.closest_mm = (uint16_t)value_mm;
            }
            if (value_x > value_p.left_edge_mm)
            {
                value_p.left_edge_mm = (int16_t)value_x;
            }
            if (value_x < value_p.right_edge_mm)
            {
                value_p.right_edge_mm = (int16_t)value_x;
            }
            if (value_y > value_p.forward_mm)
            {
                value_p.forward_mm = (uint16_t)value_y;
            }
        }
    }
    if (!found || !left || !right || !centre)
    {
        return false;
    }
    /* A point return is not an obstacle edge. With the beam already corrected
     * above, pad by S5_EDGE_INFLATE_PERMIL (sin of the padding angle, 5 deg =
     * one fine step; was 15 deg) at the FARTHEST obstacle return seen. Leg 1
     * still checks the real diagonal path with the sensor before driving it.
     * Not a promise that acoustically invisible surfaces are detectable. */
    int uncertainty = (int)((far * S5_EDGE_INFLATE_PERMIL + 999U) / 1000U);
    value_p.left_edge_mm = (int16_t)(value_p.left_edge_mm + uncertainty);
    value_p.right_edge_mm = (int16_t)(value_p.right_edge_mm - uncertainty);
    value_p.width_mm = (uint16_t)(value_p.left_edge_mm - value_p.right_edge_mm);
    value_p.valid = true;
    *p_out = value_p;
    return true;
}

bool s5_peek_merge (s5_profile_t * p_p, const s5_sample_t * p_s)
{
    unsigned angle, value_mm;
    bool valid;
    if (!p_p || !p_s || !s5_sample_unpack(p_s, &angle, &value_mm, &valid) ||
        !valid || (angle > 60U && angle < 120U))
    {
        return false;
    }
    if (!p_p->valid)
    {
        *p_p = (s5_profile_t){0};
        p_p->closest_mm = (uint16_t)value_mm;
        p_p->left_clear_mm = p_p->right_clear_mm = S5_RANGE_MAX_MM;
        p_p->valid = true;
    }
    if (angle >= 120U)
    {
        p_p->left_clear_mm = (uint16_t)value_mm;
    }
    else
    {
        p_p->right_clear_mm = (uint16_t)value_mm;
    }
    return true;
}

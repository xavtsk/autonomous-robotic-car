/** @file profile_obstacle.c
 * @brief Pack scan samples and derive obstacle profiles and side-peek updates.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 */
#include "profile_obstacle.h"
/* The sine and cosine tables cover -60..60 degrees in five-degree steps
 * and scale each value by 1000. Positive x is left and positive y is
 * forward. The servo faces straight at 90 degrees and left at 150. */
static const int16_t g_sx[25] = {
    -866, -819, -766, -707, -643, -574, -500, -423, -342, -259, -174, -87, 0,
    87,   174,  259,  342,  423,  500,  574,  643,  707,  766,  819,  866};
static const int16_t g_cy[25] = {500, 574, 643, 707,  766, 819, 866, 906, 940,
                                 966, 985, 996, 1000, 996, 985, 966, 940, 906,
                                 866, 819, 766, 707,  643, 574, 500};

/** @brief Validate angle/range before writing one canonical sample packet.
 * @see profile_obstacle.h for integer units, invalid echoes and bit layout.
 */
bool profile_obstacle_sample_pack (unsigned angle, unsigned value_mm,
                                   bool                        b_is_valid,
                                   profile_obstacle_sample_t * p_out)
{

    if (((((0 == p_out) || (angle < 30U)) || (angle > 150U)) ||
         ((angle - 30U) % 5U)) ||
        (b_is_valid &&
         ((value_mm < S5_RANGE_MIN_MM) || (value_mm > S5_RANGE_MAX_MM))))
    {
        return false;
    }
    /* Widen the Boolean zero or one before placing it in bit 17. */
    const uint32_t word = (b_is_valid ? value_mm : 0U) |
                          (((angle - 30U) / 5U) << 12) |
                          ((uint32_t)b_is_valid << 17);

    for (unsigned byte_index = 0; byte_index < sizeof p_out->bytes;
         byte_index++)
    {
        /* Keep the low byte of the shifted 18-bit sample word. */
        p_out->bytes[byte_index] = (uint8_t)(word >> (8U * byte_index));
    }

    return true;
}

/** @brief Decode sample fields before the final semantic range check.
 * @param[in] p_sample Stable three-byte packed sample.
 * @param[out] p_angle Servo degrees; may change on a range failure.
 * @param[out] p_mm Range in mm; may change on a range failure.
 * @param[out] pb_is_valid Echo usability; may change on a range failure.
 * @return True only for valid angle/padding and canonical range/validity.
 * @see profile_obstacle.h for exact ranges and unchanged early-failure cases.
 */
bool profile_obstacle_sample_unpack (const profile_obstacle_sample_t * p_sample,
                                     unsigned * p_angle, unsigned * p_mm,
                                     bool * pb_is_valid)
{

    if ((((0 == p_sample) || (0 == p_angle)) || (0 == p_mm)) ||
        (0 == pb_is_valid))
    {
        return false;
    }
    /* Widen 0..255 before shifting each input byte into the 24-bit word. */
    const uint32_t word = (uint32_t)p_sample->bytes[0] |
                          ((uint32_t)p_sample->bytes[1] << 8) |
                          ((uint32_t)p_sample->bytes[2] << 16);

    if ((word >> 18) || (((word >> 12) & 31U) > 24U))
    {
        return false;
    }

    *p_angle     = 30U + (5U * ((word >> 12) & 31U));
    *p_mm        = word & 4095U;
    *pb_is_valid = ((word >> 17) & 1U) != 0U;
    return *pb_is_valid
               ? ((*p_mm >= S5_RANGE_MIN_MM) && (*p_mm <= S5_RANGE_MAX_MM))
               : 0U == *p_mm;
}

/** @brief Clear output, validate scan coverage and derive padded geometry.
 * @see profile_obstacle.h for axes, integer rounding, threshold and failure
 * behavior. The caller supplies a valid configuration.
 * @details Require centre and both side coverage before returning a profile.
 * Use large off-centre angles for clearance. Narrower angles and returns
 * within the profile threshold define the obstacle. Pad the projected edges
 * by the uncertainty at the farthest accepted return.
 */
bool profile_obstacle_build (const profile_obstacle_sample_t * p_samples,
                             unsigned count, const s5_config_t * p_cfg,
                             profile_obstacle_t * p_out)
{
    profile_obstacle_t profile              = {0};
    bool               b_has_found_obstacle = false;
    bool               b_has_left           = false;
    bool               b_has_right          = false;
    bool               b_has_centre         = false;
    unsigned           far                  = 0;

    if (0 == p_out)
    {
        return false;
    }

    *p_out = profile;

    if ((((0 == p_samples) || (0 == p_cfg)) || (count < 5U)) ||
        (count > S5_SAMPLES_MAX))
    {
        return false;
    }

    /* clang-format off */
    profile.closest_mm    = profile.left_clear_mm = profile.right_clear_mm =
        S5_RANGE_MAX_MM;
    profile.left_edge_mm  = -4000;
    profile.right_edge_mm = 4000;
    /* clang-format on */

    for (unsigned sample_index = 0; sample_index < count; sample_index++)
    {
        bool b_is_sample_valid;

        unsigned angle;
        unsigned value_mm;
        if ((!profile_obstacle_sample_unpack(&p_samples[sample_index], &angle,
                                             &value_mm, &b_is_sample_valid)) ||
            (!b_is_sample_valid))
        {
            return false;
        }
        /* Unpacking limits valid echo distances to 30..4000 mm. All
         * following uint16_t range assignments preserve those readings. */
        const unsigned index = (angle - 30U) / 5U;
        /* Side clearance only from >=45 deg off-centre: at 60/120 the ~30 deg
         * beam still hits a close obstacle and made both sides look blocked. */
        if (angle >= 135U)
        {
            b_has_left = true;

            if (value_mm < profile.left_clear_mm)
            {
                profile.left_clear_mm = (uint16_t)value_mm;
            }
        }

        if (angle <= 45U)
        {
            b_has_right = true;

            if (value_mm < profile.right_clear_mm)
            {
                profile.right_clear_mm = (uint16_t)value_mm;
            }
        }

        if (90U == angle)
        {
            b_has_centre = true;
        }
        /* Obstacle shape only from 45..135 deg; 30/150 are side-clearance
         * readings (track surroundings there made the obstacle look 391 mm
         * wide). */
        if (((value_mm < p_cfg->profile_mm) && (angle >= 45U)) &&
            (angle <= 135U))
        {
            /* The course geometry models an echo cone of about 30 degrees.
             * Move the ray inward by at most 15 degrees (three table steps)
             * to account for a nearer return elsewhere in that cone. See
             * app_program/subsystem5/README.md for the sensor reference. */
            const unsigned projection_index =
                index > 12U ? ((index - 3U) > 12U ? index - 3U : 12U)
                            : ((index + 3U) < 12U ? index + 3U : 12U);
            /* The validated 30..4000 mm range fits in int. Multiplying it
             * by signed table entries preserves lateral direction. */
            const int lateral_mm =
                (int)value_mm * g_sx[projection_index] / 1000;
            const int forward_mm =
                (int)value_mm * g_cy[projection_index] / 1000;
            b_has_found_obstacle = true;

            if (value_mm > far)
            {
                far = value_mm;
            }

            if (value_mm < profile.closest_mm)
            {
                profile.closest_mm = (uint16_t)value_mm;
            }

            if (lateral_mm > profile.left_edge_mm)
            {
                /* Scaled sine bounds the signed coordinate to ±4000 mm. */
                profile.left_edge_mm = (int16_t)lateral_mm;
            }

            if (lateral_mm < profile.right_edge_mm)
            {
                /* The same signed coordinate range applies on the right. */
                profile.right_edge_mm = (int16_t)lateral_mm;
            }

            if (forward_mm > (int)profile.forward_mm)
            {
                /* The positive cosine entries keep y within 0..4000 mm. */
                profile.forward_mm = (uint16_t)forward_mm;
            }
        }
    }

    if ((((!b_has_found_obstacle) || (!b_has_left)) || (!b_has_right)) ||
        (!b_has_centre))
    {
        return false;
    }
    /* A point return is not an obstacle edge. With the beam already corrected
     * above, pad by S5_EDGE_INFLATE_PERMIL (sin of the padding angle, 5 deg =
     * one fine step; was 15 deg) at the FARTHEST obstacle return seen. Leg 1
     * still checks the real diagonal path with the sensor before driving it.
     * Not a promise that acoustically invisible surfaces are detectable. */
    /* The farthest accepted echo is at most 4000 mm. The padding factor
     * is below one, so each padded edge fits in int16_t within ±8000 mm. */
    const int uncertainty =
        (int)(((far * S5_EDGE_INFLATE_PERMIL) + 999U) / 1000U);
    profile.left_edge_mm  = (int16_t)(profile.left_edge_mm + uncertainty);
    profile.right_edge_mm = (int16_t)(profile.right_edge_mm - uncertainty);

    /* Maximum x minus minimum x, after padding, lies within 0..16000 mm. */
    /* clang-format off */
    profile.width_mm   =
        (uint16_t)(profile.left_edge_mm - profile.right_edge_mm);
    profile.b_is_valid = true;
    *p_out             = profile;
    /* clang-format on */
    return true;
}

/** @brief Replace the measured side clearance without recomputing geometry.
 * @param[in,out] p_profile Caller-owned profile, initialized minimally if
 * invalid.
 * @param[in] p_sample Stable packed side-peek sample with a valid echo.
 * @return True after a side update, or false with the profile unchanged.
 * @see profile_obstacle.h for accepted angles and fields retained/reset.
 */
bool profile_obstacle_merge_peek (profile_obstacle_t *              p_profile,
                                  const profile_obstacle_sample_t * p_sample)
{
    bool b_is_sample_valid;

    unsigned angle;
    unsigned value_mm;
    if (((((0 == p_profile) || (0 == p_sample)) ||
          (!profile_obstacle_sample_unpack(p_sample, &angle, &value_mm,
                                           &b_is_sample_valid))) ||
         (!b_is_sample_valid)) ||
        ((angle > 60U) && (angle < 120U)))
    {
        return false;
    }
    /* The decoder and valid flag limit value_mm to 30..4000 for all three
     * stores below, including the first initialization of the profile. */
    if (!p_profile->b_is_valid)
    {
        *p_profile                = (profile_obstacle_t){0};
        p_profile->closest_mm     = (uint16_t)value_mm;
        p_profile->right_clear_mm = S5_RANGE_MAX_MM;
        p_profile->left_clear_mm  = S5_RANGE_MAX_MM;
        p_profile->b_is_valid     = true;
    }

    if (angle >= 120U)
    {
        p_profile->left_clear_mm = (uint16_t)value_mm;
    }
    else
    {
        p_profile->right_clear_mm = (uint16_t)value_mm;
    }

    return true;
}

/* End of file. */


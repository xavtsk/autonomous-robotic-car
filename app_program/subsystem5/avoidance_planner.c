/** @file avoidance_planner.c
 * @brief Select a bounded bypass plan from the measured obstacle profile.
 * The selected unsigned bit fields fit in int before promotion.
 */
#include "avoidance_planner.h"

/** @brief Evaluate the available clearance on each side of the measured
 * obstacle in millimetres. Choose a permitted route under the
 * configured preference and fill the forward and lateral distances.
 * Return false when validation fails or neither side fits.
 */
bool avoidance_planner_build (const profile_obstacle_t * p_profile,
                              const s5_config_t *        p_cfg,
                              avoidance_planner_plan_t * p_out)
{
    unsigned left_offset_mm;
    unsigned right_offset_mm;

    if (0 == p_out)
    {
        return false;
    }

    *p_out = (avoidance_planner_plan_t){0};

    if ((((((0 == p_profile) || (0 == p_cfg)) || (!p_profile->b_is_valid)) ||
          (!p_cfg->half_width_mm)) ||
         (!p_cfg->margin_mm)) ||
        (!p_cfg->max_depth_mm))
    {
        return false;
    }

    const unsigned lateral_pad_mm = s5_outward_gap_mm(p_cfg);
    {
        /* Expand a narrow-looking obstacle about the measured centre to
         * at least S5_OBSTACLE_MAX_MM. The scan can underestimate width.
         * Both signed 16-bit edges and their sum fit the target int. */
        const int centre_mm =
            ((int)p_profile->left_edge_mm + (int)p_profile->right_edge_mm) / 2;
        const int obstacle_half_mm = S5_OBSTACLE_MAX_MM / 2;
        const int left_edge_mm =
            p_profile->left_edge_mm > (centre_mm + obstacle_half_mm)
                ? p_profile->left_edge_mm
                : centre_mm + obstacle_half_mm;
        const int right_edge_mm =
            p_profile->right_edge_mm < (centre_mm - obstacle_half_mm)
                ? p_profile->right_edge_mm
                : centre_mm - obstacle_half_mm;
        /* Keep only distances on the selected side. The sign checks
         * bound the unsigned conversions to nonnegative values. */
        /* clang-format off */
        left_offset_mm  =
            (left_edge_mm > 0 ? (unsigned)left_edge_mm : 0U) + lateral_pad_mm;
        right_offset_mm =
            (right_edge_mm < 0 ? (unsigned)-right_edge_mm : 0U) +
            lateral_pad_mm;
        /* clang-format on */
    }
    /* Use the visible front face for the approach. The bypass controller
     * measures depth later with side peeks, bounded by max_depth_mm.
     * See avoid_obstacle.c for the clearance sequence. */
    const unsigned front_distance_mm = p_profile->forward_mm;
    /* The 30- and 150-degree readings can include a wheel or track edge,
     * so they rank the two sides. The first diagonal leg needs its own
     * forward reading beyond the required travel and margin. */
    const bool b_does_left_route_fit  = left_offset_mm <= p_cfg->max_offset_mm;
    const bool b_does_right_route_fit = right_offset_mm <= p_cfg->max_offset_mm;

    if ((((!b_does_left_route_fit) && (!b_does_right_route_fit)) ||
         (0U == front_distance_mm)) ||
        (front_distance_mm > 3000U))
    {
        return false;
    }
    /* Preferred side only: never fall back to the other side. */
    if ((!S5_ALLOW_OTHER_SIDE) &&
        (((S5_PREFER_ROUTE == 1U) && (!b_does_left_route_fit)) ||
         ((S5_PREFER_ROUTE == 2U) && (!b_does_right_route_fit))))
    {
        return false;
    }

    if (S5_PREFER_ROUTE == 1U)
    {
        p_out->route = b_does_left_route_fit ? S5_ROUTE_LEFT : S5_ROUTE_RIGHT;
    }
    else if (S5_PREFER_ROUTE == 2U)
    {
        p_out->route = b_does_right_route_fit ? S5_ROUTE_RIGHT : S5_ROUTE_LEFT;
    }
    else
    {
        p_out->route =
            (b_does_left_route_fit &&
             ((!b_does_right_route_fit) ||
              (p_profile->left_clear_mm >= p_profile->right_clear_mm)))
                ? S5_ROUTE_LEFT
                : S5_ROUTE_RIGHT;
    }

    /* clang-format off */
    p_out->offset_mm       =
        S5_ROUTE_LEFT == (int)p_out->route ? left_offset_mm : right_offset_mm;
    p_out->b_has_alt_route =
        S5_ALLOW_OTHER_SIDE &&
        (S5_ROUTE_LEFT == (int)p_out->route ? b_does_right_route_fit
        : b_does_left_route_fit);
    p_out->alt_offset_mm   =
        S5_ROUTE_LEFT == (int)p_out->route ? right_offset_mm : left_offset_mm;
    p_out->forward_mm      = front_distance_mm;
    p_out->closest_mm      =
        p_profile->closest_mm > 4095U ? 4095U : p_profile->closest_mm;
    /* clang-format on */
    return true;
}

/* End of file. */


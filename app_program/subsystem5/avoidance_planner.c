/** @file avoidance_planner.c
 * @brief Select a bounded bypass plan from the measured obstacle profile.
 */
#include "avoidance_planner.h"

bool s5_plan (const s5_profile_t * p_p, const s5_config_t * p_c,
              s5_plan_t * p_out)
{
    unsigned left, right, forward, pad;
    bool value_l, value_r;
    if (!p_out)
    {
        return false;
    }
    *p_out = (s5_plan_t){0};
    if (!p_p || !p_c || !p_p->valid || !p_c->half_width_mm || !p_c->margin_mm ||
        !p_c->max_depth_mm)
    {
        return false;
    }
    pad = p_c->half_width_mm + p_c->margin_mm;
    { /* Worst case: at least a S5_OBSTACLE_MAX_MM wide obstacle about the
       * centre the scan saw (it can read a 20 cm object narrower). */
        const int centre =
                      ((int)p_p->left_edge_mm + (int)p_p->right_edge_mm) / 2,
                  half = S5_OBSTACLE_MAX_MM / 2;
        const int value_le = p_p->left_edge_mm > centre + half
                                 ? p_p->left_edge_mm
                                 : centre + half;
        const int value_re = p_p->right_edge_mm < centre - half
                                 ? p_p->right_edge_mm
                                 : centre - half;
        left = (value_le > 0 ? (unsigned)value_le : 0U) + pad;
        right = (value_re < 0 ? (unsigned)-value_re : 0U) + pad;
    }
    /* Approach to the visible front face only; the unknown depth is measured
     * afterwards by side peeks, bounded by max_depth_mm (avoid_obstacle.c).
     */
    forward = p_p->forward_mm;
    /* Diagonal 30/150 deg readings only choose the side (they also catch the
     * car's own wheels / track edges). The real check is leg 1: after turning,
     * the sensor faces the sideways path and must read past offset + margin. */
    value_l = left <= p_c->max_offset_mm;
    value_r = right <= p_c->max_offset_mm;
    if ((!value_l && !value_r) || forward == 0U || forward > 3000U)
    {
        return false;
    }
    /* Preferred side only: never fall back to the other side. */
    if (!S5_ALLOW_OTHER_SIDE && ((S5_PREFER_ROUTE == 1U && !value_l) ||
                                 (S5_PREFER_ROUTE == 2U && !value_r)))
    {
        return false;
    }
    if (S5_PREFER_ROUTE == 1U)
    {
        p_out->route = value_l ? S5_ROUTE_LEFT : S5_ROUTE_RIGHT;
    }
    else if (S5_PREFER_ROUTE == 2U)
    {
        p_out->route = value_r ? S5_ROUTE_RIGHT : S5_ROUTE_LEFT;
    }
    else
    {
        p_out->route =
            (value_l && (!value_r || p_p->left_clear_mm >= p_p->right_clear_mm))
                ? S5_ROUTE_LEFT
                : S5_ROUTE_RIGHT;
    }
    p_out->offset_mm = p_out->route == S5_ROUTE_LEFT ? left : right;
    p_out->alt_ok = S5_ALLOW_OTHER_SIDE &&
                    (p_out->route == S5_ROUTE_LEFT ? value_r : value_l);
    p_out->alt_offset_mm = p_out->route == S5_ROUTE_LEFT ? right : left;
    p_out->forward_mm = forward;
    p_out->closest_mm = p_p->closest_mm > 4095U ? 4095U : p_p->closest_mm;
    return true;
}

/* End of file. */


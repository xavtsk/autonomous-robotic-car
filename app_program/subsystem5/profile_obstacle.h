/** @file profile_obstacle.h
 * @brief Pack scan samples and derive obstacle profiles and side-peek updates.
 */
#ifndef BUDDY5_OBSTACLE_PROFILE_H
#define BUDDY5_OBSTACLE_PROFILE_H
#include "s5_config.h"
#include <stdbool.h>
#include <stdint.h>

/* 12 distance bits + 5 angle-index bits + validity = 18 bits / 3 bytes.
 * Angle index 0..24 means 30..150 degrees in 5 degree steps. */
typedef struct
{
    uint8_t bytes[3];
} profile_obstacle_sample_t;

typedef struct
{
    uint16_t closest_mm;
    uint16_t width_mm;
    uint16_t left_clear_mm;
    uint16_t right_clear_mm;
    int16_t  left_edge_mm;
    int16_t  right_edge_mm;
    uint16_t forward_mm;
    bool     b_is_valid;
    uint16_t face_width_min_mm;
    uint16_t face_width_max_mm;
} profile_obstacle_t;

/** @brief Estimate the visible width of a flat face square to the car.
 * @param[in] p_samples Stable packed echoes for the completed sweep.
 * @param count Number of stored echoes, at most S5_SAMPLES_MAX.
 * @param[in] p_cfg Stable configuration used for the near/far echo cutoff.
 * @param[in,out] p_profile Caller-owned profile from the routing sweep.
 * A null output is ignored. Width bounds are cleared if either edge is
 * unresolved, input is missing or invalid, or the profile is invalid.
 * @details Bracket the face using the last matching echo and next clear
 * bearing on either side. Output bounds are in millimetres. Assign only the
 * face width fields. Keep inputs and output distinct and stable.
 */
void profile_obstacle_face_width (const profile_obstacle_sample_t * p_samples,
                                  unsigned count, const s5_config_t * p_cfg,
                                  profile_obstacle_t * p_profile);

/** @brief Pack one servo reading into an explicit 18-bit sample.
 * @param[in] angle Servo degrees, 30..150 inclusive in exact 5-degree steps;
 * 90 is forward, larger angles point left.
 * @param[in] value_mm Range in mm, 30..4000 when valid; ignored otherwise.
 * @param[in] b_is_valid Whether the reading has a usable echo.
 * @param[out] p_out Caller-owned three-byte sample; unchanged on failure.
 * @return False for null output, invalid angle, or invalid usable range;
 * otherwise true after writing the sample.
 * @details Bits 0..11 hold range, 12..16 angle index, and 17 validity, with
 * the low byte first. Six high bits are zero. An invalid reading is encoded
 * with zero range even if value_mm is nonzero. All operations are integer;
 * no pointer is retained or synchronization performed.
 */
bool profile_obstacle_sample_pack (unsigned angle, unsigned value_mm,
                                   bool                        b_is_valid,
                                   profile_obstacle_sample_t * p_out);
/** @brief Decode the fields of a packed servo reading and validate its range.
 * @param[in] p_sample Stable readable three-byte sample.
 * @param[out] p_angle Decoded servo degrees, 30..150 in 5-degree steps.
 * @param[out] p_mm Decoded 12-bit range in mm.
 * @param[out] pb_is_valid Decoded usable-echo flag.
 * @return True for range 30..4000 when valid or zero when invalid; otherwise
 * false, including null pointers, reserved bits or angle codes 25..31.
 * @details Null/padding/angle-code rejection leaves all outputs unchanged.
 * Range validation occurs after all three outputs are written, so a false
 * return can still replace them. Callers must use the return value.
 * @pre Output locations are distinct and writable. The input stays stable;
 * no caller pointer is retained and no lock is acquired.
 */
bool profile_obstacle_sample_unpack (const profile_obstacle_sample_t * p_sample,
                                     unsigned * p_angle, unsigned * p_mm,
                                     bool * pb_is_valid);
/** @brief Derive observed obstacle geometry and side clearances from a scan.
 * @param[in] p_samples Stable array of count well-formed, valid echo samples.
 * @param[in] count Number of entries, 5..S5_SAMPLES_MAX (currently 16).
 * @param[in] p_cfg Stable configuration; only profile_mm is read, as a strict
 * upper bound on ranges considered part of the obstacle. It is not validated.
 * @param[out] p_out Caller-owned profile; cleared immediately when non-null
 * and left zero/invalid on every subsequent failure.
 * @return False for nulls, bad count, any invalid sample, no obstacle return,
 * or missing left/right/centre coverage; otherwise true with valid set.
 * @pre Output does not overlap the sample array or configuration; the caller
 * owns it exclusively. No input pointer is retained and no lock is taken.
 * @details Coverage requires a sample at 90 degrees, one at or above 135,
 * and one at or below 45; order and uniqueness are not checked. Every sample
 * must decode successfully with a usable echo. Obstacle points use only
 * angles 45..135 and ranges strictly below profile_mm. Side clearances are
 * minimum raw ranges from their respective outer angles, not lateral offsets.
 * Coordinates are in mm relative to the sensor: X positive left, Y forward.
 * Each obstacle angle is moved up to 15 degrees toward centre before table
 * projection. Sine/cosine tables have scale 1000; integer division truncates
 * toward zero. Edges are then expanded on each side by the ceiling of
 * farthest obstacle range times S5_EDGE_INFLATE_PERMIL/1000 (currently 173).
 * closest_mm is the minimum obstacle range, forward_mm the largest projected
 * Y, and width_mm the expanded left edge minus right edge. Geometry is not
 * clamped to a telemetry packet's narrower domains or a car clearance limit.
 */
bool profile_obstacle_build (const profile_obstacle_sample_t * p_samples,
                             unsigned count, const s5_config_t * p_cfg,
                             profile_obstacle_t * p_out);
/* Side peek result: replaces that side's clearance in the last profile (kept
 * for telemetry); with no previous profile, a minimal valid one is made. */
/** @brief Replace one side clearance from a valid side-peek sample.
 * @param[in,out] p_profile Caller-owned profile; unchanged on failure.
 * @param[in] p_sample Stable packed valid echo at 30..60 or 120..150 degrees.
 * @return False for nulls, malformed/invalid sample or a central angle;
 * otherwise true after updating the corresponding side.
 * @details Angles at least 120 update left_clear_mm; angles at most 60 update
 * right_clear_mm. The new range replaces the previous value, without taking
 * a minimum. An already-valid profile keeps every other field unchanged.
 * An invalid profile is first cleared, then given closest_mm from this
 * sample, both clearances at 4000 mm and valid=true before updating the side.
 * Its width, edges and forward distance remain zero. This does not build or
 * revalidate obstacle geometry, check sample age, or acquire synchronization.
 * @pre Input stays stable while read; output is exclusively caller-owned.
 */
bool profile_obstacle_merge_peek (profile_obstacle_t *              p_profile,
                                  const profile_obstacle_sample_t * p_sample);
#endif

/* End of file. */


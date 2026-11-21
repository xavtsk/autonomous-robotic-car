/** @file scan_obstacle.h
 * @brief Track scan angles, sample acceptance and scan completion.
 */
#ifndef BUDDY5_OBSTACLE_SCAN_H
#define BUDDY5_OBSTACLE_SCAN_H
#include "profile_obstacle.h"

/** @brief Mutable state for one full scan or single-angle side peek.
 * @details The obstacle task owns its instance on its stack. Initialize with
 * scan_obstacle_begin() or scan_obstacle_begin_at() before accepting
 * measurements. These helpers take no locks, retain no pointers and perform no
 * hardware I/O; callers must serialize access and position/read the sensor
 * themselves. Angles are servo degrees (90 forward, larger values left); ranges
 * are mm. count is the number of stored samples. The b_is_done and b_has_failed
 * flags describe terminal state; index is reset by initialization but not
 * advanced by these helpers.
 */
typedef struct
{
    profile_obstacle_sample_t sample[S5_SAMPLES_MAX];
    unsigned                  count : 5;
    unsigned                  index : 5;
    unsigned                  angle : 8;
    unsigned                  fine_end : 8;
    unsigned                  nearest : 8;
    unsigned                  attempts : 2;
    unsigned                  b_is_fine : 1;
    unsigned                  b_is_done : 1;
    unsigned                  b_has_failed : 1;
    unsigned                  b_is_single : 1;
    uint16_t                  nearest_mm;
    uint32_t                  started_us;
} scan_obstacle_t;

/** @brief Discard previous results and initialize a full coarse/fine scan.
 * @param[out] p_scan Non-null, exclusively owned writable scan state.
 * @param[in] now Current timestamp in microseconds modulo 2^32.
 * @details Initialization clears sample storage and flags with the state.
 * Starts at 30 degrees with a provisional nearest angle of 90 degrees and
 * nearest range S5_RANGE_MAX_MM. Saves now as the overall scan start time.
 * @pre p_scan is valid; this initializer does not check for null.
 */
void scan_obstacle_begin (scan_obstacle_t * p_scan, uint32_t now);
/* Peek: one valid reading at a fixed side angle (30..60 or 120..150, 5 deg). */
/** @brief Initialize a single-measurement peek at a supported side angle.
 * @param[out] p_scan Exclusively owned writable state; unchanged on failure.
 * @param[in] now Current timestamp in microseconds modulo 2^32.
 * @param[in] angle Servo degrees; must pass scan_obstacle_peek_angle_valid().
 * @return False for a null state or unsupported angle; otherwise true.
 * @details Validation precedes clearing the state. Success performs the same
 * reset as scan_obstacle_begin(), then selects angle and enables single-sample
 * mode.
 */
bool scan_obstacle_begin_at (scan_obstacle_t * p_scan, uint32_t now,
                             unsigned angle);
/** @brief Check whether a servo angle supports a side peek.
 * @param[in] angle Any unsigned value, interpreted as servo degrees.
 * @return True for multiples of five in 30..60 or 120..150 inclusive;
 * false otherwise. No state is read or changed.
 */
bool scan_obstacle_peek_angle_valid (unsigned angle);
/* One completed measurement per call; retries stay at the same angle. */
/** @brief Consume one completed echo and advance or terminate the scan.
 * @param[in,out] p_scan Initialized, exclusively owned state; null is ignored.
 * @param[in] now Current timestamp in microseconds modulo 2^32.
 * @param[in] b_is_valid Whether the completed measurement has a usable echo.
 * @param[in] value_mm Range in mm; must be within S5_RANGE_MIN_MM through
 * S5_RANGE_MAX_MM inclusive when valid, otherwise it is ignored.
 * @details A completed scan is unchanged. Elapsed time at least
 * S5_SCAN_TIMEOUT_US marks failed and done before considering the echo.
 * An unusable echo increments attempts without moving the angle or count;
 * the third consecutive unusable echo marks failed and done. A usable echo
 * resets attempts before checking capacity and packing exactly one sample.
 * Full storage or a packing failure marks failed and done without increasing
 * count. Successful packing increments count; a peek then completes.
 * A full scan visits 30..150 degrees in 30-degree steps, then visits the
 * nearest coarse angle +/-15 degrees in 5-degree steps, clipped to 30..150.
 * Only strictly shorter coarse ranges replace the nearest angle; ties keep
 * the previous choice, including the initial 90 when all ranges are maximum.
 * The last fine sample sets done. Existing samples remain on every failure.
 * @pre State comes from these initializers and is not externally corrupted.
 * now uses the start timestamp's clock; elapsed time must be less than one
 * complete uint32_t clock cycle. Unsigned subtraction handles one wrap.
 * @note Each call stores at most one sample. It does not schedule retries,
 * move the servo, poll hardware, validate stationarity or build a profile.
 */
void scan_obstacle_accept (scan_obstacle_t * p_scan, uint32_t now,
                           bool b_is_valid, unsigned value_mm);
#endif

/* End of file. */


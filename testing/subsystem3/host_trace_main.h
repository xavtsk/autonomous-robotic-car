/** @file host_trace_main.h
 * @brief Entry-point fixture for comparing navigation and barcode traces.
 *
 * The fixture calls each real operation once. Trace builds also observe its
 * returned state before returning the same value to the test.
 */
#ifndef TESTING_SUBSYSTEM3_HOST_TRACE_MAIN_H
#define TESTING_SUBSYSTEM3_HOST_TRACE_MAIN_H

#include "line_navigation.h"

#ifdef TEST_NAV_TRACE
void host_trace_navigation (bool                             b_did_succeed,
                            const line_navigation_output_t * p_out);
void host_trace_barcode (bc_decoder_status_t  status,
                         const bc_decoder_t * p_decoder, uint32_t now,
                         bool b_is_valid, bool b_is_black,
                         const bc_decoder_result_t * p_out);
void host_trace_runs (bool b_did_succeed, unsigned count,
                      const bc_decoder_result_t * p_out);
#endif

/** @brief Advance one frame and return the navigation result to the fixture. */
static inline bool test_nav_step (line_navigation_t *                p_context,
                                  const ir_acquisition_frame_t *     p_frame,
                                  const line_navigation_odometry_t * p_odometry,
                                  line_navigation_output_t *         p_out)
{
    const bool b_did_succeed =
        line_navigation_process(p_context, p_frame, p_odometry, p_out);
#ifdef TEST_NAV_TRACE
    host_trace_navigation(b_did_succeed, p_out);
#endif
    return b_did_succeed;
}

/** @brief Submit one timestamped sample and return its decoder status. */
static inline bc_decoder_status_t test_bar_step (bc_decoder_t * p_decoder,
                                                 uint32_t now, bool b_is_valid,
                                                 bool b_is_black,
                                                 bc_decoder_result_t * p_out)
{
    const bc_decoder_status_t status =
        bc_decoder_update(p_decoder, now, b_is_valid, b_is_black, p_out);
#ifdef TEST_NAV_TRACE
    host_trace_barcode(status, p_decoder, now, b_is_valid, b_is_black, p_out);
#endif
    return status;
}

/** @brief Decode measured widths once and return the subject's result. */
static inline bool test_runs (const uint32_t * p_runs, unsigned count,
                              uint32_t leading, uint32_t trailing, uint32_t gap,
                              const bc_decoder_config_t * p_config,
                              bc_decoder_result_t *       p_out)
{
    const bool b_did_succeed = bc_decoder_decode_runs(
        p_runs, count, leading, trailing, gap, p_config, p_out);
#ifdef TEST_NAV_TRACE
    host_trace_runs(b_did_succeed, count, p_out);
#endif
    return b_did_succeed;
}

#endif /* TESTING_SUBSYSTEM3_HOST_TRACE_MAIN_H */

/* End of file. */


/** @file op_trace.h
 * @brief Describe observations from the unchanged operator integration test.
 */
#ifndef TOOLS_COMPLIANCE_TRACES_OP_TRACE_H
#define TOOLS_COMPLIANCE_TRACES_OP_TRACE_H

#include "op_logic.h"
#include <stddef.h>

/** Host-fixture counters and inputs; never retained or published by firmware.
 */
typedef struct
{
    uint32_t clock_us;
    uint32_t remote_stop_seen;
    unsigned sleeps;
    unsigned locked;
    unsigned starts_requested;
    unsigned stops;
    bool go_down;
    bool stop_down;
    uint16_t raw[3];
} op_trace_context_t;

/** @brief Print the completed poll's state and newly appended console bytes.
 * @param p_context Nonnull stable host counters, raw ADC inputs and clock.
 * @param p_operator Nonnull initialized operator state, observed read-only.
 * @param p_go Nonnull GP20 debounce state.
 * @param p_stop Nonnull GP21 debounce state.
 * @param p_status Nonnull latest host status; may be its initial zero value.
 * @param p_console Readable console buffer through console_end bytes.
 * @param console_begin Length before this poll, no greater than console_end.
 * @param console_end Length after this poll, bounded by the fixture buffer.
 * @post Writes deterministic OPTRACE/OPCONSOLE rows to host stdout. No input
 * object, simulated clock, callback or production behavior is changed.
 * @note Used synchronously by the host fixture. Units remain microseconds,
 * milliseconds, raw ADC counts and normalized calibration summaries. Floating
 * averages are recorded as exact binary32 bits, not rounded decimal text.
 */
void op_trace_write (const op_trace_context_t * p_context,
                     const operator_t * p_operator, const op_button_t * p_go,
                     const op_button_t * p_stop,
                     const s3_integration_status_t * p_status,
                     const char * p_console, size_t console_begin,
                     size_t console_end);

#endif

/* End of file. */


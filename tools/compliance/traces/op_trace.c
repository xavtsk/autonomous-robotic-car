/** @file op_trace.c
 * @brief Emit exact operator state and console observations for comparison.
 */
#include "op_trace.h"
#include "../../../app_program/build_assert.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

BUILD_ASSERT(sizeof(((ir_calibration_summary_t *)0)->average) ==
                 sizeof(uint32_t),
             op_trace_float_storage_t);

/** @brief Append one host button's timestamps and all retained Boolean flags.
 * @param p_button Nonnull stable debounce state, owned by the calling fixture.
 * @post Writes comma-separated fields to stdout without modifying the state.
 */
static void op_trace_button (const op_logic_button_t * p_button)
{
    printf("%" PRIu32 ",%" PRIu32 ",%u,%u,%u,%u", p_button->since_ms,
           p_button->pressed_ms, (unsigned)p_button->b_has_input_level,
           (unsigned)p_button->b_is_raw_pressed,
           (unsigned)p_button->b_is_stable_pressed,
           (unsigned)p_button->b_is_press_armed);
}

/** @brief Append calibration extrema/count and the exact average bit pattern.
 * @param p_summary Nonnull readable host summary; no range validation occurs.
 * @post Writes fields to stdout. Uses memcpy to avoid aliasing or rounding.
 */
static void op_trace_summary (const ir_calibration_summary_t * p_summary)
{
    uint32_t bits;
    (void)memcpy(&bits, &p_summary->average, sizeof(bits));
    printf("%" PRIu32 ",%u,%u,%08" PRIx32 ",%u", p_summary->count,
           (unsigned)p_summary->minimum, (unsigned)p_summary->maximum, bits,
           (unsigned)p_summary->b_is_available);
}

/** @brief Observe one completed integration/operator poll.
 * @see op_trace.h for inputs, units, ownership and output boundaries.
 */
void op_trace_write (const op_trace_context_t *      p_context,
                     const op_logic_t *              p_operator,
                     const op_logic_button_t *       p_go,
                     const op_logic_button_t *       p_stop,
                     const s3_integration_status_t * p_status,
                     const char * p_console, size_t console_begin,
                     size_t console_end)
{
#if defined(OP_TRACE_LEGACY_PORT)
    const bool b_can_submit = (NULL != p_operator->port.submit);
    const bool b_can_print  = (NULL != p_operator->port.print);
#else
    const bool b_can_submit = (NULL != p_operator->port.p_submit);
    const bool b_can_print  = (NULL != p_operator->port.p_print);
#endif
    printf("OPTRACE clock=%" PRIu32 " phase=%u phase_ms=%" PRIu32
           " prompt_ms=%" PRIu32 " seq=%" PRIu32 " step=%u shown=%u",
           p_context->clock_us, (unsigned)p_operator->phase,
           p_operator->phase_ms, p_operator->prompt_ms, p_operator->sequence,
           (unsigned)p_operator->step, (unsigned)p_operator->countdown_shown);
    printf(" flags=%u,%u,%u,%u,%u,%u,%u ports=%u,%u",
           (unsigned)p_operator->b_is_awaiting_reply,
           (unsigned)p_operator->b_has_prompted,
           (unsigned)p_operator->b_has_handled_stop,
           (unsigned)p_operator->b_has_warned_stop,
           (unsigned)p_operator->b_is_go_inhibited,
           (unsigned)p_operator->b_has_warned_ignore,
           (unsigned)p_operator->b_should_reset_to_cal, (unsigned)b_can_submit,
           (unsigned)b_can_print);
    printf(" go=");
    op_trace_button(p_go);
    printf(" stop=");
    op_trace_button(p_stop);
    printf(" input=%u,%u raw=%u,%u,%u calls=%u,%u,%u,%u remote=%" PRIu32,
           (unsigned)p_context->go_down, (unsigned)p_context->stop_down,
           (unsigned)p_context->raw[0], (unsigned)p_context->raw[1],
           (unsigned)p_context->raw[2], p_context->sleeps, p_context->locked,
           p_context->starts_requested, p_context->stops,
           p_context->remote_stop_seen);
    printf(
        " status=%u,%u,%" PRIu32 ",%u,%u,%u white=", (unsigned)p_status->state,
        (unsigned)p_status->reason, p_status->command_sequence,
        (unsigned)p_status->b_did_command_succeed,
        (unsigned)p_status->b_is_collecting,
        (unsigned)p_status->calibration_sensor);
    op_trace_summary(&p_status->white);
    printf(" black=");
    op_trace_summary(&p_status->black);
    printf(" console=%zu\n", console_end);
    if (console_end != console_begin)
    {
        printf("OPCONSOLE ");
        for (size_t index = console_begin; index < console_end; ++index)
        {
            printf("%02x", (unsigned)(uint8_t)p_console[index]);
        }
        putchar('\n');
    }
}

/* End of file. */


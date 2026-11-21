/** @file op_logic.c
 * @brief Operator controls: calibration walkthrough, START/STOP/RESET.
 *
 * See header.
 * Every request goes through port.p_submit(); results are read back from the
 * published status (command_sequence/command_ok), never assumed.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 */
#include "op_logic.h"

#define OP_TAG "[OP] "

typedef struct
{
    const ir_acquisition_sensor_id_t sensor;
    const char * const               p_name;

    const unsigned gpio; /* Use the sensor mapping from ir_acquisition.h. */
} op_sensor_pin_t;

static const op_sensor_pin_t g_k_order[S3_SENSOR_COUNT] = {
    {S3_SENSOR_LEFT, "front-LEFT line sensor",
     S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT)},
    {S3_SENSOR_RIGHT, "front-RIGHT line sensor",
     S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT)},
    {S3_SENSOR_BARCODE, "BARCODE sensor",
     S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE)}};

static void enter_phase (op_logic_t * p_op, op_logic_phase_t phase,
                         uint32_t now);
static void print_status (op_logic_t * p_op, const char * p_text);
static bool submit_request (op_logic_t * p_op, s3_integration_kind_t kind,
                            ir_acquisition_sensor_id_t      sensor,
                            const s3_integration_status_t * p_st, uint32_t now);
static bool replied (const op_logic_t *              p_op,
                     const s3_integration_status_t * p_st,
                     bool *                          pb_did_succeed);
static bool deadline_reached (op_logic_t * p_op, uint32_t now);
static bool drive_is_running (s3_drive_state_t state);
static void print_range (op_logic_t * p_op, const char * p_what,
                         const ir_calibration_summary_t * p_summary);
static void cal_prompt (op_logic_t * p_op, uint32_t now,
                        op_logic_press_t                value_go,
                        const s3_integration_status_t * p_st);
static void cal_collect (op_logic_t * p_op, uint32_t now,
                         const s3_integration_status_t * p_st);
static void cal_apply (op_logic_t * p_op, uint32_t now,
                       const s3_integration_status_t * p_st);
static void handle_ready (op_logic_t * p_op, uint32_t now,
                          op_logic_press_t                value_go,
                          const s3_integration_status_t * p_st);
static void reset_wait (op_logic_t * p_op, uint32_t now,
                        const s3_integration_status_t * p_st);
static void countdown (op_logic_t * p_op, uint32_t now,
                       const s3_integration_status_t * p_st);
static void start_wait (op_logic_t * p_op, uint32_t now,
                        const s3_integration_status_t * p_st);
static void stop_pressed (op_logic_t * p_op, uint32_t now_ms);

/**
 * @brief Apply the static state-label contract in op_logic.h.
 * @param state Native state code; unknown codes select "?".
 * @return Immutable static string; no state or callback is used.
 */
const char * op_logic_drive_state_name (s3_drive_state_t state)
{
    switch (state)
    {
    case S3_DRIVE_OFF:
        return "OFF";
    case S3_DRIVE_FOLLOW:
        return "FOLLOW";
    case S3_DRIVE_BRAKING:
        return "BRAKING";
    case S3_DRIVE_MANOEUVRE:
        return "MANOEUVRE";
    case S3_DRIVE_REACQUIRE:
        return "REACQUIRE";
    case S3_DRIVE_HALTED:
        return "HALTED";
    case S3_DRIVE_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

/**
 * @brief Apply the static reason-label contract in op_logic.h.
 * @param reason Native reason code; unknown codes select "?".
 * @return Immutable static string; no state or callback is used.
 */
const char * op_logic_drive_reason_name (s3_drive_reason_t reason)
{
    switch (reason)
    {
    case S3_DRIVE_OK:
        return "ok/stopped by operator";
    case S3_DRIVE_SENSOR_FAULT:
        return "IR sensor fault";
    case S3_DRIVE_MOTION_FAULT:
        return "motor/encoder fault";
    case S3_DRIVE_REJECTED:
        return "motion request rejected";
    case S3_DRIVE_LINE_LOST:
        return "line lost";
    case S3_DRIVE_NO_DIRECTION:
        return "junction without a barcode, or two barcodes before a junction";
    case S3_DRIVE_UNMAPPED:
        return "barcode letter not in the map";
    case S3_DRIVE_TIMEOUT:
        return "timeout (barcode->junction 5 s, brake or turn)";
    case S3_DRIVE_REPORT_FAULT:
        return "motion report fault";
    case S3_DRIVE_BAD_INPUT:
        return "bad input";
    case S3_DRIVE_TERRAIN_FAULT:
        return "IMU/event fault";
    default:
        return "?";
    }
}

/**
 * @brief Clear debounce state without accessing hardware.
 * @param[out] p_button Exclusively owned state; NULL is a no-op.
 */
void op_logic_button_init (op_logic_button_t * p_button)
{
    if (p_button != 0)
    {
        *p_button = (op_logic_button_t){0};
    }
}

/**
 * @brief Apply the debounce and startup-arming contract in op_logic.h.
 * @param[in,out] p_button Initialized button state; NULL returns NONE.
 * @param b_is_down Raw pressed level after caller polarity conversion.
 * @param now_ms Wrapping millisecond time, with unsigned elapsed arithmetic.
 * @param[out] pb_has_press_edge Optional separate press-edge output, always
 * cleared first.
 * @return Release classification; no clock or callback is invoked.
 */
op_logic_press_t op_logic_button_update (op_logic_button_t * p_button,
                                         bool b_is_down, uint32_t now_ms,
                                         bool * pb_has_press_edge)
{
    op_logic_press_t press = OP_PRESS_NONE;

    if (pb_has_press_edge != 0)
    {
        *pb_has_press_edge = false;
    }

    if (0 == p_button)
    {
        return OP_PRESS_NONE;
    }

    if ((!p_button->b_has_input_level) ||
        (b_is_down != p_button->b_is_raw_pressed))
    {
        p_button->b_has_input_level = true;
        p_button->b_is_raw_pressed  = b_is_down;
        p_button->since_ms          = now_ms;
        return OP_PRESS_NONE;
    }

    if ((p_button->b_is_raw_pressed != p_button->b_is_stable_pressed) &&
        ((now_ms - p_button->since_ms) >= OP_DEBOUNCE_MS))
    {
        p_button->b_is_stable_pressed = p_button->b_is_raw_pressed;

        if (p_button->b_is_stable_pressed)
        {
            p_button->pressed_ms = p_button->since_ms;

            if ((p_button->b_is_press_armed) && (pb_has_press_edge != 0))
            {
                *pb_has_press_edge = true;
            }
        }
        else if (p_button->b_is_press_armed)
        {
            press = ((p_button->since_ms - p_button->pressed_ms) >=
                     OP_LONG_PRESS_MS)
                        ? OP_PRESS_LONG
                        : OP_PRESS_SHORT;
        }
        else
        {
            /* An unarmed release produces no button event. */
        }
    }
    /* A button held at power-up is ignored until seen released for 100 ms. */
    if ((((!p_button->b_is_press_armed) && (!p_button->b_is_raw_pressed)) &&
         (!p_button->b_is_stable_pressed)) &&
        ((now_ms - p_button->since_ms) >= OP_RELEASE_MS))
    {
        p_button->b_is_press_armed = true;
    }

    return press;
}

/**
 * @brief Read the stable button level without consuming an event.
 * @param[in] p_button Borrowed state; NULL returns false.
 * @return The stored stable level, even if press events are not yet armed.
 */
bool op_logic_button_held (const op_logic_button_t * p_button)
{
    return (p_button != 0) && (p_button->b_is_stable_pressed);
}

/**
 * @brief Validate callbacks, clear state and copy the port as documented.
 * @param[out] p_op Caller-owned state; unchanged on validation failure.
 * @param[in] p_port Borrowed port; copied after state clearing, without
 * protection against overlap with p_op.
 * @return True after initialization; false for a NULL pointer or callback.
 */
bool op_logic_init (op_logic_t * p_op, const op_logic_port_t * p_port)
{
    if ((((0 == p_op) || (0 == p_port)) || (0 == p_port->p_submit)) ||
        (0 == p_port->p_print))
    {
        return false;
    }

    *p_op       = (op_logic_t){0};
    p_op->port  = *p_port;
    p_op->phase = OP_PHASE_WAIT_STATUS;
    return true;
}

/**
 * @brief Apply the remote-STOP cancellation contract in op_logic.h.
 * @param[in,out] p_op Initialized owned state; NULL is a no-op.
 * @param now_ms Wrapping millisecond timestamp for cancellation transitions.
 * @note Sets local inhibition and never submits a duplicate STOP.
 */
void op_logic_remote_stop (op_logic_t * p_op, uint32_t now_ms)
{
    if (0 == p_op)
    {
        return;
    }

    p_op->b_is_go_inhibited = true;
    stop_pressed(p_op, now_ms);
}

/**
 * @brief Apply STOP precedence and the operator-phase contract in op_logic.h.
 * @param[in,out] p_op Initialized exclusively accessed state; NULL is a no-op.
 * @param now_ms Wrapping millisecond time used by the selected phase.
 * @param value_go Debounced GO event; inhibition can discard this event.
 * @param b_is_stop_held Debounced STOP level, serviced before consulting
 * status.
 * @param b_is_go_held Debounced GO level, used for the both-released gate.
 * @param[in] p_status Optional coherent snapshot, borrowed only for this call.
 * @note Missing status prevents phase processing, not STOP/inhibition. Invalid
 * phase values fall back to READY. Synchronous submit/print effects and exact
 * sequence-based acknowledgement remain as specified by the public contract.
 */
void op_logic_step (op_logic_t * p_op, uint32_t now_ms,
                    op_logic_press_t value_go, bool b_is_stop_held,
                    bool b_is_go_held, const s3_integration_status_t * p_status)
{
    if (0 == p_op)
    {
        return;
    }
    /* STOP first, in every phase, without waiting for status. Level-based:
     * a GP21 held since power-up, or held before GP20, still counts. */
    if (b_is_stop_held)
    {
        p_op->b_is_go_inhibited = true;

        if (!p_op->b_has_handled_stop)
        {
            const s3_integration_request_t request = {.kind = S3_CONTROL_STOP};

            if (p_op->port.p_submit(&request))
            {
                p_op->b_has_handled_stop = true;
                print_status(p_op, "STOP sent.");
            }
            else if (!p_op->b_has_warned_stop)
            {
                /* Retried every poll while held; reported once per hold. */
                print_status(p_op,
                             "STOP not accepted yet, retrying. Use the power "
                             "switch if the car moves.");
                p_op->b_has_warned_stop = true;
            }
            else
            {
                /* The operator has already received the stop warning. */
            }

            stop_pressed(p_op, now_ms);
        }
    }
    else
    {
        p_op->b_has_handled_stop = false;
        p_op->b_has_warned_stop  = false;
    }

    if (p_op->b_is_go_inhibited)
    {
        if ((value_go != (unsigned int)OP_PRESS_NONE) &&
            (!p_op->b_has_warned_ignore))
        {
            print_status(p_op, "Ignored GP20: release GP21 first.");
            p_op->b_has_warned_ignore = true;
        }

        value_go =
            OP_PRESS_NONE; /* Discard the press or release on this poll. */
        if ((!b_is_stop_held) && (!b_is_go_held))
        {
            p_op->b_is_go_inhibited   = false;
            p_op->b_has_warned_ignore = false;
        }
    }

    if (0 == p_status)
    {
        return;
    }

    switch (p_op->phase)
    {
    case OP_PHASE_WAIT_STATUS:
        print_status(p_op,
                     "Operator controls: GP20 = next/start, GP21 = STOP.");
        p_op->step = 0U;

        if (S3_ALL_SENSORS == p_status->navigation.accepted_calibration)
        {
            /* Built-in calibration was accepted at boot: no walkthrough. */
            print_status(
                p_op, "Using built-in calibration. Hold GP20 2 s at READY to "
                      "recalibrate by hand.");
            enter_phase(p_op, OP_PHASE_READY, now_ms);
        }
        else
        {
            enter_phase(p_op, OP_PHASE_CAL_PROMPT, now_ms);
        }

    break;
    case OP_PHASE_CAL_PROMPT:
        cal_prompt(p_op, now_ms, value_go, p_status);
    break;
    case OP_PHASE_CAL_COLLECT:
        cal_collect(p_op, now_ms, p_status);
    break;
    case OP_PHASE_CAL_APPLY:
        cal_apply(p_op, now_ms, p_status);
    break;
    case OP_PHASE_READY:
        handle_ready(p_op, now_ms, value_go, p_status);
    break;
    case OP_PHASE_RESET:
        reset_wait(p_op, now_ms, p_status);
    break;
    case OP_PHASE_COUNTDOWN:
        countdown(p_op, now_ms, p_status);
    break;
    case OP_PHASE_START:
        start_wait(p_op, now_ms, p_status);
    break;
    case OP_PHASE_RUN:
        if (!drive_is_running(p_status->state))
        {
            (void)p_op->port.p_print(
                (const unsigned char *)"%sRun ended: %s - %s.\n", OP_TAG,
                op_logic_drive_state_name(p_status->state),
                op_logic_drive_reason_name(p_status->reason));
            enter_phase(p_op, OP_PHASE_READY, now_ms);
        }
    break;
    default:
        enter_phase(p_op, OP_PHASE_READY, now_ms);
    break;
    }
}

/**
 * @brief Enter a phase and clear prompt, reply-wait and countdown bookkeeping.
 * @param[in,out] p_op Non-NULL initialized state owned by this update.
 * @param phase Destination phase; assigned without additional validation.
 * @param now Wrapping millisecond phase-entry timestamp.
 * @note Leaves calibration step, sequence and STOP/inhibition flags intact.
 */
static void enter_phase (op_logic_t * p_op, op_logic_phase_t phase,
                         uint32_t now)
{
    p_op->phase               = phase;
    p_op->phase_ms            = now;
    p_op->b_has_prompted      = false;
    p_op->b_is_awaiting_reply = false;
    p_op->countdown_shown     = 0U;
}

/**
 * @brief Print one tagged console line through the configured port.
 * @param[in] p_op Non-NULL initialized state with a callable print callback.
 * @param[in] p_text Borrowed NUL-terminated text, valid through the callback.
 * @note Invokes print synchronously and ignores its return value.
 */
static void print_status (op_logic_t * p_op, const char * p_text)
{
    (void)p_op->port.p_print((const unsigned char *)"%s%s\n", OP_TAG, p_text);
}

/**
 * @brief Submit one stack-owned request and begin observing sequence progress.
 * @param[in,out] p_op Non-NULL initialized state owned by this update.
 * @param kind Request discriminator passed unchanged to submit.
 * @param sensor Sensor selector passed unchanged to submit.
 * @param[in] p_st Non-NULL snapshot supplying the prior command sequence.
 * @param now Wrapping millisecond request timestamp.
 * @return Callback acceptance; false prints the existing busy message.
 * @note Sequence and phase time change even on rejection. The callback must
 * consume/copy the request before returning; acceptance does not imply
 * execution.
 */
static bool submit_request (op_logic_t * p_op, s3_integration_kind_t kind,
                            ir_acquisition_sensor_id_t      sensor,
                            const s3_integration_status_t * p_st, uint32_t now)
{
    const s3_integration_request_t request = {.kind = kind, .sensor = sensor};
    p_op->sequence                         = p_st->command_sequence;
    p_op->phase_ms                         = now;
    p_op->b_is_awaiting_reply              = p_op->port.p_submit(&request);

    if (!p_op->b_is_awaiting_reply)
    {
        print_status(p_op, "Controller busy; press GP20 again.");
    }

    return p_op->b_is_awaiting_reply;
}

/* True once the controller has executed our request; *ok is its result. */
/**
 * @brief Recognize changed controller sequence while a reply is awaited.
 * @param[in] p_op Non-NULL operator state, read without mutation.
 * @param[in] p_st Non-NULL coherent controller snapshot.
 * @param[out] pb_did_succeed Non-NULL output; written only when a changed
 * sequence is seen.
 * @return True after copying command_ok, otherwise false with pb_did_succeed
 * unchanged.
 * @note Does not clear b_is_awaiting_reply or identify a command beyond the
 * existing sequence comparison; it performs no callback or synchronization.
 */
static bool replied (const op_logic_t *              p_op,
                     const s3_integration_status_t * p_st,
                     bool *                          pb_did_succeed)
{
    if ((!p_op->b_is_awaiting_reply) ||
        (p_st->command_sequence == p_op->sequence))
    {
        return false;
    }

    *pb_did_succeed = p_st->b_did_command_succeed;
    return true;
}

/**
 * @brief Test and advance the repeating-prompt deadline.
 * @param[in,out] p_op Non-NULL state owned by this update.
 * @param now Wrapping millisecond time.
 * @return True for the first prompt or after OP_PROMPT_REPEAT_MS, updating
 * b_has_prompted/prompt_ms; false leaves both unchanged. Does not print by
 * itself.
 */
static bool deadline_reached (op_logic_t * p_op, uint32_t now)
{
    if ((!p_op->b_has_prompted) ||
        ((now - p_op->prompt_ms) >= OP_PROMPT_REPEAT_MS))
    {
        p_op->b_has_prompted = true;
        p_op->prompt_ms      = now;
        return true;
    }

    return false;
}

/**
 * @brief Test the existing inclusive active-drive state interval.
 * @param state Native drive-state code; no enum normalization is performed.
 * @return True from FOLLOW through REACQUIRE, otherwise false; no side effects.
 */
static bool drive_is_running (s3_drive_state_t state)
{
    return (state >= (unsigned int)S3_DRIVE_FOLLOW) &&
           (state <= (unsigned int)S3_DRIVE_REACQUIRE);
}

/**
 * @brief Print one calibration range using count as the validity discriminator.
 * @param[in] p_op Initialized state with a callable synchronous print port.
 * @param[in] p_what Borrowed NUL-terminated label.
 * @param[in] p_summary Non-NULL summary of raw ADC values and sample count.
 * @note Prints zero minimum/maximum when count is zero; changes no stored
 * values and ignores the callback's return value.
 */
static void print_range (op_logic_t * p_op, const char * p_what,
                         const ir_calibration_summary_t * p_summary)
{
    (void)p_op->port.p_print(
        (const unsigned char *)"%s  %s: %u samples, raw %u..%u\n", OP_TAG,
        p_what, (unsigned)p_summary->count,
        (unsigned)(p_summary->count ? p_summary->minimum : 0U),
        (unsigned)(p_summary->count ? p_summary->maximum : 0U));
}

/**
 * @brief Prompt for the selected surface or request RESET while faulted.
 * @param[in,out] p_op Owned initialized state with step in 0..OP_CAL_STEPS-1.
 * @param now Wrapping millisecond time for prompts and requests.
 * @param value_go Any non-NONE value can submit the phase's request.
 * @param[in] p_st Non-NULL snapshot; FAULT takes precedence over calibration.
 * @note Successful submission changes phase; rejection retains the prompt
 * phase while send updates its bookkeeping. Console/callback effects are
 * direct.
 */
static void cal_prompt (op_logic_t * p_op, uint32_t now,
                        op_logic_press_t                value_go,
                        const s3_integration_status_t * p_st)
{
    const unsigned index      = p_op->step / 2U;
    const bool     b_is_black = (p_op->step % 2U) != 0U;

    if ((unsigned int)S3_DRIVE_FAULT == p_st->state)
    {
        /* Calibration is refused while faulted: offer RESET, never a loop of
         * refused sampling attempts. */
        if (deadline_reached(p_op, now))
        {
            (void)p_op->port.p_print(
                (const unsigned char *)"%sFAULT: %s. Calibration is blocked. "
                                       "Press GP20 to reset (or power-cycle "
                                       "the car).\n",
                OP_TAG, op_logic_drive_reason_name(p_st->reason));
        }

        if ((value_go != (unsigned int)OP_PRESS_NONE) &&
            (submit_request(p_op, S3_CONTROL_RESET, S3_SENSOR_BARCODE, p_st,
                            now)))
        {
            p_op->b_should_reset_to_cal = true;
            p_op->phase                 = OP_PHASE_RESET;
        }

        return;
    }

    if (deadline_reached(p_op, now))
    {
        (void)p_op->port.p_print(
            (const unsigned char *)"%sCalibration %u/%u: put the %s (GP%u) "
                                   "over %s. "
                                   "Keep the car still, then press GP20.\n",
            OP_TAG, p_op->step + 1U, OP_CAL_STEPS, g_k_order[index].p_name,
            g_k_order[index].gpio,
            b_is_black ? "the BLACK line (sensor centred on the tape)"
                       : "plain WHITE mat");
    }

    if ((value_go != (unsigned int)OP_PRESS_NONE) &&
        (submit_request(p_op, b_is_black ? S3_CONTROL_BLACK : S3_CONTROL_WHITE,
                        g_k_order[index].sensor, p_st, now)))
    {
        p_op->phase = OP_PHASE_CAL_COLLECT;
    }
}

/**
 * @brief Await sampling acknowledgement/completion, then advance or apply.
 * @param[in,out] p_op Owned initialized state with a valid calibration step.
 * @param now Wrapping millisecond time; deadlines use the request timestamp.
 * @param[in] p_st Non-NULL snapshot supplying reply, sensor and sample
 * summaries.
 * @note A rejected/timed-out reply returns to the prompt. Completion prints
 * the selected range; WHITE advances the step, BLACK submits APPLY. No wait
 * loop or motor call is performed; callback acceptance remains distinct from
 * controller execution.
 */
static void cal_collect (op_logic_t * p_op, uint32_t now,
                         const s3_integration_status_t * p_st)
{
    const unsigned index         = p_op->step / 2U;
    const bool     b_is_black    = (p_op->step % 2U) != 0U;
    bool           b_did_succeed = false;

    if (p_op->b_is_awaiting_reply)
    {
        if (replied(p_op, p_st, &b_did_succeed))
        {
            p_op->b_is_awaiting_reply = false;

            if (!b_did_succeed)
            {
                (void)p_op->port.p_print(
                    (const unsigned char *)"%sSampling refused (state=%s). "
                                           "Press GP20 to retry.\n",
                    OP_TAG, op_logic_drive_state_name(p_st->state));
                enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
                return;
            }

            print_status(p_op, "Sampling for 1 s - keep still...");
        }
        else if ((now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
        {
            print_status(p_op,
                         "No reply from controller. Press GP20 to retry.");
            enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
        }
        else
        {
            /* Keep waiting for the controller's calibration reply. */
        }

        return;
    }

    if ((p_st->b_is_collecting) ||
        (p_st->calibration_sensor != g_k_order[index].sensor))
    {
        if ((now - p_op->phase_ms) >= OP_COLLECT_TIMEOUT_MS)
        {
            print_status(p_op, "Sampling did not finish. Press GP20 to retry.");
            enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
        }

        return;
    }

    print_range(p_op, b_is_black ? "BLACK" : "WHITE",
                b_is_black ? &p_st->black : &p_st->white);

    if (!b_is_black)
    {
        p_op->step++;
        enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
    }
    else if (submit_request(p_op, S3_CONTROL_APPLY_CALIBRATION,
                            g_k_order[index].sensor, p_st, now))
    {
        p_op->phase = OP_PHASE_CAL_APPLY;
    }
    else
    {
        enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
    }
}

/**
 * @brief Await APPLY and retain the existing retry/next-sensor transitions.
 * @param[in,out] p_op Owned initialized state at a valid BLACK step.
 * @param now Wrapping millisecond time for reply timeout and phase entry.
 * @param[in] p_st Non-NULL snapshot supplying sequence, result and sample
 * ranges.
 * @note Timeout retries BLACK; rejection retries both surfaces for this sensor.
 * Success advances the step, reaching READY after the final sensor. Prints
 * through the existing callback without changing calibration directly.
 */
static void cal_apply (op_logic_t * p_op, uint32_t now,
                       const s3_integration_status_t * p_st)
{
    const unsigned index         = p_op->step / 2U;
    bool           b_did_succeed = false;

    if (!replied(p_op, p_st, &b_did_succeed))
    {
        if ((now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
        {
            print_status(
                p_op, "No reply from controller. Redo BLACK for this sensor.");
            enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
        }

        return;
    }

    if (!b_did_succeed)
    {
        (void)p_op->port.p_print(
            (const unsigned char *)"%s%s (GP%u) REJECTED: white and black "
                                   "readings "
                                   "overlap (or too few samples). Redo this "
                                   "sensor; check its height and the "
                                   "surface.\n",
            OP_TAG, g_k_order[index].p_name, g_k_order[index].gpio);
        print_range(p_op, "WHITE", &p_st->white);
        print_range(p_op, "BLACK", &p_st->black);
        p_op->step = index * 2U;
        enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
        return;
    }

    (void)p_op->port.p_print((const unsigned char *)"%s%s (GP%u) calibrated.\n",
                             OP_TAG, g_k_order[index].p_name,
                             g_k_order[index].gpio);
    p_op->step++;

    if (p_op->step >= OP_CAL_STEPS)
    {
        print_status(p_op, "All 3 sensors calibrated.");
        enter_phase(p_op, OP_PHASE_READY, now);
    }
    else
    {
        enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
    }
}

/**
 * @brief Interpret a READY-phase GO event with fault RESET taking precedence.
 * @param[in,out] p_op Owned initialized state; may enter RESET or calibration.
 * @param now Wrapping millisecond time for prompts and phase transitions.
 * @param value_go SHORT begins countdown; LONG begins calibration when healthy.
 * @param[in] p_st Non-NULL snapshot; any non-NONE GO requests RESET in FAULT.
 * @note Prints prompts through the port; a countdown alone submits no START.
 */
static void handle_ready (op_logic_t * p_op, uint32_t now,
                          op_logic_press_t                value_go,
                          const s3_integration_status_t * p_st)
{
    const bool b_has_fault = (unsigned int)S3_DRIVE_FAULT == p_st->state;

    if (deadline_reached(p_op, now))
    {
        if (b_has_fault)
        {
            (void)p_op->port.p_print(
                (const unsigned char *)"%sFAULT: %s. Fix the cause, then press "
                                       "GP20 to reset. If reset keeps failing, "
                                       "power-cycle the car.\n",
                OP_TAG, op_logic_drive_reason_name(p_st->reason));
        }
        else
        {
            print_status(
                p_op,
                "READY: put the car on the line, then press GP20 to start "
                "(3 s countdown). Hold GP20 2 s to recalibrate. GP21 = STOP.");
        }
    }

    if (b_has_fault && (value_go != (unsigned int)OP_PRESS_NONE))
    {
        /* The controller refuses calibration while faulted, so every press
         * (short or long) means RESET here; recalibrate after the reset. */
        p_op->b_should_reset_to_cal = false;

        if (submit_request(p_op, S3_CONTROL_RESET, S3_SENSOR_BARCODE, p_st,
                           now))
        {
            p_op->phase = OP_PHASE_RESET;
        }
    }
    else if ((unsigned int)OP_PRESS_LONG == value_go)
    {
        print_status(p_op, "Recalibrating all sensors.");
        p_op->step = 0U;
        enter_phase(p_op, OP_PHASE_CAL_PROMPT, now);
    }
    else if ((unsigned int)OP_PRESS_SHORT == value_go)
    {
        enter_phase(p_op, OP_PHASE_COUNTDOWN, now);
    }
    else
    {
        /* Wait in READY for the next button event. */
    }
}

/**
 * @brief Await RESET result or timeout and return to the selected retry phase.
 * @param[in,out] p_op Owned initialized state with the reset destination
 * chosen.
 * @param now Wrapping millisecond time for OP_REPLY_TIMEOUT_MS.
 * @param[in] p_st Non-NULL snapshot providing sequence/result/readiness.
 * @note Reply or timeout returns to CAL_PROMPT or READY and clears reply
 * bookkeeping through enter. It prints the outcome but submits no new request.
 */
static void reset_wait (op_logic_t * p_op, uint32_t now,
                        const s3_integration_status_t * p_st)
{
    bool b_did_succeed = false;

    if (replied(p_op, p_st, &b_did_succeed))
    {
        if (b_did_succeed)
        {
            print_status(p_op, "Fault reset.");
        }
        else
        {
            (void)p_op->port.p_print(
                (const unsigned char *)"%sReset refused: motors must be "
                                       "stopped "
                                       "and sensors healthy (ready=0x%x). "
                                       "Press "
                                       "GP20 to retry, or power-cycle the "
                                       "car.\n",
                OP_TAG, (unsigned)p_st->navigation.readiness_blockers);
        }

        enter_phase(p_op,
                    p_op->b_should_reset_to_cal ? OP_PHASE_CAL_PROMPT
                                                : OP_PHASE_READY,
                    now);
    }
    else if ((now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
    {
        print_status(p_op, "No reply from controller.");
        enter_phase(p_op,
                    p_op->b_should_reset_to_cal ? OP_PHASE_CAL_PROMPT
                                                : OP_PHASE_READY,
                    now);
    }
    else
    {
        /* The reset request is still awaiting a reply. */
    }
}

/**
 * @brief Print remaining seconds, then submit START at the existing deadline.
 * @param[in,out] p_op Owned initialized state with a countdown-entry timestamp.
 * @param now Wrapping millisecond time; elapsed time uses unsigned subtraction.
 * @param[in] p_st Non-NULL snapshot used when submitting START.
 * @note Rejected submission returns to READY. Accepted submission enters START
 * to await execution; delayed polls can skip intermediate printed seconds.
 */
static void countdown (op_logic_t * p_op, uint32_t now,
                       const s3_integration_status_t * p_st)
{
    const uint32_t elapsed = now - p_op->phase_ms;
    const unsigned second  = (unsigned)(elapsed / 1000U);

    if (elapsed >= OP_COUNTDOWN_MS)
    {
        if (submit_request(p_op, S3_CONTROL_START, S3_SENSOR_BARCODE, p_st,
                           now))
        {
            p_op->phase = OP_PHASE_START;
        }
        else
        {
            enter_phase(p_op, OP_PHASE_READY, now);
        }
    }
    else if (second >= p_op->countdown_shown)
    {
        (void)p_op->port.p_print(
            (const unsigned char *)"%sStarting in %u... (GP21 cancels)\n",
            OP_TAG, (unsigned)(OP_COUNTDOWN_MS / 1000U) - second);
        p_op->countdown_shown = second + 1U;
    }
    else
    {
        /* This countdown second has already been displayed. */
    }
}

/**
 * @brief Await START execution and report its result without requesting motion.
 * @param[in,out] p_op Owned initialized state with pending reply bookkeeping.
 * @param now Wrapping millisecond time for the reply deadline.
 * @param[in] p_st Non-NULL snapshot with sequence/result/state/readiness.
 * @note An accepted execution enters RUN; rejection or timeout returns to
 * READY. Both transitions clear waiting state and print through the port.
 */
static void start_wait (op_logic_t * p_op, uint32_t now,
                        const s3_integration_status_t * p_st)
{
    bool b_did_succeed = false;

    if (replied(p_op, p_st, &b_did_succeed))
    {
        if (b_did_succeed)
        {
            print_status(p_op, "RUNNING. GP21 = STOP.");
            enter_phase(p_op, OP_PHASE_RUN, now);
        }
        else
        {
            (void)p_op->port.p_print(
                (const unsigned char *)"%sSTART refused: state=%s ready=0x%x "
                                       "(1 channels, 2 calibration, 4 sensors, "
                                       "8 odometry, 16 fault). Motors must be "
                                       "stopped and braked.\n",
                OP_TAG, op_logic_drive_state_name(p_st->state),
                (unsigned)p_st->navigation.readiness_blockers);
            enter_phase(p_op, OP_PHASE_READY, now);
        }
    }
    else if ((now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
    {
        print_status(p_op, "No reply from controller.");
        enter_phase(p_op, OP_PHASE_READY, now);
    }
    else
    {
        /* Keep waiting for the controller's start reply. */
    }
}

/**
 * @brief Cancel phase-local work after a STOP event without submitting STOP.
 * @param[in,out] p_op Owned initialized state; phase-specific fields may
 * change.
 * @param now_ms Wrapping millisecond phase-entry time.
 * @note Sampling/apply returns to its prompt (APPLY retries BLACK); pending
 * start returns to READY and prints cancellation; RESET returns to its retry
 * phase. Other phases are unchanged. The caller owns STOP
 * submission/inhibition.
 */
static void stop_pressed (op_logic_t * p_op, uint32_t now_ms)
{
    if (((unsigned int)OP_PHASE_CAL_COLLECT == p_op->phase) ||
        ((unsigned int)OP_PHASE_CAL_APPLY == p_op->phase))
    {
        if ((unsigned int)OP_PHASE_CAL_APPLY == p_op->phase)
        {
            p_op->step = ((p_op->step / 2U) * 2U) +
                         1U; /* Repeat the black calibration step. */
        }

        enter_phase(p_op, OP_PHASE_CAL_PROMPT, now_ms);
    }
    else if (((unsigned int)OP_PHASE_COUNTDOWN == p_op->phase) ||
             ((unsigned int)OP_PHASE_START == p_op->phase))
    {
        print_status(p_op, "Start cancelled.");
        enter_phase(p_op, OP_PHASE_READY, now_ms);
    }
    else if ((unsigned int)OP_PHASE_RESET == p_op->phase)
    {
        enter_phase(p_op,
                    p_op->b_should_reset_to_cal ? OP_PHASE_CAL_PROMPT
                                                : OP_PHASE_READY,
                    now_ms);
    }
    else
    {
        /* STOP needs no phase transition here. */
    }
}

/* End of file. */


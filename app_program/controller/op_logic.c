/** @file op_logic.c
 * @brief Operator controls: calibration walkthrough, START/STOP/RESET.
 */
/* Operator controls: calibration walkthrough, START/STOP/RESET. See header.
 * Every request goes through port.submit(); results are read back from the
 * published status (command_sequence/command_ok), never assumed. */
#include "op_logic.h"

#define OP_TAG "[OP] "

static const struct
{
    s3_sensor_id_t sensor;
    const char * name;
    unsigned gpio; /* from the single wiring definition in ir_acquisition.h */
} g_k_order[S3_SENSOR_COUNT] = {{S3_SENSOR_LEFT, "front-LEFT line sensor",
                                 S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT)},
                                {S3_SENSOR_RIGHT, "front-RIGHT line sensor",
                                 S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT)},
                                {S3_SENSOR_BARCODE, "BARCODE sensor",
                                 S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE)}};

/**
 * @brief Apply the static state-label contract in op_logic.h.
 * @param state Native state code; unknown codes select "?".
 * @return Immutable static string; no state or callback is used.
 */
const char * op_drive_state_name (s3_drive_state_t state)
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
const char * op_drive_reason_name (s3_drive_reason_t reason)
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
void op_button_init (op_button_t * p_button)
{
    if (p_button != 0)
    {
        *p_button = (op_button_t){0};
    }
}

/**
 * @brief Apply the debounce and startup-arming contract in op_logic.h.
 * @param[in,out] p_b Initialized button state; NULL returns NONE.
 * @param down Raw pressed level after caller polarity conversion.
 * @param now_ms Wrapping millisecond time, with unsigned elapsed arithmetic.
 * @param[out] p_edge Optional separate press-edge output, always cleared first.
 * @return Release classification; no clock or callback is invoked.
 */
op_press_t op_button_update (op_button_t * p_b, bool down, uint32_t now_ms,
                             bool * p_edge)
{
    op_press_t press = OP_PRESS_NONE;
    if (p_edge != 0)
    {
        *p_edge = false;
    }
    if (p_b == 0)
    {
        return OP_PRESS_NONE;
    }
    if (!p_b->seen || (down != p_b->raw))
    {
        p_b->seen = true;
        p_b->raw = down;
        p_b->since_ms = now_ms;
        return OP_PRESS_NONE;
    }
    if ((p_b->raw != p_b->stable) &&
        ((uint32_t)(now_ms - p_b->since_ms) >= OP_DEBOUNCE_MS))
    {
        p_b->stable = p_b->raw;
        if (p_b->stable)
        {
            p_b->pressed_ms = p_b->since_ms;
            if (p_b->armed && (p_edge != 0))
            {
                *p_edge = true;
            }
        }
        else if (p_b->armed)
        {
            press = ((uint32_t)(p_b->since_ms - p_b->pressed_ms) >=
                     OP_LONG_PRESS_MS)
                        ? OP_PRESS_LONG
                        : OP_PRESS_SHORT;
        }
    }
    /* A button held at power-up is ignored until seen released for 100 ms. */
    if (!p_b->armed && !p_b->raw && !p_b->stable &&
        ((uint32_t)(now_ms - p_b->since_ms) >= OP_RELEASE_MS))
    {
        p_b->armed = true;
    }
    return press;
}

/**
 * @brief Read the stable button level without consuming an event.
 * @param[in] p_button Borrowed state; NULL returns false.
 * @return The stored stable level, even if press events are not yet armed.
 */
bool op_button_held (const op_button_t * p_button)
{
    return (p_button != 0) && p_button->stable;
}

/**
 * @brief Validate callbacks, clear state and copy the port as documented.
 * @param[out] p_op Caller-owned state; unchanged on validation failure.
 * @param[in] p_port Borrowed port; copied after state clearing, without
 * protection against overlap with p_op.
 * @return True after initialization; false for a NULL pointer or callback.
 */
bool operator_init (operator_t * p_op, const op_port_t * p_port)
{
    if ((p_op == 0) || (p_port == 0) || (p_port->submit == 0) ||
        (p_port->print == 0))
    {
        return false;
    }
    *p_op = (operator_t){0};
    p_op->port = *p_port;
    p_op->phase = OP_PHASE_WAIT_STATUS;
    return true;
}

/**
 * @brief Enter a phase and clear prompt, reply-wait and countdown bookkeeping.
 * @param[in,out] p_op Non-NULL initialized state owned by this update.
 * @param phase Destination phase; assigned without additional validation.
 * @param now Wrapping millisecond phase-entry timestamp.
 * @note Leaves calibration step, sequence and STOP/inhibition flags intact.
 */
static void enter (operator_t * p_op, op_phase_t phase, uint32_t now)
{
    p_op->phase = phase;
    p_op->phase_ms = now;
    p_op->prompted = false;
    p_op->awaiting_reply = false;
    p_op->countdown_shown = 0U;
}

/**
 * @brief Print one tagged console line through the configured port.
 * @param[in] p_op Non-NULL initialized state with a callable print callback.
 * @param[in] p_text Borrowed NUL-terminated text, valid through the callback.
 * @note Invokes print synchronously and ignores its return value.
 */
static void say (operator_t * p_op, const char * p_text)
{
    (void)p_op->port.print((const unsigned char *)"%s%s\n", OP_TAG, p_text);
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
 * consume/copy the request before returning; acceptance does not imply execution.
 */
static bool send (operator_t * p_op, s3_control_kind_t kind,
                  s3_sensor_id_t sensor, const s3_integration_status_t * p_st,
                  uint32_t now)
{
    const s3_control_request_t request = {.kind = kind, .sensor = sensor};
    p_op->sequence = p_st->command_sequence;
    p_op->phase_ms = now;
    p_op->awaiting_reply = p_op->port.submit(&request);
    if (!p_op->awaiting_reply)
    {
        say(p_op, "Controller busy; press GP20 again.");
    }
    return p_op->awaiting_reply;
}

/* True once the controller has executed our request; *ok is its result. */
/**
 * @brief Recognize changed controller sequence while a reply is awaited.
 * @param[in] p_op Non-NULL operator state, read without mutation.
 * @param[in] p_st Non-NULL coherent controller snapshot.
 * @param[out] p_ok Non-NULL output; written only when a changed sequence is seen.
 * @return True after copying command_ok, otherwise false with p_ok unchanged.
 * @note Does not clear awaiting_reply or identify a command beyond the existing
 * sequence comparison; it performs no callback or synchronization.
 */
static bool replied (const operator_t * p_op,
                     const s3_integration_status_t * p_st, bool * p_ok)
{
    if (!p_op->awaiting_reply || (p_st->command_sequence == p_op->sequence))
    {
        return false;
    }
    *p_ok = p_st->command_ok;
    return true;
}

/**
 * @brief Test and advance the repeating-prompt deadline.
 * @param[in,out] p_op Non-NULL state owned by this update.
 * @param now Wrapping millisecond time.
 * @return True for the first prompt or after OP_PROMPT_REPEAT_MS, updating
 * prompted/prompt_ms; false leaves both unchanged. Does not print by itself.
 */
static bool due (operator_t * p_op, uint32_t now)
{
    if (!p_op->prompted ||
        ((uint32_t)(now - p_op->prompt_ms) >= OP_PROMPT_REPEAT_MS))
    {
        p_op->prompted = true;
        p_op->prompt_ms = now;
        return true;
    }
    return false;
}

/**
 * @brief Test the existing inclusive active-drive state interval.
 * @param state Native drive-state code; no enum normalization is performed.
 * @return True from FOLLOW through REACQUIRE, otherwise false; no side effects.
 */
static bool running (s3_drive_state_t state)
{
    return (state >= S3_DRIVE_FOLLOW) && (state <= S3_DRIVE_REACQUIRE);
}

/**
 * @brief Print one calibration range using count as the validity discriminator.
 * @param[in] p_op Initialized state with a callable synchronous print port.
 * @param[in] p_what Borrowed NUL-terminated label.
 * @param[in] p_r Non-NULL summary of raw ADC values and sample count.
 * @note Prints zero minimum/maximum when count is zero; changes no stored
 * values and ignores the callback's return value.
 */
static void print_range (operator_t * p_op, const char * p_what,
                         const s3_calibration_summary_t * p_r)
{
    (void)p_op->port.print(
        (const unsigned char *)"%s  %s: %u samples, raw %u..%u\n", OP_TAG,
        p_what, (unsigned)p_r->count,
        (unsigned)(p_r->count ? p_r->minimum : 0U),
        (unsigned)(p_r->count ? p_r->maximum : 0U));
}

/**
 * @brief Prompt for the selected surface or request RESET while faulted.
 * @param[in,out] p_op Owned initialized state with step in 0..OP_CAL_STEPS-1.
 * @param now Wrapping millisecond time for prompts and requests.
 * @param value_go Any non-NONE value can submit the phase's request.
 * @param[in] p_st Non-NULL snapshot; FAULT takes precedence over calibration.
 * @note Successful submission changes phase; rejection retains the prompt
 * phase while send updates its bookkeeping. Console/callback effects are direct.
 */
static void cal_prompt (operator_t * p_op, uint32_t now, op_press_t value_go,
                        const s3_integration_status_t * p_st)
{
    const unsigned index = p_op->step / 2U;
    const bool black = (p_op->step % 2U) != 0U;
    if (p_st->state == S3_DRIVE_FAULT)
    {
        /* Calibration is refused while faulted: offer RESET, never a loop of
         * refused sampling attempts. */
        if (due(p_op, now))
        {
            (void)p_op->port.print(
                (const unsigned char *)"%sFAULT: %s. Calibration is blocked. "
                                       "Press GP20 to reset (or power-cycle "
                                       "the car).\n",
                OP_TAG, op_drive_reason_name(p_st->reason));
        }
        if ((value_go != OP_PRESS_NONE) &&
            send(p_op, S3_CONTROL_RESET, S3_SENSOR_BARCODE, p_st, now))
        {
            p_op->reset_to_cal = true;
            p_op->phase = OP_PHASE_RESET;
        }
        return;
    }
    if (due(p_op, now))
    {
        (void)p_op->port.print(
            (const unsigned char *)"%sCalibration %u/%u: put the %s (GP%u) "
                                   "over %s. "
                                   "Keep the car still, then press GP20.\n",
            OP_TAG, p_op->step + 1U, OP_CAL_STEPS, g_k_order[index].name,
            g_k_order[index].gpio,
            black ? "the BLACK line (sensor centred on the tape)"
                  : "plain WHITE mat");
    }
    if ((value_go != OP_PRESS_NONE) &&
        send(p_op, black ? S3_CONTROL_BLACK : S3_CONTROL_WHITE,
             g_k_order[index].sensor, p_st, now))
    {
        p_op->phase = OP_PHASE_CAL_COLLECT;
    }
}

/**
 * @brief Await sampling acknowledgement/completion, then advance or apply.
 * @param[in,out] p_op Owned initialized state with a valid calibration step.
 * @param now Wrapping millisecond time; deadlines use the request timestamp.
 * @param[in] p_st Non-NULL snapshot supplying reply, sensor and sample summaries.
 * @note A rejected/timed-out reply returns to the prompt. Completion prints
 * the selected range; WHITE advances the step, BLACK submits APPLY. No wait
 * loop or motor call is performed; callback acceptance remains distinct from
 * controller execution.
 */
static void cal_collect (operator_t * p_op, uint32_t now,
                         const s3_integration_status_t * p_st)
{
    const unsigned index = p_op->step / 2U;
    const bool black = (p_op->step % 2U) != 0U;
    bool value_ok = false;
    if (p_op->awaiting_reply)
    {
        if (replied(p_op, p_st, &value_ok))
        {
            p_op->awaiting_reply = false;
            if (!value_ok)
            {
                (void)p_op->port.print(
                    (const unsigned char *)"%sSampling refused (state=%s). "
                                           "Press GP20 to retry.\n",
                    OP_TAG, op_drive_state_name(p_st->state));
                enter(p_op, OP_PHASE_CAL_PROMPT, now);
                return;
            }
            say(p_op, "Sampling for 1 s - keep still...");
        }
        else if ((uint32_t)(now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
        {
            say(p_op, "No reply from controller. Press GP20 to retry.");
            enter(p_op, OP_PHASE_CAL_PROMPT, now);
        }
        return;
    }
    if (p_st->collecting ||
        (p_st->calibration_sensor != g_k_order[index].sensor))
    {
        if ((uint32_t)(now - p_op->phase_ms) >= OP_COLLECT_TIMEOUT_MS)
        {
            say(p_op, "Sampling did not finish. Press GP20 to retry.");
            enter(p_op, OP_PHASE_CAL_PROMPT, now);
        }
        return;
    }
    print_range(p_op, black ? "BLACK" : "WHITE",
                black ? &p_st->black : &p_st->white);
    if (!black)
    {
        p_op->step++;
        enter(p_op, OP_PHASE_CAL_PROMPT, now);
    }
    else if (send(p_op, S3_CONTROL_APPLY_CALIBRATION, g_k_order[index].sensor,
                  p_st, now))
    {
        p_op->phase = OP_PHASE_CAL_APPLY;
    }
    else
    {
        enter(p_op, OP_PHASE_CAL_PROMPT, now);
    }
}

/**
 * @brief Await APPLY and retain the existing retry/next-sensor transitions.
 * @param[in,out] p_op Owned initialized state at a valid BLACK step.
 * @param now Wrapping millisecond time for reply timeout and phase entry.
 * @param[in] p_st Non-NULL snapshot supplying sequence, result and sample ranges.
 * @note Timeout retries BLACK; rejection retries both surfaces for this sensor.
 * Success advances the step, reaching READY after the final sensor. Prints
 * through the existing callback without changing calibration directly.
 */
static void cal_apply (operator_t * p_op, uint32_t now,
                       const s3_integration_status_t * p_st)
{
    const unsigned index = p_op->step / 2U;
    bool value_ok = false;
    if (!replied(p_op, p_st, &value_ok))
    {
        if ((uint32_t)(now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
        {
            say(p_op, "No reply from controller. Redo BLACK for this sensor.");
            enter(p_op, OP_PHASE_CAL_PROMPT, now);
        }
        return;
    }
    if (!value_ok)
    {
        (void)p_op->port.print(
            (const unsigned char *)"%s%s (GP%u) REJECTED: white and black "
                                   "readings "
                                   "overlap (or too few samples). Redo this "
                                   "sensor; check its height and the "
                                   "surface.\n",
            OP_TAG, g_k_order[index].name, g_k_order[index].gpio);
        print_range(p_op, "WHITE", &p_st->white);
        print_range(p_op, "BLACK", &p_st->black);
        p_op->step = index * 2U;
        enter(p_op, OP_PHASE_CAL_PROMPT, now);
        return;
    }
    (void)p_op->port.print((const unsigned char *)"%s%s (GP%u) calibrated.\n",
                           OP_TAG, g_k_order[index].name,
                           g_k_order[index].gpio);
    p_op->step++;
    if (p_op->step >= OP_CAL_STEPS)
    {
        say(p_op, "All 3 sensors calibrated.");
        enter(p_op, OP_PHASE_READY, now);
    }
    else
    {
        enter(p_op, OP_PHASE_CAL_PROMPT, now);
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
static void ready (operator_t * p_op, uint32_t now, op_press_t value_go,
                   const s3_integration_status_t * p_st)
{
    const bool fault = p_st->state == S3_DRIVE_FAULT;
    if (due(p_op, now))
    {
        if (fault)
        {
            (void)p_op->port.print(
                (const unsigned char *)"%sFAULT: %s. Fix the cause, then press "
                                       "GP20 to reset. If reset keeps failing, "
                                       "power-cycle the car.\n",
                OP_TAG, op_drive_reason_name(p_st->reason));
        }
        else
        {
            say(p_op,
                "READY: put the car on the line, then press GP20 to start "
                "(3 s countdown). Hold GP20 2 s to recalibrate. GP21 = STOP.");
        }
    }
    if (fault && (value_go != OP_PRESS_NONE))
    {
        /* The controller refuses calibration while faulted, so every press
         * (short or long) means RESET here; recalibrate after the reset. */
        p_op->reset_to_cal = false;
        if (send(p_op, S3_CONTROL_RESET, S3_SENSOR_BARCODE, p_st, now))
        {
            p_op->phase = OP_PHASE_RESET;
        }
    }
    else if (value_go == OP_PRESS_LONG)
    {
        say(p_op, "Recalibrating all sensors.");
        p_op->step = 0U;
        enter(p_op, OP_PHASE_CAL_PROMPT, now);
    }
    else if (value_go == OP_PRESS_SHORT)
    {
        enter(p_op, OP_PHASE_COUNTDOWN, now);
    }
}

/**
 * @brief Await RESET result or timeout and return to the selected retry phase.
 * @param[in,out] p_op Owned initialized state with reset_to_cal already chosen.
 * @param now Wrapping millisecond time for OP_REPLY_TIMEOUT_MS.
 * @param[in] p_st Non-NULL snapshot providing sequence/result/readiness.
 * @note Reply or timeout returns to CAL_PROMPT or READY and clears reply
 * bookkeeping through enter. It prints the outcome but submits no new request.
 */
static void reset_wait (operator_t * p_op, uint32_t now,
                        const s3_integration_status_t * p_st)
{
    bool value_ok = false;
    if (replied(p_op, p_st, &value_ok))
    {
        if (value_ok)
        {
            say(p_op, "Fault reset.");
        }
        else
        {
            (void)p_op->port.print(
                (const unsigned char *)"%sReset refused: motors must be "
                                       "stopped "
                                       "and sensors healthy (ready=0x%x). "
                                       "Press "
                                       "GP20 to retry, or power-cycle the "
                                       "car.\n",
                OP_TAG, (unsigned)p_st->navigation.readiness_blockers);
        }
        enter(p_op, p_op->reset_to_cal ? OP_PHASE_CAL_PROMPT : OP_PHASE_READY,
              now);
    }
    else if ((uint32_t)(now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
    {
        say(p_op, "No reply from controller.");
        enter(p_op, p_op->reset_to_cal ? OP_PHASE_CAL_PROMPT : OP_PHASE_READY,
              now);
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
static void countdown (operator_t * p_op, uint32_t now,
                       const s3_integration_status_t * p_st)
{
    const uint32_t elapsed = now - p_op->phase_ms;
    const unsigned second = (unsigned)(elapsed / 1000U);
    if (elapsed >= OP_COUNTDOWN_MS)
    {
        if (send(p_op, S3_CONTROL_START, S3_SENSOR_BARCODE, p_st, now))
        {
            p_op->phase = OP_PHASE_START;
        }
        else
        {
            enter(p_op, OP_PHASE_READY, now);
        }
    }
    else if (second >= p_op->countdown_shown)
    {
        (void)p_op->port.print(
            (const unsigned char *)"%sStarting in %u... (GP21 cancels)\n",
            OP_TAG, (unsigned)(OP_COUNTDOWN_MS / 1000U) - second);
        p_op->countdown_shown = second + 1U;
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
static void start_wait (operator_t * p_op, uint32_t now,
                        const s3_integration_status_t * p_st)
{
    bool value_ok = false;
    if (replied(p_op, p_st, &value_ok))
    {
        if (value_ok)
        {
            say(p_op, "RUNNING. GP21 = STOP.");
            enter(p_op, OP_PHASE_RUN, now);
        }
        else
        {
            (void)p_op->port.print(
                (const unsigned char *)"%sSTART refused: state=%s ready=0x%x "
                                       "(1 channels, 2 calibration, 4 sensors, "
                                       "8 odometry, 16 fault). Motors must be "
                                       "stopped and braked.\n",
                OP_TAG, op_drive_state_name(p_st->state),
                (unsigned)p_st->navigation.readiness_blockers);
            enter(p_op, OP_PHASE_READY, now);
        }
    }
    else if ((uint32_t)(now - p_op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
    {
        say(p_op, "No reply from controller.");
        enter(p_op, OP_PHASE_READY, now);
    }
}

/**
 * @brief Cancel phase-local work after a STOP event without submitting STOP.
 * @param[in,out] p_op Owned initialized state; phase-specific fields may change.
 * @param now_ms Wrapping millisecond phase-entry time.
 * @note Sampling/apply returns to its prompt (APPLY retries BLACK); pending
 * start returns to READY and prints cancellation; RESET returns to its retry
 * phase. Other phases are unchanged. The caller owns STOP submission/inhibition.
 */
static void stop_pressed (operator_t * p_op, uint32_t now_ms)
{
    if ((p_op->phase == OP_PHASE_CAL_COLLECT) ||
        (p_op->phase == OP_PHASE_CAL_APPLY))
    {
        if (p_op->phase == OP_PHASE_CAL_APPLY)
        {
            p_op->step = (p_op->step / 2U) * 2U + 1U; /* redo BLACK */
        }
        enter(p_op, OP_PHASE_CAL_PROMPT, now_ms);
    }
    else if ((p_op->phase == OP_PHASE_COUNTDOWN) ||
             (p_op->phase == OP_PHASE_START))
    {
        say(p_op, "Start cancelled.");
        enter(p_op, OP_PHASE_READY, now_ms);
    }
    else if (p_op->phase == OP_PHASE_RESET)
    {
        enter(p_op, p_op->reset_to_cal ? OP_PHASE_CAL_PROMPT : OP_PHASE_READY,
              now_ms);
    }
}

/**
 * @brief Apply the remote-STOP cancellation contract in op_logic.h.
 * @param[in,out] p_op Initialized owned state; NULL is a no-op.
 * @param now_ms Wrapping millisecond timestamp for cancellation transitions.
 * @note Sets local inhibition and never submits a duplicate STOP.
 */
void operator_remote_stop (operator_t * p_op, uint32_t now_ms)
{
    if (!p_op)
    {
        return;
    }
    p_op->inhibit = true;
    stop_pressed(p_op, now_ms);
}

/**
 * @brief Apply STOP precedence and the operator-phase contract in op_logic.h.
 * @param[in,out] p_op Initialized exclusively accessed state; NULL is a no-op.
 * @param now_ms Wrapping millisecond time used by the selected phase.
 * @param value_go Debounced GO event; inhibition can discard this event.
 * @param stop_held Debounced STOP level, serviced before consulting status.
 * @param go_held Debounced GO level, used for the both-released gate.
 * @param[in] p_st Optional coherent snapshot, borrowed only for this call.
 * @note Missing status prevents phase processing, not STOP/inhibition. Invalid
 * phase values fall back to READY. Synchronous submit/print effects and exact
 * sequence-based acknowledgement remain as specified by the public contract.
 */
void operator_step (operator_t * p_op, uint32_t now_ms, op_press_t value_go,
                    bool stop_held, bool go_held,
                    const s3_integration_status_t * p_st)
{
    if (p_op == 0)
    {
        return;
    }
    /* STOP first, in every phase, without waiting for status. Level-based:
     * a GP21 held since power-up, or held before GP20, still counts. */
    if (stop_held)
    {
        p_op->inhibit = true;
        if (!p_op->stop_handled)
        {
            const s3_control_request_t request = {.kind = S3_CONTROL_STOP};
            if (p_op->port.submit(&request))
            {
                p_op->stop_handled = true;
                say(p_op, "STOP sent.");
            }
            else if (!p_op->stop_warned)
            {
                /* Retried every poll while held; reported once per hold. */
                say(p_op, "STOP not accepted yet, retrying. Use the power "
                          "switch if the car moves.");
                p_op->stop_warned = true;
            }
            stop_pressed(p_op, now_ms);
        }
    }
    else
    {
        p_op->stop_handled = false;
        p_op->stop_warned = false;
    }
    if (p_op->inhibit)
    {
        if ((value_go != OP_PRESS_NONE) && !p_op->ignore_warned)
        {
            say(p_op, "Ignored GP20: release GP21 first.");
            p_op->ignore_warned = true;
        }
        value_go =
            OP_PRESS_NONE; /* discard, including a release on this poll */
        if (!stop_held && !go_held)
        {
            p_op->inhibit = false;
            p_op->ignore_warned = false;
        }
    }
    if (p_st == 0)
    {
        return;
    }
    switch (p_op->phase)
    {
    case OP_PHASE_WAIT_STATUS:
        say(p_op, "Operator controls: GP20 = next/start, GP21 = STOP.");
        p_op->step = 0U;
        if (p_st->navigation.accepted_calibration == S3_ALL_SENSORS)
        {
            /* Built-in calibration was accepted at boot: no walkthrough. */
            say(p_op, "Using built-in calibration. Hold GP20 2 s at READY to "
                      "recalibrate by hand.");
            enter(p_op, OP_PHASE_READY, now_ms);
        }
        else
        {
            enter(p_op, OP_PHASE_CAL_PROMPT, now_ms);
        }
        break;
    case OP_PHASE_CAL_PROMPT:
        cal_prompt(p_op, now_ms, value_go, p_st);
        break;
    case OP_PHASE_CAL_COLLECT:
        cal_collect(p_op, now_ms, p_st);
        break;
    case OP_PHASE_CAL_APPLY:
        cal_apply(p_op, now_ms, p_st);
        break;
    case OP_PHASE_READY:
        ready(p_op, now_ms, value_go, p_st);
        break;
    case OP_PHASE_RESET:
        reset_wait(p_op, now_ms, p_st);
        break;
    case OP_PHASE_COUNTDOWN:
        countdown(p_op, now_ms, p_st);
        break;
    case OP_PHASE_START:
        start_wait(p_op, now_ms, p_st);
        break;
    case OP_PHASE_RUN:
        if (!running(p_st->state))
        {
            (void)p_op->port.print(
                (const unsigned char *)"%sRun ended: %s - %s.\n", OP_TAG,
                op_drive_state_name(p_st->state),
                op_drive_reason_name(p_st->reason));
            enter(p_op, OP_PHASE_READY, now_ms);
        }
        break;
    default:
        enter(p_op, OP_PHASE_READY, now_ms);
        break;
    }
}

/* End of file. */


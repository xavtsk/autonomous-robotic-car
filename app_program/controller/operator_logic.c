/* Operator controls: calibration walkthrough, START/STOP/RESET. See header.
 * Every request goes through port.submit(); results are read back from the
 * published status (command_sequence/command_ok), never assumed. */
#include "operator_logic.h"

#define OP_TAG "[OP] "

static const struct
{
    s3_sensor_id_t sensor;
    const char *name;
    unsigned gpio; /* from the single wiring definition in ir_acquisition.h */
} k_order[S3_SENSOR_COUNT] = {
    {S3_SENSOR_LEFT, "front-LEFT line sensor",
     S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_LEFT)},
    {S3_SENSOR_RIGHT, "front-RIGHT line sensor",
     S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_RIGHT)},
    {S3_SENSOR_BARCODE, "BARCODE sensor",
     S3_GPIO_FOR_CHANNEL(S3_ADC_CHANNEL_BARCODE)}};

const char *op_drive_state_name(s3_drive_state_t state)
{
    switch (state)
    {
    case S3_DRIVE_OFF: return "OFF";
    case S3_DRIVE_FOLLOW: return "FOLLOW";
    case S3_DRIVE_BRAKING: return "BRAKING";
    case S3_DRIVE_MANOEUVRE: return "MANOEUVRE";
    case S3_DRIVE_REACQUIRE: return "REACQUIRE";
    case S3_DRIVE_HALTED: return "HALTED";
    case S3_DRIVE_FAULT: return "FAULT";
    default: return "?";
    }
}
const char *op_drive_reason_name(s3_drive_reason_t reason)
{
    switch (reason)
    {
    case S3_DRIVE_OK: return "ok/stopped by operator";
    case S3_DRIVE_SENSOR_FAULT: return "IR sensor fault";
    case S3_DRIVE_MOTION_FAULT: return "motor/encoder fault";
    case S3_DRIVE_REJECTED: return "motion request rejected";
    case S3_DRIVE_LINE_LOST: return "line lost";
    case S3_DRIVE_NO_DIRECTION:
        return "junction without a barcode, or two barcodes before a junction";
    case S3_DRIVE_UNMAPPED: return "barcode letter not in the map";
    case S3_DRIVE_TIMEOUT: return "timeout (barcode->junction 5 s, brake or turn)";
    case S3_DRIVE_REPORT_FAULT: return "motion report fault";
    case S3_DRIVE_BAD_INPUT: return "bad input";
    default: return "?";
    }
}

void op_button_init(op_button_t *button)
{
    if (button != 0)
    {
        *button = (op_button_t){0};
    }
}
op_press_t op_button_update(op_button_t *b, bool down, uint32_t now_ms,
                            bool *edge)
{
    op_press_t press = OP_PRESS_NONE;
    if (edge != 0)
    {
        *edge = false;
    }
    if (b == 0)
    {
        return OP_PRESS_NONE;
    }
    if (!b->seen || (down != b->raw))
    {
        b->seen = true;
        b->raw = down;
        b->since_ms = now_ms;
        return OP_PRESS_NONE;
    }
    if ((b->raw != b->stable) &&
        ((uint32_t)(now_ms - b->since_ms) >= OP_DEBOUNCE_MS))
    {
        b->stable = b->raw;
        if (b->stable)
        {
            b->pressed_ms = b->since_ms;
            if (b->armed && (edge != 0))
            {
                *edge = true;
            }
        }
        else if (b->armed)
        {
            press = ((uint32_t)(b->since_ms - b->pressed_ms) >=
                     OP_LONG_PRESS_MS)
                        ? OP_PRESS_LONG
                        : OP_PRESS_SHORT;
        }
    }
    /* A button held at power-up is ignored until seen released for 100 ms. */
    if (!b->armed && !b->raw && !b->stable &&
        ((uint32_t)(now_ms - b->since_ms) >= OP_RELEASE_MS))
    {
        b->armed = true;
    }
    return press;
}

bool op_button_held(const op_button_t *button)
{
    return (button != 0) && button->stable;
}

bool operator_init(operator_t *op, const op_port_t *port)
{
    if ((op == 0) || (port == 0) || (port->submit == 0) || (port->print == 0))
    {
        return false;
    }
    *op = (operator_t){0};
    op->port = *port;
    op->phase = OP_PHASE_WAIT_STATUS;
    return true;
}

static void enter(operator_t *op, op_phase_t phase, uint32_t now)
{
    op->phase = phase;
    op->phase_ms = now;
    op->prompted = false;
    op->awaiting_reply = false;
    op->countdown_shown = 0U;
}
static void say(operator_t *op, const char *text)
{
    (void)op->port.print((const unsigned char *)"%s%s\n", OP_TAG, text);
}
static bool send(operator_t *op, s3_control_kind_t kind, s3_sensor_id_t sensor,
                 const s3_integration_status_t *st, uint32_t now)
{
    const s3_control_request_t request = {.kind = kind, .sensor = sensor};
    op->sequence = st->command_sequence;
    op->phase_ms = now;
    op->awaiting_reply = op->port.submit(&request);
    if (!op->awaiting_reply)
    {
        say(op, "Controller busy; press GP20 again.");
    }
    return op->awaiting_reply;
}
/* True once the controller has executed our request; *ok is its result. */
static bool replied(const operator_t *op, const s3_integration_status_t *st,
                    bool *ok)
{
    if (!op->awaiting_reply || (st->command_sequence == op->sequence))
    {
        return false;
    }
    *ok = st->command_ok;
    return true;
}
static bool due(operator_t *op, uint32_t now)
{
    if (!op->prompted || ((uint32_t)(now - op->prompt_ms) >= OP_PROMPT_REPEAT_MS))
    {
        op->prompted = true;
        op->prompt_ms = now;
        return true;
    }
    return false;
}
static bool running(s3_drive_state_t state)
{
    return (state >= S3_DRIVE_FOLLOW) && (state <= S3_DRIVE_REACQUIRE);
}
static void print_range(operator_t *op, const char *what,
                        const s3_calibration_summary_t *r)
{
    (void)op->port.print(
        (const unsigned char *)"%s  %s: %u samples, raw %u..%u\n", OP_TAG, what,
        (unsigned)r->count, (unsigned)(r->count ? r->minimum : 0U),
        (unsigned)(r->count ? r->maximum : 0U));
}

static void cal_prompt(operator_t *op, uint32_t now, op_press_t go,
                       const s3_integration_status_t *st)
{
    const unsigned index = op->step / 2U;
    const bool black = (op->step % 2U) != 0U;
    if (st->state == S3_DRIVE_FAULT)
    {
        /* Calibration is refused while faulted: offer RESET, never a loop of
         * refused sampling attempts. */
        if (due(op, now))
        {
            (void)op->port.print(
                (const unsigned char *)"%sFAULT: %s. Calibration is blocked. "
                                       "Press GP20 to reset (or power-cycle "
                                       "the car).\n",
                OP_TAG, op_drive_reason_name(st->reason));
        }
        if ((go != OP_PRESS_NONE) &&
            send(op, S3_CONTROL_RESET, S3_SENSOR_BARCODE, st, now))
        {
            op->reset_to_cal = true;
            op->phase = OP_PHASE_RESET;
        }
        return;
    }
    if (due(op, now))
    {
        (void)op->port.print(
            (const unsigned char *)"%sCalibration %u/%u: put the %s (GP%u) over %s. "
                                   "Keep the car still, then press GP20.\n",
            OP_TAG, op->step + 1U, OP_CAL_STEPS, k_order[index].name,
            k_order[index].gpio,
            black ? "the BLACK line (sensor centred on the tape)"
                  : "plain WHITE mat");
    }
    if ((go != OP_PRESS_NONE) &&
        send(op, black ? S3_CONTROL_BLACK : S3_CONTROL_WHITE,
             k_order[index].sensor, st, now))
    {
        op->phase = OP_PHASE_CAL_COLLECT;
        }
}
static void cal_collect(operator_t *op, uint32_t now,
                        const s3_integration_status_t *st)
{
    const unsigned index = op->step / 2U;
    const bool black = (op->step % 2U) != 0U;
    bool ok = false;
    if (op->awaiting_reply)
    {
        if (replied(op, st, &ok))
        {
            op->awaiting_reply = false;
            if (!ok)
            {
                (void)op->port.print(
                    (const unsigned char *)"%sSampling refused (state=%s). "
                                           "Press GP20 to retry.\n",
                    OP_TAG, op_drive_state_name(st->state));
                enter(op, OP_PHASE_CAL_PROMPT, now);
                return;
            }
            say(op, "Sampling for 1 s - keep still...");
        }
        else if ((uint32_t)(now - op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
        {
            say(op, "No reply from controller. Press GP20 to retry.");
            enter(op, OP_PHASE_CAL_PROMPT, now);
        }
        return;
    }
    if (st->collecting || (st->calibration_sensor != k_order[index].sensor))
    {
        if ((uint32_t)(now - op->phase_ms) >= OP_COLLECT_TIMEOUT_MS)
        {
            say(op, "Sampling did not finish. Press GP20 to retry.");
            enter(op, OP_PHASE_CAL_PROMPT, now);
        }
        return;
    }
    print_range(op, black ? "BLACK" : "WHITE", black ? &st->black : &st->white);
    if (!black)
    {
        op->step++;
        enter(op, OP_PHASE_CAL_PROMPT, now);
    }
    else if (send(op, S3_CONTROL_APPLY_CALIBRATION, k_order[index].sensor, st,
                  now))
    {
        op->phase = OP_PHASE_CAL_APPLY;
    }
    else
    {
        enter(op, OP_PHASE_CAL_PROMPT, now);
    }
}
static void cal_apply(operator_t *op, uint32_t now,
                      const s3_integration_status_t *st)
{
    const unsigned index = op->step / 2U;
    bool ok = false;
    if (!replied(op, st, &ok))
    {
        if ((uint32_t)(now - op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
        {
            say(op, "No reply from controller. Redo BLACK for this sensor.");
            enter(op, OP_PHASE_CAL_PROMPT, now);
        }
        return;
    }
    if (!ok)
    {
        (void)op->port.print(
            (const unsigned char *)"%s%s (GP%u) REJECTED: white and black readings "
                                   "overlap (or too few samples). Redo this "
                                   "sensor; check its height and the surface.\n",
            OP_TAG, k_order[index].name, k_order[index].gpio);
        print_range(op, "WHITE", &st->white);
        print_range(op, "BLACK", &st->black);
        op->step = index * 2U;
        enter(op, OP_PHASE_CAL_PROMPT, now);
        return;
    }
    (void)op->port.print((const unsigned char *)"%s%s (GP%u) calibrated.\n",
                         OP_TAG, k_order[index].name, k_order[index].gpio);
    op->step++;
    if (op->step >= OP_CAL_STEPS)
    {
        say(op, "All 3 sensors calibrated.");
        enter(op, OP_PHASE_READY, now);
    }
    else
    {
        enter(op, OP_PHASE_CAL_PROMPT, now);
    }
}
static void ready(operator_t *op, uint32_t now, op_press_t go,
                  const s3_integration_status_t *st)
{
    const bool fault = st->state == S3_DRIVE_FAULT;
    if (due(op, now))
    {
        if (fault)
        {
            (void)op->port.print(
                (const unsigned char *)"%sFAULT: %s. Fix the cause, then press "
                                       "GP20 to reset. If reset keeps failing, "
                                       "power-cycle the car.\n",
                OP_TAG, op_drive_reason_name(st->reason));
        }
        else
        {
            say(op, "READY: put the car on the line, then press GP20 to start "
                    "(3 s countdown). Hold GP20 2 s to recalibrate. GP21 = STOP.");
        }
    }
    if (fault && (go != OP_PRESS_NONE))
    {
        /* The controller refuses calibration while faulted, so every press
         * (short or long) means RESET here; recalibrate after the reset. */
        op->reset_to_cal = false;
        if (send(op, S3_CONTROL_RESET, S3_SENSOR_BARCODE, st, now))
        {
            op->phase = OP_PHASE_RESET;
        }
    }
    else if (go == OP_PRESS_LONG)
    {
        say(op, "Recalibrating all sensors.");
        op->step = 0U;
        enter(op, OP_PHASE_CAL_PROMPT, now);
    }
    else if (go == OP_PRESS_SHORT)
    {
        enter(op, OP_PHASE_COUNTDOWN, now);
    }
}
static void reset_wait(operator_t *op, uint32_t now,
                       const s3_integration_status_t *st)
{
    bool ok = false;
    if (replied(op, st, &ok))
    {
        if (ok)
        {
            say(op, "Fault reset.");
        }
        else
        {
            (void)op->port.print(
                (const unsigned char *)"%sReset refused: motors must be stopped "
                                       "and sensors healthy (ready=0x%x). Press "
                                       "GP20 to retry, or power-cycle the car.\n",
                OP_TAG, (unsigned)st->navigation.readiness_blockers);
        }
        enter(op, op->reset_to_cal ? OP_PHASE_CAL_PROMPT : OP_PHASE_READY, now);
    }
    else if ((uint32_t)(now - op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
    {
        say(op, "No reply from controller.");
        enter(op, op->reset_to_cal ? OP_PHASE_CAL_PROMPT : OP_PHASE_READY, now);
    }
}
static void countdown(operator_t *op, uint32_t now,
                      const s3_integration_status_t *st)
{
    const uint32_t elapsed = now - op->phase_ms;
    const unsigned second = (unsigned)(elapsed / 1000U);
    if (elapsed >= OP_COUNTDOWN_MS)
    {
        if (send(op, S3_CONTROL_START, S3_SENSOR_BARCODE, st, now))
        {
            op->phase = OP_PHASE_START;
        }
        else
        {
            enter(op, OP_PHASE_READY, now);
        }
    }
    else if (second >= op->countdown_shown)
    {
        (void)op->port.print(
            (const unsigned char *)"%sStarting in %u... (GP21 cancels)\n",
            OP_TAG, (unsigned)(OP_COUNTDOWN_MS / 1000U) - second);
        op->countdown_shown = second + 1U;
    }
}
static void start_wait(operator_t *op, uint32_t now,
                       const s3_integration_status_t *st)
{
    bool ok = false;
    if (replied(op, st, &ok))
    {
        if (ok)
        {
            say(op, "RUNNING. GP21 = STOP.");
            enter(op, OP_PHASE_RUN, now);
        }
        else
        {
            (void)op->port.print(
                (const unsigned char *)"%sSTART refused: state=%s ready=0x%x "
                                       "(1 channels, 2 calibration, 4 sensors, "
                                       "8 odometry, 16 fault). Motors must be "
                                       "stopped and braked.\n",
                OP_TAG, op_drive_state_name(st->state),
                (unsigned)st->navigation.readiness_blockers);
            enter(op, OP_PHASE_READY, now);
        }
    }
    else if ((uint32_t)(now - op->phase_ms) >= OP_REPLY_TIMEOUT_MS)
    {
        say(op, "No reply from controller.");
        enter(op, OP_PHASE_READY, now);
    }
}

static void stop_pressed(operator_t *op, uint32_t now_ms)
{
    if ((op->phase == OP_PHASE_CAL_COLLECT) || (op->phase == OP_PHASE_CAL_APPLY))
    {
        if (op->phase == OP_PHASE_CAL_APPLY)
        {
            op->step = (op->step / 2U) * 2U + 1U; /* redo BLACK */
        }
        enter(op, OP_PHASE_CAL_PROMPT, now_ms);
    }
    else if ((op->phase == OP_PHASE_COUNTDOWN) || (op->phase == OP_PHASE_START))
    {
        say(op, "Start cancelled.");
        enter(op, OP_PHASE_READY, now_ms);
    }
    else if (op->phase == OP_PHASE_RESET)
    {
        enter(op, op->reset_to_cal ? OP_PHASE_CAL_PROMPT : OP_PHASE_READY,
              now_ms);
    }
}

void operator_step(operator_t *op, uint32_t now_ms, op_press_t go,
                   bool stop_held, bool go_held,
                   const s3_integration_status_t *st)
{
    if (op == 0)
    {
        return;
    }
    /* STOP first, in every phase, without waiting for status. Level-based:
     * a GP21 held since power-up, or held before GP20, still counts. */
    if (stop_held)
    {
        op->inhibit = true;
        if (!op->stop_handled)
        {
            const s3_control_request_t request = {.kind = S3_CONTROL_STOP};
            if (op->port.submit(&request))
            {
                op->stop_handled = true;
                say(op, "STOP sent.");
            }
            else if (!op->stop_warned)
            {
                /* Retried every poll while held; reported once per hold. */
                say(op, "STOP not accepted yet, retrying. Use the power "
                        "switch if the car moves.");
                op->stop_warned = true;
            }
            stop_pressed(op, now_ms);
        }
    }
    else
    {
        op->stop_handled = false;
        op->stop_warned = false;
    }
    if (op->inhibit)
    {
        if ((go != OP_PRESS_NONE) && !op->ignore_warned)
        {
            say(op, "Ignored GP20: release GP21 first.");
            op->ignore_warned = true;
        }
        go = OP_PRESS_NONE; /* discard, including a release on this poll */
        if (!stop_held && !go_held)
        {
            op->inhibit = false;
            op->ignore_warned = false;
        }
    }
    if (st == 0)
    {
        return;
    }
    switch (op->phase)
    {
    case OP_PHASE_WAIT_STATUS:
        say(op, "Operator controls: GP20 = next/start, GP21 = STOP.");
        op->step = 0U;
        if (st->navigation.accepted_calibration == S3_ALL_SENSORS)
        {
            /* Built-in calibration was accepted at boot: no walkthrough. */
            say(op, "Using built-in calibration. Hold GP20 2 s at READY to "
                    "recalibrate by hand.");
            enter(op, OP_PHASE_READY, now_ms);
        }
        else
        {
            enter(op, OP_PHASE_CAL_PROMPT, now_ms);
        }
        break;
    case OP_PHASE_CAL_PROMPT: cal_prompt(op, now_ms, go, st); break;
    case OP_PHASE_CAL_COLLECT: cal_collect(op, now_ms, st); break;
    case OP_PHASE_CAL_APPLY: cal_apply(op, now_ms, st); break;
    case OP_PHASE_READY: ready(op, now_ms, go, st); break;
    case OP_PHASE_RESET: reset_wait(op, now_ms, st); break;
    case OP_PHASE_COUNTDOWN: countdown(op, now_ms, st); break;
    case OP_PHASE_START: start_wait(op, now_ms, st); break;
    case OP_PHASE_RUN:
        if (!running(st->state))
        {
            (void)op->port.print(
                (const unsigned char *)"%sRun ended: %s - %s.\n", OP_TAG,
                op_drive_state_name(st->state),
                op_drive_reason_name(st->reason));
            enter(op, OP_PHASE_READY, now_ms);
        }
        break;
    default: enter(op, OP_PHASE_READY, now_ms); break;
    }
}

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

void op_button_init (op_button_t * p_button)
{
    if (p_button != 0)
    {
        *p_button = (op_button_t){0};
    }
}

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

bool op_button_held (const op_button_t * p_button)
{
    return (p_button != 0) && p_button->stable;
}

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

static void enter (operator_t * p_op, op_phase_t phase, uint32_t now)
{
    p_op->phase = phase;
    p_op->phase_ms = now;
    p_op->prompted = false;
    p_op->awaiting_reply = false;
    p_op->countdown_shown = 0U;
}

static void say (operator_t * p_op, const char * p_text)
{
    (void)p_op->port.print((const unsigned char *)"%s%s\n", OP_TAG, p_text);
}

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

static bool running (s3_drive_state_t state)
{
    return (state >= S3_DRIVE_FOLLOW) && (state <= S3_DRIVE_REACQUIRE);
}

static void print_range (operator_t * p_op, const char * p_what,
                         const s3_calibration_summary_t * p_r)
{
    (void)p_op->port.print(
        (const unsigned char *)"%s  %s: %u samples, raw %u..%u\n", OP_TAG,
        p_what, (unsigned)p_r->count,
        (unsigned)(p_r->count ? p_r->minimum : 0U),
        (unsigned)(p_r->count ? p_r->maximum : 0U));
}

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

void operator_remote_stop (operator_t * p_op, uint32_t now_ms)
{
    if (!p_op)
    {
        return;
    }
    p_op->inhibit = true;
    stop_pressed(p_op, now_ms);
}

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

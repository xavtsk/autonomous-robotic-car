/*
 * Operator controls for the S3_MOTION profile: a calibration walkthrough,
 * START with a countdown, STOP and fault RESET, driven by two buttons.
 *
 * Pure logic, no RTOS or GPIO: the caller passes time, button presses and the
 * latest s3_integration status, and this module only ever calls the port's
 * submit() (the existing s3_integration_submit() in firmware). It never
 * requests motion directly and owns no navigation state, so the single
 * controller owner (s3_integration/s3_drive) is unchanged.
 */
#ifndef OPERATOR_LOGIC_H
#define OPERATOR_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include "s3_integration.h"

#define OP_DEBOUNCE_MS (30U)
#define OP_RELEASE_MS (100U)    /* both buttons released before first use */
#define OP_LONG_PRESS_MS (2000U)
#define OP_COUNTDOWN_MS (3000U)
#define OP_REPLY_TIMEOUT_MS (1000U)
#define OP_COLLECT_TIMEOUT_MS (3000U)
#define OP_PROMPT_REPEAT_MS (5000U)
#define OP_CAL_STEPS (2U * (unsigned)S3_SENSOR_COUNT)
_Static_assert(OP_CAL_STEPS <= 7U, "operator_t.step is 3 bits");

typedef enum { OP_PRESS_NONE, OP_PRESS_SHORT, OP_PRESS_LONG } op_press_t;

/* One debounced button. Short/long is reported on release; the "pressed"
 * edge is reported as soon as the press is stable (used for STOP). */
typedef struct
{
    uint32_t since_ms, pressed_ms;
    bool seen : 1, raw : 1, stable : 1, armed : 1;
} op_button_t;

typedef enum
{
    OP_PHASE_WAIT_STATUS,
    OP_PHASE_CAL_PROMPT,
    OP_PHASE_CAL_COLLECT,
    OP_PHASE_CAL_APPLY,
    OP_PHASE_READY,
    OP_PHASE_RESET,
    OP_PHASE_COUNTDOWN,
    OP_PHASE_START,
    OP_PHASE_RUN
} op_phase_t;

typedef struct
{
    bool (*submit)(const s3_control_request_t *request);
    int (*print)(const unsigned char *format, ...);
} op_port_t;

typedef struct
{
    op_port_t port;
    op_phase_t phase;
    uint32_t phase_ms, prompt_ms, sequence;
    unsigned step : 3;            /* calibration step 0..OP_CAL_STEPS (6) */
    unsigned countdown_shown : 2; /* seconds already announced, 0..3 */
    bool awaiting_reply : 1, prompted : 1;
    bool stop_handled : 1;  /* this GP21 hold already sent/handled STOP */
    bool stop_warned : 1;   /* bounded "STOP not accepted" message */
    bool inhibit : 1;       /* GP21 was held: ignore GP20 until both released */
    bool ignore_warned : 1; /* bounded "Ignored GP20" message */
    bool reset_to_cal : 1;  /* RESET was requested from a calibration prompt */
} operator_t;

void op_button_init(op_button_t *button);
/* Feed the raw level every poll. Returns the press kind on release. *edge is
 * set true once per press, when it becomes stable. */
op_press_t op_button_update(op_button_t *button, bool down, uint32_t now_ms,
                            bool *edge);
/* Debounced level, true while held. Unlike the press events this is reported
 * even for a button held since power-up, so a held STOP is always seen. */
bool op_button_held(const op_button_t *button);

bool operator_init(operator_t *op, const op_port_t *port);
/* Call every poll. status may be 0 until the controller has published one.
 * go is the GP20 press kind; stop_held/go_held are the debounced levels.
 * While GP21 is held: STOP is sent once per hold (retried, with one message,
 * if the controller refuses it), any countdown/START/sampling is cancelled
 * and GP20 is ignored until BOTH buttons are released; then a fresh GP20
 * press is needed. */
void operator_step(operator_t *op, uint32_t now_ms, op_press_t go,
                   bool stop_held, bool go_held,
                   const s3_integration_status_t *status);

const char *op_drive_state_name(s3_drive_state_t state);
const char *op_drive_reason_name(s3_drive_reason_t reason);
#endif

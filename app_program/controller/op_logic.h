/** @file op_logic.h
 * @brief Operator controls for the S3_MOTION profile: a calibration
 * walkthrough, START with a countdown, STOP and fault RESET, driven by
 * two buttons.
 *
 * Pure logic, no RTOS or GPIO: the caller passes time, button presses and the
 * latest s3_integration status, and this module only ever calls the port's
 * p_submit() (s3_integration_submit() in firmware). It never
 * requests motion directly and owns no navigation state, so the single
 * controller owner (s3_integration/s3_drive) is unchanged.
 */
#ifndef OPERATOR_LOGIC_H
#define OPERATOR_LOGIC_H
#include "../build_assert.h"
#include <stdbool.h>
#include <stdint.h>
#include "s3_integration.h"

#define OP_DEBOUNCE_MS (30U)
#define OP_RELEASE_MS (100U) /* Wait for both buttons to stay released. */
#define OP_LONG_PRESS_MS (2000U)
#define OP_COUNTDOWN_MS (3000U)
#define OP_REPLY_TIMEOUT_MS (1000U)
#define OP_COLLECT_TIMEOUT_MS (3000U)
#define OP_PROMPT_REPEAT_MS (5000U)
#define OP_CAL_STEPS (2U * (unsigned)S3_SENSOR_COUNT)
/* Compile-time guarantee: op_logic_t.step is 3 bits. */
BUILD_ASSERT(OP_CAL_STEPS <= 7U, op_logic_h_check_a_t);

typedef enum
{
    OP_PRESS_NONE,
    OP_PRESS_SHORT,
    OP_PRESS_LONG
} op_logic_press_t;

/* One debounced button. Short/long is reported on release; the "pressed"
 * edge is reported as soon as the press is stable (used for STOP). */
typedef struct
{
    uint32_t since_ms;
    uint32_t pressed_ms;
    bool     b_has_input_level : 1;
    bool     b_is_raw_pressed : 1;
    bool     b_is_stable_pressed : 1;
    bool     b_is_press_armed : 1;
} op_logic_button_t;

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
} op_logic_phase_t;

typedef struct
{
    bool ( * p_submit)(const s3_integration_request_t * p_request);
    int  ( * p_print)(const unsigned char * p_format, ...);
} op_logic_port_t;

typedef struct
{
    op_logic_port_t  port;
    op_logic_phase_t phase;
    uint32_t         phase_ms;
    uint32_t         prompt_ms;
    uint32_t         sequence;
    unsigned         step : 3;    /* Calibration steps range from 0 to 6. */
    unsigned countdown_shown : 2; /* Track the last second announced (0..3). */
    bool     b_is_awaiting_reply : 1;
    bool     b_has_prompted : 1;
    bool     b_has_handled_stop : 1; /* This hold already handled STOP. */
    bool     b_has_warned_stop : 1; /* Limit the "STOP not accepted" warning. */
    bool b_is_go_inhibited : 1; /* Ignore GP20 until both buttons are released.
                                 */
    bool b_has_warned_ignore : 1;   /* Limit the "Ignored GP20" warning. */
    bool b_should_reset_to_cal : 1; /* RESET came from a calibration prompt. */
} op_logic_t;

/**
 * @brief Clear debounce history and require an initial stable release.
 * @param[out] p_button Caller-owned button state; NULL is a no-op.
 * @note Serialize initialization, updates and reads of each button. This
 * module does not acquire locks or retain the supplied pointer.
 */
void op_logic_button_init (op_logic_button_t * p_button);
/* Feed the raw level every poll. Returns the press kind on release. *edge is
 * set true once per press, when it becomes stable. */
/**
 * @brief Advance one button's debounce state using caller-supplied time.
 * @param[in,out] p_button Initialized, exclusively accessed state; may be NULL.
 * @param b_is_down True for a physically pressed button; polarity is
 * caller-owned.
 * @param now_ms Wrapping millisecond timestamp, normally supplied every poll.
 * @param[out] pb_has_press_edge Optional separate output, cleared first even
 * if p_button is NULL. Set only for an armed, newly debounced press.
 * @return NONE until a release reports SHORT or LONG. Press duration uses raw
 * transition times; debounce, initial release and long-press limits are the
 * OP_* constants above. A held startup button must release before arming.
 * @note No GPIO, clock, callback or allocation is used. The held-level query
 * can still report a startup hold before press events have been armed.
 */
op_logic_press_t op_logic_button_update (op_logic_button_t * p_button,
                                         bool b_is_down, uint32_t now_ms,
                                         bool * pb_has_press_edge);
/* Debounced level, true while held. Unlike the press events this is reported
 * even for a button held since power-up, so a held STOP is always seen. */
/**
 * @brief Read the debounced level independently of press-event arming.
 * @param[in] p_button Borrowed stable state; NULL returns false.
 * @return True while the stored stable level is pressed; no state is changed.
 */
bool op_logic_button_held (const op_logic_button_t * p_button);

/**
 * @brief Initialize operator state and copy the synchronous port callbacks.
 * @param[out] p_op Caller-owned state; NULL returns false.
 * @param[in] p_port Port with non-NULL p_submit and p_print callbacks. The
 * source is read after p_op is cleared; overlapping storage is not protected.
 * @return False for a missing pointer/callback without modifying p_op;
 * otherwise true with phase WAIT_STATUS. No callback is invoked here.
 * @note Serialize all operations on p_op. Callbacks must remain callable and
 * must not reenter this state. p_submit must consume/copy its borrowed request
 * during the call; acceptance is not execution. p_print results are ignored.
 */
bool op_logic_init (op_logic_t * p_op, const op_logic_port_t * p_port);
/* A remote STOP was already queued by the controller. Cancel a GP20 countdown
 * without submitting a duplicate STOP or acknowledging a different command. */
/**
 * @brief Inhibit local GO after a remote STOP has already been queued.
 * @param[in,out] p_op Exclusively accessed initialized state; NULL is a no-op.
 * @param now_ms Wrapping millisecond time for any resulting phase transition.
 * @note Cancels applicable calibration/start/reset waits, possibly printing
 * the existing cancellation message. Does not submit or acknowledge a command.
 * Both buttons must release before a fresh GO event can be accepted.
 */
void op_logic_remote_stop (op_logic_t * p_op, uint32_t now_ms);
/* Call every poll. status may be 0 until the controller has published one.
 * go is the GP20 press kind; the STOP and GO inputs are debounced levels.
 * While GP21 is held: STOP is sent once per hold (retried, with one message,
 * if the controller refuses it), any countdown/START/sampling is cancelled
 * and GP20 is ignored until BOTH buttons are released; then a fresh GP20
 * press is needed. */
/**
 * @brief Process STOP/inhibition first, then one operator-phase update.
 * @param[in,out] p_op Exclusively accessed initialized state; NULL is a no-op.
 * @param now_ms Wrapping millisecond timestamp for deadlines and prompts.
 * @param value_go Debounced GO event. Unknown values retain the phase-specific
 * comparisons; no general enum validation is added by this interface.
 * @param b_is_stop_held Debounced STOP level, including a hold since startup.
 * @param b_is_go_held Debounced GO level used to require both buttons released.
 * @param[in] p_status Borrowed coherent controller snapshot, or NULL when none
 * is available. STOP/inhibition still run without a snapshot.
 * @note May call both callbacks synchronously; neither callback directly drives
 * a motor here. A rejected held STOP is retried each poll with bounded warning
 * output. Reply detection uses changed command_sequence plus command_ok.
 * Timed-out requests return to the existing retry phase; they are not assumed
 * executed. No pointers to requests or status survive the call.
 */
void op_logic_step (op_logic_t * p_op, uint32_t now_ms,
                    op_logic_press_t value_go, bool b_is_stop_held,
                    bool                            b_is_go_held,
                    const s3_integration_status_t * p_status);

/**
 * @brief Return the fixed console label for a drive state.
 * @param state State code, including unrecognized native enum values.
 * @return Borrowed immutable string of static lifetime; "?" if unrecognized.
 */
const char * op_logic_drive_state_name (s3_drive_state_t state);
/**
 * @brief Return the fixed console explanation for a drive reason.
 * @param reason Reason code, including unrecognized native enum values.
 * @return Borrowed immutable string of static lifetime; "?" if unrecognized.
 */
const char * op_logic_drive_reason_name (s3_drive_reason_t reason);
#endif

/* End of file. */


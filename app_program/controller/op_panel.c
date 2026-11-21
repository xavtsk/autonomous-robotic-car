/** @file op_panel.c
 * @brief Poll the GP20 and GP21 buttons every 10 ms and submit operator
 * requests through the integration controller.
 */
#include "op_panel.h"
#if APP_S3_MOTION
#include "op_logic.h"
#include "remote_control.h"
#include <sys/sysdef.h>

#define OP_GO_PIN (20U)
#define OP_STOP_PIN (21U)
#define OP_POLL_MS (10U)

static ID gh_panel_task;

static bool now_ms (uint32_t * p_out);
static bool button_pressed (unsigned pin);
static void panel_task (INT code, void * p_unused);

/** @brief Configure both button inputs and start their polling task once.
 * Return E_IO on pin setup failure or the task creation error on failure.
 */
ER op_panel_start (void)
{
    if (gh_panel_task > 0)
    {
        return E_OBJ;
    }

    if ((gpio_set_pin(OP_GO_PIN, GPIO_MODE_IN) != E_OK) ||
        (gpio_set_pin(OP_STOP_PIN, GPIO_MODE_IN) != E_OK))
    {
        return E_IO;
    }

    clr_w(GPIO(OP_GO_PIN), GPIO_PDE);
    clr_w(GPIO(OP_STOP_PIN), GPIO_PDE);
    set_w(GPIO(OP_GO_PIN), GPIO_PUE);
    set_w(GPIO(OP_STOP_PIN), GPIO_PUE);
    /* The panel entry has the kernel INT start code and void * context.
     * app_start_task stores it using the kernel's FP field. */
    gh_panel_task =
        app_start_task((FP)panel_task, OP_PRI_PANEL, TP_PRC1, "operator");
    return (gh_panel_task < E_OK) ? gh_panel_task : E_OK;
}

/** @brief Write the low 32 bits of the kernel time in milliseconds.
 * A failed clock read leaves the caller's timestamp untouched.
 */
static bool now_ms (uint32_t * p_out)
{
    SYSTIM time;

    if (tk_get_otm(&time) != E_OK)
    {
        return false;
    }

    *p_out = time.lo;
    return true;
}

/** @brief Return true when the selected active-low button pin reads low. */
static bool button_pressed (unsigned pin)
{
    return gpio_get_val(pin) ==
           0U; /* Use active-low inputs with internal pull-ups. */
}

/** @brief Own the debouncers and apply physical and remote STOP precedence.
 * The task skips button processing when its millisecond clock is unavailable.
 */
static void panel_task (INT code, void * p_unused)
{
    static op_logic_t              value_op;
    static op_logic_button_t       go_button;
    static op_logic_button_t       stop_button;
    static s3_integration_status_t status;
    const op_logic_port_t          port = {s3_integration_submit, tm_printf};
    uint32_t                       remote_stop_seen = 0;
    (void)code;
    (void)p_unused;
    op_logic_button_init(&go_button);
    op_logic_button_init(&stop_button);

    if (!op_logic_init(&value_op, &port))
    {
        tm_printf((UB *)"[OP] init failed: buttons disabled\n");
        tk_ext_tsk();
    }

    uint32_t now;
    for (;;)
    {
        bool             b_has_stop_edge = false;
        bool             b_has_go_edge   = false;
        op_logic_press_t value_go;

        if (now_ms(&now))
        {
            (void)op_logic_button_update(&stop_button,
                                         button_pressed(OP_STOP_PIN), now,
                                         &b_has_stop_edge);
            value_go = op_logic_button_update(
                &go_button, button_pressed(OP_GO_PIN), now, &b_has_go_edge);
            remote_control_physical_stop(op_logic_button_held(&stop_button));
            const uint32_t remote_stop = remote_control_stop_sequence();

            if (remote_stop != remote_stop_seen)
            {
                remote_stop_seen = remote_stop;
                op_logic_remote_stop(&value_op, now);
            }

            op_logic_step(&value_op, now, value_go,
                          op_logic_button_held(&stop_button),
                          op_logic_button_held(&go_button),
                          s3_integration_status(&status) ? &status : 0);
        }

        (void)tk_dly_tsk(OP_POLL_MS);
    }
}
#else
/** @brief Return E_NOSPT when the integrated motion profile is disabled. */
ER op_panel_start (void)
{
    return E_NOSPT;
}
#endif

/* End of file. */


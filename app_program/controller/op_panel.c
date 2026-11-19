/** @file op_panel.c
 * @brief Thin RTOS/GPIO shell around operator_logic.
 */
/* Thin RTOS/GPIO shell around operator_logic. Polls the two Robo Pico user
 * buttons every 10 ms and only talks to the controller through
 * s3_integration_submit()/s3_integration_status(). No motion calls. */
#include "op_panel.h"
#if APP_S3_MOTION
#include "op_logic.h"
#include "remote_control.h"
#include <sys/sysdef.h>

#define OP_GO_PIN (20U)
#define OP_STOP_PIN (21U)
#define OP_POLL_MS (10U)

static ID g_panel;

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

static bool down (unsigned pin)
{
    return gpio_get_val(pin) == 0U; /* active low, internal pull-up */
}

static void panel (INT code, void * p_unused)
{
    static operator_t value_op;
    static op_button_t go_button, stop_button;
    static s3_integration_status_t status;
    const op_port_t port = {s3_integration_submit, tm_printf};
    uint32_t now;
    uint32_t remote_stop_seen = 0;
    (void)code;
    (void)p_unused;
    op_button_init(&go_button);
    op_button_init(&stop_button);
    if (!operator_init(&value_op, &port))
    {
        tm_printf((UB *)"[OP] init failed: buttons disabled\n");
        tk_ext_tsk();
    }
    for (;;)
    {
        bool stop_edge = false, go_edge = false;
        op_press_t value_go;
        if (now_ms(&now))
        {
            (void)op_button_update(&stop_button, down(OP_STOP_PIN), now,
                                   &stop_edge);
            value_go =
                op_button_update(&go_button, down(OP_GO_PIN), now, &go_edge);
            s3_remote_physical_stop(op_button_held(&stop_button));
            const uint32_t remote_stop = s3_remote_stop_sequence();
            if (remote_stop != remote_stop_seen)
            {
                remote_stop_seen = remote_stop;
                operator_remote_stop(&value_op, now);
            }
            operator_step(&value_op, now, value_go,
                          op_button_held(&stop_button),
                          op_button_held(&go_button),
                          s3_integration_status(&status) ? &status : 0);
        }
        (void)tk_dly_tsk(OP_POLL_MS);
    }
}

ER operator_panel_start (void)
{
    if (g_panel > 0)
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
    g_panel = app_start_task((FP)panel, OP_PRI_PANEL, TP_PRC1, "operator");
    return (g_panel < E_OK) ? (ER)g_panel : E_OK;
}
#else
ER operator_panel_start (void)
{
    return E_NOSPT;
}
#endif

/* End of file. */


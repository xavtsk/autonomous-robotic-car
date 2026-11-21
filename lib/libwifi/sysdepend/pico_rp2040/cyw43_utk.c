/** @file cyw43_utk.c
 * @brief Poll the CYW43439 on the configured micro T-Kernel owner task.
 * Retry action codes fit in the unsigned comparison domain.
 */

#include "cyw43.h"
#include "cyw43_country.h"
#include "cyw43_utk.h"

#if TM_NET_LWIP
/* Initialize lwIP before cyw43_wifi_set_up(). The driver callback
 * cyw43_cb_tcpip_init(), in the Pico SDK's lib/cyw43-driver/src/cyw43_lwip.c,
 * calls netif_add() during that setup. For NO_SYS=1 the single radio owner
 * polls both CYW43 and lwIP timeouts. See
 * lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c for the MQTT callbacks on this
 * owner.
 */
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "wifi_retry_policy.h"
#include "wifi_secrets.h"
#if TM_MQTT
#include "mqtt_utk.h"
#endif
#endif

#include <string.h>

/* tm/tmonitor.h only pulls in tk/typedef.h (plain B/H/W/D-style typedefs),
 * never tk/tkernel.h/tk/syslib.h -- so it is safe in this pico-sdk/cyw43
 * namespace file, unlike tk/tkernel.h itself (see cc.h's comment on why
 * that one must stay out of here). Needed so join/link failures are
 * actually visible on the console instead of only sitting in g_radio_status.
 */
#include <tm/tmonitor.h>

#define CYW43_POLL_MS 2
#define CYW43_SSID_MAX_BYTES 32
#define CYW43_SCAN_MAX_POLLS 1000
#define CYW43_SCAN_POLL_MS 10
#define CYW43_SCAN_SETTLE_POLLS 30

/* Overridable from the makefile if an AP needs something else. MIXED accepts
 * both TKIP and AES; the driver treats it the same as WPA2_AES_PSK for a
 * pure-AES AP, so it is a strictly wider match than AES-only. */
#ifndef WIFI_AUTH_MODE
#define WIFI_AUTH_MODE CYW43_AUTH_WPA2_MIXED_PSK
#endif

/* Off by default: it cost ~10s of every boot, flooded the console right before
 * the join, and leaves the radio channel-hopping immediately before the
 * association it is about to attempt. Build with -DWIFI_DIAGNOSTIC_SCAN=1 when
 * a network is not being found and you need to see what the chip can hear. */
#ifndef WIFI_DIAGNOSTIC_SCAN
#define WIFI_DIAGNOSTIC_SCAN 0
#endif

extern volatile uint32_t cyw43_utk_poll_requested;
extern int32_t           tk_dly_tsk (uint32_t delay);
extern uint64_t          time_us_64 (void);

#if CNF_SMP
extern int32_t tk_get_prc (void);
extern void    mp_memory_barrir (void);
#define CYW43_UTK_OWNER_PRC() tk_get_prc()
#define CYW43_UTK_MEMORY_BARRIER() mp_memory_barrir()
#else
/* Neither primitive is built in a single-core kernel -- both live behind
 * TK_SUPPORT_SMP. There is only ever processor 1, and no second core for the
 * publish to become visible to; the barrier still has to order the radio state
 * this task hands to DMA, so it stays as a real one rather than a no-op.
 * CNF_SMP is used here for the same header-avoidance reason as in
 * cyw43_utk_compat.c. */
#define CYW43_UTK_OWNER_PRC() 1
#define CYW43_UTK_MEMORY_BARRIER() __asm__ volatile("dmb" ::: "memory")
#endif

static T_CYW43_UTK_STATUS g_radio_status = {
    .init_result    = -5,
    .strongest_rssi = -32768,
};

#if TM_NET_LWIP
static const char * cyw43_utk_link_name (int32_t status);
static int32_t      cyw43_utk_try_join (void);
#endif
#if TM_NET_LWIP && WIFI_DIAGNOSTIC_SCAN
static int  cyw43_utk_log_scan (void *                         p_env,
                                const cyw43_ev_scan_result_t * p_result);
static void cyw43_utk_scan_diagnostics (void);
#endif
#if !TM_NET_LWIP
static int cyw43_utk_save_scan (void *                         p_env,
                                const cyw43_ev_scan_result_t * p_result);
#endif

/** @brief Own radio initialization, polling and optional network services.
 * @param start_code Ignored kernel task start code.
 * @param p_exinf Ignored borrowed kernel context; may be NULL, never retained.
 * @note Does not return. Runs on the configured radio-owner processor and is
 * the only writer of the shared status. Publishes initialization after the
 * existing barrier; status readers can still observe mixed-age fields.
 * Interface initialization failure parks with 1000 ms delays. Otherwise the
 * existing loop polls with 2 ms task delays; lwIP builds retain their retry
 * policy, and scan-only builds retain scan completion reporting. Polling,
 * interface setup and join/scan calls remain under the CYW43 lock; sleeps,
 * lwIP timeouts and application/MQTT callbacks remain outside it. No timeout
 * or scheduling guarantee is added.
 */
void cyw43_utk_task (int32_t start_code, void * p_exinf)
{
#if TM_NET_LWIP
    bool                b_is_interface_up = false;
    wifi_retry_policy_t retry;
#endif
#if TM_MQTT
    int app_link_reported = -1;
#endif
#if !TM_NET_LWIP
    cyw43_wifi_scan_options_t options = {0};
    bool                      b_was_scanning;
#endif
    int32_t result;

    (void)start_code;
    (void)p_exinf;
    /* In this running task the kernel reports a positive processor ID.
     * The single-core fallback is 1. Both fit the shared unsigned field
     * and the signed format argument below. */
    g_radio_status.owner_prc = (uint32_t)CYW43_UTK_OWNER_PRC();

    tm_printf((UB *)"wifi: cyw43_utk_task starting (proc %d)\n",
              (int)g_radio_status.owner_prc);

#if TM_NET_LWIP
    lwip_init();
#endif

    tm_printf((UB *)"wifi: cyw43_init() ...\n");
    cyw43_init(&cyw43_state);
    tm_printf((UB *)"wifi: cyw43_init() returned, bringing interface up ...\n");
    cyw43_thread_enter();
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true,
                      CYW43_COUNTRY_WORLDWIDE);

    if ((cyw43_state.itf_state & (1u << CYW43_ITF_STA)) == 0U)
    {
        /* cyw43_wifi_set_up() has a void API and otherwise hides its bus-init
         * error.  Report the underlying I/O class instead of letting scan()
         * replace it with the secondary "interface not up" (-4) result. */
        tm_printf((UB *)"wifi: FAILED -- interface never came up (SPI bus / "
                        "firmware load to the CYW43439 did not succeed)\n");
        result = -CYW43_EIO;
    }
    else
    {
#if TM_NET_LWIP
        b_is_interface_up = true;
#endif
        (void)cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA,
                                 g_radio_status.mac);
#if TM_NET_LWIP
        /* Join instead of scan: with lwIP wired in, the point of coming up
         * is a working network, not a diagnostic scan. cyw43_cb_tcpip_init()
         * already ran (from inside cyw43_wifi_set_up() above) and started
         * DHCP; cyw43_wifi_join() below is what actually gets a beacon to
         * respond to it. */
#if WIFI_DIAGNOSTIC_SCAN
        cyw43_utk_scan_diagnostics();
#endif

        /* Only the *request* outcome. The AP's verdict shows up later as
         * g_radio_status.link_status, polled below, which is also where a
         * failed association gets retried. */
        result = cyw43_utk_try_join();
#else
        result = cyw43_wifi_scan(&cyw43_state, &options, &g_radio_status,
                                 cyw43_utk_save_scan);
#endif
    }

    cyw43_thread_exit();

    g_radio_status.init_result = result;
    g_radio_status.ready       = (0 == result);
#if TM_NET_LWIP
    g_radio_status.link_status = CYW43_LINK_DOWN;
#else
    g_radio_status.scan_active = (0 == result);
#endif
    CYW43_UTK_MEMORY_BARRIER();
    g_radio_status.init_complete = 1;

    /* A failed bus has nothing useful to poll.  More importantly, repeatedly
     * waking a failed high-rate service must not interfere with USB or the
     * rest of the kernel qualification. */
    if (result != 0
#if TM_NET_LWIP
        && !b_is_interface_up
#endif
    )
    {
        for (;;)
        {
#if TM_MQTT
            if ((app_link_reported != 0) && (mqtt_utk_app_link(0)))
            {
                app_link_reported = 0;
            }
#endif
            (void)tk_dly_tsk(1000);
        }
    }

#if TM_NET_LWIP
    int32_t prev_link_status = g_radio_status.link_status;
    wifi_retry_policy_init(&retry, time_us_64(), result);
#endif

    for (;;)
    {
#if TM_NET_LWIP
        cyw43_thread_enter();

        if (cyw43_poll != NULL)
        {
            cyw43_poll();
        }

        g_radio_status.poll_count++;
        g_radio_status.link_status =
            cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

        if (CYW43_LINK_UP == g_radio_status.link_status)
        {
            g_radio_status.ip_addr = ip4_addr_get_u32(
                netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]));
        }
        else
        {
            g_radio_status.ip_addr = 0;
        }

        if (g_radio_status.link_status != prev_link_status)
        {
            prev_link_status = g_radio_status.link_status;

            if (CYW43_LINK_UP == g_radio_status.link_status)
            {
                uint32_t ip_address = g_radio_status.ip_addr;
                /* Each masked octet is 0..255 and fits the %d argument. */
                tm_printf((UB *)"wifi: link up, ip=%d.%d.%d.%d\n",
                          (int)(ip_address & 0xffU),
                          (int)((ip_address >> 8) & 0xffU),
                          (int)((ip_address >> 16) & 0xffU),
                          (int)((ip_address >> 24) & 0xffU));
            }
            else
            {
                tm_printf((UB *)"wifi: link status -> %s (%d)\n",
                          cyw43_utk_link_name(g_radio_status.link_status),
                          (int)g_radio_status.link_status);
            }
        }

        {
            wifi_retry_policy_action_t action = wifi_retry_policy_observe(
                &retry, time_us_64(), g_radio_status.link_status);

            if ((unsigned int)WIFI_RETRY_LEAVE == action)
            {
                int32_t leave_result;
                tm_printf((UB *)"wifi: no IP within %u ms; disconnecting, "
                                "retry in %u ms\n",
                          (unsigned)(WIFI_CONNECT_TIMEOUT_US / 1000ULL),
                          (unsigned)(retry.delay_us / 1000U));
                /* Official driver disassociation, on the existing radio owner
                 * under its lock. Keep polling DHCP/radio during the backoff;
                 * never sleep for the timeout or restart the whole stack. */
                leave_result = cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA);

                if (leave_result != 0)
                {
                    tm_printf((UB *)"wifi: leave request rejected, err=%d; "
                                    "retry remains scheduled\n",
                              (int)leave_result);
                }
            }
            else if ((unsigned int)WIFI_RETRY_JOIN == action)
            {
                result = cyw43_utk_try_join();
                wifi_retry_policy_join_result(&retry, time_us_64(), result);
                g_radio_status.ready = (0 == result);
                /* Force the next iteration to treat whatever comes back as a
                 * change, so the new outcome is always reported. Must be a
                 * value cyw43_tcpip_link_status() can never return (it yields
                 * -3..3), or a genuine repeat failure would be swallowed. */
                prev_link_status = 0x7fffffff;
            }
            else
            {
                /* Keep the current wireless link action. */
            }
        }
#else
        b_was_scanning = g_radio_status.scan_active != 0;
        cyw43_thread_enter();
        if (cyw43_poll != NULL)
        {
            cyw43_poll();
        }
        g_radio_status.poll_count++;
        g_radio_status.scan_active = cyw43_wifi_scan_active(&cyw43_state);
        if (b_was_scanning && !g_radio_status.scan_active)
        {
            CYW43_UTK_MEMORY_BARRIER();
            g_radio_status.scan_complete = 1;
        }
#endif
        cyw43_utk_poll_requested = 0;
        cyw43_thread_exit();
#if TM_NET_LWIP
        sys_check_timeouts();
#if TM_MQTT
        /* Outside the radio lock: the application takes its own short mutex.
         * Retry after startup until its objects exist. */
        {
            int b_up = CYW43_LINK_UP == g_radio_status.link_status;

            if ((b_up != app_link_reported) && (mqtt_utk_app_link(b_up)))
            {
                app_link_reported = b_up;
            }
        }
        /* mqtt_utk_start() is idempotent (see its own connecting/is_connected
         * guard), so calling it on every poll iteration once the link is up
         * is fine -- it only actually does anything on the first call, or
         * again after connection_cb() schedules a retry. */
        if (CYW43_LINK_UP == g_radio_status.link_status)
        {
            mqtt_utk_start();
            mqtt_utk_poll();
        }
#endif
#endif
        (void)tk_dly_tsk(CYW43_POLL_MS);
    }
}

/** @brief Copy the current radio status using the existing memory barriers.
 * @param[out] p_status Caller-owned destination; NULL returns without changes.
 * @note No pointer is retained. There is no writer lock or version check, so
 * barriers order access but do not make this a coherent multi-field snapshot.
 * Preserves the imported public structure and its existing volatile accesses.
 */
void cyw43_utk_get_status (T_CYW43_UTK_STATUS * p_status)
{
    if (NULL == p_status)
    {
        return;
    }

    CYW43_UTK_MEMORY_BARRIER();
    *p_status = g_radio_status;
    CYW43_UTK_MEMORY_BARRIER();
}

#if TM_NET_LWIP
/** @brief Describe a driver's link code using a static diagnostic string.
 * @param status Signed CYW43_LINK_* code; unknown values are accepted.
 * @return Read-only, process-lifetime string, including an unknown fallback.
 * @note No state changes, allocation or locking; caller must not free it.
 */
static const char * cyw43_utk_link_name (int32_t status)
{
    switch (status)
    {
    case CYW43_LINK_DOWN:
        return "down";
    case CYW43_LINK_JOIN:
        return "joined AP, waiting for IP";
    case CYW43_LINK_NOIP:
        return "joined AP, no IP yet";
    case CYW43_LINK_UP:
        return "up";
    case CYW43_LINK_FAIL:
        return "FAILED (association failure)";
    case CYW43_LINK_NONET:
        return "FAILED: SSID not found (out of range, wrong name, or "
               "5GHz-only network -- this chip is 2.4GHz only)";
    case CYW43_LINK_BADAUTH:
        return "FAILED: authentication rejected (wrong password, or AP "
               "requires an auth mode this chip doesn't support, "
               "e.g. WPA3-only)";
    default:
        return "unknown";
    }
}
#endif

#if TM_NET_LWIP && WIFI_DIAGNOSTIC_SCAN
/* Diagnostic only: prints every beacon the chip actually hears, before the
 * join is attempted. This is what separates "the radio and driver are fine,
 * the AP just rejected us" from "the chip cannot even see that SSID" (wrong
 * name, out of range, or -- the common one for phone hotspots -- a 5GHz-only
 * network, which this 2.4GHz-only chip is physically unable to detect). */
/** @brief Log one diagnostic beacon and update the owner's scan statistics.
 * @param p_env Unused callback context; may be NULL and is not retained.
 * @param p_result Borrowed driver result; NULL returns without changes.
 * @return Zero, including for a NULL result, to continue the driver scan.
 * @note Runs during locked owner polling. Copies at most 32 SSID bytes for
 * display, replacing nonprintable bytes. Counts every result and keeps the
 * strongest raw RSSI; no pointer is retained or additional lock acquired.
 */
static int cyw43_utk_log_scan (void *                         p_env,
                               const cyw43_ev_scan_result_t * p_result)
{
    UB  ssid[CYW43_SSID_MAX_BYTES + 1];
    int value_i;

    (void)p_env;
    if (NULL == p_result)
    {
        return 0;
    }

    for (value_i = 0;
         (value_i < p_result->ssid_len) && (value_i < CYW43_SSID_MAX_BYTES);
         value_i++)
    {
        UB value_c    = p_result->ssid[value_i];
        ssid[value_i] = (value_c >= 0x20 && value_c < 0x7f) ? value_c : '?';
    }
    ssid[value_i] = '\0';

    g_radio_status.scan_count++;
    if (p_result->rssi > g_radio_status.strongest_rssi)
    {
        g_radio_status.strongest_rssi = p_result->rssi;
    }

    tm_printf((UB *)"wifi:   \"%s\"  rssi=%d  ch=%d  auth=%d\n", ssid,
              (int)p_result->rssi, (int)p_result->channel,
              (int)p_result->auth_mode);
    return 0;
}

/* Drives the poll loop by hand until the scan finishes -- the main poll loop
 * further down has not been entered yet at this point, and scan results only
 * arrive as async events processed by cyw43_poll(). */
/** @brief Run the optional startup scan before the connection attempt.
 * @pre The network owner holds the CYW43 thread lock on entry.
 * @note Returns with that lock held. Request failure logs and returns without
 * releasing it. An accepted request releases it between at most 1000 polling
 * iterations, each delayed by 10 ms unless completion permits an early exit
 * after iteration 30. Task scheduling can extend elapsed wall time.
 */
static void cyw43_utk_scan_diagnostics (void)
{
    cyw43_wifi_scan_options_t opts = {0};
    int32_t                   err;
    int                       value_i;

    tm_printf((UB *)"wifi: scanning for visible 2.4GHz networks ...\n");

    err = cyw43_wifi_scan(&cyw43_state, &opts, NULL, cyw43_utk_log_scan);
    if (err != 0)
    {
        tm_printf((UB *)"wifi: scan request failed, err=%d\n", (int)err);
        return;
    }
    cyw43_thread_exit();

    /* ~10s ceiling; a scan normally settles in 2-4s. */
    for (value_i = 0; value_i < CYW43_SCAN_MAX_POLLS; value_i++)
    {
        bool b_active;

        cyw43_thread_enter();
        if (cyw43_poll != NULL)
        {
            cyw43_poll();
        }
        b_active = cyw43_wifi_scan_active(&cyw43_state);
        cyw43_thread_exit();

        if ((!b_active) && (value_i > CYW43_SCAN_SETTLE_POLLS))
        {
            break;
        }
        (void)tk_dly_tsk(CYW43_SCAN_POLL_MS);
    }

    cyw43_thread_enter();
    tm_printf(
        (UB *)"wifi: scan done -- %d network(s) seen, strongest rssi=%d\n",
        (int)g_radio_status.scan_count, (int)g_radio_status.strongest_rssi);
}
#endif

#if TM_NET_LWIP
/* Caller must hold cyw43_thread_enter(). Returns the driver's acceptance of
 * the join *request*; the actual outcome arrives later as a link status.
 *
 * The BSSID argument is NULL, so the chip picks among all APs advertising this
 * SSID. Where two of them are in range (a mesh or a repeater), that choice can
 * change between the association and the handshake and the join fails. Pass a
 * six-byte BSSID here instead of NULL to pin it to one radio. */
/** @brief Submit one join request using the configured SSID/authentication.
 * @return Unchanged driver request result; zero means accepted, not connected.
 * @pre The network owner holds the CYW43 thread lock; it remains held.
 * @note Borrows static configuration strings and uses no BSSID restriction.
 * Logs the SSID and a rejected request's code, never the password. Link status
 * and retry scheduling are handled later by the owner task.
 */
static int32_t cyw43_utk_try_join (void)
{
    int32_t err;

    tm_printf((UB *)"wifi: joining \"%s\" ...\n", WIFI_SSID);

    err = cyw43_wifi_join(&cyw43_state, strlen(WIFI_SSID),
                          (const uint8_t *)WIFI_SSID, strlen(WIFI_PASSWORD),
                          (const uint8_t *)WIFI_PASSWORD, WIFI_AUTH_MODE, NULL,
                          CYW43_CHANNEL_NONE);

    if (err != 0)
    {
        tm_printf((UB *)"wifi: cyw43_wifi_join() request rejected, err=%d\n",
                  (int)err);
    }

    return err;
}
#endif

#if !TM_NET_LWIP
/** @brief Accumulate one scan-only beacon without printing or retaining it.
 * @param p_env Caller-owned T_CYW43_UTK_STATUS; must be non-NULL for a result.
 * @param p_result Borrowed driver result; NULL leaves all state unchanged.
 * @return Zero so scanning continues.
 * @note Invoked by the locked owner poll. Increments the existing unsigned
 * scan counter and preserves the strongest raw RSSI; no new locking occurs.
 */
static int cyw43_utk_save_scan (void *                         p_env,
                                const cyw43_ev_scan_result_t * p_result)
{
    T_CYW43_UTK_STATUS * p_status = p_env;

    if (NULL == p_result)
    {
        return 0;
    }
    p_status->scan_count++;
    if (p_result->rssi > p_status->strongest_rssi)
    {
        p_status->strongest_rssi = p_result->rssi;
    }
    return 0;
}
#endif

/* End of file. */


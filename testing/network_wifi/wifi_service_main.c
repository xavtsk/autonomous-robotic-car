/** @file wifi_service_main.c
 * @brief Runs the actual production cyw43_utk_task() with fake hardware/clock
 * APIs.
 *
 * Deterministic virtual time: no radio access, real sleeps or credentials.
 */
#include "wifi_service_main.h"
#include "cyw43_utk.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

cyw43_t           cyw43_state;
volatile uint32_t cyw43_utk_poll_requested;
static uint64_t   g_clock_us;
static uint64_t   g_stop_us;
static unsigned   g_scenario;
static unsigned   g_joins;
static unsigned   g_leaves;
static unsigned   g_polls;
static unsigned   g_timeouts;
static unsigned   g_lock_depth;
static unsigned   g_app_up;
static uint64_t   g_joined_at[32];
static uint64_t   g_left_at[32];
static bool       gb_should_fail_interface;
static jmp_buf    g_stopped;

static void poll_driver (void);

void ( * cyw43_poll)(void) = poll_driver;

static void run (unsigned which, unsigned seconds);

/** @brief Clear the entire simulated radio state.
 */
void cyw43_init (cyw43_t * p_state)
{
    memset(p_state, 0, sizeof(*p_state));
}

/** @brief Acquire the simulated radio lock and fail the assertion on nested
 * entry.
 */
void cyw43_thread_enter (void)
{
    assert(g_lock_depth++ == 0U);
}

/** @brief Release the simulated radio lock and fail the assertion if it was
 * not held.
 */
void cyw43_thread_exit (void)
{
    assert(g_lock_depth-- == 1U);
}

/** @brief Record station-interface startup under the radio lock, including
 * the injected interface failure.
 */
void cyw43_wifi_set_up (cyw43_t * p_state, int itf, bool b_is_up,
                        unsigned country)
{
    assert(1U == g_lock_depth && 0 == itf && b_is_up);
    (void)country;
    p_state->itf_state = gb_should_fail_interface ? 0 : 1;
}

/** @brief Write six zero bytes as the fake interface's MAC address while the
 * radio is locked.
 */
int cyw43_wifi_get_mac (cyw43_t * p_state, int itf, uint8_t mac[6])
{
    (void)p_state;
    (void)itf;
    assert(1U == g_lock_depth);
    memset(mac, 0, 6);
    return 0;
}

/** @brief Record the join time in microseconds under the radio lock and
 * return the selected scenario's join result.
 */
int cyw43_wifi_join (cyw43_t * p_state, size_t ssid_length,
                     const uint8_t * p_ssid, size_t key_length,
                     const uint8_t * p_key, uint32_t auth,
                     const uint8_t * p_bssid, uint32_t channel)
{
    (void)p_state;
    (void)ssid_length;
    (void)p_ssid;
    (void)key_length;
    (void)p_key;
    (void)auth;
    (void)p_bssid;
    (void)channel;
    assert(1U == g_lock_depth && g_joins < 32U);
    g_joined_at[g_joins++] = g_clock_us;
    return ((2U == g_scenario) && (1U == g_joins)) ? -5 : 0;
}

/** @brief Record the leave time in microseconds and inject a driver error in
 * scenario six.
 */
int cyw43_wifi_leave (cyw43_t * p_state, int itf)
{
    (void)p_state;
    assert(1U == g_lock_depth && 0 == itf && g_leaves < 32U);
    g_left_at[g_leaves++] = g_clock_us;
    return 6U == g_scenario ? -5 : 0;
}

/** @brief Set a fixture IPv4 address and return the link status selected by
 * the scenario and simulated time.
 */
int cyw43_tcpip_link_status (cyw43_t * p_state, int itf)
{
    (void)itf;
    assert(1U == g_lock_depth);
    p_state->netif[0].address.addr = 0x01020304;

    switch (g_scenario)
    {
    case 0:
        return g_clock_us >= 1000000U ? CYW43_LINK_UP : CYW43_LINK_JOIN;
    case 1:
        return g_clock_us >= 10000000U ? CYW43_LINK_NOIP : CYW43_LINK_JOIN;
    case 2:
        return g_joins >= 2U ? CYW43_LINK_UP : CYW43_LINK_DOWN;
    case 3:
        if (g_clock_us < 1000000U)
        {
            return CYW43_LINK_UP;
        }

        if (g_clock_us < 7000000U)
        {
            return CYW43_LINK_DOWN;
        }

        if (g_clock_us < 8000000U)
        {
            return CYW43_LINK_UP;
        }

        return CYW43_LINK_DOWN;
    case 4:
        return CYW43_LINK_FAIL;
    case 5:
        return g_clock_us < 23000000U ? CYW43_LINK_NOIP : CYW43_LINK_UP;
    case 6:
        return CYW43_LINK_NOIP;
    default:
        assert(false);
        return CYW43_LINK_DOWN;
    }
}

/** @brief Check that lwIP initialization occurs outside the radio lock.
 */
void lwip_init (void)
{
    assert(0U == g_lock_depth);
}

/** @brief Count a timeout-service poll and require the radio lock to be
 * free.
 */
void sys_check_timeouts (void)
{
    assert(0U == g_lock_depth);
    ++g_timeouts;
}

/** @brief Update the fixture application-link state outside the radio lock
 * and acknowledge the notification.
 */
int mqtt_utk_app_link (int b_is_up)
{
    assert(0U == g_lock_depth);
    g_app_up = (unsigned)b_is_up;
    return 1;
}

/** @brief Require an active application link and an unlocked radio when MQTT
 * starts.
 */
void mqtt_utk_start (void)
{
    assert(0U == g_lock_depth && g_app_up);
}

/** @brief Require an active application link and an unlocked radio whenever
 * MQTT is polled.
 */
void mqtt_utk_poll (void)
{
    assert(0U == g_lock_depth && g_app_up);
}

/** @brief Discard the formatted console output and return zero.
 */
int tm_printf (const unsigned char * p_format, ...)
{
    (void)p_format;
    return 0;
}

/** @brief Report processor one to the simulated kernel caller.
 */
int32_t tk_get_prc (void)
{
    return 1;
}

/** @brief Provide the kernel barrier symbol for the single-threaded Wi-Fi
 * fixture.
 */
void mp_memory_barrir (void)
{
}

/** @brief Read the simulated hardware clock in microseconds.
 */
uint64_t time_us_64 (void)
{
    return g_clock_us;
}

/** @brief Check the polling or retry delay outside the network lock and
 * advance the simulated clock to the test cutoff.
 */
int32_t tk_dly_tsk (uint32_t delay_ms)
{
    assert(0U == g_lock_depth);
    /* No timeout/backoff blocking sleeps: normal service remains at 2 ms. */
    assert(gb_should_fail_interface ? 1000U == delay_ms : 2U == delay_ms);
    g_clock_us += (uint64_t)delay_ms * 1000U;

    if (g_clock_us >= g_stop_us)
    {
        longjmp(g_stopped, 1);
    }

    return 0;
}

/** @brief Exercise Wi-Fi joins, disconnect recovery and polling on a
 * simulated clock with the radio lock checks enabled.
 */
int main (void)
{
    T_CYW43_UTK_STATUS status;
    run(0, 100);
    assert(1U == g_joins && 0U == g_leaves && g_app_up);
    run(1, 61);
    assert(3U == g_joins && 2U == g_leaves);
    assert(20000000U == g_left_at[0] && 25000000U == g_joined_at[1]);
    assert(45000000U == g_left_at[1] && 55000000U == g_joined_at[2]);
    run(2, 8);
    assert(2U == g_joins && 5000000U == g_joined_at[1] && 0U == g_leaves &&
           g_app_up);
    run(3, 15);
    assert(3U == g_joins && 6000000U == g_joined_at[1] &&
           13000000U == g_joined_at[2]);
    cyw43_utk_get_status(&status);
    assert(0U == status.ip_addr && !g_app_up);
    run(4, 90);
    assert(5U == g_joins && 5000000U == g_joined_at[1] &&
           15002000U == g_joined_at[2]);
    assert(35004000U == g_joined_at[3] && 65006000U == g_joined_at[4] &&
           0U == g_leaves);
    run(5, 60);
    assert(1U == g_joins && 1U == g_leaves && g_app_up);
    run(6, 30);
    assert(2U == g_joins && 1U == g_leaves && 25000000U == g_joined_at[1]);
    gb_should_fail_interface = true;
    run(0, 5);
    assert(0U == g_joins && 0U == g_leaves && 0U == g_polls);
    puts("PASS: production Wi-Fi task: success, stuck JOIN/NOIP, rejected "
         "join/leave, DOWN recovery, backoff, late UP, bus failure, owner "
         "lock and nonblocking polling");
    return 0;
}

/** @brief Count a driver poll and require exactly one level of radio
 * locking.
 */
static void poll_driver (void)
{
    assert(1U == g_lock_depth);
    ++g_polls;
}

/** @brief Reset the simulated radio, run the selected recovery scenario for
 * the given seconds and check lock and poll accounting.
 */
static void run (unsigned which, unsigned seconds)
{
    /* clang-format off */
    g_scenario = which;
    g_joins    = g_leaves = g_polls = g_timeouts = g_lock_depth = g_app_up = 0;
    g_clock_us = 0;
    g_stop_us  = (uint64_t)seconds * 1000000U;
    /* clang-format on */
    memset(g_joined_at, 0, sizeof(g_joined_at));
    memset(g_left_at, 0, sizeof(g_left_at));

    if (setjmp(g_stopped) == 0)
    {
        cyw43_utk_task(0, NULL);
    }

    assert(0U == g_lock_depth);

    if (!gb_should_fail_interface)
    {
        assert(g_polls == g_timeouts && g_polls > 0U);
    }
}

/* End of file. */


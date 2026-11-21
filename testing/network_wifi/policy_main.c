/** @file policy_main.c
 * @brief Exercise Wi-Fi retry policy across radio link states.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "policy_main.h"
#include "cyw43.h"
#include "wifi_retry_policy.h"
#include <assert.h>
#include <stdio.h>

#define SATURATION_RETRY_COUNT 10
#define TIMER_WIDTH_CASE_COUNT 2

/** @brief Advance the Wi-Fi retry policy across join, backoff, recovery and
 * clock-wrap boundaries.
 */
int main (void)
{
    wifi_retry_policy_t retry_policy;
    const uint64_t      second = 1000000ULL;
    uint64_t            now;

    wifi_retry_policy_init(&retry_policy, 0, 0);
    assert(wifi_retry_policy_observe(&retry_policy, 0, CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 10U * second,
                                     CYW43_LINK_JOIN) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 19U * second,
                                     CYW43_LINK_NOIP) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 20U * second,
                                     CYW43_LINK_NOIP) ==
           (unsigned)WIFI_RETRY_LEAVE);
    assert(wifi_retry_policy_observe(&retry_policy, 24U * second,
                                     CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 25U * second,
                                     CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_JOIN);
    wifi_retry_policy_join_result(&retry_policy, 25U * second, -5);
    assert(wifi_retry_policy_observe(&retry_policy, 34U * second,
                                     CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 35U * second,
                                     CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_JOIN);
    wifi_retry_policy_join_result(&retry_policy, 35U * second, 0);
    assert(
        wifi_retry_policy_observe(&retry_policy, 36U * second, CYW43_LINK_UP) ==
        (unsigned)WIFI_RETRY_WAIT);
    assert((5U * second) == retry_policy.next_delay_us);
    assert(wifi_retry_policy_observe(&retry_policy, 40U * second,
                                     CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 45U * second,
                                     CYW43_LINK_DOWN) ==
           (unsigned)WIFI_RETRY_JOIN);

    /* Repeated terminal errors, even when the value changes, cannot keep
     * postponing an armed retry. Backoff caps and a late UP cancels it. */
    wifi_retry_policy_init(&retry_policy, 0, 0);
    assert(wifi_retry_policy_observe(&retry_policy, 0, CYW43_LINK_FAIL) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 4U * second,
                                     CYW43_LINK_BADAUTH) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 5U * second,
                                     CYW43_LINK_NONET) ==
           (unsigned)WIFI_RETRY_JOIN);
    now = 5U * second;

    for (unsigned retry_index = 0;
         retry_index < (unsigned)SATURATION_RETRY_COUNT; ++retry_index)
    {
        wifi_retry_policy_join_result(&retry_policy, now, -1);
        assert(retry_policy.delay_us <= 30U * second);
        now += retry_policy.delay_us;
        assert(wifi_retry_policy_observe(&retry_policy, now, CYW43_LINK_FAIL) ==
               (unsigned)WIFI_RETRY_JOIN);
    }

    assert((30U * second) == retry_policy.next_delay_us);
    wifi_retry_policy_join_result(&retry_policy, now, -1);
    assert(
        wifi_retry_policy_observe(&retry_policy, now + second, CYW43_LINK_UP) ==
        (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, now + 100U * second,
                                     CYW43_LINK_UP) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert((5U * second) == retry_policy.next_delay_us);

    /* Losing just the address starts a fresh bounded DHCP window, not an
     * immediate disconnect. Another JOIN/NOIP transition cannot extend it. */
    wifi_retry_policy_init(&retry_policy, 0, 0);
    (void)wifi_retry_policy_observe(&retry_policy, second, CYW43_LINK_UP);
    assert(wifi_retry_policy_observe(&retry_policy, 2U * second,
                                     CYW43_LINK_NOIP) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 21U * second,
                                     CYW43_LINK_JOIN) ==
           (unsigned)WIFI_RETRY_WAIT);
    assert(wifi_retry_policy_observe(&retry_policy, 22U * second,
                                     CYW43_LINK_NOIP) ==
           (unsigned)WIFI_RETRY_LEAVE);

    /* uint64_t elapsed arithmetic also works across the uint32_t wrap and
     * across unsigned uint64_t wrap (assuming normal monotonic intervals). */
    for (unsigned boundary_index = 0;
         boundary_index < (unsigned)TIMER_WIDTH_CASE_COUNT; ++boundary_index)
    {
        now = boundary_index ? UINT64_MAX - second : UINT32_MAX - second;
        wifi_retry_policy_init(&retry_policy, now, 0);
        assert(wifi_retry_policy_observe(&retry_policy, now + 20U * second - 1U,
                                         CYW43_LINK_JOIN) ==
               (unsigned)WIFI_RETRY_WAIT);
        assert(wifi_retry_policy_observe(&retry_policy, now + 20U * second,
                                         CYW43_LINK_NOIP) ==
               (unsigned)WIFI_RETRY_LEAVE);
        assert(wifi_retry_policy_observe(&retry_policy, now + 25U * second,
                                         CYW43_LINK_DOWN) ==
               (unsigned)WIFI_RETRY_JOIN);
    }

    puts("PASS: Wi-Fi policy: 20 s deadline, transition stability, "
         "failure/backoff cap, recovery/reset, 64-bit wrap");
    return 0;
}

/* End of file. */


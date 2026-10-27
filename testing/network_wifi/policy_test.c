#include "cyw43.h"
#include "wifi_retry_policy.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    wifi_retry_policy_t p;
    const uint64_t second = 1000000ULL;
    uint64_t now;

    wifi_retry_init(&p, 0, 0);
    assert(wifi_retry_observe(&p, 0, CYW43_LINK_DOWN) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 10 * second, CYW43_LINK_JOIN) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 19 * second, CYW43_LINK_NOIP) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 20 * second, CYW43_LINK_NOIP) == WIFI_RETRY_LEAVE);
    assert(wifi_retry_observe(&p, 24 * second, CYW43_LINK_DOWN) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 25 * second, CYW43_LINK_DOWN) == WIFI_RETRY_JOIN);
    wifi_retry_join_result(&p, 25 * second, -5);
    assert(wifi_retry_observe(&p, 34 * second, CYW43_LINK_DOWN) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 35 * second, CYW43_LINK_DOWN) == WIFI_RETRY_JOIN);
    wifi_retry_join_result(&p, 35 * second, 0);
    assert(wifi_retry_observe(&p, 36 * second, CYW43_LINK_UP) == WIFI_RETRY_WAIT);
    assert(p.next_delay_us == 5 * second);
    assert(wifi_retry_observe(&p, 40 * second, CYW43_LINK_DOWN) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 45 * second, CYW43_LINK_DOWN) == WIFI_RETRY_JOIN);

    /* Repeated terminal errors, even when the value changes, cannot keep
     * postponing an armed retry. Backoff caps and a late UP cancels it. */
    wifi_retry_init(&p, 0, 0);
    assert(wifi_retry_observe(&p, 0, CYW43_LINK_FAIL) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 4 * second, CYW43_LINK_BADAUTH) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 5 * second, CYW43_LINK_NONET) == WIFI_RETRY_JOIN);
    now = 5 * second;
    for(unsigned i = 0; i < 10; ++i) {
        wifi_retry_join_result(&p, now, -1);
        assert(p.delay_us <= 30 * second);
        now += p.delay_us;
        assert(wifi_retry_observe(&p, now, CYW43_LINK_FAIL) == WIFI_RETRY_JOIN);
    }
    assert(p.next_delay_us == 30 * second);
    wifi_retry_join_result(&p, now, -1);
    assert(wifi_retry_observe(&p, now + second, CYW43_LINK_UP) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, now + 100 * second, CYW43_LINK_UP) == WIFI_RETRY_WAIT);
    assert(p.next_delay_us == 5 * second);

    /* Losing just the address starts a fresh bounded DHCP window, not an
     * immediate disconnect. Another JOIN/NOIP transition cannot extend it. */
    wifi_retry_init(&p, 0, 0);
    (void)wifi_retry_observe(&p, second, CYW43_LINK_UP);
    assert(wifi_retry_observe(&p, 2 * second, CYW43_LINK_NOIP) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 21 * second, CYW43_LINK_JOIN) == WIFI_RETRY_WAIT);
    assert(wifi_retry_observe(&p, 22 * second, CYW43_LINK_NOIP) == WIFI_RETRY_LEAVE);

    /* uint64_t elapsed arithmetic also works across the uint32_t wrap and
     * across unsigned uint64_t wrap (assuming normal monotonic intervals). */
    for(unsigned i = 0; i < 2; ++i) {
        now = i ? UINT64_MAX - second : UINT32_MAX - second;
        wifi_retry_init(&p, now, 0);
        assert(wifi_retry_observe(&p, now + 20 * second - 1, CYW43_LINK_JOIN) == WIFI_RETRY_WAIT);
        assert(wifi_retry_observe(&p, now + 20 * second, CYW43_LINK_NOIP) == WIFI_RETRY_LEAVE);
        assert(wifi_retry_observe(&p, now + 25 * second, CYW43_LINK_DOWN) == WIFI_RETRY_JOIN);
    }
    puts("PASS: Wi-Fi policy: 20 s deadline, transition stability, failure/backoff cap, recovery/reset, 64-bit wrap");
    return 0;
}

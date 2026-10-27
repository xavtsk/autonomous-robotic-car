/* Runs the actual production cyw43_utk_task() with fake hardware/clock APIs.
 * Deterministic virtual time: no radio access, real sleeps or credentials. */
#include "cyw43.h"
#include "cyw43_utk.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

cyw43_t cyw43_state;
volatile uint32_t cyw43_utk_poll_requested;
static uint64_t clock_us, stop_us;
static unsigned scenario, joins, leaves, polls, timeouts, lock_depth, app_up;
static uint64_t joined_at[32], left_at[32];
static bool interface_failure;
static jmp_buf stopped;
static void poll_driver(void) { assert(lock_depth == 1); ++polls; }
void (*cyw43_poll)(void) = poll_driver;

void cyw43_init(cyw43_t *state) { memset(state, 0, sizeof(*state)); }
void cyw43_thread_enter(void) { assert(lock_depth++ == 0); }
void cyw43_thread_exit(void) { assert(lock_depth-- == 1); }
void cyw43_wifi_set_up(cyw43_t *state, int itf, bool up, unsigned country)
{ assert(lock_depth == 1 && itf == 0 && up); (void)country; state->itf_state = interface_failure ? 0 : 1; }
int cyw43_wifi_get_mac(cyw43_t *state, int itf, uint8_t mac[6])
{ (void)state; (void)itf; assert(lock_depth == 1); memset(mac, 0, 6); return 0; }
int cyw43_wifi_join(cyw43_t *state, size_t sn, const uint8_t *s,
                    size_t kn, const uint8_t *k, uint32_t auth,
                    const uint8_t *bssid, uint32_t channel)
{
    (void)state; (void)sn; (void)s; (void)kn; (void)k;
    (void)auth; (void)bssid; (void)channel;
    assert(lock_depth == 1 && joins < 32);
    joined_at[joins++] = clock_us;
    return (scenario == 2 && joins == 1) ? -5 : 0;
}
int cyw43_wifi_leave(cyw43_t *state, int itf)
{
    (void)state; assert(lock_depth == 1 && itf == 0 && leaves < 32);
    left_at[leaves++] = clock_us;
    return scenario == 6 ? -5 : 0;
}
int cyw43_tcpip_link_status(cyw43_t *state, int itf)
{
    (void)itf; assert(lock_depth == 1);
    state->netif[0].address.addr = 0x01020304;
    switch(scenario) {
    case 0: return clock_us >= 1000000 ? CYW43_LINK_UP : CYW43_LINK_JOIN;
    case 1: return clock_us >= 10000000 ? CYW43_LINK_NOIP : CYW43_LINK_JOIN;
    case 2: return joins >= 2 ? CYW43_LINK_UP : CYW43_LINK_DOWN;
    case 3:
        if(clock_us < 1000000) return CYW43_LINK_UP;
        if(clock_us < 7000000) return CYW43_LINK_DOWN;
        if(clock_us < 8000000) return CYW43_LINK_UP;
        return CYW43_LINK_DOWN;
    case 4: return CYW43_LINK_FAIL;
    case 5: return clock_us < 23000000 ? CYW43_LINK_NOIP : CYW43_LINK_UP;
    case 6: return CYW43_LINK_NOIP;
    default: assert(false); return CYW43_LINK_DOWN;
    }
}
void lwip_init(void) { assert(lock_depth == 0); }
void sys_check_timeouts(void) { assert(lock_depth == 0); ++timeouts; }
int mqtt_utk_app_link(int up) { assert(lock_depth == 0); app_up = (unsigned)up; return 1; }
void mqtt_utk_start(void) { assert(lock_depth == 0 && app_up); }
void mqtt_utk_poll(void) { assert(lock_depth == 0 && app_up); }
int tm_printf(const unsigned char *format, ...) { (void)format; return 0; }
int32_t tk_get_prc(void) { return 1; }
void mp_memory_barrir(void) {}
uint64_t time_us_64(void) { return clock_us; }
int32_t tk_dly_tsk(uint32_t delay_ms)
{
    assert(lock_depth == 0);
    /* No timeout/backoff blocking sleeps: normal service remains at 2 ms. */
    assert(interface_failure ? delay_ms == 1000 : delay_ms == 2);
    clock_us += (uint64_t)delay_ms * 1000;
    if(clock_us >= stop_us) longjmp(stopped, 1);
    return 0;
}
static void run(unsigned which, unsigned seconds)
{
    scenario = which; joins = leaves = polls = timeouts = lock_depth = app_up = 0;
    clock_us = 0; stop_us = (uint64_t)seconds * 1000000;
    memset(joined_at, 0, sizeof(joined_at)); memset(left_at, 0, sizeof(left_at));
    if(setjmp(stopped) == 0) cyw43_utk_task(0, NULL);
    assert(lock_depth == 0);
    if(!interface_failure) assert(polls == timeouts && polls > 0);
}
int main(void)
{
    T_CYW43_UTK_STATUS status;
    run(0, 100); assert(joins == 1 && leaves == 0 && app_up);
    run(1, 61);
    assert(joins == 3 && leaves == 2);
    assert(left_at[0] == 20000000 && joined_at[1] == 25000000);
    assert(left_at[1] == 45000000 && joined_at[2] == 55000000);
    run(2, 8); assert(joins == 2 && joined_at[1] == 5000000 && leaves == 0 && app_up);
    run(3, 15); assert(joins == 3 && joined_at[1] == 6000000 && joined_at[2] == 13000000);
    cyw43_utk_get_status(&status); assert(status.ip_addr == 0 && !app_up);
    run(4, 90);
    assert(joins == 5 && joined_at[1] == 5000000 && joined_at[2] == 15002000);
    assert(joined_at[3] == 35004000 && joined_at[4] == 65006000 && leaves == 0);
    run(5, 60); assert(joins == 1 && leaves == 1 && app_up);
    run(6, 30); assert(joins == 2 && leaves == 1 && joined_at[1] == 25000000);
    interface_failure = true; run(0, 5); assert(joins == 0 && leaves == 0 && polls == 0);
    puts("PASS: production Wi-Fi task: success, stuck JOIN/NOIP, rejected join/leave, DOWN recovery, backoff, late UP, bus failure, owner lock and nonblocking polling");
    return 0;
}

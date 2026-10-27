/* Single-owner Wi-Fi retry policy. The caller supplies CYW43_LINK_* and a
 * monotonic microsecond clock; no sleeps, allocation or driver calls here. */
#ifndef WIFI_RETRY_POLICY_H
#define WIFI_RETRY_POLICY_H

#include <stdint.h>

/* Total time allowed for a join attempt to obtain an IP address. A transition
 * from DOWN to JOIN to NOIP does not restart this deadline. Tunable, not a
 * demonstrated bound on hotspot/DHCP latency. */
#ifndef WIFI_CONNECT_TIMEOUT_US
#define WIFI_CONNECT_TIMEOUT_US (20ULL * 1000ULL * 1000ULL)
#endif
#ifndef WIFI_REJOIN_DELAY_US
#define WIFI_REJOIN_DELAY_US (5ULL * 1000ULL * 1000ULL)
#endif
#ifndef WIFI_REJOIN_MAX_DELAY_US
#define WIFI_REJOIN_MAX_DELAY_US (30ULL * 1000ULL * 1000ULL)
#endif
#if WIFI_CONNECT_TIMEOUT_US == 0 || WIFI_REJOIN_DELAY_US == 0 || \
    WIFI_REJOIN_MAX_DELAY_US < WIFI_REJOIN_DELAY_US || \
    WIFI_REJOIN_MAX_DELAY_US > UINT32_MAX
#error Invalid Wi-Fi retry timing configuration
#endif

typedef enum {
    WIFI_RETRY_WAIT,
    WIFI_RETRY_LEAVE,
    WIFI_RETRY_JOIN
} wifi_retry_action_t;

typedef struct {
    uint64_t since_us;
    uint32_t delay_us;
    uint32_t next_delay_us;
    unsigned phase : 2;
} wifi_retry_policy_t;

enum { WIFI_PHASE_ATTEMPT, WIFI_PHASE_BACKOFF, WIFI_PHASE_UP };

static inline void wifi_retry_schedule(wifi_retry_policy_t *p, uint64_t now)
{
    const uint64_t doubled = (uint64_t)p->next_delay_us * 2ULL;
    p->phase = WIFI_PHASE_BACKOFF;
    p->since_us = now;
    p->delay_us = p->next_delay_us;
    p->next_delay_us = (uint32_t)(doubled > WIFI_REJOIN_MAX_DELAY_US ?
                                WIFI_REJOIN_MAX_DELAY_US : doubled);
}

/* Record the join request result, including the initial request. An accepted
 * request is not yet connected. Failed requests also get bounded backoff. */
static inline void wifi_retry_join_result(wifi_retry_policy_t *p, uint64_t now,
                                         int32_t result)
{
    if(result != 0) {
        wifi_retry_schedule(p, now);
    } else {
        p->phase = WIFI_PHASE_ATTEMPT;
        p->since_us = now;
    }
}

static inline void wifi_retry_init(wifi_retry_policy_t *p, uint64_t now,
                                  int32_t result)
{
    *p = (wifi_retry_policy_t){0};
    p->next_delay_us = (uint32_t)WIFI_REJOIN_DELAY_US;
    wifi_retry_join_result(p, now, result);
}

static inline wifi_retry_action_t wifi_retry_observe(wifi_retry_policy_t *p,
                                                     uint64_t now, int32_t link)
{
    if(link == CYW43_LINK_UP) {
        p->phase = WIFI_PHASE_UP;
        p->next_delay_us = (uint32_t)WIFI_REJOIN_DELAY_US;
        return WIFI_RETRY_WAIT;
    }
    if(p->phase == WIFI_PHASE_UP) {
        if(link <= CYW43_LINK_DOWN) {
            wifi_retry_schedule(p, now);
        } else {
            /* Lost the address but still associated: allow DHCP a new bounded
             * chance before actively leaving the access point. */
            p->phase = WIFI_PHASE_ATTEMPT;
            p->since_us = now;
        }
    }
    if(p->phase == WIFI_PHASE_BACKOFF) {
        if((uint64_t)(now - p->since_us) >= p->delay_us) {
            p->phase = WIFI_PHASE_ATTEMPT;
            p->since_us = now;
            return WIFI_RETRY_JOIN;
        }
        return WIFI_RETRY_WAIT;
    }
    if(link < 0) {
        wifi_retry_schedule(p, now);
    } else if((uint64_t)(now - p->since_us) >= WIFI_CONNECT_TIMEOUT_US) {
        wifi_retry_schedule(p, now);
        return WIFI_RETRY_LEAVE;
    }
    return WIFI_RETRY_WAIT;
}
#endif

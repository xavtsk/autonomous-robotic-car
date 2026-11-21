/** @file wifi_retry_policy.h
 * @brief Single-owner Wi-Fi retry policy.
 *
 * The caller supplies CYW43_LINK_* and a
 * monotonic microsecond clock; no sleeps, allocation or driver calls here.
 */
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
#if WIFI_CONNECT_TIMEOUT_US == 0 || WIFI_REJOIN_DELAY_US == 0 ||               \
    WIFI_REJOIN_MAX_DELAY_US < WIFI_REJOIN_DELAY_US ||                         \
    WIFI_REJOIN_MAX_DELAY_US > UINT32_MAX
#error Invalid Wi-Fi retry timing configuration
#endif

typedef enum
{
    WIFI_RETRY_WAIT,
    WIFI_RETRY_LEAVE,
    WIFI_RETRY_JOIN
} wifi_retry_policy_action_t;

typedef struct
{
    uint64_t since_us;
    uint32_t delay_us;
    uint32_t next_delay_us;
    unsigned phase : 2;
} wifi_retry_policy_t;

typedef enum
{
    WIFI_PHASE_ATTEMPT,
    WIFI_PHASE_BACKOFF,
    WIFI_PHASE_UP
} wifi_retry_policy_phase_t;

/** @brief Enter backoff and advance the next delay with the existing cap.
 * @param[in,out] p_policy Non-NULL initialized policy owned by the caller.
 * @param now_us Current wrapping microsecond timestamp; stored unchanged.
 * @note Records the current delay, then doubles the next delay in uint64_t
 * before capping and narrowing it. The largest intermediate is twice
 * UINT32_MAX. The configuration check ensures the cap fits uint32_t.
 * No clock read, sleep, I/O or locking.
 */
static inline void wifi_retry_policy_schedule (wifi_retry_policy_t * p_policy,
                                               uint64_t              now_us)
{
    const uint64_t doubled = (uint64_t)p_policy->next_delay_us * 2ULL;
    /* clang-format off */
    p_policy->phase         = WIFI_PHASE_BACKOFF;
    p_policy->since_us      = now_us;
    p_policy->delay_us      = p_policy->next_delay_us;
    p_policy->next_delay_us =
        (uint32_t)(doubled > WIFI_REJOIN_MAX_DELAY_US ? WIFI_REJOIN_MAX_DELAY_US
                                                      : doubled);
    /* clang-format on */
}

/* Record the join request result, including the initial request. An accepted
 * request is not yet connected. Failed requests also get bounded backoff. */
/** @brief Record a join request outcome without treating acceptance as a link.
 * @param[in,out] p_policy Non-NULL initialized policy, exclusively
 * caller-owned.
 * @param now_us Current wrapping timestamp in microseconds.
 * @param result Driver request result: zero accepted, any nonzero rejected.
 * @note Rejection schedules backoff. Acceptance resets the attempt timestamp
 * and phase only. No driver call, clock read, sleep or locking.
 */
static inline void
wifi_retry_policy_join_result (wifi_retry_policy_t * p_policy, uint64_t now_us,
                               int32_t result)
{
    if (result != 0)
    {
        wifi_retry_policy_schedule(p_policy, now_us);
    }
    else
    {
        p_policy->phase    = WIFI_PHASE_ATTEMPT;
        p_policy->since_us = now_us;
    }
}

/** @brief Initialize all policy fields, then apply the initial join outcome.
 * @param[out] p_policy Non-NULL writable policy owned exclusively by the
 * caller.
 * @param now_us Current wrapping timestamp in microseconds.
 * @param result Initial driver request result; zero accepted, nonzero rejected.
 * @note Replaces prior state. The first backoff is positive and fits
 * uint32_t because it does not exceed the checked cap. A rejected request
 * schedules it immediately. No driver call, sleep or locking.
 */
static inline void wifi_retry_policy_init (wifi_retry_policy_t * p_policy,
                                           uint64_t now_us, int32_t result)
{
    *p_policy               = (wifi_retry_policy_t){0};
    p_policy->next_delay_us = (uint32_t)WIFI_REJOIN_DELAY_US;
    wifi_retry_policy_join_result(p_policy, now_us, result);
}

/** @brief Advance the policy from the current link state and elapsed time.
 * @param[in,out] p_policy Non-NULL initialized policy, exclusively
 * caller-owned.
 * @param now_us Monotonic microsecond clock, wrapping modulo 2^64; elapsed-time
 * comparisons use unsigned subtraction. Call before a whole clock wrap elapses.
 * @param link Signed CYW43_LINK_* state; unknown codes follow the same numeric
 * comparisons as the implementation and are not separately rejected.
 * @return WAIT, LEAVE or JOIN. JOIN/LEAVE also update state before returning;
 * the caller performs the driver action and records a new join's result.
 * @note UP resets backoff; JOIN/NOIP transitions do not extend an attempt's
 * deadline. No clock read, driver call, sleep or locking occurs here.
 */
static inline wifi_retry_policy_action_t
wifi_retry_policy_observe (wifi_retry_policy_t * p_policy, uint64_t now_us,
                           int32_t link)
{
    if (CYW43_LINK_UP == link)
    {
        /* The configured first delay still fits uint32_t on link recovery. */
        p_policy->phase         = WIFI_PHASE_UP;
        p_policy->next_delay_us = (uint32_t)WIFI_REJOIN_DELAY_US;
        return WIFI_RETRY_WAIT;
    }

    /* The two-bit phase fits in int in both enum comparisons. */
    if (WIFI_PHASE_UP == (int)p_policy->phase)
    {
        if (link <= CYW43_LINK_DOWN)
        {
            wifi_retry_policy_schedule(p_policy, now_us);
        }
        else
        {
            /* Lost the address but still associated: allow DHCP a new bounded
             * chance before actively leaving the access point. */
            p_policy->phase    = WIFI_PHASE_ATTEMPT;
            p_policy->since_us = now_us;
        }
    }

    if (WIFI_PHASE_BACKOFF == (int)p_policy->phase)
    {
        if ((now_us - p_policy->since_us) >= p_policy->delay_us)
        {
            p_policy->phase    = WIFI_PHASE_ATTEMPT;
            p_policy->since_us = now_us;
            return WIFI_RETRY_JOIN;
        }

        return WIFI_RETRY_WAIT;
    }

    if (link < 0)
    {
        wifi_retry_policy_schedule(p_policy, now_us);
    }
    else if ((now_us - p_policy->since_us) >= WIFI_CONNECT_TIMEOUT_US)
    {
        wifi_retry_policy_schedule(p_policy, now_us);
        return WIFI_RETRY_LEAVE;
    }
    else
    {
        /* Wait for a link update or the connection deadline. */
    }

    return WIFI_RETRY_WAIT;
}
#endif

/* End of file. */


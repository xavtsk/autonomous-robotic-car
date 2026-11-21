/** @file terr_link.c
 * @brief Integrate terrain readiness, reset and observations with the
 * controller owner.
 * Validated enum limits are nonnegative. Casts make the unsigned
 * comparison domain explicit while retaining the original limits.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 */
#include "terr_link.h"
#include "../app_types.h"
#if APP_S4
#include "run_timing.h"
#include "../subsystem4/buddy4.h"
#include "../subsystem1/buddy1.h"

#define TERRAIN_MAX_AGE_US 150000U
#define TERRAIN_RESET_WAIT_US 500000U
#define TERRAIN_EVENT_BUDGET 8U
#define TERRAIN_LIVE_PERIOD (200U / S3_SAMPLE_PERIOD_MS)
#if (200U % S3_SAMPLE_PERIOD_MS) != 0U || TERRAIN_LIVE_PERIOD == 0U ||         \
    TERRAIN_LIVE_PERIOD > UINT8_MAX
#error "200 ms terrain telemetry cadence must fit the eight-bit cycle divider"
#endif

typedef struct
{
    terr_data_result_t sample;
    uint32_t           now;
    uint32_t           reset_us;
    uint16_t           run;
    uint16_t           expected_run;
    uint16_t           humps;
    uint16_t           sent_humps;
    uint16_t           sent_run;
    /* The sampling owner polls and reports this state. Initialization runs
     * before sampling starts. Cached peak height is 0..500 whole mm. */
    unsigned sent_height : 9;
    unsigned b_has_sample : 1;
    unsigned b_is_waiting : 1;
    unsigned b_has_latched_fault : 1;
    unsigned b_does_own_run : 1;
    unsigned b_has_sent_fault : 1;
    unsigned b_has_sent_hump : 1;
    unsigned b_has_invalidated_hump : 1;
    unsigned b_sent_fault : 1;
    unsigned b_has_unreported_impact : 1;
} terrain_control_t;

static terrain_control_t g_control;
static uint8_t
    g_safety_seen; /* Latch lifting and overturning between reports. */

static bool healthy (uint32_t now);
static void terrain_live_report (void);

/** @brief Reset the controller's terrain snapshot, reporting state and
 * transient impact/safety indications.
 */
void terr_link_init (void)
{
    g_control     = (terrain_control_t){0};
    g_safety_seen = (uint8_t)TERRAIN_SAFETY_NONE;
}

/** @brief Return true only for healthy terrain data when neither a latched
 * fault nor a pending reset blocks START.
 */
bool terr_link_ready (uint32_t now)
{
    return ((!g_control.b_has_latched_fault) && (!g_control.b_is_waiting)) &&
           (healthy(now));
}

/** @brief Test readiness for clearing a fault using fresh healthy data and the
 * pending-reset state.
 */
bool terr_link_reset_ready (uint32_t now)
{
    return (!g_control.b_is_waiting) && (healthy(now));
}

/** @brief Clear the controller's terrain fault latch.
 */
void terr_link_clear_fault (void)
{
    g_control.b_has_latched_fault = false;
}

/** @brief On readiness and a successful terrain run request, retain the
 * expected run token, start the reset acknowledgement timer and
 * invalidate the previous hump report.
 */
bool terr_link_begin (uint32_t now)
{
    uint16_t token;

    if ((!terr_link_ready(now)) || (!buddy4_begin_run(&token)))
    {
        return false;
    }

    /* clang-format off */
    g_control.expected_run           = token;
    g_control.reset_us               = now;
    g_control.b_is_waiting           = true;
    g_control.b_has_sample           = false;
    g_control.b_does_own_run         = true;
    g_control.b_has_sent_hump        = false;
    g_control.b_has_invalidated_hump =
        buddy1_invalidate(BUDDY1_TOPIC_HUMP) == E_OK;
    /* clang-format on */
    return true;
}

/** @brief On the drive owner, consume a bounded number of events, evaluate
 * freshness and the run-token acknowledgement, and update terrain
 * holds or drive faults using the wrapping microsecond clock.
 */
void terr_link_poll (s3_drive_t * p_drive, uint32_t now)
{
    terr_data_result_t sample;
    terr_data_result_t event;
    uint16_t           run;
    uint16_t           humps;
    const bool         b_is_running =
        (p_drive->state >= (unsigned int)S3_DRIVE_FOLLOW) &&
        (p_drive->state <= (unsigned int)S3_DRIVE_REACQUIRE);
    const bool b_is_resetting =
        s3_drive_reset_result(p_drive) == (unsigned int)S3_RESET_PENDING;
    bool b_has_event_fault       = false;
    bool b_was_lost              = false;
    g_control.now                = now;
    p_drive->b_has_external_hold = false;

    if (!b_is_running)
    {
        g_control.b_is_waiting = false;
    }

    const bool b_has_sample = buddy4_read(&sample, &run, &humps);

    if (((b_has_sample && b_is_running) && (g_control.b_does_own_run)) &&
        (run != g_control.expected_run))
    {
        b_has_event_fault = true;
    }
    else if (b_has_sample)
    {
        g_control.sample       = sample;
        g_control.run          = run;
        g_control.humps        = humps;
        g_control.b_has_sample = true;

        if ((sample.b_has_impact) ||
            ((unsigned int)TERRAIN_IMPACT == sample.state))
        {
            g_control.b_has_unreported_impact = true;
        }

        if ((g_control.b_is_waiting) && (run == g_control.expected_run))
        {
            g_control.b_is_waiting = false;
        }
    }
    else
    {
        /* Retain the previous state when a snapshot is unavailable. */
    }
    /* Bound work per control cycle. Completed hump count comes from the
     * coherent cumulative snapshot, not this potentially lossy event queue. */
    for (unsigned event_index = 0; (event_index < TERRAIN_EVENT_BUDGET) &&
                                   (buddy4_event(&event, &b_was_lost));
         ++event_index)
    {
        if ((((b_was_lost ||
               (event.safety != (unsigned int)TERRAIN_SAFETY_NONE)) ||
              (event.b_has_impact)) ||
             ((unsigned int)TERRAIN_IMPACT == event.state)) ||
            (event.flags & TERRAIN_FAULT))
        {
            b_has_event_fault = true;
        }

        if ((event.b_has_impact) ||
            ((unsigned int)TERRAIN_IMPACT == event.state))
        {
            g_control.b_has_unreported_impact = true;
        }

        if (event.safety != (unsigned int)TERRAIN_SAFETY_NONE)
        {
            g_safety_seen = (uint8_t)event.safety;
        }
    }

    if ((!b_is_running) && (!b_is_resetting))
    {
        return;
    }

    if (((g_control.b_is_waiting) &&
         ((now - g_control.reset_us) <= TERRAIN_RESET_WAIT_US)) &&
        (!b_has_event_fault))
    {
        p_drive->b_has_external_hold = true;
        return;
    }

    if ((b_has_event_fault || (g_control.b_is_waiting)) || (!healthy(now)))
    {
        g_control.b_has_latched_fault = true;
        g_control.b_is_waiting        = false;
        s3_drive_terrain_fault(p_drive);
    }
}

/** @brief Publish changed terrain fault and completed hump state. Keep unsent
 * updates eligible for retry and invalidate the hump section when no
 * current result applies.
 */
void terr_link_report (void)
{
    const bool b_has_fault =
        (g_control.b_has_latched_fault) ||
        ((!g_control.b_is_waiting) &&
         (((((!g_control.b_has_sample) ||
             ((g_control.now - g_control.sample.timestamp_us) >
              TERRAIN_MAX_AGE_US)) ||
            (g_control.sample.flags & TERRAIN_FAULT)) ||
           (g_control.sample.b_has_impact)) ||
          (g_control.sample.safety != (unsigned int)TERRAIN_SAFETY_NONE)));

    if ((!g_control.b_has_sent_fault) ||
        (g_control.b_sent_fault != b_has_fault))
    {
        ER value_er = buddy1_update_faults(BUDDY1_FAULT_IMU,
                                           b_has_fault ? BUDDY1_FAULT_IMU : 0U);
        /* E_TMOUT: mirror updated but the live fault report was not queued;
         * retry next cycle so the dashboard sees the change promptly. */
        if (E_OK == value_er)
        {
            g_control.b_has_sent_fault = true;
            g_control.b_sent_fault     = b_has_fault;
        }
    }
    /* A previous valid run peak survives transient sensor faults. It is not
     * relabelled as a current instantaneous measurement. */
    if ((((((g_control.b_has_sample) && (g_control.b_does_own_run)) &&
           (g_control.run == g_control.expected_run)) &&
          (g_control.sample.flags & TERRAIN_PEAK_VALID)) &&
         (g_control.sample.peak_m >= 0)) &&
        (g_control.sample.peak_m <= 0.5f))
    {
        /* The accepted 0..0.5 m range rounds to 0..500 mm. */
        const uint16_t height =
            (uint16_t)((g_control.sample.peak_m * 1000.0f) + 0.5f);
        /* E_TMOUT updates the snapshot cache without queuing the live
         * report. Retry on a later cycle with the latest count and peak. */
        if (((((!g_control.b_has_sent_hump) ||
               (g_control.sent_run != g_control.run)) ||
              (g_control.sent_humps != g_control.humps)) ||
             (g_control.sent_height != height)) &&
            (E_OK == buddy1_report_hump(g_control.humps, height)))
        {
            g_control.b_has_sent_hump        = true;
            g_control.b_has_invalidated_hump = false;
            g_control.sent_humps             = g_control.humps;
            g_control.sent_height            = height;
            g_control.sent_run               = g_control.run;
        }
    }
    else
    {
        if (!g_control.b_has_invalidated_hump)
        {
            g_control.b_has_invalidated_hump =
                buddy1_invalidate(BUDDY1_TOPIC_HUMP) == E_OK;
        }

        g_control.b_has_sent_hump = false;
    }
    /* S4 exports compass-derived turn RATE, not absolute heading. Never feed
     * degrees/second into Buddy 1's heading-degrees packet. */
    terrain_live_report();
}

/** @brief Require a calibrated sample no older than 150 ms with no terrain
 * fault, impact or active safety condition. The input clock wraps in
 * microseconds.
 */
static bool healthy (uint32_t now)
{
    return ((((((g_control.b_has_sample) &&
                ((now - g_control.sample.timestamp_us) <=
                 TERRAIN_MAX_AGE_US)) &&
               (g_control.sample.flags & TERRAIN_CALIBRATED)) &&
              (!(g_control.sample.flags & TERRAIN_FAULT))) &&
             (!g_control.sample.b_has_impact)) &&
            (g_control.sample.state != (unsigned int)TERRAIN_IMPACT)) &&
           ((unsigned int)TERRAIN_SAFETY_NONE == g_control.sample.safety);
}

/** @brief Round a tilt between -90 and 90 degrees to signed tenths. An
 * unordered or out-of-range input clears the caller's validity flag
 * and returns zero.
 */
static int16_t tilt_x10 (float32_t deg, bool * pb_is_valid)
{
    const float32_t tenths_deg = deg * 10.0f;

    if (!((tenths_deg >= -900.0f) && (tenths_deg <= 900.0f)))
    {
        /* An unordered input fails the range check. */
        *pb_is_valid = false;
        return 0;
    }

    /* The ordered range check keeps the rounded result within +/-900. */
    return (int16_t)(tenths_deg >= 0.0f ? tenths_deg + 0.5f
                                        : tenths_deg - 0.5f);
}

/** @brief Convert a height between zero and half a metre to whole millimetres.
 * Return false on invalid input before writing the caller's output.
 */
static bool height_mm (float32_t height_m, uint16_t * p_mm)
{
    if (!((height_m >= 0.0f) && (height_m <= 0.5f)))
    {
        /* An unordered input fails the range check. */
        return false;
    }

    /* The validated height rounds to 0..500 mm before being stored. */
    *p_mm = (uint16_t)((height_m * 1000.0f) + 0.5f);
    return true;
}

/* Live Buddy 4 report for the laptop, every TERRAIN_LIVE_PERIOD calls (200 ms
 * at the configured control cycle), with or without a run, so tilting or
 * pushing the car on the bench shows up: pitch/roll, terrain state, the height
 * so far on a hump being crossed, and the highest completed hump since START
 * (reset at START; Buddy 4 retains it for the run). An impact anywhere in the
 * period is reported as the impact state even if the latest sample is normal.
 * Phase-shifted from the 200 ms motion/pose reports so the four-message
 * outbox is not filled in one cycle. QoS 0: a full outbox drops one report.
 * Stale or missing IMU data is "unavailable", never 0 degrees or 0 mm. */
/** @brief At the live reporting cadence, encode freshness, valid tilt and
 * heights, and accumulated impact or safety observations into the
 * terrain report.
 */
static void terrain_live_report (void)
{
    static uint8_t             divider   = TERRAIN_LIVE_PERIOD / 2U;
    telemetry_codec_terrain_t  telemetry = {0};
    const terr_data_result_t * p_terrain = &g_control.sample;

    if (++divider < TERRAIN_LIVE_PERIOD)
    {
        return;
    }

    divider = 0U;
    const bool b_is_fresh =
        (g_control.b_has_sample) &&
        ((g_control.now - p_terrain->timestamp_us) <= TERRAIN_MAX_AGE_US);
    bool b_did_succeed =
        (b_is_fresh && (p_terrain->flags & TERRAIN_TILT_VALID)) &&
        (!(p_terrain->flags & TERRAIN_FAULT));

    if (b_did_succeed)
    {
        /* clang-format off */
        telemetry.pitch_tenths_deg =
            tilt_x10(p_terrain->pitch_deg, &b_did_succeed);
        telemetry.roll_tenths_deg  =
            tilt_x10(p_terrain->roll_deg, &b_did_succeed);
        /* clang-format on */
    }

    if (b_did_succeed)
    {
        telemetry.tilt_valid = 1U;
    }
    else
    {
        telemetry.pitch_tenths_deg = 0;
        telemetry.roll_tenths_deg  = 0;
    }

    /* Fresh states must fit the three-bit field. Use the bounded unknown
     * or impact code for the existing fallback and event paths. */
    telemetry.state = b_is_fresh && ((unsigned)p_terrain->state < 8U)
                          ? (uint8_t)p_terrain->state
                          : (uint8_t)TERRAIN_UNKNOWN;

    if (g_control.b_has_unreported_impact)
    {
        telemetry.state = (uint8_t)TERRAIN_IMPACT;
    }

    if ((b_is_fresh && (p_terrain->b_is_hump_active)) &&
        (height_mm(p_terrain->hump_height_m, &telemetry.climb_mm)))
    {
        telemetry.on_hump = 1U;
    }

    if (((g_control.b_has_sample) && (p_terrain->flags & TERRAIN_PEAK_VALID)) &&
        (height_mm(p_terrain->peak_m, &telemetry.peak_mm)))
    {
        telemetry.peak_valid = 1U;
    }
    /* Compass turn rate: only with a fresh, valid estimate (needs Buddy 4's
     * magnetometer calibration); unavailable is sent as 0 with valid = 0. */
    if (((b_is_fresh && (p_terrain->flags & TERRAIN_TURN_VALID)) &&
         (p_terrain->turn_dps >= -(float32_t)BUDDY1_TURN_LIMIT_DPS)) &&
        (p_terrain->turn_dps <= (float32_t)BUDDY1_TURN_LIMIT_DPS))
    {
        telemetry.turn_valid = 1U;
        /* The ordered checks bound the rounded rate to +/-360 dps. */
        telemetry.turn_dps =
            (int16_t)(p_terrain->turn_dps >= 0.0f ? p_terrain->turn_dps + 0.5f
                                                  : p_terrain->turn_dps - 0.5f);
    }

    if (b_is_fresh &&
        ((unsigned int)TERRAIN_SAFETY_LIFTED == p_terrain->safety))
    {
        telemetry.safety = BUDDY1_SAFETY_LIFTED;
    }

    if (b_is_fresh &&
        ((unsigned int)TERRAIN_SAFETY_OVERTURNED == p_terrain->safety))
    {
        telemetry.safety = BUDDY1_SAFETY_OVERTURNED;
    }

    if ((BUDDY1_SAFETY_NONE == telemetry.safety) &&
        (TERRAIN_SAFETY_LIFTED == (int)g_safety_seen))
    {
        telemetry.safety = BUDDY1_SAFETY_LIFTED;
    }

    if (TERRAIN_SAFETY_OVERTURNED == (int)g_safety_seen)
    {
        telemetry.safety = BUDDY1_SAFETY_OVERTURNED;
    }

    if (buddy1_report_terrain(&telemetry) == E_OK)
    {
        g_control.b_has_unreported_impact = false;
        g_safety_seen                     = (uint8_t)TERRAIN_SAFETY_NONE;
    }
}
#endif

/* End of file. */


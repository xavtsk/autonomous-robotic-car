/** @file terr_link.c
 * @brief Integrate terrain readiness, reset and observations with the
 * controller owner.
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

typedef struct
{
    terrain_result_t sample;
    uint32_t now, reset_us;
    uint16_t run, expected_run, humps, sent_humps, sent_height, sent_run;
    unsigned have : 1, waiting : 1, latched : 1, run_owned : 1, fault_sent : 1,
        hump_sent : 1, hump_invalidated : 1;
    int sent_fault;
} terrain_control_t;

static terrain_control_t g_control;
static void terrain_live_report (void);
static bool g_impact_seen;    /* impact since the last live report */
static uint8_t g_safety_seen; /* lifted/overturned since the last live report */

static bool healthy (uint32_t now)
{
    return g_control.have &&
           (uint32_t)(now - g_control.sample.timestamp_us) <=
               TERRAIN_MAX_AGE_US &&
           (g_control.sample.flags & TERRAIN_CALIBRATED) &&
           !(g_control.sample.flags & TERRAIN_FAULT) &&
           !g_control.sample.impact &&
           g_control.sample.state != TERRAIN_IMPACT &&
           g_control.sample.safety == TERRAIN_SAFETY_NONE;
}

void terrain_control_init (void)
{
    g_control = (terrain_control_t){0};
    g_impact_seen = false;
    g_safety_seen = (uint8_t)TERRAIN_SAFETY_NONE;
}

bool terrain_control_ready (uint32_t now)
{
    return !g_control.latched && !g_control.waiting && healthy(now);
}

bool terrain_control_reset_ready (uint32_t now)
{
    return !g_control.waiting && healthy(now);
}

void terrain_control_clear_fault (void)
{
    g_control.latched = false;
}

bool terrain_control_begin (uint32_t now)
{
    uint16_t token;
    if (!terrain_control_ready(now) || !subsystem4_begin_run(&token))
    {
        return false;
    }
    g_control.expected_run = token;
    g_control.reset_us = now;
    g_control.waiting = true;
    g_control.have = false;
    g_control.run_owned = true;
    g_control.hump_sent = false;
    g_control.hump_invalidated = buddy1_invalidate(BUDDY1_TOPIC_HUMP) == E_OK;
    return true;
}

void terrain_control_poll (s3_drive_t * p_drive, uint32_t now)
{
    terrain_result_t sample, event;
    uint16_t run, humps;
    bool running = p_drive->state >= S3_DRIVE_FOLLOW &&
                   p_drive->state <= S3_DRIVE_REACQUIRE;
    bool resetting = s3_drive_reset_result(p_drive) == S3_RESET_PENDING;
    bool event_fault = false, lost = false;
    g_control.now = now;
    p_drive->external_hold = false;
    if (!running)
    {
        g_control.waiting = false;
    }
    if (subsystem4_read(&sample, &run, &humps))
    {
        if (running && g_control.run_owned && run != g_control.expected_run)
        {
            event_fault = true;
        }
        else
        {
            g_control.sample = sample;
            g_control.run = run;
            g_control.humps = humps;
            g_control.have = true;
            if (sample.impact || sample.state == TERRAIN_IMPACT)
            {
                g_impact_seen = true;
            }
            if (g_control.waiting && run == g_control.expected_run)
            {
                g_control.waiting = false;
            }
        }
    }
    /* Bound work per control cycle. Completed hump count comes from the
     * coherent cumulative snapshot, not this potentially lossy event queue. */
    for (unsigned value_i = 0;
         value_i < TERRAIN_EVENT_BUDGET && subsystem4_event(&event, &lost);
         ++value_i)
    {
        if (lost || event.safety != TERRAIN_SAFETY_NONE || event.impact ||
            event.state == TERRAIN_IMPACT || (event.flags & TERRAIN_FAULT))
        {
            event_fault = true;
        }
        if (event.impact || event.state == TERRAIN_IMPACT)
        {
            g_impact_seen = true;
        }
        if (event.safety != TERRAIN_SAFETY_NONE)
        {
            g_safety_seen = (uint8_t)event.safety;
        }
    }
    if (!running && !resetting)
    {
        return;
    }
    if (g_control.waiting &&
        (uint32_t)(now - g_control.reset_us) <= TERRAIN_RESET_WAIT_US &&
        !event_fault)
    {
        p_drive->external_hold = true;
        return;
    }
    if (event_fault || g_control.waiting || !healthy(now))
    {
        g_control.latched = true;
        g_control.waiting = false;
        s3_drive_terrain_fault(p_drive);
    }
}

void terrain_control_report (void)
{
    bool bad =
        g_control.latched ||
        (!g_control.waiting &&
         (!g_control.have ||
          (uint32_t)(g_control.now - g_control.sample.timestamp_us) >
              TERRAIN_MAX_AGE_US ||
          (g_control.sample.flags & TERRAIN_FAULT) || g_control.sample.impact ||
          g_control.sample.safety != TERRAIN_SAFETY_NONE));
    if (!g_control.fault_sent || g_control.sent_fault != (int)bad)
    {
        ER value_er =
            buddy1_update_faults(BUDDY1_FAULT_IMU, bad ? BUDDY1_FAULT_IMU : 0U);
        /* E_TMOUT: mirror updated but the live fault report was not queued;
         * retry next cycle so the dashboard sees the change promptly. */
        if (value_er == E_OK)
        {
            g_control.fault_sent = true;
            g_control.sent_fault = (int)bad;
        }
    }
    /* A previous valid run peak survives transient sensor faults. It is not
     * relabelled as a current instantaneous measurement. */
    if (g_control.have && g_control.run_owned &&
        g_control.run == g_control.expected_run &&
        (g_control.sample.flags & TERRAIN_PEAK_VALID) &&
        g_control.sample.peak_m >= 0 && g_control.sample.peak_m <= 0.5f)
    {
        uint16_t height = (uint16_t)(g_control.sample.peak_m * 1000.0f + 0.5f);
        if (!g_control.hump_sent || g_control.sent_run != g_control.run ||
            g_control.sent_humps != g_control.humps ||
            g_control.sent_height != height)
        {
            ER value_er = buddy1_report_hump(g_control.humps, height);
            /* E_TMOUT updates the snapshot cache but does not enqueue the
             * live report. Keep it pending until a later nonblocking attempt
             * succeeds, using the latest count/peak from this run. */
            if (value_er == E_OK)
            {
                g_control.hump_sent = true;
                g_control.hump_invalidated = false;
                g_control.sent_humps = g_control.humps;
                g_control.sent_height = height;
                g_control.sent_run = g_control.run;
            }
        }
    }
    else
    {
        if (!g_control.hump_invalidated)
        {
            g_control.hump_invalidated =
                buddy1_invalidate(BUDDY1_TOPIC_HUMP) == E_OK;
        }
        g_control.hump_sent = false;
    }
    /* S4 exports compass-derived turn RATE, not absolute heading. Never feed
     * degrees/second into Buddy 1's heading-degrees packet. */
    terrain_live_report();
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
#define TERRAIN_LIVE_PERIOD (200U / S3_SAMPLE_PERIOD_MS)
#if (200U % S3_SAMPLE_PERIOD_MS) != 0U || TERRAIN_LIVE_PERIOD == 0U ||         \
    TERRAIN_LIVE_PERIOD > UINT8_MAX
#error "200 ms terrain telemetry cadence must fit the eight-bit cycle divider"
#endif
static int16_t tilt_x10 (float32_t deg, bool * p_ok)
{
    float32_t value_x = deg * 10.0f;
    if (!(value_x >= -900.0f && value_x <= 900.0f))
    {
        *p_ok = false;
        return 0;
    } /* also NaN */
    return (int16_t)(value_x >= 0.0f ? value_x + 0.5f : value_x - 0.5f);
}

static bool height_mm (float32_t value_m, uint16_t * p_mm)
{
    if (!(value_m >= 0.0f && value_m <= 0.5f))
    {
        return false;
    } /* also NaN */
    *p_mm = (uint16_t)(value_m * 1000.0f + 0.5f);
    return true;
}

static void terrain_live_report (void)
{
    static uint8_t divider = TERRAIN_LIVE_PERIOD / 2U;
    buddy1_terrain_t value_t = {0};
    const terrain_result_t * p_s = &g_control.sample;
    bool fresh, value_ok;
    if (++divider < TERRAIN_LIVE_PERIOD)
    {
        return;
    }
    divider = 0U;
    fresh = g_control.have &&
            (uint32_t)(g_control.now - p_s->timestamp_us) <= TERRAIN_MAX_AGE_US;
    value_ok = fresh && (p_s->flags & TERRAIN_TILT_VALID) &&
               !(p_s->flags & TERRAIN_FAULT);
    if (value_ok)
    {
        value_t.pitch_x10 = tilt_x10(p_s->pitch_deg, &value_ok);
        value_t.roll_x10 = tilt_x10(p_s->roll_deg, &value_ok);
    }
    if (value_ok)
    {
        value_t.tilt_valid = 1U;
    }
    else
    {
        value_t.pitch_x10 = 0;
        value_t.roll_x10 = 0;
    }
    value_t.state = fresh && (unsigned)p_s->state < 8U
                        ? (uint8_t)p_s->state
                        : (uint8_t)TERRAIN_UNKNOWN;
    if (g_impact_seen)
    {
        value_t.state = (uint8_t)TERRAIN_IMPACT;
    }
    if (fresh && p_s->hump_active &&
        height_mm(p_s->hump_height_m, &value_t.climb_mm))
    {
        value_t.on_hump = 1U;
    }
    if (g_control.have && (p_s->flags & TERRAIN_PEAK_VALID) &&
        height_mm(p_s->peak_m, &value_t.peak_mm))
    {
        value_t.peak_valid = 1U;
    }
    /* Compass turn rate: only with a fresh, valid estimate (needs Buddy 4's
     * magnetometer calibration); unavailable is sent as 0 with valid = 0. */
    if (fresh && (p_s->flags & TERRAIN_TURN_VALID) &&
        p_s->turn_dps >= -(float32_t)BUDDY1_TURN_LIMIT_DPS &&
        p_s->turn_dps <= (float32_t)BUDDY1_TURN_LIMIT_DPS)
    {
        value_t.turn_valid = 1U;
        value_t.turn_dps =
            (int16_t)(p_s->turn_dps >= 0.0f ? p_s->turn_dps + 0.5f
                                            : p_s->turn_dps - 0.5f);
    }
    if (fresh && p_s->safety == TERRAIN_SAFETY_LIFTED)
    {
        value_t.safety = BUDDY1_SAFETY_LIFTED;
    }
    if (fresh && p_s->safety == TERRAIN_SAFETY_OVERTURNED)
    {
        value_t.safety = BUDDY1_SAFETY_OVERTURNED;
    }
    if (value_t.safety == BUDDY1_SAFETY_NONE &&
        g_safety_seen == TERRAIN_SAFETY_LIFTED)
    {
        value_t.safety = BUDDY1_SAFETY_LIFTED;
    }
    if (g_safety_seen == TERRAIN_SAFETY_OVERTURNED)
    {
        value_t.safety = BUDDY1_SAFETY_OVERTURNED;
    }
    if (buddy1_report_terrain(&value_t) == E_OK)
    {
        g_impact_seen = false;
        g_safety_seen = (uint8_t)TERRAIN_SAFETY_NONE;
    }
}
#endif

/* End of file. */


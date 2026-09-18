/*
 * telemetry.c - message building and the outbound queue.
 *
 * No Pico SDK calls anywhere in this file. That is deliberate: it means
 * every line here can be compiled and tested on a laptop with plain C99,
 * which is where most of the bugs will be found.
 */

#include "buddy1/telemetry.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* -------------------------------------------------------------------------
 * Topics
 *
 * Written as macros so the C compiler joins the root and the suffix into a
 * single string literal at COMPILE time. "abc" "def" is "abcdef" in C, with
 * no runtime work and no buffer needed. Each one ends up as one constant
 * string in flash, which is why tele_msg_t stores only a pointer to it.
 * ------------------------------------------------------------------------- */

/*
 * Topics are identified internally by index rather than by string, so the
 * snapshot cache can hold one entry per topic in a plain array. The table
 * below is the only place the index and the string are tied together.
 */
typedef enum
{
    TT_STATE = 0,
    TT_MOTION,
    TT_LINE,
    TT_BARCODE,
    TT_HUMP,
    TT_IMU,
    TT_OBSTACLE,
    TT_LOG,
    TT_COUNT
} tele_topic_t;

typedef enum
{
    PT_MOTION = 0,
    PT_LINE,
    PT_IMU,
    PT_COUNT
} periodic_topic_t;

static const char * const g_topics[] =
{
    TELE_TOPIC_ROOT "/telemetry/state",
    TELE_TOPIC_ROOT "/telemetry/motion",
    TELE_TOPIC_ROOT "/telemetry/line",
    TELE_TOPIC_ROOT "/telemetry/barcode",
    TELE_TOPIC_ROOT "/telemetry/hump",
    TELE_TOPIC_ROOT "/telemetry/imu",
    TELE_TOPIC_ROOT "/telemetry/obstacle",
    TELE_TOPIC_ROOT "/telemetry/log"
};

/*
 * Largest message body (everything between the envelope and the closing
 * brace).
 *
 * Worked out from the worst case, not guessed: the obstacle report with
 * every field at its type's extreme is 109 characters, and the motion
 * report is 97. 144 leaves real headroom, because a body that overflowed
 * would be truncated mid-string and produce JSON with no closing brace -
 * which the dashboard would reject for reasons nobody could see.
 */
#define TELE_BODY_MAX  144u

/*
 * Longest free-text log message we accept from a caller, in characters
 * BEFORE escaping. Escaping can double the length in the worst case (a
 * string of nothing but quotes), so the escaped buffer is sized for that
 * and the result still fits inside TELE_BODY_MAX.
 */
#define TELE_LOG_TEXT_MAX   80u
#define TELE_LOG_ESCAPED_MAX 128u

/* Index mask. Valid only because TELE_QUEUE_SLOTS is a power of two. */
#define TELE_QUEUE_MASK  (TELE_QUEUE_SLOTS - 1u)

/* -------------------------------------------------------------------------
 * Module state
 *
 * All static: nothing outside this file can reach these, which is Barr-C
 * rule 1.8a and stops another subsystem corrupting the queue by accident.
 *
 * Named with a leading g because they are module-global (Barr-C 7.1j).
 *
 * NOT volatile yet. These are only touched from ordinary task code, never
 * from an interrupt handler. When this module goes under the RTOS, the
 * push/pop pair needs a lock - see the note on tele_enqueue() below.
 * ------------------------------------------------------------------------- */

static tele_msg_t g_queue[TELE_QUEUE_SLOTS];
static uint32_t   g_head;      /* index of the next slot we will write   */
static uint32_t   g_tail;      /* index of the oldest unread message     */
static uint32_t   g_count;     /* how many messages are waiting          */
static uint32_t   g_seq;       /* sequence number of the next message    */
static uint32_t   g_dropped;   /* important events lost because FIFO full */

/* High-rate telemetry is stored as one pending latest value per topic. It is
 * deliberately separate from the event FIFO so motion/line/IMU traffic can
 * never evict a barcode, hump, obstacle or state transition. */
static tele_msg_t g_periodic[PT_COUNT];
static bool       g_periodic_pending[PT_COUNT];
static uint32_t   g_periodic_order[PT_COUNT];
static uint32_t   g_periodic_count;
static uint32_t   g_coalesced;

/*
 * THE SNAPSHOT CACHE
 *
 * The most recent body published on each topic, kept so the car can answer
 * "tell me everything right now" - a requirement the lecturer stated in the
 * briefing ("give me a summary... the car should be able to send back a
 * snapshot of what's happening at that point of time").
 *
 * Updated automatically inside tele_enqueue(), so no subsystem has to report
 * anything twice or know the cache exists.
 *
 * Cost: TT_COUNT x TELE_BODY_MAX = 8 x 144 = 1.1 kB of RAM.
 */
static char g_latest[TT_COUNT][TELE_BODY_MAX];
static bool g_has_latest[TT_COUNT];

/* -------------------------------------------------------------------------
 * Name tables
 *
 * The order of these MUST match the order of the enums in telemetry.h.
 * The compile-time checks below catch it if someone adds an enum value and
 * forgets the name - a mismatch would otherwise print the wrong state
 * forever and nobody would notice.
 * ------------------------------------------------------------------------- */

#define TELE_COMPILE_ASSERT(condition, name) \
    typedef char tele_assert_##name[(condition) ? 1 : -1]

TELE_COMPILE_ASSERT(
    (sizeof g_topics / sizeof g_topics[0]) == (size_t)TT_COUNT,
    topic_table_matches_enum);

/*
 * The payload buffer must hold the worst case: the longest envelope, the
 * snapshot marker, the longest body, the closing brace, and the terminator.
 * Asserting it here means a future field that breaks the arithmetic stops
 * the build rather than silently truncating a message at run time.
 */
#define TELE_ENVELOPE_WORST  33u   /* {"t":4294967295,"seq":4294967295,     */
#define TELE_SNAP_MARKER     12u   /* "snap":true,                          */

TELE_COMPILE_ASSERT(
    (TELE_ENVELOPE_WORST + TELE_SNAP_MARKER + (TELE_BODY_MAX - 1u) + 2u)
        <= TELE_PAYLOAD_MAX,
    payload_buffer_holds_worst_case);

/*
 * The name tables that used to live here now live in robot_types.c, beside
 * the enums they describe. Keeping a second copy meant telemetry and the
 * mission controller could drift apart and report different names for the
 * same state.
 */

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

void tele_init(void)
{
    unsigned i;

    tele_lock();

    g_head    = 0u;
    g_tail    = 0u;
    g_count   = 0u;
    g_seq     = 0u;
    g_dropped = 0u;
    g_periodic_count = 0u;
    g_coalesced      = 0u;

    for (i = 0u; i < (unsigned)TT_COUNT; i++)
    {
        g_has_latest[i]  = false;
        g_latest[i][0]   = '\0';
    }

    for (i = 0u; i < (unsigned)PT_COUNT; i++)
    {
        g_periodic_pending[i] = false;
        g_periodic_order[i]   = 0u;
    }

    tele_unlock();
}

/* -------------------------------------------------------------------------
 * JSON string escaping
 *
 * Every other field we publish comes from a fixed table or an integer, so it
 * cannot contain anything troublesome. tele_report_log() is the single place
 * where a caller's own text reaches the wire, and text breaks JSON in three
 * ways:
 *
 *   a quote      "sensor said "hi""   -> invalid, the dashboard rejects it
 *   a backslash  "path C:\temp"       -> \t is a TAB escape in JSON, so the
 *                                        message silently arrives wrong,
 *                                        which is worse than being rejected
 *   a newline    raw control character -> illegal inside a JSON string
 *
 * Quotes and backslashes are escaped. Control characters are replaced with a
 * space rather than escaped as \u00XX: in a log line they carry no meaning
 * worth six bytes on a constrained link.
 *
 * Bounded at both ends - it never reads past max_src and never writes past
 * dst_size - so no caller can overflow it however badly behaved they are.
 * ------------------------------------------------------------------------- */

static size_t json_escape(char * dst, size_t dst_size,
                          const char * src, size_t max_src)
{
    size_t di = 0u;
    size_t si = 0u;

    if (0u == dst_size)
    {
        return 0u;
    }

    while (('\0' != src[si]) && (si < max_src))
    {
        unsigned char ch = (unsigned char)src[si];

        if (('"' == ch) || ('\\' == ch))
        {
            if ((di + 2u) >= dst_size) { break; }
            dst[di] = '\\';       di++;
            dst[di] = (char)ch;   di++;
        }
        else if ((ch < 0x20u) || (0x7Fu == ch))
        {
            if ((di + 1u) >= dst_size) { break; }
            dst[di] = ' ';        di++;
        }
        else
        {
            if ((di + 1u) >= dst_size) { break; }
            dst[di] = (char)ch;   di++;
        }
        si++;
    }

    dst[di] = '\0';
    return di;
}

/* -------------------------------------------------------------------------
 * The heart of the module
 * ------------------------------------------------------------------------- */

/*
 * Tell GCC and Clang that tele_enqueue() takes a printf-style format string
 * (argument 2) whose values start at argument 3.
 *
 * This buys two things:
 *   - the compiler checks EVERY tele_report_*() call below, so a format
 *     string that does not match its arguments is caught at build time.
 *     That is the exact defect class in LAB6's pid.c ("%d" for a float) and
 *     in Bug Hunt #4, and it is undefined behaviour, not a style slip.
 *   - it stops -Wformat-nonliteral firing on the vsnprintf() inside, because
 *     the compiler can now see the format string is a checked parameter.
 *
 * Guarded so a compiler without the attribute still builds (Barr-C 1.1c:
 * keep compiler extensions minimal and localised - this is the only one).
 */
#if defined(__GNUC__)
#define TELE_PRINTF_LIKE(fmt_index, first_value) \
    __attribute__((format(printf, fmt_index, first_value)))
#else
#define TELE_PRINTF_LIKE(fmt_index, first_value)
#endif

static bool tele_enqueue(tele_topic_t which, const char * body_fmt, ...)
    TELE_PRINTF_LIKE(2, 3);

/*
 * Build one message and put it in the queue.
 *
 * Returns true if it was queued cleanly, false if an older message had to
 * be thrown away to make room.
 *
 * THREAD SAFETY: enqueue_body() uses the platform tele_lock() hooks around
 * the shared queue state. The host and current bare-metal Pico ports are
 * single-threaded and implement those hooks as no-ops. The future μT-Kernel
 * port must provide a short synchronization mechanism and measure its bound.
 */
/*
 * Put an already-formatted body into the queue.
 *
 * is_snapshot marks the message as a replay of cached state rather than a
 * live event, so a subscriber can tell the two apart. Without that marker a
 * snapshot would look exactly like the car suddenly re-reporting everything
 * it had ever seen, which would be misleading in a log.
 */
static periodic_topic_t periodic_index(tele_topic_t which)
{
    switch (which)
    {
        case TT_MOTION: return PT_MOTION;
        case TT_LINE:   return PT_LINE;
        case TT_IMU:    return PT_IMU;
        case TT_STATE:
        case TT_BARCODE:
        case TT_HUMP:
        case TT_OBSTACLE:
        case TT_LOG:
        case TT_COUNT:
        default:        return PT_COUNT;
    }
}

static void build_message(tele_msg_t * slot,
                          tele_topic_t which,
                          const char * body,
                          bool is_snapshot)
{
    slot->topic = g_topics[which];
    (void)snprintf(slot->payload, sizeof slot->payload,
                   "{\"t\":%" PRIu32 ",\"seq\":%" PRIu32 ",%s%.*s}",
                   tele_now_ms(), g_seq,
                   is_snapshot ? "\"snap\":true," : "",
                   (int)(TELE_BODY_MAX - 1u), body);
    g_seq++;
}

static bool enqueue_body(tele_topic_t which, const char * body, bool is_snapshot)
{
    tele_msg_t * slot;
    bool         queued_cleanly = true;
    periodic_topic_t periodic;

    /* Keep the table index safe even if a future caller accidentally casts an
     * out-of-range integer to tele_topic_t. Public report functions never do
     * this, but rejecting it here makes the module boundary robust and keeps
     * static analysis able to prove every g_topics[] access is in bounds. */
    if (((unsigned)which >= (unsigned)TT_COUNT) || (NULL == body))
    {
        return false;
    }

    periodic = periodic_index(which);

    /* Everything from here to tele_unlock() is one indivisible operation:
       the overflow check, the slot write and the index advance have to agree
       with each other or the queue corrupts. Short and bounded - no I/O. */
    tele_lock();

    if (!is_snapshot && (periodic < PT_COUNT))
    {
        slot = &g_periodic[periodic];
        if (g_periodic_pending[periodic])
        {
            g_coalesced++;
        }
        else
        {
            g_periodic_pending[periodic] = true;
            g_periodic_count++;
        }
        g_periodic_order[periodic] = g_seq;
        build_message(slot, which, body, false);
    }
    else
    {
        /* Event FIFO full: discard its oldest event. Periodic measurements
         * cannot cause this path, so sensor-rate traffic cannot be the cause. */
        if (TELE_QUEUE_SLOTS == g_count)
        {
            g_tail = (g_tail + 1u) & TELE_QUEUE_MASK;
            g_count--;
            g_dropped++;
            queued_cleanly = false;
        }

        slot = &g_queue[g_head];
        build_message(slot, which, body, is_snapshot);
        g_head = (g_head + 1u) & TELE_QUEUE_MASK;
        g_count++;
    }

    tele_unlock();

    return queued_cleanly;
}

/*
 * Build one message and put it in the queue.
 *
 * Returns true if it was queued cleanly, false if an older message had to
 * be thrown away to make room.
 *
 * THREAD SAFETY: the latest-value cache and queue are protected separately by
 * the platform hooks. No lock is held while formatting the body or while one
 * protected operation calls another, so the port need not be recursive.
 */
static bool tele_enqueue(tele_topic_t which, const char * body_fmt, ...)
{
    char    body[TELE_BODY_MAX];
    va_list args;

    if (((unsigned)which >= (unsigned)TT_COUNT) || (NULL == body_fmt))
    {
        return false;
    }

    /* Format the caller's fields. vsnprintf always NUL-terminates and never
       writes past the buffer, so a body that is too long is truncated
       rather than becoming a security problem. Every caller below bounds
       its own arguments, so truncation should never actually happen. */
    va_start(args, body_fmt);
    (void)vsnprintf(body, sizeof body, body_fmt, args);
    va_end(args);

    /*
     * Remember it for the snapshot, EXCEPT for the log topic.
     *
     * A log line is an event, not a state. Replaying an old one in a
     * snapshot would tell the reader something happened now when it did not.
     */
    if (TT_LOG != which)
    {
        tele_lock();
        (void)snprintf(g_latest[which], TELE_BODY_MAX, "%.*s",
                       (int)(TELE_BODY_MAX - 1u), body);
        g_has_latest[which] = true;
        tele_unlock();
    }

    return enqueue_body(which, body, false);
}

uint32_t tele_publish_snapshot(void)
{
    unsigned i;
    uint32_t queued = 0u;

    /*
     * Re-publish the latest value of every topic that has one, each on its
     * own topic rather than bundled into a single giant message.
     *
     * Why not one big message: the combined state is roughly 500 characters,
     * which would need a payload buffer nearly three times the size of every
     * other message, used once per run. Replaying them individually reuses
     * the queue, the formatting and the topics exactly as they already are,
     * and the subscriber's existing handling of each topic just works.
     */
    for (i = 0u; i < (unsigned)TT_COUNT; i++)
    {
        char copy[TELE_BODY_MAX];
        bool present;

        /* Copy the cached body out under the lock, then release it BEFORE
           calling enqueue_body() - which takes the same lock. Holding it
           across that call would deadlock on any non-recursive mutex, and
           requiring a recursive one would be a trap for whoever writes the
           RTOS port. */
        tele_lock();
        present = g_has_latest[i];
        if (present)
        {
            /* Explicit precision again: the compiler sees g_latest as one
               flat block and cannot tell that each row is independently
               NUL-bounded, so state the bound rather than rely on it. */
            (void)snprintf(copy, sizeof copy, "%.*s",
                           (int)(TELE_BODY_MAX - 1u), g_latest[i]);
        }
        tele_unlock();

        if (present)
        {
            (void)enqueue_body((tele_topic_t)i, copy, true);
            queued++;
        }
    }

    return queued;
}

/* -------------------------------------------------------------------------
 * The reporting API
 *
 * Each of these does exactly one thing: name a topic and name its fields.
 * Units are part of every field name (peak_mm, not peak) so that no two
 * subsystems can ever disagree about whether a number is millimetres or
 * centimetres. That bug is invisible until the demo.
 *
 * All values are integers. The Cortex-M0+ has no floating-point hardware,
 * so printing a float pulls in a large, slow software library for no gain.
 * ------------------------------------------------------------------------- */

bool tele_report_state(tele_state_t state)
{
    return tele_enqueue(TT_STATE, "\"state\":\"%s\"", tele_state_name(state));
}

bool tele_report_motion(int16_t left_mms,
                        int16_t right_mms,
                        int32_t dist_mm,
                        uint32_t left_encoder,
                        uint32_t right_encoder)
{
    /* Per-wheel, because the difference between the two IS the thing the PID
       is controlling. Raw encoder counts travel too - the write-up p.5 lists
       them by name, and they are what lets anyone verify dist_mm rather than
       take it on trust. */
    return tele_enqueue(TT_MOTION,
                        "\"spd_l_mms\":%d,\"spd_r_mms\":%d,\"dist_mm\":%" PRId32
                        ",\"enc_l\":%" PRIu32 ",\"enc_r\":%" PRIu32,
                        (int)left_mms, (int)right_mms, dist_mm,
                        left_encoder, right_encoder);
}

bool tele_report_line(int8_t pos,
                      bool at_junction,
                      uint16_t left_raw,
                      uint16_t right_raw)
{
    /* JSON has real booleans, so emit true/false rather than 1/0 - it reads
       better in mosquitto_sub, which is what the brief asks for.
       The raw ADC readings travel so IR calibration can be checked from the
       telemetry instead of needing a serial cable to the moving car. */
    return tele_enqueue(TT_LINE,
                        "\"pos\":%d,\"junction\":%s,\"ir_l\":%u,\"ir_r\":%u",
                        (int)pos,
                        at_junction ? "true" : "false",
                        (unsigned)left_raw,
                        (unsigned)right_raw);
}

bool tele_report_barcode(char decoded_char,
                         nav_action_t action,
                         bool was_reversed)
{
    /* %c would happily emit a quote or a backslash and produce broken JSON
       if the decoder ever handed us a stray byte. Reject anything that is
       not a plain capital letter. */
    char safe = ((decoded_char >= 'A') && (decoded_char <= 'Z')) ? decoded_char : '?';

    /* was_reversed is the evidence that bidirectional decoding - which the
       brief requires - actually happened, rather than being claimed. */
    return tele_enqueue(TT_BARCODE,
                        "\"char\":\"%c\",\"action\":\"%s\",\"reversed\":%s",
                        safe,
                        nav_action_name(action),
                        was_reversed ? "true" : "false");
}

bool tele_report_hump(uint16_t peak_mm,
                      uint16_t max_peak_mm,
                      uint16_t hump_count)
{
    /* max_mm carries mission requirement 5 - "measure and report the highest
       hump peak experienced during the run". The robot tracks it, so the
       requirement is met by the robot rather than by whoever is reading the
       dashboard. */
    return tele_enqueue(TT_HUMP,
                        "\"peak_mm\":%u,\"max_mm\":%u,\"count\":%u",
                        (unsigned)peak_mm,
                        (unsigned)max_peak_mm,
                        (unsigned)hump_count);
}

bool tele_report_imu(uint16_t heading_dg,
                     int16_t pitch_ddg,
                     int16_t turn_rate_dps,
                     imu_event_t event)
{
    /* pitch is in DECI-degrees so tilt keeps a decimal place with no float:
       -125 means -12.5 degrees. The Cortex-M0+ has no FPU, so this is not a
       micro-optimisation, it is the difference between an integer store and
       a call into a software floating-point library. */
    return tele_enqueue(TT_IMU,
                        "\"heading_dg\":%u,\"pitch_ddg\":%d,"
                        "\"turn_dps\":%d,\"event\":\"%s\"",
                        (unsigned)heading_dg,
                        (int)pitch_ddg,
                        (int)turn_rate_dps,
                        imu_event_name(event));
}

bool tele_report_obstacle(int16_t angle_dg,
                          uint16_t closest_mm,
                          uint16_t width_mm,
                          uint16_t clear_left_mm,
                          uint16_t clear_right_mm,
                          avoid_plan_t plan)
{
    /* All five things the write-up p.9 Stage 3 asks a profile to estimate -
       location (as a bearing), width, closest point, and both clearances -
       plus the Stage 4 decision, so the telemetry records not only what the
       car saw but what it chose to do about it. */
    return tele_enqueue(TT_OBSTACLE,
                        "\"angle_dg\":%d,\"closest_mm\":%u,\"width_mm\":%u,"
                        "\"clear_l_mm\":%u,\"clear_r_mm\":%u,\"plan\":\"%s\"",
                        (int)angle_dg,
                        (unsigned)closest_mm,
                        (unsigned)width_mm,
                        (unsigned)clear_left_mm,
                        (unsigned)clear_right_mm,
                        avoid_plan_name(plan));
}

bool tele_report_log(const char * text)
{
    char safe[TELE_LOG_ESCAPED_MAX];

    if (NULL == text)
    {
        text = "";
    }

    /* Escape first, then format. Doing it the other way round would mean
       building broken JSON and hoping to repair it afterwards. The escaper
       caps the input at TELE_LOG_TEXT_MAX characters, so the result always
       fits inside TELE_BODY_MAX and can never be truncated mid-string. */
    (void)json_escape(safe, sizeof safe, text, TELE_LOG_TEXT_MAX);

    return tele_enqueue(TT_LOG, "\"msg\":\"%s\"", safe);
}

/* -------------------------------------------------------------------------
 * The draining API
 * ------------------------------------------------------------------------- */

bool tele_queue_pop(tele_msg_t * out)
{
    bool got = false;

    if (NULL == out)
    {
        return false;
    }

    tele_lock();

    if (0u != g_count)
    {
        *out   = g_queue[g_tail];
        g_tail = (g_tail + 1u) & TELE_QUEUE_MASK;
        g_count--;
        got    = true;
    }
    else if (0u != g_periodic_count)
    {
        unsigned i;
        unsigned oldest = (unsigned)PT_COUNT;
        uint32_t oldest_order = 0u;
        bool     found = false;

        for (i = 0u; i < (unsigned)PT_COUNT; i++)
        {
            if (g_periodic_pending[i] &&
                (!found || ((int32_t)(g_periodic_order[i] - oldest_order) < 0)))
            {
                oldest       = i;
                oldest_order = g_periodic_order[i];
                found        = true;
            }
        }

        if (oldest < (unsigned)PT_COUNT)
        {
            *out = g_periodic[oldest];
            g_periodic_pending[oldest] = false;
            g_periodic_count--;
            got = true;
        }
    }

    tele_unlock();

    return got;
}

uint32_t tele_queue_depth(void)
{
    uint32_t value;
    tele_lock();
    value = g_count + g_periodic_count;
    tele_unlock();
    return value;
}

uint32_t tele_dropped_count(void)
{
    uint32_t value;
    tele_lock();
    value = g_dropped;
    tele_unlock();
    return value;
}

uint32_t tele_coalesced_count(void)
{
    uint32_t value;
    tele_lock();
    value = g_coalesced;
    tele_unlock();
    return value;
}

uint32_t tele_next_seq(void)
{
    uint32_t value;
    tele_lock();
    value = g_seq;
    tele_unlock();
    return value;
}

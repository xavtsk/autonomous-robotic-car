/*
 * host_test.c - runs the telemetry and command logic on your LAPTOP.
 *
 * No Pico, no WiFi, no broker needed. Build and run with:
 *
 *     cd tests/buddy1 && make test
 *
 * Why this exists: nearly every bug in this module is a formatting or
 * arithmetic mistake, and those are far cheaper to find in a one-second
 * laptop run than by flashing a board and squinting at a serial monitor.
 * This is the same host/target split LAB6 uses (optimise_host.c) and the
 * reason Bug Hunt #2 says the laptop finds 5 of its 7 defects.
 *
 * The tests are GOLDEN VECTORS: each one states the exact string we expect.
 * A test that only checks "it produced something" would not have caught a
 * single one of the defects these were written to prevent.
 */

#include "buddy1/telemetry.h"
#include "buddy1/command_receiver.h"

#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * The fake clock
 *
 * telemetry.c declares tele_now_ms() but does not implement it. On the Pico
 * it will return to_ms_since_boot(). Here it returns a number we control,
 * so the timestamps in our expected strings are predictable.
 * ------------------------------------------------------------------------- */

static uint32_t g_fake_ms = 0u;

uint32_t tele_now_ms(void)
{
    return g_fake_ms;
}

/* The tests are single-threaded, so there is nothing to exclude. Present so
   telemetry.c links, and so the lock calls are exercised by every test. */
void tele_lock(void)   { }
void tele_unlock(void) { }

/* -------------------------------------------------------------------------
 * Tiny test harness
 * ------------------------------------------------------------------------- */

static unsigned g_checks = 0u;
static unsigned g_failures = 0u;

static nav_action_t map_action(const nav_action_t map[ROBOT_BARCODE_COUNT],
                               char letter)
{
    if ((NULL == map) || (letter < 'A') || (letter > 'Z'))
    {
        return NAV_NONE;
    }
    return map[(unsigned)(letter - 'A')];
}

static void check(bool condition, const char * what)
{
    g_checks++;
    if (condition)
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        printf("  FAIL  %s\n", what);
        g_failures++;
    }
}

static void check_str(const char * got, const char * want, const char * what)
{
    g_checks++;
    if (0 == strcmp(got, want))
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        printf("  FAIL  %s\n", what);
        printf("        want: %s\n", want);
        printf("        got : %s\n", got);
        g_failures++;
    }
}

/* -------------------------------------------------------------------------
 * Telemetry tests
 * ------------------------------------------------------------------------- */

static void test_payload_format(void)
{
    tele_msg_t msg;

    printf("\npayload format\n");

    tele_init();
    g_fake_ms = 128450u;

    (void)tele_report_hump(43u, 51u, 2u);
    check(tele_queue_pop(&msg), "a reported hump can be popped");
    check_str(msg.topic, TELE_TOPIC_ROOT "/telemetry/hump", "hump topic");
    check_str(msg.payload,
              "{\"t\":128450,\"seq\":0,\"peak_mm\":43,\"max_mm\":51,\"count\":2}",
              "hump payload");

    g_fake_ms = 128500u;
    (void)tele_report_motion(220, 218, 1840, 4821u, 4805u);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":128500,\"seq\":1,\"spd_l_mms\":220,\"spd_r_mms\":218,"
              "\"dist_mm\":1840,\"enc_l\":4821,\"enc_r\":4805}",
              "motion payload: per-wheel speeds AND raw encoder counts");

    (void)tele_report_line(-3, false, 2100u, 1980u);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":128500,\"seq\":2,\"pos\":-3,\"junction\":false,"
              "\"ir_l\":2100,\"ir_r\":1980}",
              "line payload: position, junction flag and raw IR readings");

    (void)tele_report_state(TELE_STATE_LINE_FOLLOW);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":128500,\"seq\":3,\"state\":\"LINE_FOLLOW\"}",
              "state payload");

    (void)tele_report_barcode('C', NAV_RIGHT, true);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":128500,\"seq\":4,\"char\":\"C\",\"action\":\"RIGHT\","
              "\"reversed\":true}",
              "barcode payload, including the bidirectional-decode evidence");

    (void)tele_report_imu(90, -125, 42, IMU_EVENT_CLIMBING);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":128500,\"seq\":5,\"heading_dg\":90,\"pitch_ddg\":-125,"
              "\"turn_dps\":42,\"event\":\"CLIMBING\"}",
              "imu payload: pitch in deci-degrees means -12.5 needs no float");

    (void)tele_report_obstacle(15, 250u, 120u, 300u, 90u, AVOID_LEFT);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":128500,\"seq\":6,\"angle_dg\":15,\"closest_mm\":250,"
              "\"width_mm\":120,\"clear_l_mm\":300,\"clear_r_mm\":90,"
              "\"plan\":\"LEFT\"}",
              "obstacle payload: all five profile fields plus the plan");
}

/*
 * TELE_PAYLOAD_MAX was sized by working out the worst case on paper. This
 * test checks the arithmetic was right, because if it was not, the failure
 * mode is a truncated payload with no closing brace - valid-looking output
 * that the dashboard silently rejects.
 */
static void test_worst_case_payload_fits(void)
{
    tele_msg_t msg;
    size_t     len;

    printf("\nthe largest possible message still fits\n");

    tele_init();
    g_fake_ms = 4294967295u;   /* largest timestamp */

    (void)tele_report_obstacle(-32768, 65535u, 65535u, 65535u, 65535u,
                               AVOID_CONTINUE);
    (void)tele_queue_pop(&msg);

    len = strlen(msg.payload);
    printf("        worst-case obstacle payload is %u of %u bytes\n",
           (unsigned)len, (unsigned)TELE_PAYLOAD_MAX);
    check(len < TELE_PAYLOAD_MAX, "worst-case obstacle payload is not truncated");
    check('}' == msg.payload[len - 1u], "worst-case obstacle payload closes its brace");

    (void)tele_report_motion(-32768, -32768, -2147483647 - 1, 4294967295u, 4294967295u);
    (void)tele_queue_pop(&msg);
    len = strlen(msg.payload);
    printf("        worst-case motion payload is %u of %u bytes\n",
           (unsigned)len, (unsigned)TELE_PAYLOAD_MAX);
    check(len < TELE_PAYLOAD_MAX, "worst-case motion payload is not truncated");
    check('}' == msg.payload[len - 1u], "worst-case motion payload closes its brace");
}

static void test_bad_input_cannot_break_json(void)
{
    tele_msg_t msg;
    char       huge[300];
    size_t     len;

    printf("\nmalformed input still produces valid JSON\n");

    tele_init();
    g_fake_ms = 1000u;

    /* A quote character from a misbehaving decoder must not escape into the
       payload and break the JSON. */
    (void)tele_report_barcode('"', NAV_LEFT, false);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":1000,\"seq\":0,\"char\":\"?\",\"action\":\"LEFT\",\"reversed\":false}",
              "a non-letter barcode char becomes '?'");

    /* An over-long log message must be cut at a safe point, not mid-string. */
    memset(huge, 'x', sizeof huge - 1u);
    huge[sizeof huge - 1u] = '\0';

    (void)tele_report_log(huge);
    (void)tele_queue_pop(&msg);
    len = strlen(msg.payload);
    check(len < TELE_PAYLOAD_MAX, "over-long log fits the buffer");
    check((len > 0u) && ('}' == msg.payload[len - 1u]),
          "over-long log still ends with a closing brace");

    (void)tele_report_log(NULL);
    (void)tele_queue_pop(&msg);
    check_str(msg.payload, "{\"t\":1000,\"seq\":2,\"msg\":\"\"}",
              "a NULL log message does not crash");
}

static void test_queue_fifo(void)
{
    tele_msg_t msg;
    unsigned   i;

    printf("\nqueue keeps order\n");

    tele_init();
    g_fake_ms = 0u;

    for (i = 0u; i < 3u; i++)
    {
        (void)tele_report_hump((uint16_t)(i + 1u), 99u, (uint16_t)i);
    }

    check(3u == tele_queue_depth(), "three messages are waiting");

    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"seq\":0"), "first out is the first in");
    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"seq\":1"), "then the second");
    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"seq\":2"), "then the third");

    check(0u == tele_queue_depth(), "queue is empty again");
    check(!tele_queue_pop(&msg), "popping an empty queue returns false");
}

static void test_queue_overflow_drops_oldest(void)
{
    tele_msg_t msg;
    unsigned   i;
    bool       clean;

    printf("\nqueue overflow discards the OLDEST message\n");

    tele_init();
    g_fake_ms = 0u;

    /* Fill it exactly. One aggregated check, so sixteen identical lines of
       "ok" do not bury the interesting results below. */
    bool all_clean = true;
    for (i = 0u; i < TELE_QUEUE_SLOTS; i++)
    {
        clean = tele_report_hump((uint16_t)i, 0u, 0u);
        if (!clean)
        {
            all_clean = false;
        }
    }
    check(all_clean, "all 16 messages queued cleanly while there was room");

    check(TELE_QUEUE_SLOTS == tele_queue_depth(), "queue is full");
    check(0u == tele_dropped_count(), "nothing dropped yet");

    /* One more. This must push out seq 0, not refuse the new message. */
    clean = tele_report_hump(999u, 0u, 0u);
    check(!clean, "the overflowing call reports that something was dropped");
    check(1u == tele_dropped_count(), "the drop counter moved");
    check(TELE_QUEUE_SLOTS == tele_queue_depth(), "queue is still full, not overfull");

    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"seq\":1"),
          "seq 0 was discarded, so the oldest survivor is seq 1");

    /* Drain, and confirm the newest message really is still in there. */
    while (tele_queue_pop(&msg))
    {
        /* keep the last one */
    }
    check(NULL != strstr(msg.payload, "\"peak_mm\":999"),
          "the newest message survived the overflow");
}

static void test_periodic_telemetry_cannot_evict_events(void)
{
    tele_msg_t msg;
    unsigned   i;

    printf("\nperiodic telemetry is coalesced behind important events\n");

    tele_init();
    g_fake_ms = 100u;

    for (i = 0u; i < 50u; i++)
    {
        (void)tele_report_motion((int16_t)i, (int16_t)i,
                                 (int32_t)i, i, i);
    }

    check(1u == tele_queue_depth(),
          "fifty unsent motion samples occupy one latest-value slot");
    check(49u == tele_coalesced_count(),
          "stale periodic samples are counted as coalesced");
    check(0u == tele_dropped_count(),
          "coalescing periodic samples loses no important event");

    (void)tele_report_barcode('W', NAV_LEFT, false);
    check(2u == tele_queue_depth(), "the barcode event is retained separately");

    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"char\":\"W\""),
          "important event is sent before periodic telemetry");

    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"spd_l_mms\":49"),
          "the surviving motion sample is the newest one");
}

/* -------------------------------------------------------------------------
 * Command tests
 * ------------------------------------------------------------------------- */

static void test_topic_routing(void)
{
    printf("\ncommand topics are recognised\n");

    check(CMD_BARCODE_MAP == cmd_kind_from_topic("grp07/car/cmd/barcode_map"),
          "barcode_map");
    check(CMD_SNAPSHOT == cmd_kind_from_topic("grp07/car/cmd/snapshot"),
          "snapshot");
    check(CMD_UNKNOWN == cmd_kind_from_topic("grp07/car/cmd/stop"),
          "remote stop is not an assessed MQTT command");
    check(CMD_UNKNOWN == cmd_kind_from_topic("grp07/car/cmd/resume"),
          "remote resume is not an assessed MQTT command");

    check(CMD_SNAPSHOT == cmd_kind_from_topic("anything/at/all/cmd/snapshot"),
          "works whatever the group prefix is");

    check(CMD_UNKNOWN == cmd_kind_from_topic("grp07/car/cmd/wibble"),
          "an unknown command under cmd/ is reported, not ignored");

    check(CMD_NONE == cmd_kind_from_topic("grp07/car/telemetry/stop"),
          "a matching name outside cmd/ is NOT treated as a command");
    check(CMD_NONE == cmd_kind_from_topic("stop"), "a bare word is not a command");
    check(CMD_NONE == cmd_kind_from_topic(NULL), "NULL topic does not crash");
}

static void test_barcode_map_parsing(void)
{
    robot_config_t config;
    nav_action_t   map[ROBOT_BARCODE_COUNT];

    printf("\nbarcode remap parsing\n");

    robot_config_init(&config);
    check(NAV_STRAIGHT == robot_config_action_for_letter(&config, 'A'),
          "default A is STRAIGHT");
    check(NAV_UTURN == robot_config_action_for_letter(&config, 'D'),
          "default D is UTURN");
    check(NAV_NONE == robot_config_action_for_letter(&config, 'Z'),
          "default Z is unassigned");

    /* The demo-day case: the whole mapping is replaced over the air. */
    check(cmd_parse_barcode_map(
              "{\"D\":\"LEFT\",\"C\":\"RIGHT\",\"W\":\"STRAIGHT\",\"F\":\"UTURN\"}",
              map),
          "a well-formed remap is accepted");
    check(NAV_LEFT == map_action(map, 'D'), "D now means LEFT");
    check(NAV_RIGHT == map_action(map, 'C'), "C now means RIGHT");
    check(NAV_STRAIGHT == map_action(map, 'W'), "W now means STRAIGHT");
    check(NAV_NONE == map_action(map, 'A'),
          "A is unassigned now, because the payload did not mention it");

    check(cmd_parse_barcode_map("  { \"Q\" : \"LEFT\" }  ", map),
          "whitespace around the pairs is tolerated");
    check(NAV_LEFT == map_action(map, 'Q'), "Q parsed despite the spaces");

    check(NAV_NONE == map_action(map, 'a'),
          "lowercase letters are not valid barcode keys");
    check(NAV_NONE == map_action(map, '1'), "digits are not either");
}

static void test_bad_remap_leaves_the_old_map_alone(void)
{
    robot_config_t config;
    nav_action_t   map[ROBOT_BARCODE_COUNT];

    printf("\na malformed remap must NOT be half-applied\n");

    robot_config_init(&config);
    memcpy(map, config.barcode_map, sizeof map);

    /* Second pair is broken. The first pair must not survive. */
    check(!cmd_parse_barcode_map("{\"A\":\"LEFT\",\"B\":\"SIDEWAYS\"}", map),
          "an unknown action name is rejected");
    check(NAV_STRAIGHT == map_action(map, 'A'),
          "A still has its ORIGINAL meaning, not the half-applied one");

    check(!cmd_parse_barcode_map("{\"AB\":\"LEFT\"}", map),
          "a two-character key is rejected");
    check(!cmd_parse_barcode_map("{\"A\":\"LEFT\"", map),
          "a truncated payload with no closing brace is rejected");
    check(!cmd_parse_barcode_map("{\"A\":\"LEF", map),
          "a value with no closing quote is rejected");
    check(!cmd_parse_barcode_map("{\"A\" \"LEFT\"}", map),
          "a missing colon is rejected");
    check(!cmd_parse_barcode_map("{\"A\":\"LEFT\" \"B\":\"RIGHT\"}", map),
          "a missing comma between pairs is rejected");
    check(!cmd_parse_barcode_map("{\"A\":\"LEFT\",}", map),
          "a trailing comma is rejected");
    check(!cmd_parse_barcode_map("{\"A\":\"LEFT\"} rubbish", map),
          "junk after the closing brace is rejected");
    check(!cmd_parse_barcode_map("{}", map), "an empty map is rejected");
    check(!cmd_parse_barcode_map("", map), "an empty payload is rejected");
    check(!cmd_parse_barcode_map(NULL, map), "a NULL payload does not crash");

    check(NAV_STRAIGHT == map_action(map, 'A'),
          "after all that, the original map is intact");
}


/* -------------------------------------------------------------------------
 * A structural JSON checker
 *
 * Every golden-vector test above checks one exact string. This checks a
 * PROPERTY that must hold for every message we will ever emit: that it is
 * structurally valid JSON. Applying it to messages built from hostile inputs
 * catches whole classes of defect that no fixed expected-string could,
 * because you cannot write a golden vector for a bug you did not predict.
 * ------------------------------------------------------------------------- */

static bool json_is_well_formed(const char * s)
{
    int  depth     = 0;
    bool in_string = false;
    bool escaped   = false;
    size_t i;

    if ('{' != s[0])
    {
        return false;
    }

    for (i = 0u; '\0' != s[i]; i++)
    {
        char c = s[i];

        if (in_string)
        {
            if (escaped)
            {
                /* Only these may follow a backslash in JSON. */
                if (('"' != c) && ('\\' != c) && ('/' != c) && ('b' != c) &&
                    ('f' != c) && ('n' != c) && ('r' != c) && ('t' != c) &&
                    ('u' != c))
                {
                    return false;
                }
                escaped = false;
            }
            else if ('\\' == c)   { escaped = true; }
            else if ('"' == c)    { in_string = false; }
            else if ((unsigned char)c < 0x20u)
            {
                return false;     /* raw control char inside a string */
            }
        }
        else
        {
            if      ('"' == c) { in_string = true; }
            else if ('{' == c) { depth++; }
            else if ('}' == c) { depth--; if (depth < 0) { return false; } }
        }
    }

    return ((0 == depth) && !in_string && !escaped);
}

static void check_json(const char * payload, const char * what)
{
    g_checks++;
    if (json_is_well_formed(payload))
    {
        printf("  ok    %s\n", what);
    }
    else
    {
        printf("  FAIL  %s\n", what);
        printf("        not valid JSON: %s\n", payload);
        g_failures++;
    }
}

static void test_free_text_cannot_break_json(void)
{
    tele_msg_t msg;
    char       nasty[256];
    size_t     i;

    printf("\nfree text is escaped, not trusted\n");

    tele_init();
    g_fake_ms = 1000u;

    (void)tele_report_log("sensor said \"hi\"");
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":1000,\"seq\":0,\"msg\":\"sensor said \\\"hi\\\"\"}",
              "a quote is escaped, not emitted raw");
    check_json(msg.payload, "  and the result is valid JSON");

    (void)tele_report_log("path C:\\temp");
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":1000,\"seq\":1,\"msg\":\"path C:\\\\temp\"}",
              "a backslash is doubled - otherwise \\t would mean TAB");
    check_json(msg.payload, "  and the result is valid JSON");

    (void)tele_report_log("line1\nline2\tend");
    (void)tele_queue_pop(&msg);
    check_str(msg.payload,
              "{\"t\":1000,\"seq\":2,\"msg\":\"line1 line2 end\"}",
              "control characters become spaces");
    check_json(msg.payload, "  and the result is valid JSON");

    /* Worst case for the escaper: every character needs two bytes out. */
    for (i = 0u; i < 200u; i++) { nasty[i] = '"'; }
    nasty[200] = '\0';

    (void)tele_report_log(nasty);
    (void)tele_queue_pop(&msg);
    check(strlen(msg.payload) < TELE_PAYLOAD_MAX,
          "200 quotes still fits the payload buffer");
    check_json(msg.payload, "200 quotes still produces valid JSON");

    /* Every byte value a caller could possibly pass. */
    for (i = 0u; i < 255u; i++) { nasty[i] = (char)(i + 1u); }
    nasty[255] = '\0';

    (void)tele_report_log(nasty);
    (void)tele_queue_pop(&msg);
    check_json(msg.payload, "every byte value 1..255 produces valid JSON");
}

static void test_every_message_is_valid_json(void)
{
    tele_msg_t msg;

    printf("\nevery message type is structurally valid JSON\n");

    tele_init();
    g_fake_ms = 4294967295u;

    (void)tele_report_state(TELE_STATE_BYPASS);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "state");

    (void)tele_report_motion(-32768, 32767, -2147483647 - 1, 4294967295u, 0u);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "motion at type extremes");

    (void)tele_report_line(-128, true, 4095u, 0u);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "line at type extremes");

    (void)tele_report_barcode('Z', NAV_UTURN, true);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "barcode");

    (void)tele_report_hump(65535u, 65535u, 65535u);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "hump at type extremes");

    (void)tele_report_imu(359u, -900, -32768, IMU_EVENT_IMPACT);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "imu at type extremes");

    (void)tele_report_obstacle(-32768, 65535u, 65535u, 65535u, 65535u, AVOID_REVERSE);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "obstacle at type extremes");

    /* Out-of-range enums must not read past the name tables. */
    (void)tele_report_state((tele_state_t)999);
    (void)tele_queue_pop(&msg); check_json(msg.payload, "state with a bogus enum");
    check(NULL != strstr(msg.payload, "INVALID"),
          "  a bogus enum prints INVALID rather than reading past the table");
}

static void test_name_tables_are_bounded(void)
{
    printf("\nname lookups reject out-of-range values\n");

    check(0 == strcmp("INVALID", tele_state_name((tele_state_t)TELE_STATE_COUNT)),
          "tele_state_name rejects COUNT");
    check(0 == strcmp("INVALID", tele_state_name((tele_state_t)12345)),
          "tele_state_name rejects a wild value");
    check(0 == strcmp("INVALID", nav_action_name((nav_action_t)NAV_COUNT)),
          "nav_action_name rejects COUNT");
    check(0 == strcmp("INVALID", imu_event_name((imu_event_t)IMU_EVENT_COUNT)),
          "imu_event_name rejects COUNT");
    check(0 == strcmp("INVALID", avoid_plan_name((avoid_plan_t)AVOID_COUNT)),
          "avoid_plan_name rejects COUNT");

    check(0 == strcmp("LINE_FOLLOW", tele_state_name(TELE_STATE_LINE_FOLLOW)),
          "a valid state still resolves");
}

static void test_action_name_round_trip(void)
{
    unsigned i;
    bool     all_ok = true;

    printf("\naction names survive a round trip\n");

    /* Every action must convert to a name and back to itself, or the
       barcode remap command and the telemetry would disagree about what a
       letter means. Start at 1 - index 0 is NAV_NONE, which is the failure
       value rather than an action. */
    for (i = 1u; i < (unsigned)NAV_COUNT; i++)
    {
        nav_action_t a = (nav_action_t)i;
        if (a != nav_action_from_name(nav_action_name(a)))
        {
            all_ok = false;
        }
    }
    check(all_ok, "every action name parses back to the same action");

    check(NAV_NONE == nav_action_from_name("SIDEWAYS"), "an unknown name is NAV_NONE");
    check(NAV_NONE == nav_action_from_name("left"), "names are case-sensitive");
    check(NAV_NONE == nav_action_from_name(""), "an empty name is NAV_NONE");
    check(NAV_NONE == nav_action_from_name(NULL), "a NULL name does not crash");
}

static void test_queue_survives_many_wraps(void)
{
    tele_msg_t msg;
    unsigned   i;
    const unsigned TOTAL = 500u;
    unsigned   popped = 0u;
    bool       seq_ok = true;
    uint32_t   expect_first;

    printf("\nqueue stays correct across many wraps\n");

    tele_init();
    g_fake_ms = 0u;

    /* Push far more than the queue can hold, so the indices wrap many times
       over. A mask that was subtly wrong would corrupt long before 500. */
    for (i = 0u; i < TOTAL; i++)
    {
        (void)tele_report_hump((uint16_t)(i & 0xFFFFu), 0u, 0u);
    }

    check(TELE_QUEUE_SLOTS == tele_queue_depth(),
          "queue holds exactly its capacity, never more");
    check((TOTAL - TELE_QUEUE_SLOTS) == tele_dropped_count(),
          "dropped count is exactly pushes minus capacity");
    check(TOTAL == tele_next_seq(),
          "sequence counted every message, including the dropped ones");

    /* The survivors must be the LAST 16, in order. */
    expect_first = TOTAL - TELE_QUEUE_SLOTS;
    while (tele_queue_pop(&msg))
    {
        char want[32];
        (void)snprintf(want, sizeof want, "\"seq\":%lu",
                       (unsigned long)(expect_first + popped));
        if (NULL == strstr(msg.payload, want))
        {
            seq_ok = false;
        }
        popped++;
    }

    check(TELE_QUEUE_SLOTS == popped, "exactly capacity messages came back out");
    check(seq_ok, "the survivors are the newest 16, still in order");
}

static void test_pop_guards(void)
{
    printf("\npop guards\n");

    tele_init();
    check(!tele_queue_pop(NULL), "popping into NULL returns false, does not crash");

    (void)tele_report_log("x");
    check(!tele_queue_pop(NULL), "still refuses NULL when a message is waiting");
    check(1u == tele_queue_depth(), "  and the message is still queued");
}

static void test_barcode_map_hard_cases(void)
{
    robot_config_t config;
    nav_action_t   map[ROBOT_BARCODE_COUNT];
    char         all26[512];
    size_t       used = 0u;
    unsigned     i;
    bool         all_ok = true;

    printf("\nbarcode remap, harder cases\n");

    /* The realistic worst case: the lecturer assigns all 26 letters. */
    used += (size_t)snprintf(all26 + used, sizeof all26 - used, "{");
    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        used += (size_t)snprintf(all26 + used, sizeof all26 - used,
                                 "%s\"%c\":\"STRAIGHT\"",
                                 (0u == i) ? "" : ",", (char)('A' + i));
    }
    used += (size_t)snprintf(all26 + used, sizeof all26 - used, "}");

    check(cmd_parse_barcode_map(all26, map), "all 26 letters in one payload parse");
    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        if (NAV_STRAIGHT != map_action(map, (char)('A' + i)))
        {
            all_ok = false;
        }
    }
    check(all_ok, "  and every one of the 26 took effect");
    printf("        payload was %u bytes\n", (unsigned)used);

    /* A duplicated key is not malformed JSON; last one wins, like every
       other JSON parser. Worth pinning down so it cannot change silently. */
    robot_config_init(&config);
    memcpy(map, config.barcode_map, sizeof map);
    check(cmd_parse_barcode_map("{\"A\":\"LEFT\",\"A\":\"RIGHT\"}", map),
          "a duplicated key is accepted");
    check(NAV_RIGHT == map_action(map, 'A'), "  and the last one wins");

    /* An action name longer than any real one must be rejected without
       overrunning the fixed-size name buffer. */
    robot_config_init(&config);
    memcpy(map, config.barcode_map, sizeof map);
    check(!cmd_parse_barcode_map("{\"A\":\"STRAIGHTAWAY\"}", map),
          "an over-long action name is rejected, not truncated into a match");
    check(NAV_STRAIGHT == map_action(map, 'A'),
          "  and the old map survived");

    /* Exactly at the boundary: STRAIGHT is the longest legal name. */
    check(cmd_parse_barcode_map("{\"B\":\"STRAIGHT\"}", map),
          "the longest legal action name still parses");

    check(!cmd_parse_barcode_map("{\"A\":\"\"}", map),
          "an empty action name is rejected");
    check(!cmd_parse_barcode_map("{\"\":\"LEFT\"}", map),
          "an empty key is rejected");
    check(!cmd_parse_barcode_map("[\"A\",\"LEFT\"]", map),
          "a JSON array is not an object and is rejected");
}

static void test_shared_configuration_lifecycle(void)
{
    robot_config_t config;
    nav_action_t   map[ROBOT_BARCODE_COUNT];
    char           reply[192];
    unsigned       i;

    printf("\nshared configuration is frozen for the autonomous run\n");

    robot_config_init(&config);
    check(RUN_PHASE_CONFIGURING == config.phase,
          "startup accepts pre-run configuration");
    check(0u == config.version, "development defaults are version zero");

    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        map[i] = NAV_NONE;
    }
    map[(unsigned)('W' - 'A')] = NAV_LEFT;
    map[(unsigned)('F' - 'A')] = NAV_RIGHT;

    check(ROBOT_CONFIG_APPLIED ==
              robot_config_apply_barcode_map(&config, map),
          "a valid map is accepted while configuring");
    check(1u == config.version, "accepted configuration increments its version");
    check(NAV_LEFT == robot_config_action_for_letter(&config, 'W'),
          "navigation reads the shared accepted map");
    check(2u == robot_config_assigned_count(&config),
          "the shared configuration reports assigned entries");
    check(cmd_format_barcode_map_reply(reply, sizeof reply, 1234u, &config,
                                       CMD_MAP_REPLY_ACCEPTED),
          "an accepted map gets a structured acknowledgement");
    check_str(reply,
              "{\"t\":1234,\"accepted\":true,\"version\":1,\"entries\":2,"
              "\"phase\":\"CONFIGURING\"}",
              "the acknowledgement identifies version, count and phase");
    check_json(reply, "the accepted acknowledgement is valid JSON");

    check(robot_config_set_phase(&config, RUN_PHASE_ARMED),
          "configuration can be armed");
    check(ROBOT_CONFIG_LOCKED ==
              robot_config_apply_barcode_map(&config, map),
          "an armed robot rejects remapping");
    check(cmd_format_barcode_map_reply(reply, sizeof reply, 1300u, &config,
                                       CMD_MAP_REPLY_LOCKED),
          "a locked map request gets a structured rejection");
    check(NULL != strstr(reply, "\"reason\":\"configuration_locked\""),
          "the rejection explains why configuration was refused");
    check_json(reply, "the locked acknowledgement is valid JSON");
    check(robot_config_set_phase(&config, RUN_PHASE_AUTONOMOUS),
          "the armed robot can begin autonomous operation");
    check(ROBOT_CONFIG_LOCKED ==
              robot_config_apply_barcode_map(&config, map),
          "an autonomous robot rejects remapping");
    check(!robot_config_set_phase(&config, RUN_PHASE_CONFIGURING),
          "the lifecycle cannot move backwards without reset");
    check(robot_config_set_phase(&config, RUN_PHASE_FINISHED),
          "the autonomous run can finish");
    check(0 == strcmp("FINISHED", run_phase_name(config.phase)),
          "the run phase has a stable readable name");

    config.phase = (run_phase_t)99;
    check(!robot_config_set_phase(&config, RUN_PHASE_FINISHED),
          "a corrupted current phase cannot authorize a transition");
    check(0 == strcmp("INVALID", run_phase_name(config.phase)),
          "an invalid run phase is reported safely");
}

static void test_shared_configuration_rejects_bad_inputs(void)
{
    robot_config_t config;
    nav_action_t   map[ROBOT_BARCODE_COUNT];
    char           tiny_reply[8];
    unsigned       i;

    printf("\nshared configuration rejects invalid boundaries\n");

    robot_config_init(&config);
    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        map[i] = NAV_NONE;
    }

    check(ROBOT_CONFIG_INVALID_ARGUMENT ==
              robot_config_apply_barcode_map(NULL, map),
          "a NULL configuration is rejected");
    check(ROBOT_CONFIG_INVALID_ARGUMENT ==
              robot_config_apply_barcode_map(&config, NULL),
          "a NULL map is rejected");
    check(ROBOT_CONFIG_INVALID_MAP ==
              robot_config_apply_barcode_map(&config, map),
          "a map with no assigned action is rejected");

    map[0] = (nav_action_t)99;
    check(ROBOT_CONFIG_INVALID_MAP ==
              robot_config_apply_barcode_map(&config, map),
          "an out-of-range action is rejected");
    check(NAV_STRAIGHT == robot_config_action_for_letter(&config, 'A'),
          "a rejected map cannot alter the previous mapping");
    check(0u == config.version,
          "a rejected map cannot increment the configuration version");

    check(!robot_config_set_phase(&config, RUN_PHASE_AUTONOMOUS),
          "the lifecycle cannot skip ARMED");
    check(RUN_PHASE_CONFIGURING == config.phase,
          "a rejected phase transition leaves the phase unchanged");
    check(robot_config_set_phase(&config, RUN_PHASE_CONFIGURING),
          "repeating the current phase is idempotent");

    check(NAV_NONE == robot_config_action_for_letter(NULL, 'A'),
          "a NULL configuration has no barcode action");
    check(NAV_NONE == robot_config_action_for_letter(&config, 'a'),
          "a lowercase barcode cannot index the table");
    check(0u == robot_config_assigned_count(NULL),
          "a NULL configuration reports no assignments");

    check(!cmd_format_barcode_map_reply(NULL, 1u, 0u, &config,
                                        CMD_MAP_REPLY_ACCEPTED),
          "the reply formatter rejects a NULL output");
    check(!cmd_format_barcode_map_reply(tiny_reply, 0u, 0u, &config,
                                        CMD_MAP_REPLY_ACCEPTED),
          "the reply formatter rejects a zero-sized output");
    check(!cmd_format_barcode_map_reply(tiny_reply, sizeof tiny_reply, 0u,
                                        &config, CMD_MAP_REPLY_ACCEPTED),
          "the reply formatter reports truncation");
    check(!cmd_format_barcode_map_reply(tiny_reply, sizeof tiny_reply, 0u,
                                        &config, (cmd_map_reply_t)99),
          "the reply formatter rejects an invalid result enum");
}

static void test_topic_routing_edges(void)
{
    char deep[200];

    printf("\ncommand topic routing, edge cases\n");

    check(CMD_NONE == cmd_kind_from_topic(""), "an empty topic is not a command");
    check(CMD_NONE == cmd_kind_from_topic("/"), "a bare slash is not a command");
    check(CMD_NONE == cmd_kind_from_topic("cmd/"),
          "a topic ending in a slash names no command");
    check(CMD_SNAPSHOT == cmd_kind_from_topic("cmd/snapshot"),
          "a topic with no group prefix still routes");
    check(CMD_NONE == cmd_kind_from_topic("xcmd/snapshot"),
          "a segment merely ENDING in cmd does not count");
    check(CMD_NONE == cmd_kind_from_topic("grp07/car/cmdx/snapshot"),
          "a segment merely STARTING with cmd does not count");
    check(CMD_NONE == cmd_kind_from_topic("grp07/car/cmd/snapshot/extra"),
          "a deeper topic under cmd does not match a known command name");

    (void)snprintf(deep, sizeof deep,
                   "a/very/long/prefix/that/keeps/going/and/going/and/going/"
                   "on/for/quite/a/while/indeed/cmd/stop");
    check(CMD_UNKNOWN == cmd_kind_from_topic(deep),
          "a very long remote-motion topic remains disabled");
}


/* -------------------------------------------------------------------------
 * Snapshot
 * ------------------------------------------------------------------------- */

/* Drain the queue, returning how many messages came out, and record whether
   every one of them was valid JSON and carried the snapshot marker. */
static uint32_t drain(bool * all_json, bool * all_snap)
{
    tele_msg_t msg;
    uint32_t   n = 0u;

    *all_json = true;
    *all_snap = true;

    while (tele_queue_pop(&msg))
    {
        if (!json_is_well_formed(msg.payload)) { *all_json = false; }
        if (NULL == strstr(msg.payload, "\"snap\":true")) { *all_snap = false; }
        n++;
    }
    return n;
}

static void test_snapshot(void)
{
    tele_msg_t msg;
    bool       all_json;
    bool       all_snap;
    uint32_t   n;

    printf("\nsnapshot replays current state on request\n");

    tele_init();
    g_fake_ms = 5000u;

    check(0u == tele_publish_snapshot(),
          "a snapshot before anything is reported queues nothing");
    check(0u == tele_queue_depth(), "  and leaves the queue empty");

    /* Report three subsystems, then throw the live messages away - the
       snapshot must come from the cache, not from the queue. */
    (void)tele_report_state(TELE_STATE_HUMP);
    (void)tele_report_hump(43u, 51u, 2u);
    (void)tele_report_motion(220, 218, 1840, 4821u, 4805u);
    while (tele_queue_pop(&msg)) { /* discard the live traffic */ }
    check(0u == tele_queue_depth(), "queue emptied before the snapshot");

    n = tele_publish_snapshot();
    check(3u == n, "snapshot queues exactly the three topics that reported");

    n = drain(&all_json, &all_snap);
    check(3u == n, "  and three messages come back out");
    check(all_json, "  every snapshot message is valid JSON");
    check(all_snap, "  every snapshot message is marked \"snap\":true");

    /* The values must be the ones last reported, not defaults. */
    tele_init();
    g_fake_ms = 6000u;
    (void)tele_report_hump(43u, 51u, 2u);
    while (tele_queue_pop(&msg)) { }
    (void)tele_publish_snapshot();
    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"peak_mm\":43,\"max_mm\":51,\"count\":2"),
          "snapshot carries the values that were last reported");
    check_str(msg.topic, TELE_TOPIC_ROOT "/telemetry/hump",
              "  and goes out on that subsystem's own topic");
}

static void test_snapshot_tracks_latest(void)
{
    tele_msg_t msg;
    uint32_t   n;
    bool       a;
    bool       b;

    printf("\nsnapshot follows the newest value, and repeats cleanly\n");

    tele_init();
    g_fake_ms = 7000u;

    (void)tele_report_hump(10u, 10u, 1u);
    (void)tele_report_hump(60u, 60u, 2u);      /* a taller one */
    while (tele_queue_pop(&msg)) { }

    (void)tele_publish_snapshot();
    (void)tele_queue_pop(&msg);
    check(NULL != strstr(msg.payload, "\"peak_mm\":60"),
          "snapshot reflects the most recent report, not the first");

    /* Taking a snapshot must not consume the cache - a second request has to
       work exactly like the first. */
    n = tele_publish_snapshot();
    check(1u == n, "a second snapshot still finds the cached state");
    n = drain(&a, &b);
    check(1u == n, "  and produces the same one message");
    check(a && b, "  still valid JSON and still marked as a snapshot");
}

static void test_snapshot_excludes_log(void)
{
    tele_msg_t msg;
    uint32_t   n;

    printf("\nsnapshot excludes free-text log lines\n");

    tele_init();
    g_fake_ms = 8000u;

    (void)tele_report_log("calibration finished");
    while (tele_queue_pop(&msg)) { }

    n = tele_publish_snapshot();
    check(0u == n,
          "a log line alone produces no snapshot - it is an event, not a state");

    (void)tele_report_state(TELE_STATE_BOOT);
    (void)tele_report_log("another log line");
    while (tele_queue_pop(&msg)) { }

    n = tele_publish_snapshot();
    check(1u == n, "only the state is replayed, the log is not");
    (void)tele_queue_pop(&msg);
    check(NULL == strstr(msg.payload, "msg"),
          "  and the replayed message is not the log line");
}

static void test_snapshot_all_topics(void)
{
    tele_msg_t msg;
    bool       all_json;
    bool       all_snap;
    uint32_t   n;

    printf("\nsnapshot with every subsystem reporting\n");

    tele_init();
    g_fake_ms = 9000u;

    (void)tele_report_state(TELE_STATE_LINE_FOLLOW);
    (void)tele_report_motion(220, 218, 1840, 4821u, 4805u);
    (void)tele_report_line(-3, false, 2100u, 1980u);
    (void)tele_report_barcode('C', NAV_RIGHT, true);
    (void)tele_report_hump(43u, 51u, 2u);
    (void)tele_report_imu(90u, -125, 42, IMU_EVENT_CLIMBING);
    (void)tele_report_obstacle(15, 250u, 120u, 300u, 90u, AVOID_LEFT);
    while (tele_queue_pop(&msg)) { }

    n = tele_publish_snapshot();
    check(7u == n, "all seven state topics are replayed");

    /* Seven fits the 16-slot queue with room to spare, which is the reason
       the queue is not smaller. */
    check(7u == tele_queue_depth(), "a full snapshot fits the queue without dropping");
    check(0u == tele_dropped_count(), "  and drops nothing");

    n = drain(&all_json, &all_snap);
    check(7u == n, "seven come back out");
    check(all_json, "  all seven are valid JSON");
    check(all_snap, "  all seven are marked as snapshot replays");
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
    printf("INF2004 Buddy 1 - telemetry and command logic\n");
    printf("topic root: %s\n", TELE_TOPIC_ROOT);

    test_payload_format();
    test_worst_case_payload_fits();
    test_bad_input_cannot_break_json();
    test_queue_fifo();
    test_queue_overflow_drops_oldest();
    test_periodic_telemetry_cannot_evict_events();
    test_free_text_cannot_break_json();
    test_every_message_is_valid_json();
    test_name_tables_are_bounded();
    test_action_name_round_trip();
    test_queue_survives_many_wraps();
    test_pop_guards();
    test_topic_routing();
    test_topic_routing_edges();
    test_barcode_map_parsing();
    test_bad_remap_leaves_the_old_map_alone();
    test_barcode_map_hard_cases();
    test_shared_configuration_lifecycle();
    test_shared_configuration_rejects_bad_inputs();
    test_snapshot();
    test_snapshot_tracks_latest();
    test_snapshot_excludes_log();
    test_snapshot_all_topics();

    printf("\n%u checks, %u failures\n", g_checks, g_failures);

    if (0u == g_failures)
    {
        printf("ALL TESTS PASS  (0 failures)\n");
        return 0;
    }

    printf("TESTS FAILED\n");
    return 1;
}

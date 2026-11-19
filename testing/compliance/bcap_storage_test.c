/** @file bcap_storage_test.c
 * @brief Characterize capture state and staged serializers before compaction.
 *
 * Compile this candidate fixture against each subject's public bc_capture.h
 * and separately compiled bc_capture.c. Synthetic metadata includes rejected
 * inputs; recorder counters are seeded only within their initialized domain.
 * Traces print logical fields, never struct padding or layout-dependent bytes.
 */
#include "app_program/controller/bc_capture.h"
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    unsigned char before[8];
    unsigned char bytes[192];
    unsigned char after[8];
} bcap_test_output_t;

static bcap_t g_capture;
static bcap_t g_before;
static unsigned g_states;
static unsigned g_binary_cases;
static unsigned g_text_cases;

/** @brief Print every byte of an addressable buffer in a stable hex form. */
static void print_bytes (const unsigned char * p_bytes, unsigned count)
{
    for (unsigned index = 0U; index < count; ++index)
    {
        printf("%02x", (unsigned)p_bytes[index]);
    }
}

/** @brief Print named cursor values independently of bitfield allocation. */
static void print_cursor (const bcap_cursor_t * p_cursor)
{
    printf(" %u %u %u %u %u", (unsigned)p_cursor->episode,
           (unsigned)p_cursor->sample, (unsigned)p_cursor->header_done,
           (unsigned)p_cursor->summary_done, (unsigned)p_cursor->end_done);
}

/** @brief Print all logical sample fields, including invalid stored bytes. */
static void print_sample (const bcap_sample_t * p_sample)
{
    printf("%04x%04x%02x%02x%02x", (unsigned)p_sample->raw,
           (unsigned)p_sample->mm, (unsigned)p_sample->dark4,
           (unsigned)p_sample->flags, (unsigned)p_sample->dt_ms);
}

/** @brief Observe the entire capture without depending on member offsets. */
static void trace_state (const char * p_label)
{
    printf("BCSTORE STATE %u %s %" PRIu32 " %" PRIu32
           " %u %u %u %u %u %u %u %u %u %u %u %u %u ",
           g_states++, p_label, g_capture.run_start_us, g_capture.last_us,
           (unsigned)g_capture.used, (unsigned)g_capture.pre_head,
           (unsigned)g_capture.pre_count, (unsigned)g_capture.quiet,
           (unsigned)g_capture.episodes, (unsigned)g_capture.dropped,
           (unsigned)g_capture.active, (unsigned)g_capture.have_last,
           (unsigned)g_capture.decoded, (unsigned)g_capture.rejected,
           (unsigned)g_capture.timing, (unsigned)g_capture.overflow,
           (unsigned)g_capture.letter_count);
    print_bytes((const unsigned char *)g_capture.letters,
                (unsigned)sizeof(g_capture.letters));
    putchar('\n');
    printf("BCSTORE POOL ");
    for (unsigned index = 0U; index < BCAP_POOL; ++index)
    {
        print_sample(&g_capture.pool[index]);
    }
    printf("\nBCSTORE PRE ");
    for (unsigned index = 0U; index < BCAP_PRE; ++index)
    {
        print_sample(&g_capture.pre[index]);
        printf("%016" PRIx64, (uint64_t)g_capture.pre_qmm[index]);
    }
    printf("\nBCSTORE EPISODES");
    for (unsigned index = 0U; index < BCAP_MAX_EPISODES; ++index)
    {
        const bcap_episode_t * p_episode = &g_capture.episode[index];
        printf(" %u:%u:%" PRIu32 ":%016" PRIx64, (unsigned)p_episode->first,
               (unsigned)p_episode->count, p_episode->start_ms,
               (uint64_t)p_episode->origin_qmm);
    }
    putchar('\n');
}

/** @brief Verify canaries outside the advertised write region remain intact. */
static void check_guards (const bcap_test_output_t * p_output, unsigned used)
{
    for (unsigned index = 0U; index < 8U; ++index)
    {
        assert(p_output->before[index] == 0xA5U);
        assert(p_output->after[index] == 0xA5U);
    }
    for (unsigned index = used; index < sizeof(p_output->bytes); ++index)
    {
        assert(p_output->bytes[index] == 0xA5U);
    }
}

/** @brief Observe binary bytes and atomic failure semantics at one stage.
 * @param null_argument 0 for none; 1/2/3 pass a null capture/cursor/output.
 * @return Number of bytes produced, with the caller's cursor updated in place.
 */
static unsigned binary_case (const char * p_label, bcap_cursor_t * p_cursor,
                             unsigned capacity, unsigned null_argument)
{
    bcap_test_output_t output;
    bcap_cursor_t before;
    unsigned length;
    memset(&output, 0xA5, sizeof(output));
    memcpy(&before, p_cursor, sizeof(before));
    memcpy(&g_before, &g_capture, sizeof(g_before));
    length =
        bcap_binary_next(null_argument == 1U ? NULL : &g_capture,
                         null_argument == 2U ? NULL : p_cursor,
                         null_argument == 3U ? NULL : output.bytes, capacity);
    assert(memcmp(&g_before, &g_capture, sizeof(g_before)) == 0);
    assert(length <= capacity && length <= BCAP_WIRE_MAX);
    check_guards(&output, length);
    if (length == 0U)
    {
        assert(memcmp(&before, p_cursor, sizeof(before)) == 0);
    }
    printf("BCSTORE BINARY %u %s %u %u %u", g_binary_cases++, p_label, capacity,
           null_argument, length);
    print_cursor(p_cursor);
    putchar(' ');
    print_bytes(output.bytes, (unsigned)sizeof(output.bytes));
    putchar('\n');
    return length;
}

/** @brief Observe text's truncation and cursor advancement, including size0. */
static bool text_case (const char * p_label, bcap_cursor_t * p_cursor,
                       unsigned capacity)
{
    bcap_test_output_t output;
    bool result;
    memset(&output, 0xA5, sizeof(output));
    memcpy(&g_before, &g_capture, sizeof(g_before));
    result =
        bcap_format_next(&g_capture, p_cursor, (char *)output.bytes, capacity);
    assert(memcmp(&g_before, &g_capture, sizeof(g_before)) == 0);
    check_guards(&output, capacity == 0U ? 1U : capacity);
    assert(memchr(output.bytes, 0, capacity == 0U ? 1U : capacity) != NULL);
    printf("BCSTORE TEXT %u %s %u %u", g_text_cases++, p_label, capacity,
           (unsigned)result);
    print_cursor(p_cursor);
    putchar(' ');
    print_bytes(output.bytes, (unsigned)sizeof(output.bytes));
    putchar('\n');
    return result;
}

/** @brief Supply a deterministic sample through the real recorder API. */
static void record_sample (uint32_t stamp, uint16_t raw, uint16_t darkness,
                           uint8_t surface, uint8_t status, char letter,
                           bool collecting, double centre_m)
{
    bcap_input_t input = {0};
    input.t_us = stamp;
    input.raw = raw;
    input.darkness = darkness;
    input.surface = surface;
    input.status = status;
    input.letter = letter;
    input.collecting = collecting;
    input.rearming = true;
    input.centre_m = centre_m;
    bcap_record(&g_capture, &input);
}

/** @brief Exercise circular history, saturation and both trigger-fit edges. */
static void history_cases (void)
{
    bcap_begin_run(&g_capture, UINT32_MAX - 1000U);
    trace_state("begin");
    for (unsigned index = 0U; index <= BCAP_PRE; ++index)
    {
        record_sample((uint32_t)(UINT32_MAX - 1000U + index * 5000U),
                      (uint16_t)index, (uint16_t)(index * 3U), 1U, 0U, 0, false,
                      (double)index / 1000.0);
        if (index + 1U >= BCAP_PRE - 1U)
        {
            trace_state("history-boundary");
        }
    }
    assert(g_capture.pre_head == 1U && g_capture.pre_count == BCAP_PRE);
    assert(g_capture.have_last && g_capture.pre[0].dt_ms == 5U);
    record_sample(2000000U, 4095U, 1000U, 2U, 1U, 'A', false, 1.0);
    assert(g_capture.active && g_capture.used == BCAP_PRE + 1U);
    bcap_end_run(&g_capture);
    trace_state("end-once");
    bcap_end_run(&g_capture);
    assert(g_capture.episodes == 1U && !g_capture.active);
    trace_state("end-twice");
    for (unsigned over = 0U; over <= 1U; ++over)
    {
        bcap_begin_run(&g_capture, 0U);
        for (unsigned index = 0U; index < BCAP_PRE; ++index)
        {
            record_sample(index * 5000U, (uint16_t)index, 0U, 1U, 0U, 0, false,
                          0.0);
        }
        g_capture.used = (uint16_t)(BCAP_POOL - BCAP_PRE - 1U + over);
        record_sample(2000000U, 123U, 1000U, 2U, 0U, 0, false, 0.0);
        assert(g_capture.dropped == over);
        assert(g_capture.used ==
               (over == 0U ? BCAP_POOL : BCAP_POOL - BCAP_PRE));
        trace_state(over == 0U ? "pool-exact-fit" : "pool-one-over");
    }
}

/** @brief Cover quiet threshold, collecting override and terminal pool4000. */
static void quiet_cases (void)
{
    for (unsigned collecting = 0U; collecting <= 1U; ++collecting)
    {
        bcap_begin_run(&g_capture, 0U);
        record_sample(0U, 0U, 1000U, 2U, 0U, 0, false, 0.0);
        for (unsigned index = 1U; index <= BCAP_END_QUIET; ++index)
        {
            record_sample(index * 5000U, 1U, 0U, 1U, 0U, 0, collecting != 0U,
                          0.0);
            if (index >= BCAP_END_QUIET - 1U)
            {
                trace_state("quiet-threshold");
            }
        }
        assert(g_capture.quiet == BCAP_END_QUIET);
        assert(g_capture.active == (collecting != 0U));
        record_sample(500000U, 1U, 0U, 1U, 0U, 0, false, 0.0);
        assert(!g_capture.active && g_capture.episodes == 1U);
        trace_state("quiet-close");
    }
    bcap_begin_run(&g_capture, 0U);
    for (unsigned index = 0U; index < BCAP_POOL; ++index)
    {
        record_sample(index * 5000U, 1U, 0U, 1U, 0U, 0, true, 0.0);
    }
    assert(g_capture.quiet == BCAP_POOL && g_capture.used == BCAP_POOL);
    assert(!g_capture.active && g_capture.episodes == 1U);
    trace_state("quiet-pool-terminal");
    bcap_begin_run(&g_capture, 123U);
    assert(!g_capture.active && !g_capture.have_last && !g_capture.quiet);
    assert(!g_capture.pre_head && !g_capture.pre_count);
    trace_state("begin-clears-terminal");
}

/** @brief Preserve raw input, time wrap, saturations and unavailable position.
 */
static void input_cases (void)
{
    const uint32_t gaps[] = {0U,      1U,      30000U,    30001U,
                             255000U, 256000U, UINT32_MAX};
    const uint16_t raw[] = {0U, 4095U, 4096U, UINT16_MAX};
    const unsigned char letters[] = {0U, '@', '[', 0xFFU, 'A', 'Z'};
    const double positions[] = {NAN,    INFINITY, -INFINITY,     -1.0,
                                -0.0,   0.0,      0.00000390625, 0.000499,
                                0.0005, 65.535,   1.0e20,        -1.0e20};
    for (unsigned index = 0U; index < sizeof(gaps) / sizeof(gaps[0]); ++index)
    {
        bcap_begin_run(&g_capture, UINT32_MAX - 10U);
        record_sample(UINT32_MAX - 10U, 0U, 0U, 1U, 0U, 0, false, 0.0);
        assert(g_capture.pre[0].dt_ms == 0U);
        record_sample((uint32_t)(UINT32_MAX - 10U + gaps[index]), 1U, 0U, 1U,
                      0U, 0, false, 0.0);
        assert(((g_capture.pre[1].flags & 0x80U) != 0U) ==
               (gaps[index] > 30000U));
        trace_state("time-gap");
    }
    bcap_begin_run(&g_capture, 0U);
    for (unsigned index = 0U; index < sizeof(raw) / sizeof(raw[0]); ++index)
    {
        record_sample(index * 5000U, raw[index], UINT16_MAX, 3U, UINT8_MAX, 0,
                      false, 0.0);
        assert(g_capture.pre[index].raw == raw[index]);
        assert(g_capture.pre[index].dark4 == 250U);
    }
    trace_state("unvalidated-raw-status-surface");
    for (unsigned index = 0U; index < sizeof(letters); ++index)
    {
        record_sample(50000U + index * 5000U, 0U, 0U, 1U, 1U,
                      (char)letters[index], false, 0.0);
    }
    trace_state("unvalidated-letters");
    g_capture.decoded = g_capture.rejected = UINT16_MAX;
    g_capture.timing = g_capture.overflow = UINT16_MAX;
    for (uint8_t status = 1U; status <= 4U; ++status)
    {
        record_sample(100000U + status * 5000U, 0U, 0U, 1U, status, 'B', false,
                      0.0);
    }
    assert(!g_capture.decoded && !g_capture.rejected && !g_capture.timing &&
           !g_capture.overflow && g_capture.letter_count == 7U);
    record_sample(130000U, 0U, 0U, 1U, 1U, 'C', false, 0.0);
    record_sample(135000U, 0U, 0U, 1U, 1U, 'D', false, 0.0);
    assert(g_capture.letter_count == 8U);
    g_capture.episodes = BCAP_MAX_EPISODES;
    g_capture.dropped = UINT8_MAX - 1U;
    record_sample(140000U, 0U, 0U, 2U, 0U, 0, false, 0.0);
    record_sample(145000U, 0U, 0U, 2U, 0U, 0, false, 0.0);
    assert(g_capture.dropped == UINT8_MAX);
    trace_state("wrapping-and-saturating-counters");
    bcap_begin_run(&g_capture, 0U);
    for (unsigned index = 0U; index < sizeof(positions) / sizeof(positions[0]);
         ++index)
    {
        record_sample(index * 5000U, 0U, 0U, 1U, 0U, 0, false,
                      positions[index]);
    }
    record_sample(100000U, 0U, 1000U, 2U, 0U, 0, false, 1.0);
    trace_state("position-boundaries");
    for (unsigned index = 0U; index < sizeof(positions) / sizeof(positions[0]);
         ++index)
    {
        bcap_begin_run(&g_capture, 0U);
        record_sample(0U, 0U, 1000U, 2U, 0U, 0, false,
                      index % 2U == 0U ? 0.0 : -1.0e20);
        record_sample(5000U, 0U, 1000U, 2U, 0U, 0, false, positions[index]);
        trace_state("relative-position");
    }
}

/** @brief Prepare one empty and one populated episode for staged validation. */
static void serializer_seed (void)
{
    bcap_begin_run(&g_capture, UINT32_MAX);
    g_capture.episodes = 2U;
    g_capture.used = 2U;
    g_capture.episode[1].count = 2U;
    g_capture.episode[1].start_ms = 1U;
    g_capture.pool[0] = (bcap_sample_t){4095U, UINT16_MAX, 250U, 0xF2U, 255U};
    g_capture.pool[1] = (bcap_sample_t){0U, 0U, 0U, 1U, 0U};
    g_capture.letters[0] = 'A';
    g_capture.letters[1] = 'Z';
    g_capture.letter_count = 2U;
}

/** @brief Select summary, header, samples, or a completed empty episode. */
static bcap_cursor_t stage_cursor (unsigned stage)
{
    bcap_cursor_t cursor = {0};
    if (stage != 0U)
    {
        cursor.summary_done = true;
        cursor.episode = stage == 3U ? 0U : 1U;
        cursor.header_done = stage >= 2U;
    }
    return cursor;
}

/** @brief Mutate one public validation value without pre-truncating its input.
 */
static void mutate_metadata (unsigned field, uint32_t value)
{
    switch (field)
    {
    case 0U:
        g_capture.episodes = (uint8_t)value;
        break;
    case 1U:
        g_capture.letter_count = (uint8_t)value;
        break;
    case 2U:
        g_capture.used = (uint16_t)value;
        break;
    case 3U:
        g_capture.episode[1].first = (uint16_t)value;
        break;
    case 4U:
        g_capture.episode[1].count = (uint16_t)value;
        break;
    case 5U:
        g_capture.episode[1].start_ms = value;
        break;
    case 6U:
        g_capture.pool[0].raw = (uint16_t)value;
        break;
    case 7U:
        g_capture.pool[0].dark4 = (uint8_t)value;
        break;
    case 8U:
        g_capture.pool[0].flags = (uint8_t)value;
        break;
    default:
        g_capture.letters[0] = (char)value;
        break;
    }
}

/** @brief Compare summary/header/sample validation order and failed local
 * skips.
 */
static void metadata_cases (void)
{
    static const uint32_t values[][8] = {
        {0U, 1U, 2U, 12U, 13U, 15U, 16U, 255U},
        {0U, 1U, 2U, 8U, 9U, 15U, 16U, 255U},
        {0U, 1U, 2U, 3999U, 4000U, 4001U, 4095U, UINT16_MAX},
        {0U, 1U, 2U, 3999U, 4000U, 4001U, 4095U, UINT16_MAX},
        {0U, 1U, 2U, 3999U, 4000U, 4001U, 4095U, UINT16_MAX},
        {0U, 1U, 4294966U, 4294967U, 4294968U, 8388607U, 8388608U, UINT32_MAX},
        {0U, 1U, 4094U, 4095U, 4096U, 8191U, 8192U, UINT16_MAX},
        {0U, 1U, 249U, 250U, 251U, 252U, 254U, 255U},
        {0U, 1U, 2U, 3U, 16U, 20U, 0xF2U, 255U},
        {0U, '@', 'A', 'Z', '[', 127U, 128U, 255U}};
    for (unsigned field = 0U; field < 10U; ++field)
    {
        for (unsigned index = 0U; index < 8U; ++index)
        {
            serializer_seed();
            mutate_metadata(field, values[field][index]);
            for (unsigned stage = 0U; stage < 4U; ++stage)
            {
                bcap_cursor_t cursor = stage_cursor(stage);
                (void)binary_case("metadata", &cursor, BCAP_WIRE_MAX, 0U);
            }
        }
    }
    serializer_seed();
    g_capture.episode[1].start_ms = 8388608U;
    bcap_cursor_t cursor = stage_cursor(0U);
    assert(binary_case("late-invalid-summary", &cursor, BCAP_WIRE_MAX, 0U) >
           0U);
    cursor = stage_cursor(3U);
    assert(binary_case("late-invalid-after-skip", &cursor, BCAP_WIRE_MAX, 0U) ==
           0U);
}

/** @brief Exercise exact capacity, one-short, null arguments and text
 * traversal.
 */
static void serializer_cases (void)
{
    const unsigned text_sizes[] = {0U, 1U, 2U, 10U, 11U, 159U, 160U, 192U};
    serializer_seed();
    for (unsigned stage = 0U; stage < 4U; ++stage)
    {
        bcap_cursor_t cursor = stage_cursor(stage);
        unsigned exact =
            binary_case("capacity-reference", &cursor, BCAP_WIRE_MAX, 0U);
        assert(exact > 0U);
        const unsigned sizes[] = {0U, exact - 1U, exact, exact + 1U,
                                  BCAP_WIRE_MAX};
        for (unsigned index = 0U; index < sizeof(sizes) / sizeof(sizes[0]);
             ++index)
        {
            cursor = stage_cursor(stage);
            unsigned actual =
                binary_case("capacity", &cursor, sizes[index], 0U);
            assert(actual == (sizes[index] < exact ? 0U : exact));
        }
        for (unsigned index = 0U;
             index < sizeof(text_sizes) / sizeof(text_sizes[0]); ++index)
        {
            cursor = stage_cursor(stage);
            (void)text_case("text-capacity", &cursor, text_sizes[index]);
        }
    }
    for (unsigned null_argument = 1U; null_argument <= 3U; ++null_argument)
    {
        bcap_cursor_t cursor = {0};
        assert(binary_case("null", &cursor, BCAP_WIRE_MAX, null_argument) ==
               0U);
    }
    for (unsigned index = 0U;
         index < sizeof(text_sizes) / sizeof(text_sizes[0]); ++index)
    {
        bcap_cursor_t cursor = {0};
        unsigned calls = 0U;
        while (text_case("text-traversal", &cursor, text_sizes[index]))
        {
            assert(++calls < 10U);
        }
        assert(cursor.end_done);
    }
    bcap_cursor_t cursor = {0};
    unsigned calls = 0U;
    while (binary_case("binary-traversal", &cursor, BCAP_WIRE_MAX, 0U) != 0U)
    {
        assert(++calls < 10U);
    }
    assert(cursor.end_done);
    cursor.episode = 15U;
    assert(binary_case("invalid-cursor", &cursor, BCAP_WIRE_MAX, 0U) == 0U);
}

/** @brief Cover packet count boundaries and drain a full4000-sample pool. */
static void packet_count_cases (void)
{
    const unsigned counts[] = {0U, 1U, 23U, 24U, 25U, BCAP_POOL};
    for (unsigned index = 0U; index < sizeof(counts) / sizeof(counts[0]);
         ++index)
    {
        serializer_seed();
        g_capture.used = (uint16_t)counts[index];
        g_capture.episode[1].count = (uint16_t)counts[index];
        bcap_cursor_t cursor = stage_cursor(2U);
        unsigned exact =
            binary_case("sample-count", &cursor, BCAP_WIRE_MAX, 0U);
        assert(exact > 0U);
        cursor = stage_cursor(2U);
        assert(binary_case("sample-count-short", &cursor, exact - 1U, 0U) ==
               0U);
    }
    bcap_cursor_t cursor = {0};
    unsigned calls = 0U;
    while (binary_case("full-pool-traversal", &cursor, BCAP_WIRE_MAX, 0U) != 0U)
    {
        assert(++calls < 200U);
    }
    assert(cursor.end_done && cursor.episode == 2U);
}

/** @brief Run shared characterization; physical sensors and networking are
 * unused.
 */
int main (void)
{
    history_cases();
    quiet_cases();
    input_cases();
    metadata_cases();
    serializer_cases();
    packet_count_cases();
    assert(g_states == 39U && g_binary_cases >= 500U && g_text_cases == 80U);
    fprintf(stderr,
            "bcap_storage: states=%u binary=%u text=%u sizeof(bcap_t)=%zu\n",
            g_states, g_binary_cases, g_text_cases, sizeof(bcap_t));
    return 0;
}

/* End of file. */


/** @file bcap_storage_main.c
 * @brief Characterize capture state and staged serializers before compaction.
 *
 * Compile this candidate fixture against each subject's public bc_capture.h
 * and separately compiled bc_capture.c. Synthetic metadata includes rejected
 * inputs; recorder counters are seeded only within their initialized domain.
 * Traces print logical fields, never struct padding or layout-dependent bytes.
 */
#include "bcap_storage_main.h"
#include "../../app_program/app_types.h"
#include "app_program/controller/bc_capture.h"
#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CAPTURE_STATUS_LAST (4U)
#define CAPTURE_CURSOR_CASES (4U)
#define NULL_ARGUMENT_CASES (3U)

/* The shared comparison runner can select the old baseline field name. */
#ifndef BCAP_STORAGE_DISTANCE_MEMBER
#define BCAP_STORAGE_DISTANCE_MEMBER distance_mm
#endif

typedef struct
{
    uint8_t before[8];
    uint8_t bytes[192];
    uint8_t after[8];
} bcap_test_output_t;

static bc_capture_t g_capture;
static bc_capture_t g_before;
static unsigned     g_states;
static unsigned     g_binary_cases;
static unsigned     g_text_cases;

/** @brief Print every byte of an addressable buffer in a stable hex form. */
static void print_bytes (const uint8_t * p_bytes, unsigned count)
{
    for (unsigned index = 0U; index < count; ++index)
    {
        printf("%02x", (unsigned)p_bytes[index]);
    }
}

/** @brief Print named cursor values independently of bitfield allocation. */
static void print_cursor (const bc_capture_cursor_t * p_cursor)
{
    printf(" %u %u %u %u %u", (unsigned)p_cursor->episode,
           (unsigned)p_cursor->sample, (unsigned)p_cursor->b_is_header_done,
           (unsigned)p_cursor->b_is_summary_done,
           (unsigned)p_cursor->b_is_end_done);
}

/** @brief Print all logical sample fields, including invalid stored bytes. */
static void print_sample (const bc_capture_sample_t * p_sample)
{
    printf("%04x%04x%02x%02x%02x", (unsigned)p_sample->raw,
           (unsigned)p_sample->BCAP_STORAGE_DISTANCE_MEMBER,
           (unsigned)p_sample->darkness_code, (unsigned)p_sample->flags,
           (unsigned)p_sample->dt_ms);
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
           (unsigned)g_capture.b_is_active,
           (unsigned)g_capture.b_has_last_sample, (unsigned)g_capture.decoded,
           (unsigned)g_capture.rejected, (unsigned)g_capture.timing,
           (unsigned)g_capture.overflow, (unsigned)g_capture.letter_count);
    print_bytes((const uint8_t *)g_capture.letters,
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
        const bc_capture_episode_t * p_episode = &g_capture.episode[index];
        printf(" %u:%u:%" PRIu32 ":%016" PRIx64, (unsigned)p_episode->first,
               (unsigned)p_episode->count, p_episode->start_ms,
               (uint64_t)p_episode->origin_qmm);
    }
    putchar('\n');
}

/** @brief Verify canaries outside the advertised write region remain intact. */
static void check_guards (const bcap_test_output_t * p_output, unsigned used)
{
    for (unsigned index = 0U; index < sizeof(p_output->before); ++index)
    {
        assert(0xA5U == p_output->before[index]);
        assert(0xA5U == p_output->after[index]);
    }
    for (unsigned index = used; index < sizeof(p_output->bytes); ++index)
    {
        assert(0xA5U == p_output->bytes[index]);
    }
}

/** @brief Observe binary bytes and atomic failure semantics at one stage.
 * @param[in] p_label Case name printed with the trace.
 * @param[in,out] p_cursor Writable cursor, copied before the packer runs.
 * @param[in] capacity Byte capacity passed to the packer.
 * @param[in] null_argument Use zero for all three inputs, or 1, 2 or 3
 * to pass a null capture, cursor or output buffer, respectively.
 * @return Number of bytes. Zero leaves the caller's cursor unchanged.
 */
static unsigned binary_case (const char *          p_label,
                             bc_capture_cursor_t * p_cursor, unsigned capacity,
                             unsigned null_argument)
{
    bcap_test_output_t  output;
    bc_capture_cursor_t before;
    unsigned            length;
    memset(&output, 0xA5, sizeof(output));
    memcpy(&before, p_cursor, sizeof(before));
    memcpy(&g_before, &g_capture, sizeof(g_before));
    length = bc_capture_binary_next(1U == null_argument ? NULL : &g_capture,
                                    2U == null_argument ? NULL : p_cursor,
                                    3U == null_argument ? NULL : output.bytes,
                                    capacity);
    assert(memcmp(&g_before, &g_capture, sizeof(g_before)) == 0);
    assert(length <= capacity && length <= BCAP_WIRE_MAX);
    check_guards(&output, length);
    if (0U == length)
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
static bool text_case (const char * p_label, bc_capture_cursor_t * p_cursor,
                       unsigned capacity)
{
    bcap_test_output_t output;
    bool               b_did_format;
    memset(&output, 0xA5, sizeof(output));
    memcpy(&g_before, &g_capture, sizeof(g_before));
    b_did_format = bc_capture_format_next(&g_capture, p_cursor,
                                          (char *)output.bytes, capacity);
    assert(memcmp(&g_before, &g_capture, sizeof(g_before)) == 0);
    check_guards(&output, 0U == capacity ? 1U : capacity);
    assert(memchr(output.bytes, 0, 0U == capacity ? 1U : capacity) != NULL);
    printf("BCSTORE TEXT %u %s %u %u", g_text_cases++, p_label, capacity,
           (unsigned)b_did_format);
    print_cursor(p_cursor);
    putchar(' ');
    print_bytes(output.bytes, (unsigned)sizeof(output.bytes));
    putchar('\n');
    return b_did_format;
}

/** @brief Supply a deterministic sample through the real recorder API. */
static void record_sample (uint32_t stamp, uint16_t raw, uint16_t darkness,
                           uint8_t surface, uint8_t status, char letter,
                           bool b_is_collecting, float64_t centre_m)
{
    bc_capture_input_t input = {0};
    input.t_us               = stamp;
    input.raw                = raw;
    input.darkness           = darkness;
    input.surface            = surface;
    input.status             = status;
    input.letter             = letter;
    input.b_is_collecting    = b_is_collecting;
    input.b_is_rearming      = true;
    input.centre_m           = centre_m;
    bc_capture_record(&g_capture, &input);
}

/** @brief Exercise circular history, saturation and both trigger-fit edges. */
static void history_cases (void)
{
    bc_capture_begin_run(&g_capture, UINT32_MAX - 1000U);
    trace_state("begin");
    for (unsigned index = 0U; index <= BCAP_PRE; ++index)
    {
        record_sample((uint32_t)(UINT32_MAX - 1000U + index * 5000U),
                      (uint16_t)index, (uint16_t)(index * 3U), 1U, 0U, 0, false,
                      (float64_t)index / 1000.0);
        if (index + 1U >= BCAP_PRE - 1U)
        {
            trace_state("history-boundary");
        }
    }
    assert(1U == g_capture.pre_head && BCAP_PRE == g_capture.pre_count);
    assert(g_capture.b_has_last_sample && 5U == g_capture.pre[0].dt_ms);
    record_sample(2000000U, 4095U, 1000U, 2U, 1U, 'A', false, 1.0);
    assert(g_capture.b_is_active && (BCAP_PRE + 1U) == g_capture.used);
    bc_capture_end_run(&g_capture);
    trace_state("end-once");
    bc_capture_end_run(&g_capture);
    assert(1U == g_capture.episodes && !g_capture.b_is_active);
    trace_state("end-twice");
    for (unsigned over = 0U; over <= 1U; ++over)
    {
        bc_capture_begin_run(&g_capture, 0U);
        for (unsigned index = 0U; index < BCAP_PRE; ++index)
        {
            record_sample(index * 5000U, (uint16_t)index, 0U, 1U, 0U, 0, false,
                          0.0);
        }
        g_capture.used = (uint16_t)(BCAP_POOL - BCAP_PRE - 1U + over);
        record_sample(2000000U, 123U, 1000U, 2U, 0U, 0, false, 0.0);
        assert(g_capture.dropped == over);
        assert(g_capture.used ==
               (0U == over ? BCAP_POOL : BCAP_POOL - BCAP_PRE));
        trace_state(0U == over ? "pool-exact-fit" : "pool-one-over");
    }
}

/** @brief Cover quiet threshold, collecting override and terminal pool4000. */
static void quiet_cases (void)
{
    for (unsigned collecting_case = 0U; collecting_case <= 1U;
         ++collecting_case)
    {
        bc_capture_begin_run(&g_capture, 0U);
        record_sample(0U, 0U, 1000U, 2U, 0U, 0, false, 0.0);
        for (unsigned index = 1U; index <= BCAP_END_QUIET; ++index)
        {
            record_sample(index * 5000U, 1U, 0U, 1U, 0U, 0,
                          collecting_case != 0U, 0.0);
            if (index >= BCAP_END_QUIET - 1U)
            {
                trace_state("quiet-threshold");
            }
        }
        assert(BCAP_END_QUIET == g_capture.quiet);
        assert(g_capture.b_is_active == (collecting_case != 0U));
        record_sample(500000U, 1U, 0U, 1U, 0U, 0, false, 0.0);
        assert(!g_capture.b_is_active && 1U == g_capture.episodes);
        trace_state("quiet-close");
    }
    bc_capture_begin_run(&g_capture, 0U);
    for (unsigned index = 0U; index < BCAP_POOL; ++index)
    {
        record_sample(index * 5000U, 1U, 0U, 1U, 0U, 0, true, 0.0);
    }
    assert(BCAP_POOL == g_capture.quiet && BCAP_POOL == g_capture.used);
    assert(!g_capture.b_is_active && 1U == g_capture.episodes);
    trace_state("quiet-pool-terminal");
    bc_capture_begin_run(&g_capture, 123U);
    assert(!g_capture.b_is_active && !g_capture.b_has_last_sample &&
           !g_capture.quiet);
    assert(!g_capture.pre_head && !g_capture.pre_count);
    trace_state("begin-clears-terminal");
}

/** @brief Preserve raw input, time wrap, saturations and unavailable position.
 * @details Start the time cases near clock rollover. Trace original raw ADC
 * codes alongside clamped darkness, then trace letters and counters at
 * their storage limits. Position cases include non-finite readings, zero,
 * rounding boundaries and extreme signed travel.
 */
static void input_cases (void)
{
    const uint32_t gaps[]    = {0U,      1U,      30000U,    30001U,
                                255000U, 256000U, UINT32_MAX};
    const uint16_t raw[]     = {0U, 4095U, 4096U, UINT16_MAX};
    const uint8_t  letters[] = {0U, '@', '[', 0xFFU, 'A', 'Z'};

    const float64_t positions[] = {NAN,    INFINITY, -INFINITY,     -1.0,
                                   -0.0,   0.0,      0.00000390625, 0.000499,
                                   0.0005, 65.535,   1.0e20,        -1.0e20};
    for (unsigned index = 0U; index < sizeof(gaps) / sizeof(gaps[0]); ++index)
    {
        bc_capture_begin_run(&g_capture, UINT32_MAX - 10U);
        record_sample(UINT32_MAX - 10U, 0U, 0U, 1U, 0U, 0, false, 0.0);
        assert(0U == g_capture.pre[0].dt_ms);
        record_sample((uint32_t)(UINT32_MAX - 10U + gaps[index]), 1U, 0U, 1U,
                      0U, 0, false, 0.0);
        assert(((g_capture.pre[1].flags & 0x80U) != 0U) ==
               (gaps[index] > 30000U));
        trace_state("time-gap");
    }
    bc_capture_begin_run(&g_capture, 0U);
    for (unsigned index = 0U; index < sizeof(raw) / sizeof(raw[0]); ++index)
    {
        record_sample(index * 5000U, raw[index], UINT16_MAX, 3U, UINT8_MAX, 0,
                      false, 0.0);
        assert(g_capture.pre[index].raw == raw[index]);
        assert(250U == g_capture.pre[index].darkness_code);
    }
    trace_state("unvalidated-raw-status-surface");
    for (unsigned index = 0U; index < sizeof(letters); ++index)
    {
        record_sample(50000U + index * 5000U, 0U, 0U, 1U, 1U,
                      (char)letters[index], false, 0.0);
    }
    trace_state("unvalidated-letters");
    /* clang-format off */
    g_capture.decoded = g_capture.rejected = UINT16_MAX;
    g_capture.timing  = g_capture.overflow = UINT16_MAX;
    /* clang-format on */
    for (uint8_t status = 1U; status <= CAPTURE_STATUS_LAST; ++status)
    {
        record_sample(100000U + status * 5000U, 0U, 0U, 1U, status, 'B', false,
                      0.0);
    }
    assert(!g_capture.decoded && !g_capture.rejected && !g_capture.timing &&
           !g_capture.overflow && 7U == g_capture.letter_count);
    record_sample(130000U, 0U, 0U, 1U, 1U, 'C', false, 0.0);
    record_sample(135000U, 0U, 0U, 1U, 1U, 'D', false, 0.0);
    assert(8U == g_capture.letter_count);
    g_capture.episodes = BCAP_MAX_EPISODES;
    g_capture.dropped  = UINT8_MAX - 1;
    record_sample(140000U, 0U, 0U, 2U, 0U, 0, false, 0.0);
    record_sample(145000U, 0U, 0U, 2U, 0U, 0, false, 0.0);
    assert(UINT8_MAX == g_capture.dropped);
    trace_state("wrapping-and-saturating-counters");
    bc_capture_begin_run(&g_capture, 0U);
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
        bc_capture_begin_run(&g_capture, 0U);
        record_sample(0U, 0U, 1000U, 2U, 0U, 0, false,
                      index % 2U == 0U ? 0.0 : -1.0e20);
        record_sample(5000U, 0U, 1000U, 2U, 0U, 0, false, positions[index]);
        trace_state("relative-position");
    }
}

/** @brief Prepare one empty and one populated episode for staged validation. */
static void serializer_seed (void)
{
    bc_capture_begin_run(&g_capture, UINT32_MAX);
    /* clang-format off */
    g_capture.episodes            = 2U;
    g_capture.used                = 2U;
    g_capture.episode[1].count    = 2U;
    g_capture.episode[1].start_ms = 1U;
    g_capture.pool[0]             =
        (bc_capture_sample_t){4095U, UINT16_MAX, 250U, 0xF2U, 255U};
    g_capture.pool[1]             = (bc_capture_sample_t){0U, 0U, 0U, 1U, 0U};
    g_capture.letters[0]          = 'A';
    g_capture.letters[1]          = 'Z';
    g_capture.letter_count        = 2U;
    /* clang-format on */
}

/** @brief Select summary, header, samples, or a completed empty episode. */
static bc_capture_cursor_t stage_cursor (unsigned stage)
{
    bc_capture_cursor_t cursor = {0};
    if (stage != 0U)
    {
        cursor.b_is_summary_done = true;
        cursor.episode           = 3U == stage ? 0U : 1U;
        cursor.b_is_header_done  = stage >= 2U;
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
        g_capture.pool[0].darkness_code = (uint8_t)value;
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
    for (unsigned field = 0U; field < (sizeof values / sizeof values[0]);
         ++field)
    {
        for (unsigned index = 0U;
             index < (sizeof values[0] / sizeof values[0][0]); ++index)
        {
            serializer_seed();
            mutate_metadata(field, values[field][index]);
            for (unsigned stage = 0U; stage < CAPTURE_CURSOR_CASES; ++stage)
            {
                bc_capture_cursor_t cursor = stage_cursor(stage);
                (void)binary_case("metadata", &cursor, BCAP_WIRE_MAX, 0U);
            }
        }
    }
    serializer_seed();
    g_capture.episode[1].start_ms = 8388608U;
    bc_capture_cursor_t cursor    = stage_cursor(0U);
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
    for (unsigned stage = 0U; stage < CAPTURE_CURSOR_CASES; ++stage)
    {
        bc_capture_cursor_t cursor = stage_cursor(stage);
        unsigned            exact =
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
    for (unsigned null_argument = 1U; null_argument <= NULL_ARGUMENT_CASES;
         ++null_argument)
    {
        bc_capture_cursor_t cursor = {0};
        assert(binary_case("null", &cursor, BCAP_WIRE_MAX, null_argument) ==
               0U);
    }
    for (unsigned index = 0U;
         index < sizeof(text_sizes) / sizeof(text_sizes[0]); ++index)
    {
        bc_capture_cursor_t cursor = {0};
        unsigned            calls  = 0U;
        while (text_case("text-traversal", &cursor, text_sizes[index]))
        {
            assert(++calls < 10U);
        }
        assert(cursor.b_is_end_done);
    }
    bc_capture_cursor_t cursor = {0};
    unsigned            calls  = 0U;
    while (binary_case("binary-traversal", &cursor, BCAP_WIRE_MAX, 0U) != 0U)
    {
        assert(++calls < 10U);
    }
    assert(cursor.b_is_end_done);
    cursor.episode = 15U;
    assert(binary_case("invalid-cursor", &cursor, BCAP_WIRE_MAX, 0U) == 0U);
}

/** @brief Cover packet count boundaries and drain a full 4000-sample pool. */
static void packet_count_cases (void)
{
    const unsigned counts[] = {0U, 1U, 23U, 24U, 25U, BCAP_POOL};
    for (unsigned index = 0U; index < sizeof(counts) / sizeof(counts[0]);
         ++index)
    {
        serializer_seed();
        g_capture.used             = (uint16_t)counts[index];
        g_capture.episode[1].count = (uint16_t)counts[index];
        bc_capture_cursor_t cursor = stage_cursor(2U);
        unsigned            exact =
            binary_case("sample-count", &cursor, BCAP_WIRE_MAX, 0U);
        assert(exact > 0U);
        cursor = stage_cursor(2U);
        assert(binary_case("sample-count-short", &cursor, exact - 1U, 0U) ==
               0U);
    }
    bc_capture_cursor_t cursor = {0};
    unsigned            calls  = 0U;
    while (binary_case("full-pool-traversal", &cursor, BCAP_WIRE_MAX, 0U) != 0U)
    {
        assert(++calls < 200U);
    }
    assert(cursor.b_is_end_done && 2U == cursor.episode);
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
    assert(39U == g_states && g_binary_cases >= 500U && 80U == g_text_cases);
    fprintf(stderr,
            "bcap_storage: states=%u binary=%u text=%u sizeof(bcap_t)=%zu\n",
            g_states, g_binary_cases, g_text_cases, sizeof(bc_capture_t));
    return 0;
}

/* End of file. */


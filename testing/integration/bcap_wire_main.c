/** @file bcap_wire_main.c
 * @brief Real capture encoder; vectors are independently decoded by Python.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
#include "bcap_wire_main.h"
#include "bc_capture.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define CAPTURE_CHUNK_SAMPLES 24U

#define MAX_SAMPLE_CHUNK_COUNT 24

static bc_capture_t g_cap;

static void vector (const char * p_tag, const uint8_t * p_packet,
                    unsigned packet_size);

/** @brief Emit real capture packet vectors for independent decoding and
 * check cursor behavior and reserved encodings.
 * @details The serializer must preserve the cursor and destination on
 * capacity failure. Validate reserved surface, darkness, ADC and summary
 * values before emitting packets. Follow the cursor over multiple episodes
 * and partial batches, then the exact 4000-sample endpoint and end packet.
 */
int main (void)
{
    uint8_t             packet[BCAP_WIRE_MAX];
    uint8_t             guard[BCAP_WIRE_MAX];
    bc_capture_cursor_t cur = {0};
    bc_capture_cursor_t before;
    unsigned            packet_size;
    unsigned            packets = 0;
    g_cap.run_start_us          = 0xfffffff0U;
    g_cap.episodes              = 1;
    g_cap.used                  = 3;
    g_cap.dropped               = 7;
    g_cap.decoded               = 1;
    g_cap.letters[0]            = 'Z';
    g_cap.letter_count          = 1;

    /* clang-format off */
    g_cap.episode[0] =
        (bc_capture_episode_t){.first = 0, .count = 3, .start_ms = 20};
    g_cap.pool[0]    = (bc_capture_sample_t){.raw           = 100,
                                          .darkness_code = 0,
                                          .flags         = 1,
                                          .distance_mm   = 0,
                                          .dt_ms         = 9};
    g_cap.pool[1]    = (bc_capture_sample_t){.raw           = 3500,
                                          .darkness_code = 250,
                                          .flags         = 34,
                                          .distance_mm   = 1,
                                          .dt_ms         = 10};
    g_cap.pool[2]    = (bc_capture_sample_t){.raw           = 200,
                                          .darkness_code = 4,
                                          .flags         = 5,
                                          .distance_mm   = 2,
                                          .dt_ms         = 0};
    /* clang-format on */
    memset(packet, 0xa5, sizeof packet);
    memcpy(guard, packet, sizeof guard);
    before = cur;
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, 5));
    assert(!memcmp(&cur, &before, sizeof cur) &&
           !memcmp(packet, guard, sizeof guard));

    for (;;)
    {
        packet_size =
            bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet);

        if (0U == packet_size)
        {
            break;
        }

        assert(packet_size <= BCAP_WIRE_MAX && 2 == (int)packet[0]);
        vector("BCAP_VECTOR", packet, packet_size);
        packets++;
    }

    assert(4U == packets && cur.b_is_end_done);
    /* Max-length summary, header timestamp and all 24 maximum-valued samples.
     */
    g_cap.letter_count = 8;
    memcpy(g_cap.letters, "ABCDEFGZ", 8);
    g_cap.used                = MAX_SAMPLE_CHUNK_COUNT;
    g_cap.episode[0].count    = MAX_SAMPLE_CHUNK_COUNT;
    g_cap.episode[0].start_ms = 4294967U;

    for (unsigned sample_index = 0;
         sample_index < (unsigned)MAX_SAMPLE_CHUNK_COUNT; ++sample_index)
    {
        g_cap.pool[sample_index] = (bc_capture_sample_t){.raw           = 4095,
                                                         .darkness_code = 250,
                                                         .flags         = 0xf2,
                                                         .distance_mm   = 65535,
                                                         .dt_ms         = 255};
    }

    cur = (bc_capture_cursor_t){0};
    assert(bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet) == 21U);
    assert(bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet) == 11U);
    before = cur;
    memset(packet, 0xa5, sizeof packet);
    memcpy(guard, packet, sizeof guard);
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, 163));
    assert(!memcmp(&cur, &before, sizeof cur) &&
           !memcmp(packet, guard, sizeof guard));
    assert(bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet) == 164U);
    vector("BCAP_BOUNDARY", packet, 164);
    /* Wireless batches do not change the six-sample USB text cursor. */
    bc_capture_cursor_t usb = {.b_is_summary_done = 1, .b_is_header_done = 1};
    char                line[160];
    assert(bc_capture_format_next(&g_cap, &usb, line, sizeof line) &&
           6 == (int)usb.sample);
    /* Invalid data never advances the cursor or silently truncates. */
    /* clang-format off */
    cur               =
        (bc_capture_cursor_t){.b_is_summary_done = 1, .b_is_header_done = 1};
    before            = cur;
    g_cap.pool[0].raw = 4096;
    /* clang-format on */
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet));
    assert(!memcmp(&cur, &before, sizeof cur));
    g_cap.pool[0].raw           = 0;
    g_cap.pool[0].darkness_code = 251;
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet));
    g_cap.pool[0].darkness_code = 0;
    g_cap.pool[0].flags         = 3;
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet));
    g_cap.pool[0].flags = 0;
    g_cap.letter_count  = 9;
    cur                 = (bc_capture_cursor_t){0};
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet));
    /* Multiple batches and episodes retain every sample, including short tails.
     */
    memset(&g_cap, 0, sizeof g_cap);
    g_cap.run_start_us = 123;
    g_cap.episodes     = 2;
    g_cap.used         = 76;

    g_cap.episode[0] =
        (bc_capture_episode_t){.first = 0, .count = 49, .start_ms = 0};
    g_cap.episode[1] =
        (bc_capture_episode_t){.first = 49, .count = 27, .start_ms = 1000};

    for (unsigned sample_index = 0; sample_index < g_cap.used; ++sample_index)
    {
        g_cap.pool[sample_index] =
            (bc_capture_sample_t){.raw           = sample_index,
                                  .darkness_code = sample_index,
                                  .flags         = 0,
                                  .distance_mm   = sample_index,
                                  .dt_ms         = 10};
    }

    cur     = (bc_capture_cursor_t){0};
    packets = 0;

    for (;;)
    {
        packet_size =
            bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet);

        if (0U == packet_size)
        {
            break;
        }

        vector("BCAP_BATCH_VECTOR", packet, packet_size);
        packets++;
    }

    assert(9U == packets && cur.b_is_end_done && 2 == (int)cur.episode);
    before = cur;
    memcpy(guard, packet, sizeof guard);
    assert(!bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet));
    assert(!memcmp(&cur, &before, sizeof cur) &&
           !memcmp(packet, guard, sizeof guard));
    /* Final full batch reaches exactly the capture pool's boundary. */
    g_cap.episodes         = 1;
    g_cap.used             = BCAP_POOL;
    g_cap.episode[0].count = BCAP_POOL;

    for (unsigned sample_index = BCAP_POOL - CAPTURE_CHUNK_SAMPLES;
         sample_index < BCAP_POOL; ++sample_index)
    {
        g_cap.pool[sample_index] =
            (bc_capture_sample_t){.raw           = sample_index,
                                  .darkness_code = 0,
                                  .flags         = 0,
                                  .distance_mm   = sample_index,
                                  .dt_ms         = 0};
    }

    cur = (bc_capture_cursor_t){.b_is_summary_done = 1,
                                .b_is_header_done  = 1,
                                .sample            = BCAP_POOL - 24U};
    assert(bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet) ==
               164U &&
           BCAP_POOL == cur.sample);
    vector("BCAP_TAIL", packet, 164);
    assert(bc_capture_binary_next(&g_cap, &cur, packet, sizeof packet) == 6U &&
           cur.b_is_end_done);
    puts("PASS: binary barcode boundaries, zero bytes, exact lengths, "
         "cursor safety and rejected reserved ranges");
}

/** @brief Print the supplied label and exactly n bytes as hexadecimal for
 * wire-vector comparison.
 */
static void vector (const char * p_tag, const uint8_t * p_packet,
                    unsigned packet_size)
{
    printf("%s ", p_tag);

    for (unsigned byte_index = 0; byte_index < packet_size; ++byte_index)
    {
        printf("%02x", p_packet[byte_index]);
    }

    puts("");
}

/* End of file. */


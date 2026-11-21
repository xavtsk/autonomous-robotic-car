/** @file bc_capture.c
 * @brief Barcode-sensor capture for diagnosing barcode reading on the course.
 * Capture flags are a nonnegative octet. Widen before extracting status bits.
 * @see docs/BARCODE_WIRE_PROTOCOL.md for the on-wire field widths and
 * packet kinds.
 * Byte and half-word values fit in int, and uint32_t fits in int64_t.
 * The casts make the existing arithmetic promotions explicit.
 * Byte and flag masks are widened unsigned before bitwise operations.
 */
#include "bc_capture.h"
#include "../app_types.h"
#include <math.h>
#include <stddef.h>

#define BCAP_GAP_FLAG_US (30000U)
#define BCAP_POSITION_FRAC_BITS 8U
#define BCAP_POSITION_INVALID INT64_MIN

static void append_text (char * p_out, unsigned int size,
                         unsigned int * p_length, const char * p_text);
static void append_unsigned (char * p_out, unsigned int size,
                             unsigned int * p_length, uint32_t remaining);

static bc_capture_sample_t make_sample (bc_capture_t *             p_capture,
                                        const bc_capture_input_t * p_in);
static int64_t             position_qmm (float64_t metres);
static uint16_t            relative_distance_mm (int64_t origin, int64_t now);
static void                close_episode (bc_capture_t * p_capture);
static void write_bits (uint8_t * p_out, unsigned * p_bit, uint32_t field_value,
                        unsigned width);

/** @brief Initialize the caller's capture storage for a new run using the
 * wrapping microsecond start time.
 */
void bc_capture_begin_run (bc_capture_t * p_store, uint32_t now_us)
{
    *p_store              = (bc_capture_t){0};
    p_store->run_start_us = now_us;
}

/** @brief Update run counters and record one timestamped barcode sample.
 * @param[in,out] p_store Initialized run storage owned by the sampling task.
 * @param[in] p_in Borrowed observation, with microseconds from the run clock
 * and centre distance in metres.
 * @details Keep recent samples ahead of the trigger. Start an episode on a
 * black classification or an active recorder. A full episode table or
 * sample pool can prevent an opening while the outcome counters continue.
 * @note This captures evidence and does not change the decoder decision.
 */
void bc_capture_record (bc_capture_t * p_store, const bc_capture_input_t * p_in)
{
    bc_capture_sample_t sample   = make_sample(p_store, p_in);
    const int64_t       position = position_qmm(p_in->centre_m);

    if (1U == p_in->status)
    {
        p_store->decoded++;

        if (p_store->letter_count < (sizeof p_store->letters - 1U))
        {
            p_store->letters[p_store->letter_count++] = p_in->letter;
            p_store->letters[p_store->letter_count]   = '\0';
        }
    }
    else if (2U == p_in->status)
    {
        p_store->rejected++;
    }
    else if (3U == p_in->status)
    {
        p_store->timing++;
    }
    else if (4U == p_in->status)
    {
        p_store->overflow++;
    }
    else
    {
        /* Other decoder statuses do not increment these capture counters. */
    }

    /* Start on BLACK, or as soon as a barcode reader starts collecting: the
     * record-then-decide reader starts on bars too faint to read BLACK. */
    const bool b_should_open = (!p_store->b_is_active) &&
                               ((2U == p_in->surface) ||
                                (p_in->b_is_collecting)); /* IR_SURFACE_BLACK */
    /* The sum of the two unsigned counters fits in a 32-bit int. */
    if (b_should_open &&
        ((p_store->episodes >= BCAP_MAX_EPISODES) ||
         (((unsigned)(p_store->used + p_store->pre_count) + 1U) > BCAP_POOL)))
    {
        if (p_store->dropped < 255U)
        {
            p_store->dropped++;
        }
    }
    else if (b_should_open)
    {
        bc_capture_episode_t * p_new_episode =
            &p_store->episode[p_store->episodes];
        const unsigned int oldest =
            (p_store->pre_head + BCAP_PRE - p_store->pre_count) % BCAP_PRE;
        /* clang-format off */
        p_new_episode->first      = p_store->used;
        p_new_episode->count      = 0U;
        p_new_episode->origin_qmm =
            (p_store->pre_count != 0U) ? p_store->pre_qmm[oldest] : position;
        /* clang-format on */
        {
            /* pre_count is at most BCAP_PRE. Widen before converting
             * the buffered sample count to milliseconds. */
            const uint32_t now_ms =
                (p_in->t_us - p_store->run_start_us) / 1000U;
            const uint32_t back_ms =
                (uint32_t)p_store->pre_count * S3_SAMPLE_PERIOD_MS;
            p_new_episode->start_ms = now_ms > back_ms ? now_ms - back_ms : 0U;
        }
        unsigned int history_index = 0U;
        unsigned int ring_index    = oldest;

        while (history_index < p_store->pre_count)
        {
            bc_capture_sample_t prior = p_store->pre[ring_index];
            prior.distance_mm         = relative_distance_mm(
                p_new_episode->origin_qmm, p_store->pre_qmm[ring_index]);

            p_store->pool[p_store->used++] = prior;
            p_new_episode->count++;

            if (BCAP_PRE == ++ring_index)
            {
                ring_index = 0U;
            }

            history_index++;
        }

        p_store->b_is_active = true;
        p_store->quiet       = 0U;
    }
    else
    {
        /* The existing capture or pre-trigger history continues below. */
    }

    if (p_store->b_is_active)
    {
        bc_capture_episode_t * p_new_episode =
            &p_store->episode[p_store->episodes];

        if (p_store->used < BCAP_POOL)
        {
            sample.distance_mm =
                relative_distance_mm(p_new_episode->origin_qmm, position);

            p_store->pool[p_store->used++] = sample;
            p_new_episode->count++;
        }

        /* Count consecutive white samples. The capture pool bounds the quiet
         * count, so the increment still fits in uint16_t. */
        p_store->quiet =
            (1U == p_in->surface) ? (uint16_t)(p_store->quiet + 1U) : 0U;

        /* End the episode after enough quiet and once the reader finishes.
         * A full sample pool ends the episode immediately. */
        if (((p_store->quiet >= BCAP_END_QUIET) && (!p_in->b_is_collecting)) ||
            (p_store->used >= BCAP_POOL))
        {
            close_episode(p_store);
            p_store->pre_count = 0U; /* Start a fresh pre-trigger window. */
        }

        return;
    }

    p_store->pre[p_store->pre_head]     = sample;
    p_store->pre_qmm[p_store->pre_head] = position;
    /* The modulo leaves a history index from zero through BCAP_PRE - 1.
     * The current 300-entry history fits both uint16_t and the retained
     * nine-bit field. */
    p_store->pre_head = (uint16_t)((p_store->pre_head + 1U) % BCAP_PRE);

    if (p_store->pre_count < BCAP_PRE)
    {
        p_store->pre_count++;
    }
}

/** @brief Finalize an active barcode episode at the end of the run.
 */
void bc_capture_end_run (bc_capture_t * p_store)
{
    close_episode(p_store);
}

/** @brief Format and advance the next line of a USB capture export.
 * @param[in] p_store Stable capture storage locked by the caller.
 * @param[in,out] p_cur Initialized cursor into the same capture.
 * @param[out] p_out Writable buffer of size bytes, with room for a terminator.
 * @param size Buffer capacity in bytes. At least one byte is required.
 * @return True after consuming a record. A short buffer truncates the text
 * without rolling back the cursor. False leaves the output as an empty string.
 */
bool bc_capture_format_next (const bc_capture_t *  p_store,
                             bc_capture_cursor_t * p_cur, char * p_out,
                             unsigned int size)
{
    unsigned int characters = 0U;
    p_out[0]                = '\0';

    if (!p_cur->b_is_summary_done)
    {
        p_cur->b_is_summary_done = true;
        append_text(p_out, size, &characters, "[BCAP] run summary: episodes=");
        append_unsigned(p_out, size, &characters, p_store->episodes);
        append_text(p_out, size, &characters, " not_stored=");
        append_unsigned(p_out, size, &characters, p_store->dropped);
        append_text(p_out, size, &characters, " decoded=");
        append_unsigned(p_out, size, &characters, p_store->decoded);
        append_text(p_out, size, &characters, " letters=");
        append_text(p_out, size, &characters,
                    p_store->letter_count ? p_store->letters : "-");
        append_text(p_out, size, &characters, " rejected=");
        append_unsigned(p_out, size, &characters, p_store->rejected);
        append_text(p_out, size, &characters, " timing_error=");
        append_unsigned(p_out, size, &characters, p_store->timing);
        append_text(p_out, size, &characters, " overflow=");
        append_unsigned(p_out, size, &characters, p_store->overflow);
        append_text(p_out, size, &characters, " | raw:dark4:flags:mm:dt");
        return true;
    }

    while (p_cur->episode < p_store->episodes)
    {
        const bc_capture_episode_t * p_episode =
            &p_store->episode[p_cur->episode];
        unsigned int line_sample_index;

        if (!p_cur->b_is_header_done)
        {
            p_cur->b_is_header_done = true;
            p_cur->sample           = 0U;
            append_text(p_out, size, &characters, "[BCAP] episode ");
            append_unsigned(p_out, size, &characters, p_cur->episode + 1U);
            append_text(p_out, size, &characters, " t=");
            append_unsigned(p_out, size, &characters, p_episode->start_ms);
            append_text(p_out, size, &characters, "ms samples=");
            append_unsigned(p_out, size, &characters, p_episode->count);
            return true;
        }

        if (p_cur->sample >= p_episode->count)
        {
            p_cur->episode++;
            p_cur->b_is_header_done = false;
            continue;
        }

        append_text(p_out, size, &characters, "BC");
        append_unsigned(p_out, size, &characters, p_cur->episode + 1U);
        append_text(p_out, size, &characters, " ");
        append_unsigned(p_out, size, &characters, p_cur->sample);
        append_text(p_out, size, &characters, ":");

        for (line_sample_index = 0U; (line_sample_index < BCAP_PER_LINE) &&
                                     (p_cur->sample < p_episode->count);
             line_sample_index++)
        {
            const bc_capture_sample_t * p_sample =
                &p_store->pool[p_episode->first + p_cur->sample];
            append_text(p_out, size, &characters, " ");
            append_unsigned(p_out, size, &characters, p_sample->raw);
            append_text(p_out, size, &characters, ":");
            append_unsigned(p_out, size, &characters, p_sample->darkness_code);
            append_text(p_out, size, &characters, ":");
            append_unsigned(p_out, size, &characters, p_sample->flags);
            append_text(p_out, size, &characters, ":");
            append_unsigned(p_out, size, &characters, p_sample->distance_mm);
            append_text(p_out, size, &characters, ":");
            append_unsigned(p_out, size, &characters, p_sample->dt_ms);
            p_cur->sample++;
        }

        return true;
    }

    if (!p_cur->b_is_end_done)
    {
        p_cur->b_is_end_done = true;
        append_text(p_out, size, &characters, "[BCAP] end");
        return true;
    }

    return false;
}

/** @brief Serialize the next versioned capture record for a borrowed store.
 * @details The cursor visits the summary, then each episode header and its
 * sample chunks, then one end record. Select and validate that record in a
 * temporary cursor before zeroing and filling the requested output range.
 * @return The encoded byte length. Zero means invalid input, insufficient
 * capacity or a finished stream. These early returns preserve both outputs.
 * @see docs/BARCODE_WIRE_PROTOCOL.md for widths and cursor sequence.
 */
unsigned bc_capture_binary_next (const bc_capture_t *  p_store,
                                 bc_capture_cursor_t * p_cur, uint8_t * p_out,
                                 unsigned size)
{
    bc_capture_cursor_t          next;
    unsigned                     bit = 0;
    unsigned                     kind;
    unsigned                     count = 0;
    unsigned                     bits;
    const bc_capture_episode_t * p_episode = NULL;

    if ((((((0 == p_store) || (0 == p_cur)) || (0 == p_out)) ||
          (p_store->episodes > BCAP_MAX_EPISODES)) ||
         (p_store->letter_count > 8U)) ||
        (p_cur->episode > p_store->episodes))
    {
        return 0;
    }

    next = *p_cur;

    if (!next.b_is_summary_done)
    {
        kind = 0;
        bits = 122U + (5U * p_store->letter_count);

        for (unsigned letter_index = 0; letter_index < p_store->letter_count;
             ++letter_index)
        {
            if (((int)p_store->letters[letter_index] < 'A') ||
                ((int)p_store->letters[letter_index] > 'Z'))
            {
                return 0;
            }
        }
    }
    else
    {
        while (next.episode < p_store->episodes)
        {
            p_episode = &p_store->episode[next.episode];

            if ((((p_episode->count > BCAP_POOL) ||
                  (((unsigned)p_episode->first + p_episode->count) >
                   p_store->used)) ||
                 (p_store->used > BCAP_POOL)) ||
                (p_episode->start_ms > 4294967U))
            {
                return 0;
            }

            if ((!next.b_is_header_done) || (next.sample < p_episode->count))
            {
                break;
            }

            next.episode++;
            next.b_is_header_done = false;
        }

        if (next.episode < p_store->episodes)
        {
            p_episode = &p_store->episode[next.episode];

            if (!next.b_is_header_done)
            {
                kind = 1;
                bits = 81U;
            }
            else
            {
                kind  = 2;
                count = p_episode->count - next.sample;

                count = (count > BCAP_WIRE_SAMPLES) ? BCAP_WIRE_SAMPLES : count;

                bits = 63U + (52U * count);

                for (unsigned sample_index = 0; sample_index < count;
                     ++sample_index)
                {
                    /* The two nonnegative offsets fit in int before conversion.
                     */
                    const bc_capture_sample_t * p_capture_sample =
                        &p_store
                             ->pool[(unsigned)(p_episode->first + next.sample) +
                                    sample_index];

                    if ((((p_capture_sample->raw > 4095U) ||
                          (p_capture_sample->darkness_code > 250U)) ||
                         ((p_capture_sample->flags & 3U) > 2U)) ||
                        ((((unsigned)p_capture_sample->flags >> 2U) & 7U) > 4U))
                    {
                        return 0;
                    }
                }
            }
        }
        else if (next.b_is_end_done)
        {
            return 0;
        }
        else
        {
            kind = 3;
            bits = 42U;
        }
    }

    const unsigned length = (bits + 7U) / 8U;

    if (size < length)
    {
        return 0;
    }

    for (unsigned byte_index = 0; byte_index < length; ++byte_index)
    {
        p_out[byte_index] = 0;
    }

    write_bits(p_out, &bit, BCAP_WIRE_VERSION, 8);
    write_bits(p_out, &bit, p_store->run_start_us, 32);
    write_bits(p_out, &bit, kind, 2);

    if (0U == kind)
    {
        write_bits(p_out, &bit, p_store->episodes, 4);
        write_bits(p_out, &bit, p_store->dropped, 8);
        write_bits(p_out, &bit, p_store->decoded, 16);
        write_bits(p_out, &bit, p_store->rejected, 16);
        write_bits(p_out, &bit, p_store->timing, 16);
        write_bits(p_out, &bit, p_store->overflow, 16);
        write_bits(p_out, &bit, p_store->letter_count, 4);

        for (unsigned letter_index = 0; letter_index < p_store->letter_count;
             ++letter_index)
        {
            write_bits(p_out, &bit, (int)p_store->letters[letter_index] - 'A',
                       5);
        }

        next.b_is_summary_done = true;
    }
    else if (1U == kind)
    {
        write_bits(p_out, &bit, next.episode, 4);
        write_bits(p_out, &bit, p_episode->start_ms, 23);
        write_bits(p_out, &bit, p_episode->count, 12);
        next.b_is_header_done = true;
        next.sample           = 0;
    }
    else if (2U == kind)
    {
        write_bits(p_out, &bit, next.episode, 4);
        write_bits(p_out, &bit, next.sample, 12);
        write_bits(p_out, &bit, count - 1U, 5);

        for (unsigned sample_index = 0; sample_index < count; ++sample_index)
        {
            /* The two nonnegative offsets fit in int before conversion. */
            const bc_capture_sample_t * p_capture_sample =
                &p_store->pool[(unsigned)(p_episode->first + next.sample) +
                               sample_index];
            write_bits(p_out, &bit, p_capture_sample->raw, 12);
            write_bits(p_out, &bit, p_capture_sample->darkness_code, 8);
            write_bits(p_out, &bit, p_capture_sample->flags, 8);
            write_bits(p_out, &bit, p_capture_sample->distance_mm, 16);
            write_bits(p_out, &bit, p_capture_sample->dt_ms, 8);
        }

        next.sample += count;
    }
    else
    {
        next.b_is_end_done = true;
    }

    *p_cur = next;
    return length;
}

/** @brief Append a terminated string to the capture line, bounded by the
 * caller's buffer and cursor.
 */
static void append_text (char * p_out, unsigned int size,
                         unsigned int * p_length, const char * p_text)
{
    while (((int)*p_text != '\0') && ((*p_length + 1U) < size))
    {
        p_out[(*p_length)++] = *p_text++;
    }

    p_out[*p_length] = '\0';
}

/** @brief Append an unsigned value in decimal to the bounded capture line and
 * advance its character cursor.
 */
static void append_unsigned (char * p_out, unsigned int size,
                             unsigned int * p_length, uint32_t remaining)
{
    char         digits[11];
    unsigned int digit_count = 0U;

    do
    {
        /* The remainder is zero through nine. C digits are contiguous. */
        digits[digit_count++] = (char)('0' + (int)(remaining % 10U));
        remaining            /= 10U;
    } while ((remaining != 0U) && (digit_count < sizeof digits));

    while ((digit_count > 0U) && ((*p_length + 1U) < size))
    {
        p_out[(*p_length)++] = digits[--digit_count];
    }

    p_out[*p_length] = '\0';
}

/** @brief Quantize one capture input, mark long inter-sample gaps and update
 * the owner's previous timestamp. The result retains raw reading and
 * capture flags.
 */
static bc_capture_sample_t make_sample (bc_capture_t *             p_capture,
                                        const bc_capture_input_t * p_in)
{
    bc_capture_sample_t sample;

    /* Compute elapsed microseconds modulo 2^32 across timer rollover. */
    const uint32_t sample_gap_us =
        p_capture->b_has_last_sample
            ? (uint32_t)(p_in->t_us - p_capture->last_us)
            : 0U;
    sample.raw         = p_in->raw;
    sample.distance_mm = 0U;
    /* The clamped darkness becomes 0..250 after division. The masks and
     * flags occupy exactly eight bits. Whole milliseconds saturate at 255. */
    /* clang-format off */
    sample.darkness_code =
        (uint8_t)((p_in->darkness > 1000U ? 1000U : p_in->darkness) / 4U);
    sample.flags         =
        (uint8_t)((p_in->surface & 3U) | ((p_in->status & 7U) << 2) |
                             (p_in->b_is_collecting ? 0x20U : 0U) |
                             (p_in->b_is_rearming ? 0x40U : 0U) |
                             (sample_gap_us > BCAP_GAP_FLAG_US ? 0x80U : 0U));
    sample.dt_ms         =
        (uint8_t)((sample_gap_us / 1000U) > 255U ? 255U
                                                 : sample_gap_us / 1000U);
    /* clang-format on */

    p_capture->last_us           = p_in->t_us;
    p_capture->b_has_last_sample = true;
    return sample;
}

/** @brief Convert metres to signed units of one 256th of a millimetre. Reserve
 * INT64_MIN for non-finite input and clamp finite overflow before
 * narrowing.
 */
static int64_t position_qmm (float64_t metres)
{
    /* One soft-float conversion per arriving sample. In particular, copying
     * a trigger must not reconvert the 300 samples in the history.
     * The integrated sampling task calls this path every 5 ms.
     * Q8 millimetres retain sub-0.008 mm relative precision before the wire's
     * existing whole-millimetre rounding, over practical absolute odometry.
     * Clamp before the cast: 2^63 is exactly representable as a double, while
     * INT64_MAX is not. Reserve INT64_MIN for unavailable/non-finite input. */
    if (!isfinite(metres))
    {
        return BCAP_POSITION_INVALID;
    }

    const float64_t units = metres * 256000.0;

    if (units >= 0x1p63)
    {
        return INT64_MAX;
    }

    if (units <= -0x1p63)
    {
        return INT64_MIN + 1;
    }

    return (int64_t)units;
}

/** @brief Return rounded nonnegative travel in millimetres from two
 * fixed-point positions. Invalid or non-increasing positions yield
 * zero. Large distances saturate at UINT16_MAX.
 */
static uint16_t relative_distance_mm (int64_t origin, int64_t now)
{
    if (((BCAP_POSITION_INVALID == origin) || (BCAP_POSITION_INVALID == now)) ||
        (now <= origin))
    {
        return 0U;
    }
    /* Unsigned subtraction avoids signed overflow for widely separated
     * finite/clamped positions. Narrow only after the wire-limit check. */
    const uint64_t delta = (uint64_t)now - (uint64_t)origin;

    /* Widen the 16-bit limit before shifting it into Q8 millimetres. */
    if (delta >= ((uint32_t)UINT16_MAX << BCAP_POSITION_FRAC_BITS))
    {
        return UINT16_MAX;
    }
    /* The remaining Q8 distance fits in 24 bits. Rounding can reach 65535. */
    return (uint16_t)(((uint32_t)delta + 128U) >> BCAP_POSITION_FRAC_BITS);
}

/** @brief Close a currently active episode and count it once. An inactive
 * episode leaves the count unchanged.
 */
static void close_episode (bc_capture_t * p_capture)
{
    if (p_capture->b_is_active)
    {
        p_capture->b_is_active = false;
        p_capture->episodes++;
    }
}

/** @brief Append a field's bits to a zero-initialized binary output, least
 * significant bit first, advancing the caller's bit cursor.
 */
static void write_bits (uint8_t * p_out, unsigned * p_bit, uint32_t field_value,
                        unsigned width)
{
    unsigned bit_offset = 0;

    while (bit_offset < width)
    {
        /* A packet bit modulo eight gives a one-bit mask from 1 to 128. */
        p_out[*p_bit / 8U] =
            (uint8_t)((unsigned)p_out[*p_bit / 8U] |
                      (((field_value >> bit_offset) & 1U) << (*p_bit % 8U)));
        ++bit_offset;
        ++*p_bit;
    }
}

/* End of file. */


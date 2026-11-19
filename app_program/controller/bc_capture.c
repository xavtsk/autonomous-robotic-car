#include "bc_capture.h"
#include <math.h>

#define BCAP_GAP_FLAG_US (30000U)
#define BCAP_POSITION_FRAC_BITS 8U
#define BCAP_POSITION_INVALID INT64_MIN

static void put (char * p_out, unsigned int size, unsigned int * p_n,
                 const char * p_s)
{
    while ((*p_s != '\0') && ((*p_n + 1U) < size))
    {
        p_out[(*p_n)++] = *p_s++;
    }
    p_out[*p_n] = '\0';
}

static void put_u (char * p_out, unsigned int size, unsigned int * p_n,
                   uint32_t value_v)
{
    char tmp[11];
    unsigned int value_i = 0U;
    do
    {
        tmp[value_i++] = (char)('0' + (value_v % 10U));
        value_v /= 10U;
    } while ((value_v != 0U) && (value_i < sizeof tmp));
    while ((value_i > 0U) && ((*p_n + 1U) < size))
    {
        p_out[(*p_n)++] = tmp[--value_i];
    }
    p_out[*p_n] = '\0';
}

void bcap_begin_run (bcap_t * p_c, uint32_t now_us)
{
    *p_c = (bcap_t){0};
    p_c->run_start_us = now_us;
}

static bcap_sample_t make_sample (bcap_t * p_c, const bcap_input_t * p_in)
{
    bcap_sample_t value_s;
    uint32_t value_dt =
        p_c->have_last ? (uint32_t)(p_in->t_us - p_c->last_us) : 0U;
    value_s.raw = p_in->raw;
    value_s.mm = 0U;
    value_s.dark4 =
        (uint8_t)((p_in->darkness > 1000U ? 1000U : p_in->darkness) / 4U);
    value_s.flags =
        (uint8_t)((p_in->surface & 3U) | ((p_in->status & 7U) << 2) |
                  (p_in->collecting ? 0x20U : 0U) |
                  (p_in->rearming ? 0x40U : 0U) |
                  (value_dt > BCAP_GAP_FLAG_US ? 0x80U : 0U));
    value_s.dt_ms =
        (uint8_t)(value_dt / 1000U > 255U ? 255U : value_dt / 1000U);
    p_c->last_us = p_in->t_us;
    p_c->have_last = true;
    return value_s;
}

static int64_t position_qmm (double metres)
{
    /* One soft-float conversion per arriving sample. In particular, copying
     * the 300-sample history at a 200 Hz trigger must not repeat that work.
     * Q8 millimetres retain sub-0.008 mm relative precision before the wire's
     * existing whole-millimetre rounding, over practical absolute odometry.
     * Clamp before the cast: 2^63 is exactly representable as a double, while
     * INT64_MAX is not. Reserve INT64_MIN for unavailable/non-finite input. */
    if (!isfinite(metres))
    {
        return BCAP_POSITION_INVALID;
    }
    const double units = metres * 256000.0;
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

static uint16_t rel_mm (int64_t origin, int64_t now)
{
    if (origin == BCAP_POSITION_INVALID || now == BCAP_POSITION_INVALID ||
        now <= origin)
    {
        return 0U;
    }
    /* Unsigned subtraction avoids signed overflow for widely separated
     * finite/clamped positions. Narrow only after the wire-limit check. */
    const uint64_t delta = (uint64_t)now - (uint64_t)origin;
    if (delta >= ((uint32_t)UINT16_MAX << BCAP_POSITION_FRAC_BITS))
    {
        return UINT16_MAX;
    }
    return (uint16_t)(((uint32_t)delta + 128U) >> BCAP_POSITION_FRAC_BITS);
}

static void close_episode (bcap_t * p_c)
{
    if (p_c->active)
    {
        p_c->active = false;
        p_c->episodes++;
    }
}

void bcap_record (bcap_t * p_c, const bcap_input_t * p_in)
{
    bcap_sample_t value_s = make_sample(p_c, p_in);
    const int64_t position = position_qmm(p_in->centre_m);
    if (p_in->status == 1U)
    {
        p_c->decoded++;
        if (p_c->letter_count < (sizeof p_c->letters - 1U))
        {
            p_c->letters[p_c->letter_count++] = p_in->letter;
            p_c->letters[p_c->letter_count] = '\0';
        }
    }
    else if (p_in->status == 2U)
    {
        p_c->rejected++;
    }
    else if (p_in->status == 3U)
    {
        p_c->timing++;
    }
    else if (p_in->status == 4U)
    {
        p_c->overflow++;
    }

    /* Start on BLACK, or as soon as a barcode reader starts collecting: the
     * record-then-decide reader starts on bars too faint to read BLACK. */
    if (!p_c->active &&
        ((p_in->surface == 2U) || p_in->collecting)) /* IR_SURFACE_BLACK */
    {
        if ((p_c->episodes >= BCAP_MAX_EPISODES) ||
            ((p_c->used + p_c->pre_count + 1U) > BCAP_POOL))
        {
            if (p_c->dropped < 255U)
            {
                p_c->dropped++;
            }
        }
        else
        {
            bcap_episode_t * p_e = &p_c->episode[p_c->episodes];
            unsigned int value_i, value_k;
            const unsigned int oldest =
                (p_c->pre_head + BCAP_PRE - p_c->pre_count) % BCAP_PRE;
            p_e->first = p_c->used;
            p_e->count = 0U;
            p_e->origin_qmm =
                (p_c->pre_count != 0U) ? p_c->pre_qmm[oldest] : position;
            {
                const uint32_t now_ms =
                    (p_in->t_us - p_c->run_start_us) / 1000U;
                const uint32_t back_ms =
                    (uint32_t)p_c->pre_count * S3_SAMPLE_PERIOD_MS;
                p_e->start_ms = now_ms > back_ms ? now_ms - back_ms : 0U;
            }
            for (value_i = 0U, value_k = oldest; value_i < p_c->pre_count;
                 value_i++)
            {
                bcap_sample_t value_p = p_c->pre[value_k];
                value_p.mm = rel_mm(p_e->origin_qmm, p_c->pre_qmm[value_k]);
                p_c->pool[p_c->used++] = value_p;
                p_e->count++;
                if (++value_k == BCAP_PRE)
                {
                    value_k = 0U;
                }
            }
            p_c->active = true;
            p_c->quiet = 0U;
        }
    }
    if (p_c->active)
    {
        bcap_episode_t * p_e = &p_c->episode[p_c->episodes];
        if (p_c->used < BCAP_POOL)
        {
            value_s.mm = rel_mm(p_e->origin_qmm, position);
            p_c->pool[p_c->used++] = value_s;
            p_e->count++;
        }
        p_c->quiet = (p_in->surface == 1U) ? (uint16_t)(p_c->quiet + 1U)
                                           : 0U; /* WHITE */
        /* Keep one barcode in one episode: not while a reader still collects.
         */
        if (((p_c->quiet >= BCAP_END_QUIET) && !p_in->collecting) ||
            (p_c->used >= BCAP_POOL))
        {
            close_episode(p_c);
            p_c->pre_count = 0U; /* the next pre-trigger window starts afresh */
        }
        return;
    }
    p_c->pre[p_c->pre_head] = value_s;
    p_c->pre_qmm[p_c->pre_head] = position;
    p_c->pre_head = (uint16_t)((p_c->pre_head + 1U) % BCAP_PRE);
    if (p_c->pre_count < BCAP_PRE)
    {
        p_c->pre_count++;
    }
}

void bcap_end_run (bcap_t * p_c)
{
    close_episode(p_c);
}

bool bcap_format_next (const bcap_t * p_c, bcap_cursor_t * p_cur, char * p_out,
                       unsigned int size)
{
    unsigned int value_n = 0U;
    p_out[0] = '\0';
    if (!p_cur->summary_done)
    {
        p_cur->summary_done = true;
        put(p_out, size, &value_n, "[BCAP] run summary: episodes=");
        put_u(p_out, size, &value_n, p_c->episodes);
        put(p_out, size, &value_n, " not_stored=");
        put_u(p_out, size, &value_n, p_c->dropped);
        put(p_out, size, &value_n, " decoded=");
        put_u(p_out, size, &value_n, p_c->decoded);
        put(p_out, size, &value_n, " letters=");
        put(p_out, size, &value_n, p_c->letter_count ? p_c->letters : "-");
        put(p_out, size, &value_n, " rejected=");
        put_u(p_out, size, &value_n, p_c->rejected);
        put(p_out, size, &value_n, " timing_error=");
        put_u(p_out, size, &value_n, p_c->timing);
        put(p_out, size, &value_n, " overflow=");
        put_u(p_out, size, &value_n, p_c->overflow);
        put(p_out, size, &value_n, " | raw:dark4:flags:mm:dt");
        return true;
    }
    while (p_cur->episode < p_c->episodes)
    {
        const bcap_episode_t * p_e = &p_c->episode[p_cur->episode];
        unsigned int value_i;
        if (!p_cur->header_done)
        {
            p_cur->header_done = true;
            p_cur->sample = 0U;
            put(p_out, size, &value_n, "[BCAP] episode ");
            put_u(p_out, size, &value_n, p_cur->episode + 1U);
            put(p_out, size, &value_n, " t=");
            put_u(p_out, size, &value_n, p_e->start_ms);
            put(p_out, size, &value_n, "ms samples=");
            put_u(p_out, size, &value_n, p_e->count);
            return true;
        }
        if (p_cur->sample >= p_e->count)
        {
            p_cur->episode++;
            p_cur->header_done = false;
            continue;
        }
        put(p_out, size, &value_n, "BC");
        put_u(p_out, size, &value_n, p_cur->episode + 1U);
        put(p_out, size, &value_n, " ");
        put_u(p_out, size, &value_n, p_cur->sample);
        put(p_out, size, &value_n, ":");
        for (value_i = 0U;
             (value_i < BCAP_PER_LINE) && (p_cur->sample < p_e->count);
             value_i++)
        {
            const bcap_sample_t * p_s = &p_c->pool[p_e->first + p_cur->sample];
            put(p_out, size, &value_n, " ");
            put_u(p_out, size, &value_n, p_s->raw);
            put(p_out, size, &value_n, ":");
            put_u(p_out, size, &value_n, p_s->dark4);
            put(p_out, size, &value_n, ":");
            put_u(p_out, size, &value_n, p_s->flags);
            put(p_out, size, &value_n, ":");
            put_u(p_out, size, &value_n, p_s->mm);
            put(p_out, size, &value_n, ":");
            put_u(p_out, size, &value_n, p_s->dt_ms);
            p_cur->sample++;
        }
        return true;
    }
    if (!p_cur->end_done)
    {
        p_cur->end_done = true;
        put(p_out, size, &value_n, "[BCAP] end");
        return true;
    }
    return false;
}

static void binary_put (unsigned char * p_out, unsigned * p_bit,
                        uint32_t value_v, unsigned width)
{
    for (unsigned value_i = 0; value_i < width; ++value_i, ++*p_bit)
    {
        p_out[*p_bit / 8U] |=
            (unsigned char)(((value_v >> value_i) & 1U) << (*p_bit % 8U));
    }
}

unsigned bcap_binary_next (const bcap_t * p_c, bcap_cursor_t * p_cur,
                           unsigned char * p_out, unsigned size)
{
    bcap_cursor_t next;
    unsigned bit = 0, kind, count = 0, bits;
    const bcap_episode_t * p_e = 0;
    if (!p_c || !p_cur || !p_out || p_c->episodes > BCAP_MAX_EPISODES ||
        p_c->letter_count > 8U || p_cur->episode > p_c->episodes)
    {
        return 0;
    }
    next = *p_cur;
    if (!next.summary_done)
    {
        kind = 0;
        bits = 122U + 5U * p_c->letter_count;
        for (unsigned value_i = 0; value_i < p_c->letter_count; ++value_i)
        {
            if (p_c->letters[value_i] < 'A' || p_c->letters[value_i] > 'Z')
            {
                return 0;
            }
        }
    }
    else
    {
        while (next.episode < p_c->episodes)
        {
            p_e = &p_c->episode[next.episode];
            if (p_e->count > BCAP_POOL ||
                (unsigned)p_e->first + p_e->count > p_c->used ||
                p_c->used > BCAP_POOL || p_e->start_ms > 4294967U)
            {
                return 0;
            }
            if (!next.header_done || next.sample < p_e->count)
            {
                break;
            }
            next.episode++;
            next.header_done = false;
        }
        if (next.episode < p_c->episodes)
        {
            p_e = &p_c->episode[next.episode];
            if (!next.header_done)
            {
                kind = 1;
                bits = 81U;
            }
            else
            {
                kind = 2;
                count = p_e->count - next.sample;
                if (count > BCAP_WIRE_SAMPLES)
                {
                    count = BCAP_WIRE_SAMPLES;
                }
                bits = 63U + 52U * count;
                for (unsigned value_i = 0; value_i < count; ++value_i)
                {
                    const bcap_sample_t * p_s =
                        &p_c->pool[p_e->first + next.sample + value_i];
                    if (p_s->raw > 4095U || p_s->dark4 > 250U ||
                        (p_s->flags & 3U) > 2U || ((p_s->flags >> 2) & 7U) > 4U)
                    {
                        return 0;
                    }
                }
            }
        }
        else
        {
            if (next.end_done)
            {
                return 0;
            }
            kind = 3;
            bits = 42U;
        }
    }
    const unsigned length = (bits + 7U) / 8U;
    if (size < length)
    {
        return 0;
    }
    for (unsigned value_i = 0; value_i < length; ++value_i)
    {
        p_out[value_i] = 0;
    }
    binary_put(p_out, &bit, BCAP_WIRE_VERSION, 8);
    binary_put(p_out, &bit, p_c->run_start_us, 32);
    binary_put(p_out, &bit, kind, 2);
    if (kind == 0U)
    {
        binary_put(p_out, &bit, p_c->episodes, 4);
        binary_put(p_out, &bit, p_c->dropped, 8);
        binary_put(p_out, &bit, p_c->decoded, 16);
        binary_put(p_out, &bit, p_c->rejected, 16);
        binary_put(p_out, &bit, p_c->timing, 16);
        binary_put(p_out, &bit, p_c->overflow, 16);
        binary_put(p_out, &bit, p_c->letter_count, 4);
        for (unsigned value_i = 0; value_i < p_c->letter_count; ++value_i)
        {
            binary_put(p_out, &bit, p_c->letters[value_i] - 'A', 5);
        }
        next.summary_done = true;
    }
    else if (kind == 1U)
    {
        binary_put(p_out, &bit, next.episode, 4);
        binary_put(p_out, &bit, p_e->start_ms, 23);
        binary_put(p_out, &bit, p_e->count, 12);
        next.header_done = true;
        next.sample = 0;
    }
    else if (kind == 2U)
    {
        binary_put(p_out, &bit, next.episode, 4);
        binary_put(p_out, &bit, next.sample, 12);
        binary_put(p_out, &bit, count - 1U, 5);
        for (unsigned value_i = 0; value_i < count; ++value_i)
        {
            const bcap_sample_t * p_s =
                &p_c->pool[p_e->first + next.sample + value_i];
            binary_put(p_out, &bit, p_s->raw, 12);
            binary_put(p_out, &bit, p_s->dark4, 8);
            binary_put(p_out, &bit, p_s->flags, 8);
            binary_put(p_out, &bit, p_s->mm, 16);
            binary_put(p_out, &bit, p_s->dt_ms, 8);
        }
        next.sample += count;
    }
    else
    {
        next.end_done = true;
    }
    *p_cur = next;
    return length;
}

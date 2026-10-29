#include "barcode_capture.h"

#define BCAP_GAP_FLAG_US (30000U)

static void put(char *out, unsigned int size, unsigned int *n, const char *s)
{
    while ((*s != '\0') && ((*n + 1U) < size))
    {
        out[(*n)++] = *s++;
    }
    out[*n] = '\0';
}
static void put_u(char *out, unsigned int size, unsigned int *n, uint32_t v)
{
    char tmp[11];
    unsigned int i = 0U;
    do
    {
        tmp[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while ((v != 0U) && (i < sizeof tmp));
    while ((i > 0U) && ((*n + 1U) < size))
    {
        out[(*n)++] = tmp[--i];
    }
    out[*n] = '\0';
}

void bcap_begin_run(bcap_t *c, uint32_t now_us)
{
    *c = (bcap_t){0};
    c->run_start_us = now_us;
}

static bcap_sample_t make_sample(bcap_t *c, const bcap_input_t *in)
{
    bcap_sample_t s;
    uint32_t dt = c->have_last ? (uint32_t)(in->t_us - c->last_us) : 0U;
    s.raw = in->raw;
    s.mm = 0U;
    s.dark4 = (uint8_t)((in->darkness > 1000U ? 1000U : in->darkness) / 4U);
    s.flags = (uint8_t)((in->surface & 3U) | ((in->status & 7U) << 2) |
                        (in->collecting ? 0x20U : 0U) |
                        (in->rearming ? 0x40U : 0U) |
                        (dt > BCAP_GAP_FLAG_US ? 0x80U : 0U));
    s.dt_ms = (uint8_t)(dt / 1000U > 255U ? 255U : dt / 1000U);
    c->last_us = in->t_us;
    c->have_last = true;
    return s;
}
static uint16_t rel_mm(double origin, double now)
{
    const double d = (now - origin) * 1000.0;
    if (!(d > 0.0))
    {
        return 0U;
    }
    return (uint16_t)(d > 65535.0 ? 65535.0 : d + 0.5);
}
static void close_episode(bcap_t *c)
{
    if (c->active)
    {
        c->active = false;
        c->episodes++;
    }
}

void bcap_record(bcap_t *c, const bcap_input_t *in)
{
    bcap_sample_t s = make_sample(c, in);
    if (in->status == 1U)
    {
        c->decoded++;
        if (c->letter_count < (sizeof c->letters - 1U))
        {
            c->letters[c->letter_count++] = in->letter;
            c->letters[c->letter_count] = '\0';
        }
    }
    else if (in->status == 2U) { c->rejected++; }
    else if (in->status == 3U) { c->timing++; }
    else if (in->status == 4U) { c->overflow++; }

    if (!c->active && (in->surface == 2U)) /* IR_SURFACE_BLACK */
    {
        if ((c->episodes >= BCAP_MAX_EPISODES) ||
            ((c->used + c->pre_count + 1U) > BCAP_POOL))
        {
            if (c->dropped < 255U) { c->dropped++; }
        }
        else
        {
            bcap_episode_t *e = &c->episode[c->episodes];
            unsigned int i;
            const unsigned int oldest =
                (c->pre_head + BCAP_PRE - c->pre_count) % BCAP_PRE;
            e->first = c->used;
            e->count = 0U;
            e->origin_m = (c->pre_count != 0U) ? c->pre_m[oldest] : in->centre_m;
            {
                const uint32_t now_ms = (in->t_us - c->run_start_us) / 1000U;
                const uint32_t back_ms = (uint32_t)c->pre_count * 10U;
                e->start_ms = now_ms > back_ms ? now_ms - back_ms : 0U;
            }
            for (i = 0U; i < c->pre_count; i++)
            {
                const unsigned int k = (oldest + i) % BCAP_PRE;
                bcap_sample_t p = c->pre[k];
                p.mm = rel_mm(e->origin_m, c->pre_m[k]);
                c->pool[c->used++] = p;
                e->count++;
            }
            c->active = true;
            c->quiet = 0U;
        }
    }
    if (c->active)
    {
        bcap_episode_t *e = &c->episode[c->episodes];
        if (c->used < BCAP_POOL)
        {
            s.mm = rel_mm(e->origin_m, in->centre_m);
            c->pool[c->used++] = s;
            e->count++;
        }
        c->quiet = (in->surface == 1U) ? (uint16_t)(c->quiet + 1U) : 0U; /* WHITE */
        if ((c->quiet >= BCAP_END_QUIET) || (c->used >= BCAP_POOL))
        {
            close_episode(c);
            c->pre_count = 0U; /* the next pre-trigger window starts afresh */
        }
        return;
    }
    c->pre[c->pre_head] = s;
    c->pre_m[c->pre_head] = in->centre_m;
    c->pre_head = (uint16_t)((c->pre_head + 1U) % BCAP_PRE);
    if (c->pre_count < BCAP_PRE) { c->pre_count++; }
}

void bcap_end_run(bcap_t *c) { close_episode(c); }

bool bcap_format_next(const bcap_t *c, bcap_cursor_t *cur, char *out, unsigned int size)
{
    unsigned int n = 0U;
    out[0] = '\0';
    if (!cur->summary_done)
    {
        cur->summary_done = true;
        put(out, size, &n, "[BCAP] run summary: episodes=");
        put_u(out, size, &n, c->episodes);
        put(out, size, &n, " not_stored=");
        put_u(out, size, &n, c->dropped);
        put(out, size, &n, " decoded=");
        put_u(out, size, &n, c->decoded);
        put(out, size, &n, " letters=");
        put(out, size, &n, c->letter_count ? c->letters : "-");
        put(out, size, &n, " rejected=");
        put_u(out, size, &n, c->rejected);
        put(out, size, &n, " timing_error=");
        put_u(out, size, &n, c->timing);
        put(out, size, &n, " overflow=");
        put_u(out, size, &n, c->overflow);
        put(out, size, &n, " | raw:dark4:flags:mm:dt");
        return true;
    }
    while (cur->episode < c->episodes)
    {
        const bcap_episode_t *e = &c->episode[cur->episode];
        unsigned int i;
        if (!cur->header_done)
        {
            cur->header_done = true;
            cur->sample = 0U;
            put(out, size, &n, "[BCAP] episode ");
            put_u(out, size, &n, cur->episode + 1U);
            put(out, size, &n, " t=");
            put_u(out, size, &n, e->start_ms);
            put(out, size, &n, "ms samples=");
            put_u(out, size, &n, e->count);
            return true;
        }
        if (cur->sample >= e->count)
        {
            cur->episode++;
            cur->header_done = false;
            continue;
        }
        put(out, size, &n, "BC");
        put_u(out, size, &n, cur->episode + 1U);
        put(out, size, &n, " ");
        put_u(out, size, &n, cur->sample);
        put(out, size, &n, ":");
        for (i = 0U; (i < BCAP_PER_LINE) && (cur->sample < e->count); i++)
        {
            const bcap_sample_t *s = &c->pool[e->first + cur->sample];
            put(out, size, &n, " ");
            put_u(out, size, &n, s->raw);
            put(out, size, &n, ":");
            put_u(out, size, &n, s->dark4);
            put(out, size, &n, ":");
            put_u(out, size, &n, s->flags);
            put(out, size, &n, ":");
            put_u(out, size, &n, s->mm);
            put(out, size, &n, ":");
            put_u(out, size, &n, s->dt_ms);
            cur->sample++;
        }
        return true;
    }
    if (!cur->end_done)
    {
        cur->end_done = true;
        put(out, size, &n, "[BCAP] end");
        return true;
    }
    return false;
}

static void binary_put(unsigned char *out, unsigned *bit, uint32_t v, unsigned width)
{
    for (unsigned i=0; i<width; ++i, ++*bit)
        out[*bit/8U] |= (unsigned char)(((v>>i)&1U)<<(*bit%8U));
}
unsigned bcap_binary_next(const bcap_t *c, bcap_cursor_t *cur,
                          unsigned char *out, unsigned size)
{
    bcap_cursor_t next;
    unsigned bit=0, kind, count=0, bits;
    const bcap_episode_t *e=0;
    if (!c || !cur || !out || c->episodes>BCAP_MAX_EPISODES ||
        c->letter_count>8U || cur->episode>c->episodes) return 0;
    next=*cur;
    if (!next.summary_done) {
        kind=0; bits=122U+5U*c->letter_count;
        for (unsigned i=0;i<c->letter_count;++i)
            if (c->letters[i]<'A' || c->letters[i]>'Z') return 0;
    } else {
        while (next.episode<c->episodes) {
            e=&c->episode[next.episode];
            if (e->count>BCAP_POOL || (unsigned)e->first+e->count>c->used ||
                c->used>BCAP_POOL || e->start_ms>4294967U) return 0;
            if (!next.header_done || next.sample<e->count) break;
            next.episode++; next.header_done=false;
        }
        if (next.episode<c->episodes) {
            e=&c->episode[next.episode];
            if (!next.header_done) { kind=1; bits=81U; }
            else {
                kind=2; count=e->count-next.sample;
                if (count>BCAP_PER_LINE) count=BCAP_PER_LINE;
                bits=61U+52U*count;
                for (unsigned i=0;i<count;++i) {
                    const bcap_sample_t *s=&c->pool[e->first+next.sample+i];
                    if (s->raw>4095U || s->dark4>250U ||
                        (s->flags&3U)>2U || ((s->flags>>2)&7U)>4U) return 0;
                }
            }
        } else {
            if (next.end_done) return 0;
            kind=3; bits=42U;
        }
    }
    const unsigned length=(bits+7U)/8U;
    if (size<length) return 0;
    for (unsigned i=0;i<length;++i) out[i]=0;
    binary_put(out,&bit,BCAP_WIRE_VERSION,8);
    binary_put(out,&bit,c->run_start_us,32);
    binary_put(out,&bit,kind,2);
    if (kind==0U) {
        binary_put(out,&bit,c->episodes,4); binary_put(out,&bit,c->dropped,8);
        binary_put(out,&bit,c->decoded,16); binary_put(out,&bit,c->rejected,16);
        binary_put(out,&bit,c->timing,16); binary_put(out,&bit,c->overflow,16);
        binary_put(out,&bit,c->letter_count,4);
        for (unsigned i=0;i<c->letter_count;++i) binary_put(out,&bit,c->letters[i]-'A',5);
        next.summary_done=true;
    } else if (kind==1U) {
        binary_put(out,&bit,next.episode,4);
        binary_put(out,&bit,e->start_ms,23); binary_put(out,&bit,e->count,12);
        next.header_done=true; next.sample=0;
    } else if (kind==2U) {
        binary_put(out,&bit,next.episode,4); binary_put(out,&bit,next.sample,12);
        binary_put(out,&bit,count-1U,3);
        for (unsigned i=0;i<count;++i) {
            const bcap_sample_t *s=&c->pool[e->first+next.sample+i];
            binary_put(out,&bit,s->raw,12); binary_put(out,&bit,s->dark4,8);
            binary_put(out,&bit,s->flags,8); binary_put(out,&bit,s->mm,16);
            binary_put(out,&bit,s->dt_ms,8);
        }
        next.sample+=count;
    } else { next.end_done=true; }
    *cur=next;
    return length;
}

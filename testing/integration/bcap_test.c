/* Barcode capture: episode detection, pre-trigger samples, quiet end, pool
 * limits, decoder counters and the USB dump format. */
#include "barcode_capture.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static bcap_t cap;
static uint32_t t;
static double pos;
static void feed(uint16_t dark, uint8_t status, char letter)
{
    const bcap_input_t in = {t, (uint16_t)(150U + dark), dark,
                             dark >= 600U ? 2U : 1U, status, letter,
                             false, false, pos};
    bcap_record(&cap, &in);
    t += 10000U;
    pos += 0.001;
}
static unsigned dump(char *all, size_t size)
{
    bcap_cursor_t cur = {0};
    char line[200];
    unsigned lines = 0U;
    all[0] = '\0';
    while (bcap_format_next(&cap, &cur, line, sizeof line))
    {
        assert(strlen(line) <= 160U);
        strncat(all, line, size - strlen(all) - 2U);
        strcat(all, "\n");
        lines++;
    }
    return lines;
}

int main(void)
{
    static char text[200000];
    unsigned i;
    t = 1000000U; pos = 1.0;
    bcap_begin_run(&cap, t);
    for (i = 0; i < 50; i++) feed(20, 0, 0);              /* white mat */
    assert(cap.episodes == 0 && !cap.active);
    for (i = 0; i < 6; i++) { feed(900, 0, 0); feed(40, 0, 0); } /* bars */
    assert(cap.active);
    for (i = 0; i < BCAP_END_QUIET - 2U; i++) feed(30, 0, 0); /* the last gap counted 1 */
    assert(cap.active);                                    /* not yet quiet */
    feed(30, 1, 'Z');                                      /* decoded at end */
    assert(!cap.active && cap.episodes == 1);
    assert(cap.decoded == 1 && strcmp(cap.letters, "Z") == 0);
    /* Pre-trigger samples kept; distance counts from the first stored one. */
    assert(cap.episode[0].count == BCAP_PRE + 12U + BCAP_END_QUIET - 1U);
    assert(cap.pool[cap.episode[0].first].mm == 0U);
    assert(cap.pool[cap.episode[0].first + BCAP_PRE].mm == BCAP_PRE);
    assert((cap.pool[cap.episode[0].first + BCAP_PRE].flags & 3U) == 2U);
    /* Status counters outside episodes too. */
    feed(10, 2, 0); feed(10, 3, 0); feed(10, 4, 0);
    assert(cap.rejected == 1 && cap.timing == 1 && cap.overflow == 1);
    /* A run that ends mid-episode closes it. */
    feed(800, 0, 0);
    assert(cap.active);
    bcap_end_run(&cap);
    assert(!cap.active && cap.episodes == 2);
    {
        const unsigned lines = dump(text, sizeof text);
        assert(strstr(text, "[BCAP] run summary: episodes=2 not_stored=0 decoded=1 letters=Z rejected=1 timing_error=1 overflow=1"));
        assert(strstr(text, "[BCAP] episode 1 t="));
        assert(strstr(text, "BC1 0: "));
        assert(strstr(text, "[BCAP] end\n"));
        assert(lines == 2U + 2U + (cap.episode[0].count + BCAP_PER_LINE - 1U) / BCAP_PER_LINE +
                        (cap.episode[1].count + BCAP_PER_LINE - 1U) / BCAP_PER_LINE);
    }
    /* Pool and episode limits: later episodes are counted, never overflow. */
    bcap_begin_run(&cap, t);
    for (unsigned e = 0; e < BCAP_MAX_EPISODES + 5U; e++)
    {
        for (i = 0; i < 300; i++) feed(i % 2 ? 900 : 50, 0, 0);
        for (i = 0; i < BCAP_END_QUIET; i++) feed(10, 0, 0);
    }
    assert(cap.used <= BCAP_POOL && cap.episodes <= BCAP_MAX_EPISODES);
    assert(cap.dropped > 0);
    (void)dump(text, sizeof text);
    /* Letters buffer never overflows. */
    bcap_begin_run(&cap, t);
    for (i = 0; i < 20; i++) feed(10, 1, (char)('A' + i));
    assert(cap.decoded == 20 && strlen(cap.letters) == sizeof cap.letters - 1U);
    puts("PASS: barcode capture episodes, pre-trigger, quiet end, run end, counters, bounded pool/letters and USB dump format");
    return 0;
}

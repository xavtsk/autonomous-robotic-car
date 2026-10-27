/* Barcode-sensor capture for diagnosing barcode reading on the course.
 *
 * Observation only: records what the barcode sensor and Buddy 3's decoder saw
 * during a run, so the raw trace can be printed over USB afterwards and
 * replayed offline. Never changes navigation or motion.
 *
 * An "episode" starts when the barcode sensor reports BLACK (a bar, or any
 * tape it crosses; same cut-offs as the decoder) and ends after
 * BCAP_END_QUIET consecutive WHITE samples; BCAP_PRE samples before the
 * trigger are kept so the leading quiet zone is visible. Episodes share one
 * fixed pool; when it is full, later episodes are only counted.
 *
 * Single-writer: the sampling owner calls bcap_record()/bcap_begin_run();
 * readers must hold the same lock as the writer (s3_integration's mutex). */
#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include <stdbool.h>
#include <stdint.h>

#define BCAP_POOL (2400U)       /* samples, ~24 s at 10 ms */
#define BCAP_MAX_EPISODES (12U)
#define BCAP_PRE (20U)          /* samples kept before the trigger */
#define BCAP_END_QUIET (40U)    /* white samples that end an episode */

/* flags: bits 0-1 surface (0 unknown, 1 white, 2 black), bits 2-4 decoder
 * status (0 none, 1 decoded, 2 rejected, 3 timing error, 4 overflow),
 * bit 5 decoder collecting a frame, bit 6 decoder re-arming (blind),
 * bit 7 sample gap above 30 ms. */
typedef struct __attribute__((packed))
{
    uint16_t raw;   /* 12-bit ADC */
    uint16_t mm;    /* odometry since the episode's first sample */
    uint8_t dark4;  /* darkness / 4 (0..250) */
    uint8_t flags;
    uint8_t dt_ms;  /* time since the previous sample, saturating */
} bcap_sample_t;

typedef struct
{
    uint16_t first, count;  /* slice of the pool */
    uint32_t start_ms;      /* ms since the run started */
    double origin_m;        /* odometry at the first stored sample */
} bcap_episode_t;

typedef struct
{
    bcap_sample_t pool[BCAP_POOL];
    bcap_sample_t pre[BCAP_PRE];
    double pre_m[BCAP_PRE];
    bcap_episode_t episode[BCAP_MAX_EPISODES];
    uint32_t run_start_us, last_us;
    uint16_t used, pre_head, pre_count, quiet;
    uint8_t episodes, dropped;
    bool active, have_last;
    /* Decoder results seen during the run (also outside episodes). */
    uint16_t decoded, rejected, timing, overflow;
    char letters[9];  /* decoded letters in order, NUL terminated */
    uint8_t letter_count;
} bcap_t;

typedef struct
{
    uint32_t t_us;
    uint16_t raw, darkness;
    uint8_t surface;     /* ir_surface_t */
    uint8_t status;      /* s3_barcode_status_t */
    char letter;         /* valid when status == decoded */
    bool collecting, rearming;
    double centre_m;     /* odometry; NaN-free, 0 when unavailable */
} bcap_input_t;

void bcap_begin_run(bcap_t *c, uint32_t now_us);
void bcap_record(bcap_t *c, const bcap_input_t *in);
void bcap_end_run(bcap_t *c);
/* Dump cursor: line 0 is the summary, then per episode one header line and
 * BCAP_PER_LINE samples per line, then "[BCAP] end". Returns false when
 * nothing is left. Every line fits in 160 characters. */
#define BCAP_PER_LINE (6U)
typedef struct { uint16_t episode, sample; bool header_done, summary_done, end_done; } bcap_cursor_t;
bool bcap_format_next(const bcap_t *c, bcap_cursor_t *cur, char *out, unsigned int size);
#endif

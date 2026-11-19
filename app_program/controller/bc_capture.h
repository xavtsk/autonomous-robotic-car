/** @file bc_capture.h
 * @brief Barcode-sensor capture for diagnosing barcode reading on the course.
 */
/* Barcode-sensor capture for diagnosing barcode reading on the course.
 *
 * Observation only: records what the barcode sensor and Buddy 3's decoder saw
 * during a run, so the raw trace can be printed over USB afterwards and
 * replayed offline. Never changes navigation or motion.
 *
 * An "episode" starts when the barcode sensor reports BLACK (a bar, or any
 * tape it crosses; same cut-offs as the decoder) or a barcode reader starts
 * collecting, and ends after BCAP_END_QUIET consecutive WHITE samples once no
 * reader is collecting; BCAP_PRE samples before the
 * trigger are kept so the leading quiet zone is visible. Episodes share one
 * fixed pool; when it is full, later episodes are only counted.
 *
 * Single-writer: the sampling owner calls bcap_record()/bcap_begin_run();
 * readers must hold the same lock as the writer (s3_integration's mutex). */
#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "run_timing.h"
#include "../subsystem3/bc_decoder.h"

#define BCAP_POOL (4000U) /* ~20 s at 200 Hz; fits the 12-bit wire count */
#if BCAP_POOL > 4095U
#error BCAP_POOL must fit the binary capture count and cursor
#endif
#define BCAP_MAX_EPISODES (12U)
#define BCAP_PRE                                                               \
    (S3_REC_PRE_US / S3_SAMPLE_PERIOD_US) /* retain the decoder's history */
#if BCAP_PRE > 511U
#error BCAP_PRE must fit nine-bit history counters
#endif
#define BCAP_END_QUIET (400U / S3_SAMPLE_PERIOD_MS) /* keep 400 ms of white */

/* flags: bits 0-1 surface (0 unknown, 1 white, 2 black), bits 2-4 decoder
 * status (0 none, 1 decoded, 2 rejected, 3 timing error, 4 overflow),
 * bit 5 decoder collecting a frame, bit 6 decoder re-arming (blind),
 * bit 7 sample gap above 30 ms. */
typedef struct __attribute__((packed))
{
    uint16_t raw;  /* 12-bit ADC */
    uint16_t mm;   /* odometry since the episode's first sample */
    uint8_t dark4; /* darkness / 4 (0..250) */
    uint8_t flags;
    uint8_t dt_ms; /* time since the previous sample, saturating */
} bcap_sample_t;

typedef struct
{
    uint16_t first, count; /* slice of the pool */
    uint32_t start_ms;     /* ms since the run started */
    int64_t origin_qmm;    /* first stored odometry, in 1/256 mm units */
} bcap_episode_t;

typedef struct
{
    bcap_sample_t pool[BCAP_POOL];
    /* Keep naturally aligned 64-bit storage before the seven-byte samples. */
    int64_t pre_qmm[BCAP_PRE]; /* quantized once on arrival, not during trigger
                                  copy */
    bcap_episode_t episode[BCAP_MAX_EPISODES];
    bcap_sample_t pre[BCAP_PRE];
    uint32_t run_start_us, last_us;

    /* Recorder-owned after bcap_begin_run(); every reader uses its mutex.
     * History wraps at BCAP_PRE; collecting can retain quiet through the
     * terminal BCAP_POOL sample. All five fields share one ARM storage word.
     * Native struct layout is never serialized. */
    bool active : 1;
    bool have_last : 1;
    uint32_t pre_head : 9;  /* 0..299 */
    uint32_t quiet : 12;    /* 0..4000, including terminal pool exhaustion */
    uint32_t pre_count : 9; /* 0..300, including the full-history state */

    /* Public serializer metadata retains every pre-validation input value. */
    uint16_t used;
    uint8_t episodes, dropped;
    /* Decoder results seen during the run (also outside episodes). */
    uint16_t decoded, rejected, timing, overflow;
    char letters[9]; /* decoded letters in order, NUL terminated */
    uint8_t letter_count;
} bcap_t;

typedef struct
{
    uint32_t t_us;
    uint16_t raw, darkness;
    uint8_t surface; /* ir_surface_t */
    uint8_t status;  /* s3_barcode_status_t */
    char letter;     /* valid when status == decoded */
    bool collecting, rearming;
    float64_t
        centre_m; /* odometry; non-finite values yield unavailable distance */
} bcap_input_t;

void bcap_begin_run (bcap_t * p_c, uint32_t now_us);
void bcap_record (bcap_t * p_c, const bcap_input_t * p_in);
void bcap_end_run (bcap_t * p_c);
/* Dump cursor: line 0 is the summary, then per episode one header line and
 * BCAP_PER_LINE samples per line, then "[BCAP] end". Returns false when
 * nothing is left. Every line fits in 160 characters. */
#define BCAP_PER_LINE (6U)

typedef struct
{
    unsigned episode : 4; /* 0..12 */
    unsigned sample : 12; /* 0..4000, including the full-buffer state */
    unsigned header_done : 1, summary_done : 1, end_done : 1;
} bcap_cursor_t;

bool bcap_format_next (const bcap_t * p_c, bcap_cursor_t * p_cur, char * p_out,
                       unsigned int size);
/* Binary wireless format v2. Common header: version8, run_start_us32,
 * kind2 (summary/header/samples/end); all fields LSB-first, padding zero.
 * At most 24 52-bit samples per packet, with a five-bit count-minus-one.
 * USB text format above retains six samples per line.
 * Zero means finished/invalid/insufficient capacity; cursor changes only when
 * a complete packet is produced. Caller holds the existing capture mutex. */
#define BCAP_WIRE_VERSION 2U
#define BCAP_WIRE_SAMPLES 24U
#define BCAP_WIRE_MAX                                                          \
    164U /* ceil((63 header bits + 24 * 52 sample bits) / 8) */
unsigned bcap_binary_next (const bcap_t * p_c, bcap_cursor_t * p_cur,
                           unsigned char * p_out, unsigned size);
#endif

/* End of file. */


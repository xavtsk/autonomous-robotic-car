/** @file bc_capture.h
 * @brief Barcode-sensor capture for diagnosing barcode reading on the course.
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
 * Single-writer: the sampling owner calls
 * bc_capture_record()/bc_capture_begin_run(); readers must hold the same lock
 * as the writer (s3_integration's mutex).
 */
#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "../app_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "run_timing.h"
#include "../subsystem3/bc_decoder.h"

#define BCAP_POOL                                                              \
    (4000U) /* At 200 Hz, the 12-bit count can report this 20 s pool. */
#if BCAP_POOL > 4095U
#error BCAP_POOL must fit the binary capture count and cursor
#endif
#define BCAP_MAX_EPISODES (12U)
#define BCAP_PRE                                                               \
    (S3_REC_PRE_US / S3_SAMPLE_PERIOD_US) /* Retain the decoder's history. */
#if BCAP_PRE > 511U
#error BCAP_PRE must fit nine-bit history counters
#endif
/* Keep 400 ms of white. */
#define BCAP_END_QUIET (400U / S3_SAMPLE_PERIOD_MS)

/* flags: bits 0-1 surface (0 unknown, 1 white, 2 black), bits 2-4 decoder
 * status (0 none, 1 decoded, 2 rejected, 3 timing error, 4 overflow),
 * bit 5 decoder collecting a frame, bit 6 decoder re-arming (blind),
 * bit 7 sample gap above 30 ms. */
typedef struct __attribute__((packed))
{
    uint16_t raw;           /* Store the raw 12-bit ADC reading. */
    uint16_t distance_mm;   /* Measure travel since the first sample. */
    uint8_t  darkness_code; /* Normalized darkness / 4 (0..250). */
    uint8_t  flags;
    uint8_t  dt_ms; /* Saturate elapsed milliseconds at 255. */
} bc_capture_sample_t;

typedef struct
{
    uint16_t first;
    uint16_t count;      /* This is the number of stored samples. */
    uint32_t start_ms;   /* Measure milliseconds since START. */
    int64_t  origin_qmm; /* Store the first position in 1/256 mm. */
} bc_capture_episode_t;

typedef struct
{
    bc_capture_sample_t pool[BCAP_POOL];
    /* Keep naturally aligned 64-bit storage before the seven-byte samples. */
    int64_t pre_qmm[BCAP_PRE]; /* Quantize each sample on arrival so the
                                * trigger can copy stored measurements.
                                */
    bc_capture_episode_t episode[BCAP_MAX_EPISODES];
    bc_capture_sample_t  pre[BCAP_PRE];
    uint32_t             run_start_us;
    uint32_t             last_us;

    /* Recorder-owned after bc_capture_begin_run(); every reader uses its mutex.
     * History wraps at BCAP_PRE; collecting can retain quiet through the
     * terminal BCAP_POOL sample. All five fields share one ARM storage word.
     * Native struct layout is never serialized. */
    bool     b_is_active : 1;
    bool     b_has_last_sample : 1;
    uint32_t pre_head : 9;  /* 0..299 */
    uint32_t quiet : 12;    /* Quiet samples can fill the pool (4000). */
    uint32_t pre_count : 9; /* Up to 300 history samples are stored. */

    /* Public serializer metadata retains every pre-validation input value. */
    uint16_t used;
    uint8_t  episodes;
    uint8_t  dropped;
    /* Decoder results seen during the run (also outside episodes). */
    uint16_t decoded;
    uint16_t rejected;
    uint16_t timing;
    uint16_t overflow;
    char     letters[9]; /* NUL-terminate letters in decode order. */
    uint8_t  letter_count;
} bc_capture_t;

typedef struct
{
    uint32_t  t_us;
    uint16_t  raw;
    uint16_t  darkness;
    uint8_t   surface; /* ir_reflectance_surface_t */
    uint8_t   status;  /* bc_decoder_status_t */
    char      letter;  /* The letter is valid when decoded. */
    bool      b_is_collecting;
    bool      b_is_rearming;
    float64_t centre_m; /* Non-finite odometry has no distance. */
} bc_capture_input_t;

void bc_capture_begin_run (bc_capture_t * p_store, uint32_t now_us);
void bc_capture_record (bc_capture_t *             p_store,
                        const bc_capture_input_t * p_in);
void bc_capture_end_run (bc_capture_t * p_store);
/* Dump cursor: line 0 is the summary, then per episode one header line and
 * BCAP_PER_LINE samples per line, then "[BCAP] end". Returns false when
 * nothing is left. Every line fits in 160 characters. */
#define BCAP_PER_LINE (6U)

typedef struct
{
    unsigned episode : 4; /* 0..12 */
    unsigned sample : 12; /* The end cursor can reach 4000. */
    unsigned b_is_header_done : 1;
    unsigned b_is_summary_done : 1;
    unsigned b_is_end_done : 1;
} bc_capture_cursor_t;

bool bc_capture_format_next (const bc_capture_t *  p_store,
                             bc_capture_cursor_t * p_cur, char * p_out,
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
    164U /* Round 63 header and 24 * 52 sample bits up to bytes. */
unsigned bc_capture_binary_next (const bc_capture_t *  p_store,
                                 bc_capture_cursor_t * p_cur, uint8_t * p_out,
                                 unsigned size);
#endif

/* End of file. */


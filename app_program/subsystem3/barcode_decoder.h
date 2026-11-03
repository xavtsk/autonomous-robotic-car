#ifndef BUDDY3_BARCODE_DECODER_H
#define BUDDY3_BARCODE_DECODER_H
#include <stdbool.h>
#include <stdint.h>
#define S3_BARCODE_RUNS (29U) /* * + gap + letter + gap + * */
typedef struct {
    uint32_t max_gap_us, min_narrow_us, max_run_us, max_frame_us;
    uint8_t quiet_modules;
} s3_barcode_config_t;
typedef struct __attribute__((packed)) {
    /* quiet_modules >= 10 and max_run*quiet_modules < 2^31; the
     * existing validator also requires gap/narrow <= max_run. */
    uint32_t max_gap_us : 28;
    uint32_t min_narrow_us : 28;
    uint32_t max_run_us : 28;
    uint32_t max_frame_us : 31;
    uint32_t quiet_modules : 6;
} s3_barcode_stored_config_t;
typedef struct { char letter; bool reversed; uint32_t timestamp_us; } s3_barcode_result_t;
typedef struct __attribute__((packed)) {
    uint8_t letter_code : 5; /* 0 none, 1..26 A..Z */
    uint8_t reversed : 1;
    /* Final result timestamp is assigned when the quiet zone is confirmed. */
} s3_barcode_stored_result_t;
typedef enum {
    S3_BARCODE_NONE=0, S3_BARCODE_DECODED, S3_BARCODE_REJECTED,
    S3_BARCODE_TIMING_ERROR, S3_BARCODE_OVERFLOW
} s3_barcode_status_t;
typedef struct {
    s3_barcode_stored_config_t config;
    uint32_t runs[S3_BARCODE_RUNS];
    uint32_t last_us, run_start_us, frame_start_us, leading_us, observed_gap_us;
    uint32_t trailing_required_us;
    uint16_t count : 5;
    uint16_t configured : 1;
    uint16_t seen : 1;
    uint16_t black : 1;
    uint16_t collecting : 1;
    uint16_t pending : 1;
    s3_barcode_stored_result_t candidate;
} s3_barcode_decoder_t;
s3_barcode_config_t s3_barcode_defaults(void);
bool s3_barcode_init(s3_barcode_decoder_t *decoder, const s3_barcode_config_t *config);
void s3_barcode_reset(s3_barcode_decoder_t *decoder);
/* Durations alternate black/white, starting/ending black. No checksum or
 * extended/multiple-character frames. Independent batch interface for fixtures. */
bool s3_barcode_decode_runs(const uint32_t *runs, unsigned int count,
    uint32_t leading_us, uint32_t trailing_us, uint32_t observed_gap_us,
    const s3_barcode_config_t *config, s3_barcode_result_t *result);
s3_barcode_status_t s3_barcode_update(s3_barcode_decoder_t *decoder,
    uint32_t timestamp_us, bool valid, bool black, s3_barcode_result_t *result);

/* ---- Record-then-decide reader (runs beside s3_barcode_update) ----------
 * Thin bars of a small barcode blur to grey (e.g. darkness 500 with grey gaps
 * at 300) and never cross fixed black/white limits. This reader makes no
 * per-sample black/white decision: it records the darkness of the whole
 * barcode, then finds bumps (bars) and dips (gaps) that differ from their
 * neighbours by at least S3_REC_PROMINENCE, trims brief/shallow wiggles if
 * there are too many, places each edge where the reading has moved from one
 * level to the next (time-weighted), and passes the 29 stripe widths to the
 * unchanged s3_barcode_decode_runs() Code 39 rules.
 * Change Requirement 1: each stored reading is 16 bits (10-bit darkness
 * 0..1000 + 6-bit time step in 0.5 ms units, 0..31.5 ms). */
#define S3_REC_MAX (1024U)        /* readings; Z needs ~800 at 0.04 m/s     */
#define S3_REC_RISE (150U)        /* start: this much darker than the paper */
#define S3_REC_WHITE_BAND (75U)   /* within this of the paper = white       */
#define S3_REC_PROMINENCE (100U)  /* bump/dip must differ by at least this  */
#define S3_REC_STEP_US (500U)     /* time-step unit of a stored reading     */
#define S3_REC_STEP_MAX (63U)     /* 6 bits: 31.5 ms > 30 ms sample-age limit */
#define S3_REC_EXT_MAX (63U)      /* bumps+dips found before wiggles are trimmed */
typedef enum { S3_REC_WAITING = 0, S3_REC_RECORDING, S3_REC_QUIET } s3_rec_state_t;
typedef struct {
    uint16_t sample[S3_REC_MAX];  /* darkness:10 | step:6 (S3_REC_STEP_US) */
    uint32_t last_us;             /* previous reading's timestamp           */
    uint32_t white_us;            /* white before the barcode (waiting) or after it */
    uint32_t leading_us;          /* white measured before the first bar    */
    uint32_t frame_us;            /* recording length                       */
    uint32_t required_us;         /* trailing quiet zone the decoder needs  */
    uint16_t prev;                /* last waiting reading (becomes sample 0) */
    uint16_t residual_us;         /* rounding carried into the next step    */
    uint16_t count : 11;          /* 0..1024 */
    uint16_t state : 2;           /* s3_rec_state_t */
    uint16_t seen : 1;
    uint16_t paper : 10;          /* lightest recent darkness = white paper */
    uint16_t best : 10;           /* darkest/lightest since the last turn   */
    uint16_t falling : 1;         /* 1 = looking for a dip, 0 = for a bump  */
    uint16_t blocked : 1;         /* after a failure: wait for white first  */
    uint32_t turn_us;             /* time since the last bump/dip turn      */
} s3_barcode_record_t;
void s3_barcode_record_reset(s3_barcode_record_t *record);
bool s3_barcode_record_active(const s3_barcode_record_t *record);
s3_barcode_status_t s3_barcode_record_update(s3_barcode_record_t *record,
    const s3_barcode_config_t *config, uint32_t timestamp_us, bool valid,
    uint16_t darkness, s3_barcode_result_t *result);
/* Pure step used above, exposed for tests: stripe widths from the recorded
 * readings. Returns the number of stripes (black first); 0 when none or more
 * than S3_BARCODE_RUNS. first/last edge are microseconds from sample 0. */
unsigned int s3_barcode_record_runs(const s3_barcode_record_t *record,
    uint32_t runs[S3_BARCODE_RUNS], uint32_t *first_edge_us,
    uint32_t *last_edge_us, uint32_t *end_us, uint32_t *max_step_us);
#endif

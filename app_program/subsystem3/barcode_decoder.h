#ifndef BUDDY3_BARCODE_DECODER_H
#define BUDDY3_BARCODE_DECODER_H
#include <stdbool.h>
#include <stdint.h>
#define S3_BARCODE_RUNS (29U) /* * + gap + letter + gap + * */
#define S3_BARCODE_WIDTH_MIN_PERCENT (65U)
#define S3_BARCODE_WIDTH_MAX_PERCENT (135U)
/* One narrow white gap inside the letter may be elongated by a grey bar edge.
 * Sentinels, bars, wide gaps and all other narrow gaps retain the strict limit. */
#define S3_BARCODE_PAYLOAD_SPACE_MAX_PERCENT (150U)
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
 * neighbours by a prominence learned from the preceding paper noise, trims brief/shallow wiggles if
 * there are too many, uses the first rising / last falling local midpoint
 * crossing around each retained bump, and passes the 29 stripe widths to the
 * s3_barcode_decode_runs() Code 39 rules. If necessary, the two
 * sentinel stars alone estimate a bounded black/white width bias, shared
 * first, then varying between the stars if no shared fit exists.
 * On a quiet, near-zero paper background, an independent bounded recovery
 * groups ink above a 5-unit noise band into exactly 15
 * complete regions. Weak regions use the half-height core around their peak;
 * strong regions retain outer edges. Each accepted interpretation must pass
 * full validation; conflicting valid letters are rejected. No letter or width rule
 * selects regions.
 * Change Requirement 1: each stored reading is 16 bits (10-bit darkness
 * 0..1000 + 6-bit time step in 0.5 ms units, 0..31.5 ms). */
#define S3_REC_MAX (2048U)        /* room for a frame plus history at 100 Hz */
#define S3_REC_RISE (50U)         /* start: this much darker than the paper */
#define S3_REC_PRE_US (1500000U)  /* real history can include a faint opening star */
#define S3_REC_PRE_MAX (320U)     /* bounds history storage at 100/200 Hz */
#define S3_REC_WHITE_BAND (75U)   /* within this of the paper = white       */
#define S3_REC_PROMINENCE (100U)  /* maximum/default peak prominence       */
#define S3_REC_MIN_PROMINENCE (15U) /* minimum contrast on the 0..1000 scale */
#define S3_REC_FAINT_BAND (5U)     /* recovery noise band above near-zero paper */
#define S3_REC_FAINT_MAX_NOISE (15U) /* refuse recovery on noisier recordings */
#define S3_REC_FAINT_DIP_US (30000U) /* bridge brief low dips inside one bar */
#define S3_REC_STEP_US (500U)     /* time-step unit of a stored reading     */
#define S3_REC_STEP_MAX (63U)     /* 6 bits: 31.5 ms > 30 ms sample-age limit */
#define S3_REC_EXT_MAX (63U)      /* bumps+dips found before wiggles are trimmed */
typedef enum { S3_REC_WAITING = 0, S3_REC_RECORDING, S3_REC_QUIET } s3_rec_state_t;
typedef struct {
    uint16_t sample[S3_REC_MAX];  /* darkness:10 | step:6 (S3_REC_STEP_US) */
    uint16_t pre[S3_REC_PRE_MAX]; /* waiting ring: same packed darkness/time format */
    uint16_t pre_head, pre_count; /* oldest entry and number of real samples */
    uint32_t pre_us;              /* elapsed time between oldest and newest entries */
    uint32_t prefix_us;           /* retained history; excluded from post-trigger timeout */
    uint32_t last_us;             /* previous reading's timestamp           */
    uint32_t white_us;            /* white before the barcode (waiting) or after it */
    uint32_t leading_us;          /* white before retained history; see rec_pre_drop */
    uint32_t frame_us;            /* recording length                       */
    uint32_t required_us;         /* earliest frame_us with enough trailing white */
    uint16_t prev;                /* last waiting reading (becomes sample 0) */
    uint16_t residual_us;         /* rounding carried into the next step    */
    uint16_t noise_q4;            /* quiet-paper absolute differences, /16  */
    uint16_t prominence;          /* frozen for the duration of a recording */
    s3_barcode_stored_result_t candidate; /* released only after real quiet */
    uint16_t count : 12;          /* 0..2048, including the full-buffer state */
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
/* Pure diagnostic extraction: stripe widths from the recorded
 * readings. Returns the number of stripes (black first); 0 when none or more
 * than S3_BARCODE_RUNS. first/last edge are microseconds from sample 0.
 * This count-only interface uses faint recovery if the primary count is wrong;
 * the live reader validates both candidates independently before selection. */
unsigned int s3_barcode_record_runs(const s3_barcode_record_t *record,
    uint32_t runs[S3_BARCODE_RUNS], uint32_t *first_edge_us,
    uint32_t *last_edge_us, uint32_t *end_us, uint32_t *max_step_us);
/* Sentinel-only fit. On success, adds bias to black widths and subtracts it
 * from white widths in place. Does not inspect/decode the payload letter.
 * Caller must still validate all widths, gaps and both corrected quiet zones.
 * On failure runs are unchanged. Exposed for replay/host diagnostics. */
bool s3_barcode_record_correct_runs(uint32_t runs[S3_BARCODE_RUNS],
    uint32_t observed_gap_us, const s3_barcode_config_t *config, int32_t *bias_us);
/* Bounded fallback when the shared fit fails. Fits each star independently,
 * holds its bias constant within that star and interpolates shared edge
 * offsets by elapsed time between them. bias_us[0/1] are the first/last star
 * biases (even microseconds); outer quiet zones lose bias_us[0/1]/2 exactly.
 * No sign reversal or drift above half the smaller narrow mean is allowed.
 * Payload does not choose the fit. All full-frame checks remain the caller's
 * responsibility. On failure runs are unchanged. */
bool s3_barcode_record_correct_varying_runs(uint32_t runs[S3_BARCODE_RUNS],
    uint32_t observed_gap_us, const s3_barcode_config_t *config, int32_t bias_us[2]);
#endif

/** @file bc_decoder.h
 * @brief Define Code 39 decoding configuration, state, results and bounded-work
 * interfaces.
 */
#ifndef BUDDY3_BARCODE_DECODER_H
#define BUDDY3_BARCODE_DECODER_H
#include <stdbool.h>
#include <stdint.h>
#include "bc_quiet.h"
#define S3_BARCODE_LETTERS (26U)
#define S3_BARCODE_RUNS (29U) /* * + gap + letter + gap + * */
#define S3_BARCODE_WIDTH_MIN_PERCENT (65U)
#define S3_BARCODE_WIDTH_MAX_PERCENT (135U)
/* One narrow white gap inside the letter may be elongated by a grey bar edge.
 * Sentinels, bars, wide gaps and all other narrow gaps retain the strict limit.
 */
#define S3_BARCODE_PAYLOAD_SPACE_MAX_PERCENT (150U)

typedef struct
{
    uint32_t max_gap_us;
    uint32_t min_narrow_us;
    uint32_t max_run_us;
    uint32_t max_frame_us;
    uint8_t  quiet_modules;
} bc_decoder_config_t;

typedef struct __attribute__((packed))
{
    /* quiet_modules >= 10 and max_run*quiet_modules < 2^31; the
     * existing validator also requires gap/narrow <= max_run. */
    uint32_t max_gap_us : 28;
    uint32_t min_narrow_us : 28;
    uint32_t max_run_us : 28;
    uint32_t max_frame_us : 31;
    uint32_t quiet_modules : 6;
} bc_decoder_stored_config_t;

typedef struct
{
    char     letter;
    bool     b_is_reversed;
    uint32_t timestamp_us;
} bc_decoder_result_t;

typedef struct __attribute__((packed))
{
    uint8_t
        letter_code : 5; /* Use zero for no letter and 1 to 26 for A to Z. */
    uint8_t b_is_reversed : 1;
    /* Final result timestamp is assigned when the quiet zone is confirmed. */
} bc_decoder_stored_result_t;

typedef enum
{
    S3_BARCODE_NONE = 0,
    S3_BARCODE_DECODED,
    S3_BARCODE_REJECTED,
    S3_BARCODE_TIMING_ERROR,
    S3_BARCODE_OVERFLOW
} bc_decoder_status_t;

typedef struct
{
    bc_decoder_stored_config_t config;
    uint32_t                   runs[S3_BARCODE_RUNS];
    uint32_t                   last_us;
    uint32_t                   run_start_us;
    uint32_t                   frame_start_us;
    uint32_t                   leading_us;
    uint32_t                   observed_gap_us;
    uint32_t                   trailing_required_us;
    uint16_t                   count : 5;
    uint16_t                   b_is_configured : 1;
    uint16_t                   b_has_sample : 1;
    uint16_t                   b_is_black : 1;
    uint16_t                   b_is_collecting : 1;
    uint16_t                   b_is_pending : 1;
    bc_decoder_stored_result_t candidate;
} bc_decoder_t;

/** Return the supported timing and quiet-zone defaults. */
bc_decoder_config_t bc_decoder_defaults (void);
/** Validate the configuration and initialize the edge-based reader. */
bool bc_decoder_init (bc_decoder_t *              p_decoder,
                      const bc_decoder_config_t * p_config);
/** Clear a partial edge-based scan while retaining its configuration. */
void bc_decoder_reset (bc_decoder_t * p_decoder);
/** Decode a complete alternating sequence of black and white durations.
 * Durations start and end on black. No checksum or
 * extended/multiple-character frames. Independent batch interface for fixtures.
 */
bool bc_decoder_decode_runs (const uint32_t * p_runs, unsigned int count,
                             uint32_t leading_us, uint32_t trailing_us,
                             uint32_t                    observed_gap_us,
                             const bc_decoder_config_t * p_config,
                             bc_decoder_result_t *       p_result);
/** Process one classified sample and report a completed scan or failure. */
bc_decoder_status_t bc_decoder_update (bc_decoder_t * p_decoder,
                                       uint32_t timestamp_us, bool b_is_valid,
                                       bool                  b_is_black,
                                       bc_decoder_result_t * p_result);

/* ---- Record-then-decide reader (runs beside bc_decoder_update) ----------
 * Thin bars of a small barcode blur to grey (e.g. darkness 500 with grey gaps
 * at 300) and never cross fixed black/white limits. This reader makes no
 * per-sample black/white decision: it records the darkness of the whole
 * barcode, then finds bumps (bars) and dips (gaps) that differ from their
 * neighbours by a prominence learned from the preceding paper noise, trims
 * brief/shallow wiggles if there are too many, uses the first rising / last
 * falling local midpoint crossing around each retained bump, and passes the 29
 * stripe widths to the bc_decoder_decode_runs() Code 39 rules. If necessary,
 * the two sentinel stars alone estimate a bounded black/white width bias,
 * shared first, then varying between the stars if no shared fit exists. On a
 * quiet, near-zero paper background, an independent bounded recovery groups ink
 * above a 5-unit noise band into exactly 15 complete regions. Weak regions use
 * the half-height core around their peak; strong regions retain outer edges.
 * Each accepted interpretation must pass full validation; conflicting valid
 * letters are rejected. No letter or width rule selects regions. Change
 * Requirement 1: each stored reading is 16 bits (10-bit darkness 0..1000 +
 * 6-bit time step in 0.5 ms units, 0..31.5 ms). */
#define S3_REC_MAX (4096U) /* Retain a frame and its 200 Hz history. */
#define S3_REC_RISE (50U)  /* Start this far above paper darkness. */
#define S3_REC_PRE_US                                                          \
    (1500000U) /* Keep measured history for a faint opening star. */
#define S3_REC_PRE_MAX (320U)   /* Bound history storage at 100 and 200 Hz. */
#define S3_REC_WHITE_BAND (75U) /* Treat this band above paper as white. */
/* Use this default and maximum prominence. */
#define S3_REC_PROMINENCE (100U)
/* This is the minimum contrast on the 0..1000 scale. */
#define S3_REC_MIN_PROMINENCE (15U)
#define S3_REC_FAINT_BAND (5U) /* Set the faint band above near-zero paper. */
/* Reject faint recovery above this noise limit. */
#define S3_REC_FAINT_MAX_NOISE (15U)
/* Bridge dips shorter than this inside one bar. */
#define S3_REC_FAINT_DIP_US (30000U)
#define S3_REC_STEP_US (500U) /* Store elapsed time in this unit. */
#define S3_REC_STEP_MAX                                                        \
    (63U) /* Six bits cover 31.5 ms. The age limit is 30 ms. */
/* Retain plateau ripples until the complete frame can be pruned globally.
 * The noisy W capture needs 71 extrema for its 15 real bars. Keeping 127
 * bounds the scratch array at 254 bytes without changing width/quiet rules. */
#define S3_REC_EXT_MAX (127U)
#define S3_REC_WORK_UNITS (128U)
#define S3_REC_FIT_TICKS (32U)

/* Per-record workspace: no shared job, no unbounded work in the sample task.
 * Extrema timestamps make each pruning comparison constant-time. */
typedef struct
{
    uint16_t ext[S3_REC_EXT_MAX];
    uint32_t ext_us[S3_REC_EXT_MAX];
    uint32_t runs[S3_BARCODE_RUNS];
    uint32_t total_us;
    uint32_t best_us;
    uint32_t peak_us;
    uint32_t dip_us;
    uint32_t low_us;
    uint32_t first_us;
    uint32_t last_us;
    uint32_t max_step_us;
    uint32_t edge_us;
    uint32_t integral_us;
    uint32_t segment_us;
    uint32_t edge_previous_us;
    uint32_t minimum;
    uint32_t best_score;
    uint32_t narrow[2];
    uint32_t required_us;
    uint32_t quiet_required_us;
    uint64_t smallest;
    int32_t  bound;
    int32_t  best_bias;
    int32_t  chosen[2];
    int32_t  trial_bias[2];
    uint16_t frozen_count;
    uint16_t cursor;
    uint16_t extrema_count;
    uint16_t best;
    uint16_t peak;
    uint16_t dip;
    uint16_t scan;
    uint16_t cut;
    uint16_t copy;
    uint16_t segment;
    uint16_t edge_cursor;
    uint16_t edges;
    int16_t  tick;
    uint8_t  phase;
    uint8_t  orientation;
    uint8_t  star_index;
    uint8_t  found;
    bool     b_is_faint : 1;
    bool     b_is_rising : 1;
    bool     b_has_crossed : 1;
    bool     b_is_strong : 1;
    bool     b_does_bias_vary : 1;
    bool     b_has_candidate : 1;
    bool     b_allows_noise : 1;
    bool     b_is_ambiguous : 1;
    bool     b_is_provisional : 1;
    bool     b_is_strict_fit_broken : 1;
    bool     b_is_early : 1; /* Before max_run_us quiet; distinct from tentative
                                noise. */
    bc_decoder_stored_result_t candidate;
} bc_decoder_work_t;

typedef enum
{
    S3_REC_WAITING = 0,
    S3_REC_RECORDING,
    S3_REC_QUIET,
    S3_REC_PROCESSING
} bc_decoder_record_state_t;

typedef struct
{
    uint16_t sample[S3_REC_MAX];  /* Store ten darkness bits and six time-step
                                     bits. */
    uint16_t pre[S3_REC_PRE_MAX]; /* Use the same format in the history ring. */
    uint16_t pre_head;
    uint16_t pre_count;   /* Count the real samples in the history ring. */
    uint32_t pre_us;      /* Measure the interval covered by the ring. */
    uint32_t prefix_us;   /* Exclude this history from the trigger timeout. */
    uint32_t last_us;     /* Store the last reading time in microseconds. */
    uint32_t white_us;    /* Accumulate white before or after ink. */
    uint32_t leading_us;  /* Measure white before retained history. */
    uint32_t frame_us;    /* Measure recorded duration in microseconds. */
    uint32_t required_us; /* Require this much elapsed recording. */
    uint32_t quiet_required_us; /* Require this much uninterrupted quiet. */
    uint16_t prev;              /* Seed an empty frame from this sample. */
    uint16_t residual_us;       /* Carry the time remainder to the next step. */
    uint16_t noise_estimate;    /* Q4 paper noise from absolute differences. */
    uint16_t prominence;        /* Keep prominence fixed during a recording. */
    bc_decoder_stored_result_t
               candidate;  /* Release only after qualified quiet. */
    uint16_t   count : 13; /* The sample count can reach 4096 when full. */
    uint16_t   state : 2;  /* bc_decoder_record_state_t */
    uint16_t   b_has_sample : 1;
    uint16_t   paper : 10;       /* Track the paper darkness reference. */
    uint16_t   best : 10;        /* Track the extremum since the last turn. */
    uint16_t   b_is_falling : 1; /* Set for a dip and clear for a peak. */
    uint16_t   b_is_blocked : 1; /* Wait for white after a failed read. */
    uint32_t   turn_us;          /* Measure time since the last reversal. */
    bc_quiet_t quiet;
    bc_decoder_work_t work;
    uint16_t          work_units;
    uint16_t fit_ticks; /* Track work used by the most recent update. */
    bool     b_accepts_faint_quiet : 1;
    bool     b_blocks_retry : 1;
    bool     b_did_try_early_fit : 1;
    bool     b_has_early_candidate : 1;
} bc_decoder_record_t;

/** Discard recorded samples, pre-trigger history and pending decisions. */
void bc_decoder_record_reset (bc_decoder_record_t * p_record);
/** Return whether this record currently retains an unfinished scan. */
bool bc_decoder_record_active (const bc_decoder_record_t * p_record);
/** Advance the bounded recording/analysis state machine by one timed sample.
 * The caller retains ownership of the record and supplies the configuration.
 * A decoded result is released only after the measured quiet zone qualifies.
 */
bc_decoder_status_t
bc_decoder_record_update (bc_decoder_record_t *       p_record,
                          const bc_decoder_config_t * p_config,
                          uint32_t timestamp_us, bool b_is_valid,
                          uint16_t darkness, bc_decoder_result_t * p_result);
/** Extract diagnostic stripe widths from the recorded
 * readings. Returns the number of stripes (black first); 0 when none or more
 * than S3_BARCODE_RUNS. first/last edge are microseconds from sample 0.
 * This count-only interface uses faint recovery if the primary count is wrong;
 * the live reader validates both candidates independently before selection. */
unsigned int bc_decoder_record_runs (const bc_decoder_record_t * p_record,
                                     uint32_t   p_runs[S3_BARCODE_RUNS],
                                     uint32_t * p_first_edge_us,
                                     uint32_t * p_last_edge_us,
                                     uint32_t * p_end_us,
                                     uint32_t * p_max_step_us);
/** Fit the sentinel widths without inspecting the payload letter.
 * On success, adds bias to black widths and subtracts it
 * from white widths in place. Does not inspect/decode the payload letter.
 * Caller must still validate all widths, gaps and both corrected quiet zones.
 * On failure runs are unchanged. Exposed for replay/host diagnostics. */
bool bc_decoder_record_correct_runs (uint32_t p_runs[S3_BARCODE_RUNS],
                                     uint32_t observed_gap_us,
                                     const bc_decoder_config_t * p_config,
                                     int32_t *                   p_bias_us);
/** Fit each star independently when the shared fit fails.
 * This bounded fallback
 * holds its bias constant within that star and interpolates shared edge
 * offsets by elapsed time between them. bias_us[0/1] are the first/last star
 * biases (even microseconds); outer quiet zones lose bias_us[0/1]/2 exactly.
 * No sign reversal or drift above half the smaller narrow mean is allowed.
 * Payload does not choose the fit. All full-frame checks remain the caller's
 * responsibility. On failure runs are unchanged. */
bool bc_decoder_record_fix_varying (uint32_t p_runs[S3_BARCODE_RUNS],
                                    uint32_t observed_gap_us,
                                    const bc_decoder_config_t * p_config,
                                    int32_t                     p_bias_us[2]);
#endif

/* End of file. */


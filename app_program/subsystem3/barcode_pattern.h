#ifndef BUDDY3_BARCODE_PATTERN_H
#define BUDDY3_BARCODE_PATTERN_H

#include <stdbool.h>
#include <stdint.h>

#define S3_CODE39_SYMBOL_WIDTHS (9U)
#define S3_CODE39_FRAME_WIDTHS (29U)

typedef enum {
    S3_CODE39_OK=0,
    S3_CODE39_INVALID_INPUT,
    S3_CODE39_INSUFFICIENT_SEPARATION,
    S3_CODE39_UNKNOWN_PATTERN,
    S3_CODE39_INVALID_FRAME,
    S3_CODE39_AMBIGUOUS
} s3_code39_status_t;

typedef struct {
    uint32_t bar_threshold_us;
    uint32_t space_threshold_us;
    /* Smallest separation between the wide and narrow classes of one
     * colour. A colour containing only narrow elements is excluded. */
    uint32_t min_separation_us;
    uint16_t pattern;
    char character; /* Full Code 39 alphabet, including '*'; zero if unknown. */
} s3_code39_symbol_t;

typedef struct {
    uint32_t min_separation_us; /* Minimum over the three classified symbols. */
    char letter;
    bool reversed;
} s3_code39_frame_t;

/* Experimental, pure width classification. Widths alternate black/white,
 * beginning and ending with black. Bars and spaces have independent narrow/
 * wide cutoffs; no optical-bias fit or expected character is used.
 *
 * widths_us and out must be nonnull. Every width and max_sample_gap_us must
 * be positive, and the sum of the nine widths must fit uint32_t. Invalid
 * input clears out when it is nonnull. Other failures retain the computed
 * pattern, thresholds and class separation for diagnosis.
 *
 * A known pattern needs more than 2 * max_sample_gap_us between its closest
 * wide/narrow widths of each split colour. This is an experimental sampling
 * margin, not a Code 39 specification tolerance or a noise guarantee.
 *
 * Success says only that these nine widths form a recognizable pattern.
 * It does not verify capture completeness, physical module dimensions,
 * sample timing limits, signal contrast, quiet zones or motion safety. */
s3_code39_status_t s3_code39_classify9(
    const uint32_t widths_us[S3_CODE39_SYMBOL_WIDTHS],
    uint32_t max_sample_gap_us, s3_code39_symbol_t *out);

/* Pattern-only application frame: exactly * + gap + A-Z + gap + *.
 * Both orientations are checked; conflicting valid interpretations refuse.
 * All 29 widths must be positive and their sum must fit uint32_t. The two
 * inter-character spaces are present, but their permissible durations are
 * deliberately not validated here. Output is cleared on every failure.
 *
 * This function does NOT establish leading/trailing quiet, a complete
 * capture, safe timing or a command that is ready to drive the car. Those
 * checks belong to the capture/framing layer before a result is acted on. */
s3_code39_status_t s3_code39_classify_frame(
    const uint32_t *widths_us, unsigned int count,
    uint32_t max_sample_gap_us, s3_code39_frame_t *out);

#endif

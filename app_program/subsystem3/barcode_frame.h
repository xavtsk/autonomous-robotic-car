#ifndef BUDDY3_BARCODE_FRAME_H
#define BUDDY3_BARCODE_FRAME_H

#include "barcode_decoder.h"

typedef enum {
    S3_FRAME_OK=0,
    S3_FRAME_WAIT_QUIET,
    S3_FRAME_INVALID_INPUT,
    S3_FRAME_TIMING,
    S3_FRAME_PATTERN,
    S3_FRAME_GEOMETRY,
    S3_FRAME_LEADING_QUIET
} s3_barcode_frame_status_t;

typedef struct {
    /* Colour-balanced module estimates in acquisition order, not known
     * physical print dimensions and not reordered by scan direction. */
    uint32_t module_us[3];
    uint32_t required_leading_us;
    uint32_t required_trailing_us;
    uint32_t min_separation_us;
    char letter;
    bool reversed;
} s3_barcode_frame_t;

bool s3_barcode_frame_config_valid(const s3_barcode_config_t *config);

/* Validate one candidate frame extracted from real signal edges. The 29
 * positive widths alternate black/white and include both inter-character
 * gaps. Leading/trailing quiet are actual contiguous white durations directly
 * adjacent to the candidate, not guessed/padded or accumulated across ink.
 * observed_gap_us is the maximum actual sample interval over that evidence.
 *
 * Exact *A-Z* patterns and sampling separation come from barcode_pattern.c.
 * A local module estimate is the colour-balanced mean of narrow widths. A
 * shared additive optical width bias would cancel in that estimate without
 * shifting measured edges; this does not establish the physical dimensions.
 * Giving each narrow/wide mean +/- one observed sample interval, its ratio
 * interval must overlap 1.8..3.5. This accounts for finite sampling, not all
 * noise or optical distortion. Each inter-character gap must be 0.5..2 times
 * its neighbouring white-narrow estimate. These are bounded experimental
 * geometry guards, not a conformance claim or an inferred print measurement.
 *
 * Each end needs quiet_modules local modules plus any positive estimated
 * apparent-quiet extension from shortened black bars, plus one sample gap.
 * Pattern span plus required trailing quiet must fit max_frame_us. Long
 * approach white is not part of that deadline.
 *
 * OK means all supplied frame/timing/quiet evidence passed. WAIT_QUIET means
 * everything except trailing quiet passed: the caller must keep monitoring
 * real white and cancel on a new mark, invalid sample or timing failure.
 * Both return a candidate in out; every other status clears out. This API
 * cannot itself verify signal contrast or whether edge extraction omitted a
 * mark. Those are the signal layer's responsibility. It never drives motors.
 */
s3_barcode_frame_status_t s3_barcode_frame_validate(
    const uint32_t *widths_us, unsigned int count,
    uint32_t leading_us, uint32_t trailing_us, uint32_t observed_gap_us,
    const s3_barcode_config_t *config, s3_barcode_frame_t *out);

#endif

#ifndef BUDDY3_BARCODE_SIGNAL_H
#define BUDDY3_BARCODE_SIGNAL_H
#include "barcode_frame.h"

/* Raw ADC is explicit: 0..4095, with polarity supplied by calibration.
 * A causal median of three rejects isolated electrical spikes. Stored values
 * keep all 12 ADC bits; timestamps use six 0.5 ms bits with carried remainder.
 * One bounded ring serves both approach history and the frozen recording.
 * There is no invented leading-white credit when old history is discarded. */
#define S3_SIGNAL_CAPACITY 2560U
#define S3_SIGNAL_HISTORY 512U
#define S3_SIGNAL_BYTES ((S3_SIGNAL_CAPACITY * 18U) / 8U)
#define S3_SIGNAL_EXTREMA 96U

typedef struct {
    uint8_t samples[S3_SIGNAL_BYTES];
    uint32_t last_us, previous_us, sample_us, elapsed_us, white_us, turn_us;
    uint32_t observed_gap_us, next_analysis_us;
    uint16_t previous_raw, older_raw, paper_q4, noise_q4, previous_filtered;
    uint16_t residual_us, best;
    uint16_t head : 12;
    uint16_t count : 12;
    uint16_t prominence : 7;
    uint16_t seen : 2;
    uint16_t recording : 1;
    uint16_t blocked : 1;
    uint16_t falling : 1;
    uint16_t polarity : 1;
    uint16_t frame_status : 3; /* Last batch validation diagnostic, not a command. */
} s3_barcode_signal_t;

void s3_barcode_signal_reset(s3_barcode_signal_t *signal);
bool s3_barcode_signal_active(const s3_barcode_signal_t *signal);
s3_barcode_status_t s3_barcode_signal_update(
    s3_barcode_signal_t *signal, const s3_barcode_config_t *config,
    uint32_t timestamp_us, bool valid, uint16_t raw, bool black_high,
    s3_barcode_result_t *result);
#endif

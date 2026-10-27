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
#endif

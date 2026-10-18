#ifndef BUDDY3_LINE_DETECTION_H
#define BUDDY3_LINE_DETECTION_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    uint16_t signal_floor;  /* Sum of normalised darkness, 0..2000. */
    uint16_t junction_dark; /* Both sensors at/above this, 1..1000. */
    uint32_t lost_us;
    uint32_t reacquire_us;
    uint32_t junction_us;
    uint32_t rearm_us;
    uint32_t max_gap_us;
} s3_line_config_t;
typedef struct
{
    bool active;
    uint32_t since_us;
} s3_persistence_t;
typedef struct
{
    bool valid;
    bool present;
    bool
        junction_candidate; /* Includes a pending, not yet reported junction. */
    int16_t error;          /* -100..100, negative = line left. */
    bool lost_event;
    bool reacquired_event;
    bool junction_event;
} s3_line_observation_t;
typedef struct
{
    s3_line_config_t config;
    s3_persistence_t lost, acquired, junction, clear;
    uint32_t last_us;
    bool configured, seen_sample, present, junction_latched, loss_reported;
} s3_line_detector_t;
s3_line_config_t s3_line_defaults(void);
bool s3_line_init(s3_line_detector_t *detector, const s3_line_config_t *config);
void s3_line_reset(s3_line_detector_t *detector);
s3_line_observation_t s3_line_update(s3_line_detector_t *detector,
                                     uint32_t timestamp_us, bool inputs_valid,
                                     uint16_t left_dark, uint16_t right_dark);
#endif

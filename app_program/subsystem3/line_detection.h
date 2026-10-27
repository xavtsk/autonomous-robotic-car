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
    /* false (default): original design, each sensor sits on one tape edge and
     * "both white" means no line (lost after lost_us).
     * true: the tape runs BETWEEN the two sensors. Both white is the normal,
     * centred state, so the line is never reported lost while following;
     * signal_floor is then a per-sensor "touching the tape" level. Steering
     * error and junction detection are identical in both modes. */
    bool between;
} s3_line_config_t;
typedef struct
{
    uint8_t active : 1;
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
    bool touching; /* between mode: a sensor is at/above signal_floor */
} s3_line_observation_t;
typedef struct
{
    s3_line_config_t config;
    s3_persistence_t lost, acquired, junction, clear;
    uint32_t last_us;
    uint8_t configured : 1;
    uint8_t seen_sample : 1;
    uint8_t present : 1;
    uint8_t junction_latched : 1;
    uint8_t loss_reported : 1;
} s3_line_detector_t;
s3_line_config_t s3_line_defaults(void);
bool s3_line_init(s3_line_detector_t *detector, const s3_line_config_t *config);
void s3_line_reset(s3_line_detector_t *detector);
s3_line_observation_t s3_line_update(s3_line_detector_t *detector,
                                     uint32_t timestamp_us, bool inputs_valid,
                                     uint16_t left_dark, uint16_t right_dark);
#endif

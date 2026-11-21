/** @file line_detection.h
 * @brief Track line error, loss, reacquisition and junction persistence.
 */
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
    bool b_is_tape_between;
} line_detection_config_t;

typedef struct
{
    uint8_t  b_is_active : 1;
    uint32_t since_us;
} line_detection_persistence_t;

typedef struct
{
    bool b_is_valid;
    bool b_is_line_present;
    bool b_has_junction_candidate; /* Includes a pending, not yet reported
                                      junction. */
    int16_t error;                 /* -100..100, negative = line left. */
    bool    b_has_lost_event;
    bool    b_has_reacquired_event;
    bool    b_has_junction_event;
    bool    b_is_touching_tape; /* In between-line mode, the sensor is active at
                                 * or above signal_floor.
                                 */
} line_detection_observation_t;

typedef struct
{
    line_detection_config_t      config;
    line_detection_persistence_t lost;
    line_detection_persistence_t acquired;
    line_detection_persistence_t junction;
    line_detection_persistence_t clear;
    uint32_t                     last_us;
    uint8_t                      b_is_configured : 1;
    uint8_t                      b_has_seen_sample : 1;
    uint8_t                      b_is_line_present : 1;
    uint8_t                      b_is_junction_latched : 1;
    uint8_t                      b_was_loss_reported : 1;
} line_detection_t;

/** @brief Return default darkness thresholds and persistence settings. */
line_detection_config_t line_detection_defaults (void);

/** @brief Initialize the detector with the borrowed thresholds and timing. */
bool line_detection_init (line_detection_t *              p_detector,
                          const line_detection_config_t * p_config);

/** @brief Clear observation history while retaining the configured inputs. */
void line_detection_reset (line_detection_t * p_detector);

/** @brief Observe 0..1000 darkness readings at a wrapping microsecond time. */
line_detection_observation_t
line_detection_update (line_detection_t * p_detector, uint32_t timestamp_us,
                       bool b_are_inputs_valid, uint16_t left_dark,
                       uint16_t right_dark);
#endif

/* End of file. */


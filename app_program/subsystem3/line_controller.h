#ifndef BUDDY3_LINE_CONTROLLER_H
#define BUDDY3_LINE_CONTROLLER_H
#include "line_detection.h"
typedef struct { float base_mps, kp_mps, maximum_mps; } s3_steering_config_t;
typedef struct { bool valid; float left_mps, right_mps; } s3_steering_request_t;
s3_steering_config_t s3_steering_defaults(void);
bool s3_steering_config_valid(const s3_steering_config_t *config);
/* Pure proposal only; no motion API calls. No reverse wheel commands. */
s3_steering_request_t s3_steering_compute(const s3_steering_config_t *config,
                                         const s3_line_observation_t *line);
#endif

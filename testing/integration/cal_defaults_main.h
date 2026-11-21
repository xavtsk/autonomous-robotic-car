/** @file cal_defaults_main.h
 * @brief Kernel callbacks for the built-in calibration fixture.
 *
 * The fixture supplies the clock, ADC and task boundaries for the controller
 * sampling task.
 */
#ifndef TESTING_INTEGRATION_CAL_DEFAULTS_MAIN_H
#define TESTING_INTEGRATION_CAL_DEFAULTS_MAIN_H

#include "runtime_main.h"
#include "../../app_program/controller/s3_integration.h"

#define TA_HLNG 2U
#define TSK_SELF 0

const s3_drive_t * s3_integration_test_drive (void);
void               s3_integration_test_sample_period (void);
float32_t          s3_integration_test_floor_mps (void);
motion_result_t    s3_integration_test_velocity (float32_t left_mps,
                                                 float32_t right_mps);

#endif /* TESTING_INTEGRATION_CAL_DEFAULTS_MAIN_H */

/* End of file. */


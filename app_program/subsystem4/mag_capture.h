/** @file mag_capture.h
 * @brief Report vehicle-axis magnetic measurements during USB calibration.
 */
#ifndef MAG_CAPTURE_H
#define MAG_CAPTURE_H
#include "terr_data.h"

#if APP_MAG_CAL_CAPTURE
/** @brief Offer one valid reading to the USB console at most ten times/s.
 * @param[in] p_sample Borrowed acquisition result in vehicle axes and gauss.
 * @param[in] p_odometry Borrowed motion feedback containing the brake state.
 * @pre Called only by the terrain task with a non-null sample.
 * @details Invalid samples, unavailable feedback, released brakes and an
 * absent USB reader produce no output. Full console buffers drop bytes.
 */
void mag_capture_sample (const imu_sensor_sample_t *  p_sample,
                         const terr_data_odometry_t * p_odometry);
#endif
#endif

/* End of file. */

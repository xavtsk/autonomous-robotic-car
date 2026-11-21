/** @file course_sim_main.h
 * @brief Declare the wheel driver interfaces implemented by the course
 * simulator.
 *
 * The simulator supplies encoder counts and motor actuation for the motion
 * controller. Its motion_request_drive stub uses the motion.h declaration
 * provided by course_profile.h. The bypass fixture reads the pose and drive
 * state through this interface.
 */
#ifndef TESTING_INTEGRATION_COURSE_SIM_MAIN_H
#define TESTING_INTEGRATION_COURSE_SIM_MAIN_H

#include "../../app_program/app_types.h"
#include <stdint.h>
#include "motor_control.h"
#include "wheel_encoder.h"

#include "s3_drive.h"

typedef struct
{
    float64_t x_m;
    float64_t y_m;
    float64_t heading_rad;
} course_sim_pose_t;

/** @brief Read the wrapping simulated microsecond clock. */
uint32_t course_sim_time_us (void);
/** @brief Return the axle position and heading in course coordinates. */
course_sim_pose_t course_sim_pose (void);
/** @brief Borrow the single-threaded simulator's read-only drive state. */
const s3_drive_t * course_sim_drive (void);
/** @brief Empty the simulated course and lift the previous tape cut-off. */
void course_sim_clear_track (void);
/** @brief Set the simulated ratio of achieved to requested turn angle. */
void course_sim_set_turn_scale (float64_t scale);
/** @brief Return the number of failing scenario verdicts. */
int course_sim_failures (void);

/** @brief Append a straight tape segment. Endpoints use metres in the course
 * frame. */
void course_sim_add_line (float64_t start_x, float64_t start_y, float64_t end_x,
                          float64_t end_y);
/** @brief Append a circular tape segment. Its centre and radius are in metres
 * and the two angles are in radians. */
void course_sim_add_arc (float64_t centre_x, float64_t centre_y,
                         float64_t radius_m, float64_t start_rad,
                         float64_t end_rad);
/** @brief Return the distance in metres from a course point to its nearest tape
 * centreline. */
float64_t course_sim_dist_to_tape (float64_t x_coord, float64_t y_coord);
/** @brief Submit a simulated turn in degrees at the requested speed using the
 * configured turn scale. Return the motion controller result. */
motion_result_t course_sim_turn_deg (float32_t deg, float32_t speed);
/** @brief Write the simulated sensor position in metres. Index zero selects the
 * left head and one selects the right. */
void course_sim_sensor_xy (int which, float64_t * p_x_coord,
                           float64_t * p_y_coord);
/** @brief Advance the simulated course by one control step and collect the
 * resulting motion and navigation feedback. */
void course_sim_tick (void);
/** @brief Reset the car pose and controllers for a run. Coordinates and advance
 * are in metres and heading is in radians. */
void course_sim_setup (float64_t x_coord, float64_t y_coord, float64_t heading,
                       float64_t advance_m);
/** @brief Return the distance in metres between the midpoint of the two heads
 * and the tape centreline. */
float64_t course_sim_centre_error (void);
/** @brief Return the display name for a valid drive state. */
const char * course_sim_state_name (s3_drive_state_t state);
/** @brief Return the display name for a valid drive reason. */
const char * course_sim_reason_name (s3_drive_reason_t reason);
/** @brief Print one scenario result and increment the simulated failure count
 * for a failing result. */
void course_sim_verdict (const char * p_name, bool b_is_ok,
                         const char * p_detail);

#endif /* TESTING_INTEGRATION_COURSE_SIM_MAIN_H */

/* End of file. */


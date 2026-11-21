/** @file transport_main.h
 * @brief Provide the host RTOS and terrain transport test interface.
 *
 * The simulated wheel feedback carries timestamps used by the terrain task.
 * IMU and task substitutions remain in the fixture.
 */
#ifndef TESTING_SUBSYSTEM4_TRANSPORT_MAIN_H
#define TESTING_SUBSYSTEM4_TRANSPORT_MAIN_H

#include "../../app_program/subsystem2/motion.h"
#include "../../app_program/subsystem4/buddy4.h"

#define TA_HLNG 2U
#define TA_RNG3 0U
#define TSK_SELF 0
#define EXPORT

/* The transport fixture captures the task and cycle handlers as callbacks. */
typedef struct
{
    FP       task;
    unsigned itskpri;
    unsigned stksz;
    unsigned tskatr;
} T_CTSK;

typedef struct
{
    FP       cychdr;
    unsigned cycatr;
    unsigned cyctim;
    unsigned cycphs;
} T_CCYC;

ID   tk_cre_tsk (const T_CTSK * p_task);
ER   tk_sta_tsk (ID h_task, INT value);
void tk_ext_tsk (void);
ID   tk_cre_cyc (const T_CCYC * p_cycle);

bool buddy4_transport_imu_init (const imu_sensor_bus_t * p_bus);
bool buddy4_transport_imu_read (const imu_sensor_bus_t * p_bus, uint32_t now,
                                imu_sensor_sample_t * p_sample);
void buddy4_transport_observe (terr_data_monitor_t *        p_monitor,
                               const imu_sensor_sample_t *  p_sample,
                               const terr_data_odometry_t * p_odometry);
void buddy4_transport_publish (const terr_data_result_t * p_terrain);
void buddy4_transport_process (terr_data_monitor_t *        p_monitor,
                               const imu_sensor_sample_t *  p_sample,
                               const terr_data_odometry_t * p_odometry);
terr_data_hump_geometry_t buddy4_transport_geometry (void);
float32_t                 buddy4_transport_track_m (void);
void                      buddy4_transport_set_hump_count (uint16_t count);
void                      buddy4_transport_set_run (uint16_t token);
#if APP_IMU_DEBUG
void     buddy4_transport_clear_debug (void);
uint32_t buddy4_transport_debug_sequence (void);
uint16_t buddy4_transport_debug_run (void);
unsigned buddy4_transport_debug_capacity (void);
void     buddy4_transport_record_debug (const imu_sensor_sample_t * p_sample,
                                        const terr_data_monitor_t * p_monitor);
#endif

#endif /* TESTING_SUBSYSTEM4_TRANSPORT_MAIN_H */

/* End of file. */


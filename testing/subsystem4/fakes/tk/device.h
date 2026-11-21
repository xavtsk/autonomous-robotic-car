/** @file device.h
 * @brief Provide device API declarations used by terrain transport host tests.
 */
#ifndef S4_TEST_DEVICE_H
#define S4_TEST_DEVICE_H
typedef int SZ;
#define TD_UPDATE 3
ER tk_swri_dev (ID h_device, int start, const void * p_buffer, SZ length,
                SZ * p_actual);
ID tk_opn_dev (UB * p_name, unsigned mode);
#endif

/* End of file. */


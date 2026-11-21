/** @file map_freeze_main.h
 * @brief Kernel callbacks for the barcode-map locking fixture.
 *
 * The test supplies the mailbox and lock operations used by the configuration
 * owner and the controller.
 */
#ifndef TESTING_INTEGRATION_MAP_FREEZE_MAIN_H
#define TESTING_INTEGRATION_MAP_FREEZE_MAIN_H

#include "runtime_main.h"
#ifdef S3_MAP_FREEZE_TEST
#include "../../app_program/controller/s3_integration.h"
#define TA_HLNG 2U
#define TSK_SELF 0

const s3_drive_t * s3_integration_test_drive (void);
void               s3_integration_test_sample_period (void);
void               s3_integration_test_set_calibrated (void);
#endif

#define E_SYS (-5)
ER  tk_snd_mbf (ID h_message_buffer, const void * p_message, INT size,
                TMO timeout);
INT tk_rcv_mbf (ID h_message_buffer, void * p_message, TMO timeout);
ER  tk_dly_tsk (unsigned delay_ms);
INT tm_printf (const UB * p_format, ...);

#endif /* TESTING_INTEGRATION_MAP_FREEZE_MAIN_H */

/* End of file. */


/** @file work_quota_main.h
 * @brief Declare the decoder probes used by the bounded-work host fixture.
 */
#ifndef TESTING_SUBSYSTEM3_WORK_QUOTA_MAIN_H
#define TESTING_SUBSYSTEM3_WORK_QUOTA_MAIN_H

#include "bc_decoder.h"

bool bc_decoder_work_test_candidate (const bc_decoder_record_t * p_record,
                                     const bc_decoder_config_t * p_config,
                                     bool                        b_is_faint,
                                     bc_decoder_result_t *       p_candidate,
                                     uint32_t *                  p_required_us);
bool bc_decoder_work_test_faint_eligible (const bc_decoder_record_t * p_record);
void bc_decoder_work_test_advance (bc_decoder_record_t *       p_record,
                                   const bc_decoder_config_t * p_config);
bool bc_decoder_work_test_is_done (const bc_decoder_record_t * p_record);

#endif /* TESTING_SUBSYSTEM3_WORK_QUOTA_MAIN_H */

/* End of file. */


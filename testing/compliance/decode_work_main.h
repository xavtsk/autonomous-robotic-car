/** @file decode_work_main.h
 * @brief Identify the standalone barcode work-budget regression fixture.
 *
 * The runner invokes main. These probes run the subject's private decoder
 * steps using recorder objects owned by the fixture.
 */
#ifndef TESTING_COMPLIANCE_DECODE_WORK_MAIN_H
#define TESTING_COMPLIANCE_DECODE_WORK_MAIN_H

#include "bc_decoder.h"

bool bc_decoder_boundary_test_config_valid (
    const bc_decoder_config_t * p_config);
void bc_decoder_boundary_test_begin (bc_decoder_record_t * p_record,
                                     bool b_is_provisional, bool b_is_early);
void bc_decoder_boundary_test_advance (bc_decoder_record_t *       p_record,
                                       const bc_decoder_config_t * p_config);

#endif /* TESTING_COMPLIANCE_DECODE_WORK_MAIN_H */

/* End of file. */


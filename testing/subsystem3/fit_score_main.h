/** @file fit_score_main.h
 * @brief Declare the decoder entry points built only for the fit fixture.
 */
#ifndef TESTING_SUBSYSTEM3_FIT_SCORE_MAIN_H
#define TESTING_SUBSYSTEM3_FIT_SCORE_MAIN_H

#include "bc_decoder.h"

uint32_t bc_decoder_fit_test_score (const uint32_t p_adjusted[9], uint16_t star,
                                    uint32_t narrow, uint32_t wide);
bool bc_decoder_fit_test_config_valid (const bc_decoder_config_t * p_config);
bool bc_decoder_fit_test_character (const uint32_t p_widths[9], uint32_t gap,
                                    const bc_decoder_config_t * p_config,
                                    uint16_t * p_pattern, uint32_t * p_narrow);

#endif /* TESTING_SUBSYSTEM3_FIT_SCORE_MAIN_H */

/* End of file. */


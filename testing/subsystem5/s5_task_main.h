/** @file s5_task_main.h
 * @brief Kernel callbacks for the obstacle task fixture.
 *
 * The fixture controls locking, task entry and sensor completion while the
 * obstacle task publishes scan state.
 */
#ifndef TESTING_SUBSYSTEM5_S5_TASK_MAIN_H
#define TESTING_SUBSYSTEM5_S5_TASK_MAIN_H

#include "../integration/ctrl_test_kernel.h"
#include <stdbool.h>

void s5_task_test_reset_mailbox (void);
void s5_task_test_set_reserved_bit (bool b_should_set);

#endif /* TESTING_SUBSYSTEM5_S5_TASK_MAIN_H */

/* End of file. */


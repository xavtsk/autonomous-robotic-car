/** @file smoke_motion.h
 * @brief Declare the separately enabled hardware motion smoke-test entry point.
 */
#ifndef MOTION_SMOKE_TEST_H
#define MOTION_SMOKE_TEST_H
#ifndef APP_MOTION_SMOKE
#define APP_MOTION_SMOKE 0
#endif
#if APP_MOTION_SMOKE
void motion_smoke_run (void);
#endif
#endif

/* End of file. */


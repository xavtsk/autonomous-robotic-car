/** @file test_navigation.h
 * @brief Expose the existing navigation integration test entry point.
 */
#ifndef TESTING_SUBSYSTEM3_TEST_NAVIGATION_H
#define TESTING_SUBSYSTEM3_TEST_NAVIGATION_H

/** @brief Run synthetic navigation, barcode and odometry assertions.
 * @post Returns after all selected cases finish. With assertions enabled,
 * a failed assertion terminates the test process; there is no error result.
 * @note Exercises readiness/updates, faults, rearming/timing, line modes,
 * diagnostics and clearance/epochs. S3_BASELINE_TRACE omits the faint-barcode
 * case. This standalone test uses synthetic frames and does not drive hardware.
 */
void test_navigation (void);

#endif /* TESTING_SUBSYSTEM3_TEST_NAVIGATION_H */

/* End of file. */


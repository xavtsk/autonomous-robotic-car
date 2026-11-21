/** @file motion_trace.h
 * @brief Record wheel feedback and reports for the differential host run.
 */
#ifndef TOOLS_COMPLIANCE_TRACES_MOTION_TRACE_H
#define TOOLS_COMPLIANCE_TRACES_MOTION_TRACE_H

#include "motion.h"

void motion_trace_feedback (void);
void motion_trace_report (bool received, const motion_report_t * p_report);

#endif /* TOOLS_COMPLIANCE_TRACES_MOTION_TRACE_H */

/* End of file. */


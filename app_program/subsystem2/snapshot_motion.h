/** @file snapshot_motion.h
 * @brief Read-only compatibility view of the motion owner's status.
 */
#ifndef MOTION_SNAPSHOT_ADAPTER_H
#define MOTION_SNAPSHOT_ADAPTER_H

#include "codec_motion.h"

/** Copy and pack the latest motion status without consuming a completion. */
bool motion_read_snapshot (motion_snapshot_t * p_snapshot);

#endif /* MOTION_SNAPSHOT_ADAPTER_H */

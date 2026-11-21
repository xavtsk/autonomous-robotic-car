/** @file terr_link.h
 * @brief Integrate terrain readiness, reset and observations with the
 * controller owner.
 */
#ifndef TERRAIN_INTEGRATION_H
#define TERRAIN_INTEGRATION_H
#include "s3_drive.h"
#if APP_S4
/* All calls belong to the existing sampling/controller task. No new motor
 * requester, report consumer, task or shared unprotected snapshot. */
void terr_link_init (void);
void terr_link_poll (s3_drive_t * p_drive, uint32_t now);
bool terr_link_ready (uint32_t now);
bool terr_link_begin (uint32_t now);
bool terr_link_reset_ready (uint32_t now);
/* Clear the latch only after the drive owner acknowledges a successful RESET.
 */
void terr_link_clear_fault (void);
void terr_link_report (void);
#else
/** @brief Leave the controller ready when the terrain task is disabled. */
static inline void terr_link_init (void)
{
}

/** @brief Accept a controller tick without reading or modifying the drive. */
static inline void terr_link_poll (s3_drive_t * p_drive, uint32_t now_us)
{
    (void)p_drive;
    (void)now_us;
}

/** @brief Allow the run to start at any time when terrain sensing is off. */
static inline bool terr_link_ready (uint32_t now_us)
{
    (void)now_us;
    return true;
}

/** @brief Accept a new run without initiating a terrain handshake. */
static inline bool terr_link_begin (uint32_t now_us)
{
    (void)now_us;
    return true;
}

/** @brief Allow a reset at any time in the profile without terrain sensing. */
static inline bool terr_link_reset_ready (uint32_t now_us)
{
    (void)now_us;
    return true;
}

/** @brief Complete the clear request without a terrain fault latch. */
static inline void terr_link_clear_fault (void)
{
}

/** @brief Finish the reporting step without publishing a terrain message. */
static inline void terr_link_report (void)
{
}
#endif
#endif

/* End of file. */


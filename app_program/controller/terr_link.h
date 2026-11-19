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
void terrain_control_init (void);
void terrain_control_poll (s3_drive_t * p_drive, uint32_t now);
bool terrain_control_ready (uint32_t now);
bool terrain_control_begin (uint32_t now);
bool terrain_control_reset_ready (uint32_t now);
/* Clear the latch only after the drive owner acknowledges a successful RESET.
 */
void terrain_control_clear_fault (void);
void terrain_control_report (void);
#else
static inline void terrain_control_init (void)
{
}

static inline void terrain_control_poll (s3_drive_t * d, uint32_t n)
{
    (void)d;
    (void)n;
}

static inline bool terrain_control_ready (uint32_t n)
{
    (void)n;
    return true;
}

static inline bool terrain_control_begin (uint32_t n)
{
    (void)n;
    return true;
}

static inline bool terrain_control_reset_ready (uint32_t n)
{
    (void)n;
    return true;
}

static inline void terrain_control_clear_fault (void)
{
}

static inline void terrain_control_report (void)
{
}
#endif
#endif

/* End of file. */


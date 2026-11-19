#include "line_recovery.h"

void s5_recovery_reset (s5_recovery_t * p_r)
{
    *p_r = (s5_recovery_t){0};
}

bool s5_recovery_observe (s5_recovery_t * p_r, uint32_t now, bool healthy,
                          bool tape, bool junction, bool corridor)
{
    if (!healthy || !tape || junction || !corridor)
    {
        p_r->touching = false;
        return p_r->confirmed;
    }
    if (!p_r->touching)
    {
        p_r->touching = true;
        p_r->since_us = now;
    }
    if ((uint32_t)(now - p_r->since_us) >= 30000U)
    {
        p_r->confirmed = true;
    }
    return p_r->confirmed;
}

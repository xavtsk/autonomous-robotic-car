#include "line_recovery.h"
void s5_recovery_reset(s5_recovery_t *r) { *r=(s5_recovery_t){0}; }
bool s5_recovery_observe(s5_recovery_t *r,uint32_t now,bool healthy,
                         bool tape,bool junction,bool corridor)
{
    if(!healthy || !tape || junction || !corridor) { r->touching=false; return r->confirmed; }
    if(!r->touching) { r->touching=true; r->since_us=now; }
    if((uint32_t)(now-r->since_us)>=30000U) { r->confirmed=true; }
    return r->confirmed;
}

#ifndef BUDDY5_LINE_RECOVERY_H
#define BUDDY5_LINE_RECOVERY_H
#include "s5_config.h"
typedef struct { uint32_t since_us; unsigned touching:1, confirmed:1; } s5_recovery_t;
void s5_recovery_reset(s5_recovery_t *r);
/* Evidence must be near the predicted original corridor, fresh, sustained,
 * non-junction tape. White-only 'between sensors' tracking is NOT evidence. */
bool s5_recovery_observe(s5_recovery_t *r,uint32_t now,bool healthy,
                         bool tape,bool junction,bool in_corridor);
#endif

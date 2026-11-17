#ifndef BUDDY5_OBSTACLE_SCAN_H
#define BUDDY5_OBSTACLE_SCAN_H
#include "obstacle_profile.h"
typedef struct {
    s5_sample_t sample[S5_SAMPLES_MAX];
    unsigned count:5, index:5, angle:8, fine_end:8, nearest:8;
    unsigned attempts:2, fine:1, done:1, failed:1, single:1;
    uint16_t nearest_mm;
    uint32_t started_us;
} s5_scan_t;
void s5_scan_begin(s5_scan_t *scan,uint32_t now);
/* Peek: one valid reading at a fixed side angle (30..60 or 120..150, 5 deg). */
bool s5_scan_begin_at(s5_scan_t *scan,uint32_t now,unsigned angle);
bool s5_peek_angle_valid(unsigned angle);
/* One completed measurement per call; retries stay at the same angle. */
void s5_scan_accept(s5_scan_t *scan,uint32_t now,bool valid,unsigned mm);
#endif

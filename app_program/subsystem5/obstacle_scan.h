#ifndef BUDDY5_OBSTACLE_SCAN_H
#define BUDDY5_OBSTACLE_SCAN_H
#include "obstacle_profile.h"
typedef struct {
    s5_sample_t sample[S5_SAMPLES_MAX];
    s5_config_t config;
    uint8_t fine_angle[10];
    uint16_t readings[3];
    unsigned count:5, index:5, angle:8, fine_count:4, fine_index:4;
    unsigned attempts:2, reading_count:2, fine:1, done:1, failed:1, single:1;
    uint32_t started_us;
} s5_scan_t;
void s5_scan_begin(s5_scan_t *scan,uint32_t now);
void s5_scan_begin_profile(s5_scan_t *scan,uint32_t now,const s5_config_t *config);
/* Peek: median of three valid readings at a fixed side angle
 * (30..60 or 120..150, 5 deg). */
bool s5_scan_begin_at(s5_scan_t *scan,uint32_t now,unsigned angle);
bool s5_peek_angle_valid(unsigned angle);
/* One completed measurement per call; three valid readings form one median
 * sample. Three consecutive invalid returns or whole-scan timeout fail. */
void s5_scan_accept(s5_scan_t *scan,uint32_t now,bool valid,unsigned mm);
#endif

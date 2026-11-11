#ifndef BUDDY4_HUMP_DETECTION_H
#define BUDDY4_HUMP_DETECTION_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float height_m, peak_m, highest_m, travel_m;
    uint32_t started_us;
    unsigned level_samples;
    bool active, descended, highest_valid, completed, armed;
} hump_detector_t;
void hump_reset(hump_detector_t *h);
/* Caller supplies signed distance delta, pitch positive nose-up. Only complete
 * ascent/descent cycles on valid, low-dynamic, forward travel become run peaks. */
void hump_update(hump_detector_t *h, float pitch, float distance_m, bool valid, uint32_t now);
/* Right wheel above a level left wheel: height = track*sin(roll)*cos(pitch).
 * Positive roll raises the right side. Require forward travel and a complete
 * level -> raised -> level excursion; do not integrate pitch in this mode. */
void hump_update_right_wheel(hump_detector_t *h, float pitch, float roll,
                            float track_m, float distance_m, bool valid, uint32_t now);
#endif

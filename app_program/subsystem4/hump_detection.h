#ifndef BUDDY4_HUMP_DETECTION_H
#define BUDDY4_HUMP_DETECTION_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float height_m, peak_m, highest_m;
    uint32_t started_us;
    unsigned level_samples;
    bool active, descended, highest_valid, completed;
} hump_detector_t;
void hump_reset(hump_detector_t *h);
/* Caller supplies signed distance delta, pitch positive nose-up. Only complete
 * ascent/descent cycles on valid, low-dynamic, forward travel become run peaks. */
void hump_update(hump_detector_t *h, float pitch, float distance_m, bool valid, uint32_t now);
#endif

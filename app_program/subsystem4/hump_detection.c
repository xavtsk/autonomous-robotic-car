#include "hump_detection.h"
#include <math.h>
void hump_reset(hump_detector_t *h) { *h=(hump_detector_t){0}; }
void hump_update(hump_detector_t *h,float pitch,float ds,bool valid,uint32_t now)
{
    h->completed=false;
    if(!valid || !isfinite(ds) || !isfinite(pitch) || ds<0 || ds>0.05f || fabsf(pitch)>40 ||
       (h->active && (uint32_t)(now-h->started_us)>10000000U)) {
        h->active=false; h->height_m=0; h->peak_m=0; h->descended=false; h->level_samples=0; return;
    }
    if(!h->active && pitch>4.0f && ds>0.0001f) {
        h->active=true; h->started_us=now; h->height_m=0; h->peak_m=0; h->descended=false; h->level_samples=0;
    }
    if(!h->active) { return; }
    h->height_m+=ds*sinf(pitch*0.01745329252f);
    if(h->height_m>h->peak_m) { h->peak_m=h->height_m; }
    if(h->peak_m>0.50f || h->height_m < -0.03f) { h->active=false; return; }
    if(pitch < -4.0f) { h->descended=true; }
    h->level_samples=(h->descended && fabsf(pitch)<2.0f)?h->level_samples+1:0;
    if(h->level_samples>=5) {
        if(h->peak_m>=0.005f && fabsf(h->height_m)<0.03f) {
            if(!h->highest_valid || h->peak_m>h->highest_m) { h->highest_m=h->peak_m; }
            h->highest_valid=true; h->completed=true;
        }
        h->active=false;
    }
}

static void right_discard(hump_detector_t *h)
{
    h->active=false; h->armed=false; h->height_m=0; h->peak_m=0;
    h->travel_m=0; h->level_samples=0;
}
void hump_update_right_wheel(hump_detector_t *h,float pitch,float roll,
                            float track_m,float ds,bool valid,uint32_t now)
{
    const float start_m=0.005f, level_m=0.003f;
    float height;
    h->completed=false;
    if(!valid || !isfinite(ds) || ds<0 || ds>0.05f ||
       !isfinite(pitch) || !isfinite(roll) || fabsf(pitch)>40 || fabsf(roll)>=10 ||
       !isfinite(track_m) || track_m<=0 || track_m>0.5f ||
       (h->active && (uint32_t)(now-h->started_us)>10000000U)) {
        right_discard(h); return;
    }
    height=track_m*sinf(roll*0.01745329252f)*cosf(pitch*0.01745329252f);
    if(height < -level_m) { right_discard(h); return; } /* left wheel raised */
    if(!h->active) {
        if(fabsf(height)<=level_m) {
            if(h->level_samples<5) { ++h->level_samples; }
            if(h->level_samples>=5) { h->armed=true; }
        } else { h->level_samples=0; }
        if(!h->armed || height<start_m || ds<=0.0001f) { return; }
        h->active=true; h->armed=false; h->started_us=now;
        h->peak_m=0; h->travel_m=0; h->level_samples=0;
    }
    h->height_m=height<0?0:height;
    if(height>h->peak_m) { h->peak_m=height; }
    /* A brief roll spike followed by flat travel must not qualify as a hump. */
    if(height>=start_m) { h->travel_m+=ds; }
    h->level_samples=fabsf(height)<=level_m?h->level_samples+1:0;
    if(h->level_samples>=5) {
        if(h->peak_m>=start_m && h->travel_m>=0.005f) {
            if(!h->highest_valid || h->peak_m>h->highest_m) { h->highest_m=h->peak_m; }
            h->highest_valid=true; h->completed=true;
        }
        h->active=false; h->height_m=0; h->armed=true;
    }
}

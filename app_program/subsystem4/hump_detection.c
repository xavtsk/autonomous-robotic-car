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

static hump_reason_t right_discard(hump_detector_t *h,hump_reason_t reason)
{
    h->active=false; h->armed=false; h->height_m=0; h->peak_m=0;
    h->travel_m=0; h->level_samples=0; h->skipped_acceleration=false;
    return reason;
}

static hump_reason_t right_step(hump_detector_t *h,float pitch,float roll,
                                float track_m,float ds,hump_sample_quality_t quality,uint32_t now)
{
    const float start_m=0.005f, level_m=0.003f;
    float height;
    h->completed=false;
    if(quality==HUMP_SAMPLE_INVALID) {
        /* The caller supplies the sensor or motion gate that rejected it. */
        return right_discard(h,HUMP_REASON_NONE);
    }
    if(!isfinite(ds) || ds>0.05f) { return right_discard(h,HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE); }
    if(ds<0) { return right_discard(h,HUMP_REASON_REVERSE_TRAVEL); }
    if(!isfinite(pitch) || !isfinite(roll) || fabsf(pitch)>40 || fabsf(roll)>=10 ||
       !isfinite(track_m) || track_m<=0 || track_m>0.5f) {
        return right_discard(h,HUMP_REASON_INVALID_GEOMETRY);
    }
    if(h->active && (uint32_t)(now-h->started_us)>10000000U) {
        return right_discard(h,HUMP_REASON_CROSSING_TIMEOUT);
    }
    if(quality==HUMP_SAMPLE_ACCELERATION_SPIKE) {
        if(h->active && !h->skipped_acceleration) {
            /* Hold the last accepted height and travel through one sample. */
            h->skipped_acceleration=true;
            h->level_samples=0;
            return HUMP_REASON_ACCELERATION_SPIKE;
        }
        return right_discard(h,HUMP_REASON_ACCELERATION_SPIKE);
    }
    h->skipped_acceleration=false;
    height=track_m*sinf(roll*0.01745329252f)*cosf(pitch*0.01745329252f);
    if(height < -level_m) { return right_discard(h,HUMP_REASON_LEFT_WHEEL_RAISED); }
    if(!h->active) {
        if(fabsf(height)<=level_m) {
            if(h->level_samples<5) { ++h->level_samples; }
            if(h->level_samples>=5) { h->armed=true; }
        } else { h->level_samples=0; }
        if(!h->armed) { return HUMP_REASON_WAITING_FOR_LEVEL; }
        if(height<start_m) { return HUMP_REASON_BELOW_START_HEIGHT; }
        if(ds<=0.0001f) { return HUMP_REASON_NO_FORWARD_TRAVEL; }
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
        return h->completed ? HUMP_REASON_NONE : HUMP_REASON_INSUFFICIENT_CROSSING_TRAVEL;
    }
    return HUMP_REASON_NONE;
}

void hump_update_right_wheel(hump_detector_t *h,float pitch,float roll,
                            float track_m,float ds,hump_sample_quality_t quality,uint32_t now)
{
#if APP_IMU_DEBUG
    const bool was_active=h->active, was_armed=h->armed;
    const float prior_travel=h->travel_m;
    h->debug.reason=(uint8_t)right_step(h,pitch,roll,track_m,ds,quality,now);
    h->debug.travel_m=h->active ? h->travel_m : (was_active ? prior_travel : 0);
    h->debug.flags=(uint8_t)((!was_armed && h->armed ? HUMP_DEBUG_ARMED : 0) |
                             (!was_active && h->active ? HUMP_DEBUG_STARTED : 0) |
                             (was_active && !h->active && !h->completed ? HUMP_DEBUG_DISCARDED : 0) |
                             (h->completed ? HUMP_DEBUG_COMPLETED : 0) |
                             (h->skipped_acceleration ? HUMP_DEBUG_SKIPPED : 0) |
                             (h->active ? HUMP_DEBUG_ACTIVE : 0) |
                             (h->armed ? HUMP_DEBUG_IS_ARMED : 0));
#else
    (void)right_step(h,pitch,roll,track_m,ds,quality,now);
#endif
}

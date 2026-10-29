#include "imu_calibration.h"
#include <math.h>
void imu_calibration_reset(imu_calibration_t *c) { *c=(imu_calibration_t){0}; }
bool imu_calibration_feed(imu_calibration_t *c,const imu_sample_t *s,bool stationary)
{
    unsigned i;
    if (c->ready) { return true; }
    /* Overrun only means a sample was skipped; values stay valid (BDU). */
    if (!stationary || !imu_sample_is_valid(s) ||
        fabsf(s->accel_g[0])>0.15f || fabsf(s->accel_g[1])>0.15f ||
        fabsf(s->accel_g[2]-1.0f)>0.15f) { imu_calibration_reset(c); return false; }
    for (i=0;i<3;++i) {
        float a=s->accel_g[i];
        if (!c->count) { c->min[i]=a; c->max[i]=a; }
        if (a<c->min[i]) { c->min[i]=a; }
        if (a>c->max[i]) { c->max[i]=a; }
        if (c->max[i]-c->min[i]>0.04f) { imu_calibration_reset(c); return false; }
        c->sum[i]+=a;
    }
    if (++c->count==IMU_CAL_SAMPLES) {
        for (i=0;i<3;++i) { c->bias_g[i]=c->sum[i]/IMU_CAL_SAMPLES; }
        c->bias_g[2]-=1.0f; c->ready=true;
    }
    return c->ready;
}

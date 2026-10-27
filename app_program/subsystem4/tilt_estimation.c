#include "tilt_estimation.h"
#include <math.h>
#define DEG 57.295779513f
bool tilt_estimation(const float a[3],float *pitch,float *roll)
{
    float n=a[0]*a[0]+a[1]*a[1]+a[2]*a[2];
    if (!isfinite(n) || n<0.81f || n>1.21f || a[2]<=0.0f) { return false; }
    /* Positive pitch = nose up. Accelerometer measures support against gravity. */
    *pitch=atan2f(a[0],sqrtf(a[1]*a[1]+a[2]*a[2]))*DEG;
    *roll=atan2f(a[1],a[2])*DEG;
    return true;
}
bool tilt_heading(const float a[3],const float m[3],float *heading)
{
    /* Project vehicle forward and magnetic north onto the gravity-normal plane.
     * Heading increases anticlockwise, matching Buddy 2. */
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    float norm=m[0]*m[0]+m[1]*m[1]+m[2]*m[2];
    float u[3], f[3], left[3], fn, x,y;
    unsigned i;
    if (!isfinite(n) || n<0.9f || n>1.1f || !isfinite(norm) || norm<0.01f || norm>0.64f) { return false; }
    for(i=0;i<3;++i) { u[i]=a[i]/n; f[i]=(i==0?1.0f:0.0f)-u[0]*u[i]; }
    fn=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
    if (fn<0.2f) { return false; }
    for(i=0;i<3;++i) { f[i]/=fn; }
    left[0]=u[1]*f[2]-u[2]*f[1]; left[1]=u[2]*f[0]-u[0]*f[2]; left[2]=u[0]*f[1]-u[1]*f[0];
    x=m[0]*f[0]+m[1]*f[1]+m[2]*f[2]; y=m[0]*left[0]+m[1]*left[1]+m[2]*left[2];
    if (x*x+y*y<0.0025f) { return false; }
    *heading=-atan2f(y,x)*DEG;
    if (*heading<0) { *heading+=360.0f; }
    return true;
}
float tilt_angle_delta(float now,float previous)
{
    float d=now-previous;
    if(d>180.0f) { d-=360.0f; } else if(d < -180.0f) { d+=360.0f; }
    return d;
}

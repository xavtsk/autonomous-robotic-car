/* LSM303DLHC, ST DocID018771 Rev 2. 7-bit I2C addresses. */
#include "imu_sensor.h"
#include <math.h>
static int32_t signed16(uint8_t hi, uint8_t lo)
{
    uint32_t value = ((uint32_t)hi << 8) | lo;
    return (value & 0x8000U) ? (int32_t)value - 65536 : (int32_t)value;
}
bool imu_sample_is_valid(const imu_sample_t *s)
{
    unsigned i;
    if (!s || !s->valid) { return false; }
    for (i=0; i<3; ++i) {
        if (!isfinite(s->accel_g[i])) { return false; }
        if (s->mag_valid && !isfinite(s->mag_gauss[i])) { return false; }
    }
    return true;
}
bool imu_sensor_init(const imu_bus_t *b)
{
    uint8_t id[3], value;
    if (!b || !b->read || !b->write) { return false; }
    /* Magnetometer identity H43. Accel reserved WHO_AM_I is not relied upon. */
    if (!b->read(b->context,0x1e,0x0a,id,3) || id[0]!=0x48 || id[1]!=0x34 || id[2]!=0x33) { return false; }
    /* 50 Hz acceleration, BDU, high resolution, +/-4g (2 mg/LSB).
     * Magnetometer 30 Hz, +/-1.3 gauss, continuous conversion. */
    if (!b->write(b->context,0x19,0x20,0x47) ||
        !b->write(b->context,0x19,0x21,0x00) ||
        !b->write(b->context,0x19,0x23,0x98) ||
        !b->write(b->context,0x1e,0x00,0x14) ||
        !b->write(b->context,0x1e,0x01,0x20) ||
        !b->write(b->context,0x1e,0x02,0x00)) { return false; }
    return b->read(b->context,0x19,0x23,&value,1) && value==0x98;
}
bool imu_sensor_read(const imu_bus_t *b, uint32_t now, imu_sample_t *out)
{
    uint8_t raw[6], status;
    int32_t x,y,z;
    if (!out) { return false; }
    *out=(imu_sample_t){0}; out->timestamp_us=now;
    if (!b || !b->read || !b->read(b->context,0x19,0x27,&status,1) || !(status & 8)) { return false; }
    out->overrun=(status & 0x80)!=0;
    if (!b->read(b->context,0x19,0xa8,raw,6)) { return false; }
    x=signed16(raw[1],raw[0])/16; y=signed16(raw[3],raw[2])/16; z=signed16(raw[5],raw[4])/16;
    /* Reject rail saturation, preserving negative samples. */
    if (x<=-2048 || x>=2047 || y<=-2048 || y>=2047 || z<=-2048 || z>=2047) { return false; }
    out->accel_g[0]=-y*0.002f; out->accel_g[1]=x*0.002f; out->accel_g[2]=z*0.002f;
    out->valid=true;
    if (b->read(b->context,0x1e,0x09,&status,1) && (status & 3) &&
        b->read(b->context,0x1e,0x03,raw,6)) {
        /* Mag byte order is big endian X,Z,Y; a full burst releases LOCK. */
        x=signed16(raw[0],raw[1]); z=signed16(raw[2],raw[3]); y=signed16(raw[4],raw[5]);
        if (x>=-2048 && x<=2047 && y>=-2048 && y<=2047 && z>=-2048 && z<=2047) {
            out->mag_gauss[0]=-y/1100.0f; out->mag_gauss[1]=x/1100.0f; out->mag_gauss[2]=z/980.0f;
            out->mag_valid=true;
        }
    }
    return true;
}

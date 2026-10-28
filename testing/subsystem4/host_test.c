#include "imu_sensor.h"
#include "terrain_monitor.h"
#include "tilt_estimation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint8_t acc[256],mag[256]; static bool fail;
static bool rd(void *c,uint8_t addr,uint8_t reg,uint8_t *out,unsigned n)
{ (void)c; if(fail)return false; memcpy(out,(addr==0x19?acc:mag)+(addr==0x19?(reg&0x7f):reg),n); return true; }
static bool wr(void *c,uint8_t addr,uint8_t reg,uint8_t v)
{ (void)c; if(fail)return false; (addr==0x19?acc:mag)[reg]=v; return true; }
static void le(unsigned reg,int value) { unsigned v=(unsigned)(value*16)&65535U; acc[reg]=(uint8_t)v; acc[reg+1]=(uint8_t)(v>>8); }
static uint32_t bits(const uint8_t *p,unsigned start,unsigned count)
{ uint32_t v=0; unsigned i; for(i=0;i<count;i++) v|=((p[(start+i)/8]>>((start+i)%8))&1U)<<i; return v; }
int main(void)
{
 imu_bus_t bus={0,rd,wr}; imu_sample_t s={0}; terrain_monitor_t t; terrain_odometry_t o={0};
 float pitch,roll; uint8_t packed[TERRAIN_PACKET_SIZE]; unsigned i;
 mag[10]='H';mag[11]='4';mag[12]='3';
 assert(imu_sensor_init(&bus)); assert(acc[0x20]==0x47 && acc[0x23]==0x98);
 mag[10]=0;assert(!imu_sensor_init(&bus));mag[10]='H';
 acc[0x27]=8;le(0x28,-100);le(0x2a,200);le(0x2c,500);
 assert(imu_sensor_read(&bus,123,&s));assert(fabsf(s.accel_g[0]+0.4f)<0.001f);
 assert(fabsf(s.accel_g[1]+0.2f)<0.001f && fabsf(s.accel_g[2]-1)<0.001f);
 fail=true;assert(!imu_sensor_read(&bus,124,&s) && !s.valid);fail=false;
 le(0x28,-2048);assert(!imu_sensor_read(&bus,125,&s));le(0x28,0);
 mag[9]=1;mag[3]=0;mag[4]=110;mag[5]=0;mag[6]=98;mag[7]=0xff;mag[8]=0x92;
 assert(imu_sensor_read(&bus,126,&s) && s.mag_valid);
 assert(fabsf(s.mag_gauss[0]-0.1f)<0.001f && fabsf(s.mag_gauss[1]-0.1f)<0.001f && fabsf(s.mag_gauss[2]-0.1f)<0.001f);
 mag[3]=0xf0;mag[4]=0;assert(imu_sensor_read(&bus,127,&s) && !s.mag_valid);
 {float a[3]={0.5f,0,0.8660254f};assert(tilt_estimation(a,&pitch,&roll));assert(fabsf(pitch-30)<0.01f);}
 assert(fabsf(tilt_angle_delta(1,359)-2)<0.001f);
 terrain_monitor_init(&t);o.stationary=true;s=(imu_sample_t){0};s.valid=true;s.accel_g[2]=1;
 for(i=1;i<=100;i++){s.timestamp_us=i*20000;terrain_monitor_process(&t,&s,&o);}
 assert(t.calibration.ready);
 s.timestamp_us+=20000;terrain_monitor_process(&t,&s,&o);assert(t.result.flags&TERRAIN_TILT_VALID);
 assert(!(t.result.flags&TERRAIN_TURN_VALID));
 s.timestamp_us+=200000;terrain_monitor_process(&t,&s,&o);assert(t.result.flags&TERRAIN_FAULT);
 {hump_detector_t h={0};
  for(i=0;i<10;i++)hump_update(&h,30,0.01f,true,i*20000);
  for(i=10;i<20;i++)hump_update(&h,-30,0.01f,true,i*20000);
  for(i=20;i<25;i++)hump_update(&h,0,0.01f,true,i*20000);
  assert(h.highest_valid && fabsf(h.highest_m-0.05f)<0.00001f && h.completed);
  hump_reset(&h);hump_update(&h,20,0.01f,true,0);hump_update(&h,20,0.01f,false,20000);assert(!h.active && !h.highest_valid);
 }
 t.result=(terrain_result_t){.timestamp_us=123,.pitch_deg=-12.3f,.roll_deg=45.6f,.turn_dps=-30,.peak_m=0.123f,.flags=15,.state=TERRAIN_CLIMBING};
 terrain_pack(&t.result,packed);assert(bits(packed,0,32)==123 && bits(packed,32,3)==TERRAIN_CLIMBING);
 assert(bits(packed,40,11)==(2048-123) && bits(packed,51,11)==456 && bits(packed,62,10)==1024-30 && bits(packed,72,9)==123);
 t.result.peak_m=0.501f;terrain_pack(&t.result,packed);assert(!(bits(packed,35,5)&TERRAIN_PEAK_VALID));
 t.result.pitch_deg=NAN;terrain_pack(&t.result,packed);assert(!(bits(packed,35,5)&TERRAIN_TILT_VALID));
 t.hump.highest_valid=true;t.hump.highest_m=0.1f;terrain_monitor_reset_run(&t);assert(!t.hump.highest_valid && t.calibration.ready);
 { terrain_result_t decoded;
  for(int angle=-900;angle<=900;angle++) {
   t.result=(terrain_result_t){.pitch_deg=angle/10.0f,.roll_deg=-angle/10.0f,
       .flags=TERRAIN_TILT_VALID|TERRAIN_TURN_VALID|TERRAIN_PEAK_VALID,
       .turn_dps=(float)(angle%361),.peak_m=0.5f,.hump_completed=true,.impact=true};
   terrain_pack(&t.result,packed);terrain_unpack(packed,&decoded);
   assert(fabsf(decoded.pitch_deg-t.result.pitch_deg)<0.001f);
   assert(fabsf(decoded.roll_deg-t.result.roll_deg)<0.001f);
   assert(decoded.turn_dps==t.result.turn_dps && decoded.peak_m==0.5f);
   assert(decoded.hump_completed && decoded.impact && (packed[10]&0xf8U)==0);
  }
 }
 puts("Buddy 4 host tests PASS: driver, axes, signed data, saturation, calibration, tilt, gaps, hump, packing, reset");
 return 0;
}

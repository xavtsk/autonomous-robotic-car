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
static float roll_for_height(float height,float pitch,float track)
{ return asinf(height/(track*cosf(pitch*0.01745329252f)))*57.295779513f; }
static void right_samples(hump_detector_t *h,uint32_t *now,float height,float ds,unsigned count)
{
 for(unsigned i=0;i<count;i++) {
  *now+=20000;
  hump_update_right_wheel(h,0,roll_for_height(height,0,0.116f),0.116f,ds,true,*now);
 }
}
typedef struct {
 terrain_monitor_t monitor;
 imu_sample_t sample;
 terrain_odometry_t odometry;
 unsigned completed,max_live_mm;
} terrain_fixture_t;
static void terrain_step(terrain_fixture_t *f,float pitch,float roll,float ds,uint32_t gap)
{
 const float p=pitch*0.01745329252f,r=roll*0.01745329252f;
 uint8_t packet[TERRAIN_PACKET_SIZE];terrain_result_t decoded;
 f->sample.timestamp_us+=gap;
 f->sample.accel_g[0]=sinf(p);f->sample.accel_g[1]=-sinf(r)*cosf(p);
 f->sample.accel_g[2]=cosf(r)*cosf(p);
 f->odometry.distance_m+=ds;f->odometry.stationary=ds==0;
 terrain_monitor_process(&f->monitor,&f->sample,&f->odometry);
 terrain_pack(&f->monitor.result,packet);terrain_unpack(packet,&decoded);
 if(decoded.hump_completed) { ++f->completed; }
 if(decoded.hump_active) {
  unsigned mm=(unsigned)lroundf(decoded.hump_height_m*1000);
  if(mm>f->max_live_mm) { f->max_live_mm=mm; }
 }
}
static void terrain_fixture_init(terrain_fixture_t *f,terrain_hump_geometry_t geometry)
{
 *f=(terrain_fixture_t){0};terrain_monitor_init(&f->monitor);
 assert(f->monitor.hump_geometry==TERRAIN_HUMP_LONGITUDINAL);
 assert(terrain_monitor_set_hump_geometry(&f->monitor,geometry,0.116f));
 f->sample.valid=true;f->odometry.valid=true;f->odometry.epoch=1;
 for(unsigned i=0;i<110;i++) { terrain_step(f,0,0,0,20000); }
 assert(f->monitor.calibration.ready && (f->monitor.result.flags&TERRAIN_TILT_VALID));
}
static void right_wheel_tests(void)
{
 hump_detector_t h={0};uint32_t now=0;
 /* No count without an observed level baseline, or from stationary tilting. */
 right_samples(&h,&now,0.01f,0.001f,20);assert(!h.active && !h.highest_valid);
 right_samples(&h,&now,0,0,5);
 right_samples(&h,&now,0.01f,0,20);assert(!h.active && !h.highest_valid);
 right_samples(&h,&now,0,0,5);
 right_samples(&h,&now,0.004f,0.001f,20);assert(!h.active && !h.highest_valid);
 right_samples(&h,&now,0,0,5);
 /* One 10 mm right-wheel hump. A held plateau is active, not completed. */
 right_samples(&h,&now,0.01f,0.0008f,100);
 assert(h.active && !h.highest_valid && !h.completed && fabsf(h.height_m-0.01f)<0.000001f);
 right_samples(&h,&now,0,0.0008f,4);assert(h.active && !h.completed);
 right_samples(&h,&now,0,0.0008f,1);
 assert(!h.active && h.completed && h.highest_valid && fabsf(h.highest_m-0.01f)<0.000001f);
 right_samples(&h,&now,0,0,20);assert(!h.completed);
 right_samples(&h,&now,0.015f,0.001f,10);right_samples(&h,&now,0,0.001f,5);
 assert(h.completed && fabsf(h.highest_m-0.015f)<0.000001f);
 /* Invalid/reverse/timeout cancels the candidate, preserving prior peaks. */
 right_samples(&h,&now,0.02f,0.001f,10);
 hump_update_right_wheel(&h,0,5,0.116f,-0.001f,true,now+=20000);
 assert(!h.active && !h.armed && fabsf(h.highest_m-0.015f)<0.000001f);
 right_samples(&h,&now,0,0.001f,5);right_samples(&h,&now,0.02f,0.001f,10);
 hump_update_right_wheel(&h,0,5,0.116f,0.001f,false,now+=20000);
 assert(!h.active && !h.armed && h.highest_valid);
 right_samples(&h,&now,0,0.001f,5);right_samples(&h,&now,0.01f,0.001f,10);
 now=h.started_us+10000001U;
 hump_update_right_wheel(&h,0,5,0.116f,0.001f,true,now);
 assert(!h.active && !h.armed);
 right_samples(&h,&now,0.01f,0.001f,20);assert(!h.active); /* no restart on same plateau */
 /* Left-only elevation, pitch-only humps, brief roll spikes and steep roll do not count. */
 hump_reset(&h);right_samples(&h,&now,0,0.001f,5);
 right_samples(&h,&now,-0.01f,0.001f,20);right_samples(&h,&now,0,0.001f,5);
 assert(!h.highest_valid);
 for(unsigned i=0;i<20;i++) { hump_update_right_wheel(&h,15,0,0.116f,0.001f,true,now+=20000); }
 assert(!h.active && !h.highest_valid);
 right_samples(&h,&now,0.01f,0.0008f,1);right_samples(&h,&now,0,0.001f,5);
 assert(!h.highest_valid && !h.completed);
 hump_update_right_wheel(&h,0,10,0.116f,0.001f,true,now+=20000);
 assert(!h.active && !h.armed);
 /* Width and pitch affect the calculation; the result is not fixed at 1 cm. */
 for(unsigned variant=0;variant<3;variant++) {
  const float track=variant==0?0.116f:0.2f,pitch=variant==2?20:0;
  const float roll=roll_for_height(0.012f,pitch,track);
  hump_reset(&h);
  for(unsigned i=0;i<5;i++) { hump_update_right_wheel(&h,0,0,track,0.001f,true,now+=20000); }
  for(unsigned i=0;i<10;i++) { hump_update_right_wheel(&h,pitch,roll,track,0.001f,true,now+=20000); }
  assert(h.active && fabsf(h.height_m-0.012f)<0.000001f);
 }
 {
  terrain_fixture_t f;
  const float roll=roll_for_height(0.01f,0,0.116f);
  terrain_fixture_init(&f,TERRAIN_HUMP_RIGHT_WHEEL);
  for(unsigned pass=0;pass<2;pass++) {
   for(unsigned i=0;i<40;i++) { terrain_step(&f,0,roll,0.0008f,20000); }
   assert(f.monitor.result.hump_active && f.completed==pass);
   for(unsigned i=0;i<25;i++) { terrain_step(&f,0,0,0.0008f,20000); }
   assert(f.completed==pass+1 && (f.monitor.result.flags&TERRAIN_PEAK_VALID));
  }
  assert(f.max_live_mm==10 && fabsf(f.monitor.result.peak_m-0.01f)<0.0001f);
  /* Raw invalidity and real sample gaps must disarm immediately. */
  for(unsigned i=0;i<20;i++) { terrain_step(&f,0,roll,0.0008f,20000); }
  f.sample.valid=false;terrain_step(&f,0,roll,0.0008f,20000);f.sample.valid=true;
  assert(!f.monitor.hump.active && !f.monitor.hump.armed && f.monitor.hump.highest_valid);
  for(unsigned i=0;i<20;i++) { terrain_step(&f,0,roll,0.0008f,20000); }
  assert(!f.monitor.hump.active && f.completed==2);
  for(unsigned i=0;i<25;i++) { terrain_step(&f,0,0,0.0008f,20000); }
  for(unsigned i=0;i<20;i++) { terrain_step(&f,0,roll,0.0008f,20000); }
  terrain_step(&f,0,roll,0.0008f,120000);
  assert(!f.monitor.hump.active && !f.monitor.hump.armed && f.completed==2);
  terrain_monitor_reset_run(&f.monitor);
  assert(f.monitor.hump_geometry==TERRAIN_HUMP_RIGHT_WHEEL && f.monitor.wheel_track_m==0.116f);
  assert(f.monitor.calibration.ready && !f.monitor.hump.highest_valid);
  assert(!terrain_monitor_set_hump_geometry(&f.monitor,TERRAIN_HUMP_RIGHT_WHEEL,0));
  assert(!terrain_monitor_set_hump_geometry(&f.monitor,(terrain_hump_geometry_t)99,0.116f));
 }
 {
  terrain_fixture_t f;const float roll=roll_for_height(0.01f,0,0.116f);
  terrain_fixture_init(&f,TERRAIN_HUMP_RIGHT_WHEEL);
  for(unsigned i=0;i<40;i++) { terrain_step(&f,0,roll,0,20000); }
  for(unsigned i=0;i<25;i++) { terrain_step(&f,0,0,0,20000); }
  assert(f.completed==0 && !f.monitor.hump.highest_valid); /* hand tilt, no wheel distance */
  for(unsigned i=0;i<40;i++) { terrain_step(&f,0,roll,0.0008f,20000); }
  assert(f.monitor.hump.active);
  f.odometry.valid=false;terrain_step(&f,0,roll,0.0008f,20000);f.odometry.valid=true;
  assert(!f.monitor.hump.active && !f.monitor.hump.armed);
  for(unsigned i=0;i<25;i++) { terrain_step(&f,0,0,0.0008f,20000); }
  for(unsigned i=0;i<40;i++) { terrain_step(&f,0,roll,0.0008f,20000); }
  assert(f.monitor.hump.active);
  /* The low-pass angle is still plausible, but the abrupt lateral change
   * must cancel the estimate rather than create an artificial taller peak. */
  terrain_step(&f,0,20,0.0008f,20000);
  assert((f.monitor.result.flags&TERRAIN_TILT_VALID) && fabsf(f.monitor.result.roll_deg)<10);
  assert(!f.monitor.hump.active && !f.monitor.hump.armed && f.completed==0);
 }
 {
  terrain_fixture_t legacy,right;
  terrain_fixture_init(&legacy,TERRAIN_HUMP_LONGITUDINAL);
  terrain_fixture_init(&right,TERRAIN_HUMP_RIGHT_WHEEL);
  /* A smooth both-wheel pitch hump still belongs to the original profile. */
  for(unsigned segment=0;segment<6;segment++) {
   for(unsigned i=0;i<20;i++) {
    float pitch=segment==0?(float)(i+1):segment==1?20:segment==2?(float)(19-i):
        segment==3?-(float)(i+1):segment==4?-20:-(float)(19-i);
    terrain_step(&legacy,pitch,0,0.001f,20000);terrain_step(&right,pitch,0,0.001f,20000);
   }
  }
  for(unsigned i=0;i<25;i++) { terrain_step(&legacy,0,0,0.001f,20000);terrain_step(&right,0,0,0.001f,20000); }
  assert(legacy.completed==1 && legacy.monitor.hump.highest_valid);
  assert(right.completed==0 && !right.monitor.hump.highest_valid);
 }
}
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
 /* Right side up = positive roll: the vehicle-left axis then points down. */
 {float a[3]={0,-0.5f,0.8660254f};assert(tilt_estimation(a,&pitch,&roll));assert(fabsf(roll-30)<0.01f && fabsf(pitch)<0.01f);}
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
   assert(decoded.hump_completed && decoded.impact && !decoded.hump_active && (packed[11]&0xf8U)==0);
  }
  /* Live climb height: 1 + 9 bits after the impact bit, 0..500 mm, zero when no hump. */
  for(int mm=0;mm<=500;mm++) {
   t.result=(terrain_result_t){.hump_active=true,.hump_height_m=mm/1000.0f};
   terrain_pack(&t.result,packed);terrain_unpack(packed,&decoded);
   assert(bits(packed,83,1)==1 && bits(packed,84,9)==(unsigned)mm && (packed[11]&0xe0U)==0);
   assert(decoded.hump_active && fabsf(decoded.hump_height_m-mm/1000.0f)<0.0001f);
  }
  t.result=(terrain_result_t){.hump_active=true,.hump_height_m=0.501f};
  terrain_pack(&t.result,packed);assert(bits(packed,83,10)==0);
  t.result=(terrain_result_t){.hump_active=false,.hump_height_m=0.2f};
  terrain_pack(&t.result,packed);assert(bits(packed,83,10)==0);
 }
 /* The monitor reports the live climb while crossing a hump. */
 {hump_detector_t h={0};
  for(i=0;i<10;i++)hump_update(&h,30,0.01f,true,i*20000);
  assert(h.active && h.height_m>0.04f);
 }
 right_wheel_tests();
 puts("Buddy 4 host tests PASS: driver, calibration, tilt, longitudinal/right-wheel hump geometry, rejection/rearm, packing, repeat/reset and live height");
 return 0;
}

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ir_acquisition.h"
#include "ir_calibration.h"
#include "line_controller.h"
#include "barcode_decoder.h"
#include "line_navigation.h"
#include "barcode_fixtures.h"

typedef struct { unsigned int calls; uint32_t now; uint8_t fail; } mock_adc_t;
static int32_t mock_read(void *context, uint8_t channel, uint16_t *raw)
{
    mock_adc_t *mock = context;
    mock->calls++;
    *raw = (uint16_t)(100U + channel);
    return (mock->fail == channel) ? -9 : 0;
}
static uint32_t mock_clock(void *context)
{
    mock_adc_t *mock = context;
    mock->now += 10U;
    return mock->now;
}
static void test_acquisition(void)
{
    s3_acquisition_config_t cfg = {{0U, 1U, 2U}, 1U};
    s3_sensor_frame_t frame;
    mock_adc_t mock = {0U, 0U, 255U};
    assert(s3_acquisition_capture(&cfg, mock_read, mock_clock, &mock, &frame));
    assert(mock.calls == 1U && frame.sensor[S3_SENSOR_BARCODE].raw == 100U);
    assert(!frame.sensor[S3_SENSOR_LEFT].enabled && !frame.sensor[S3_SENSOR_LEFT].valid);
    cfg.enabled_mask = 7U;
    cfg.channel[S3_SENSOR_LEFT] = 2U;
    cfg.channel[S3_SENSOR_RIGHT] = 1U;
    mock.fail = 2U;
    assert(s3_acquisition_capture(&cfg, mock_read, mock_clock, &mock, &frame));
    assert(frame.sensor[S3_SENSOR_BARCODE].valid);
    assert(!frame.sensor[S3_SENSOR_LEFT].valid && frame.sensor[S3_SENSOR_LEFT].error == -9);
    assert(frame.sensor[S3_SENSOR_LEFT].raw == 0U);
    assert(frame.sensor[S3_SENSOR_RIGHT].raw == 101U);
    assert(frame.sensor[S3_SENSOR_BARCODE].timestamp_us < frame.sensor[S3_SENSOR_RIGHT].timestamp_us);
    cfg.channel[1] = cfg.channel[0];
    assert(!s3_acquisition_config_valid(&cfg));
    cfg.channel[1] = 3U;
    assert(!s3_acquisition_config_valid(&cfg));
    cfg.channel[1] = 2U;
    cfg.enabled_mask = 8U;
    assert(!s3_acquisition_config_valid(&cfg));
    cfg.enabled_mask = 0U;
    assert(!s3_acquisition_capture(&cfg, mock_read, mock_clock, &mock, &frame));
    assert(!frame.sensor[0].valid);
}
static void test_calibration(void)
{
    s3_calibration_collector_t collector;
    ir_calibration_t active = {150U, 280U, 400U, 600U};
    ir_reflectance_t left, right;
    uint16_t dark;
    unsigned int i;
    s3_calibration_begin(&collector);
    assert(!s3_calibration_finish(&collector, &active));
    assert(!s3_calibration_summary(&collector.white).available);
    assert(!s3_calibration_add(&collector, IR_SURFACE_UNKNOWN, 10U));
    assert(!s3_calibration_add(&collector, IR_SURFACE_WHITE, 4096U));
    for (i = 0U; i < 10U; i++) {
        assert(s3_calibration_add(&collector, IR_SURFACE_WHITE, (uint16_t)(3000U+i)));
        assert(s3_calibration_add(&collector, IR_SURFACE_BLACK, (uint16_t)(200U+i)));
    }
    assert(s3_calibration_finish(&collector, &active));
    assert(active.white_raw == 3004U && active.black_raw == 204U);
    assert(collector.white.minimum == 3000U && collector.white.maximum == 3009U);
    assert(s3_calibration_summary(&collector.white).average == 3004.5f);
    assert(ir_reflectance_init(&left, &active));
    active = (ir_calibration_t){150U, 280U, 400U, 600U};
    assert(ir_reflectance_init(&right, &active));
    assert(ir_reflectance_update(&left, 3004U, &dark) && dark == 0U);
    assert(ir_reflectance_update(&right, 280U, &dark) && dark == 1000U);
    assert(left.surface == IR_SURFACE_WHITE && right.surface == IR_SURFACE_BLACK);
    s3_calibration_begin(&collector);
    for (i = 0U; i < 10U; i++) {
        assert(s3_calibration_add(&collector, IR_SURFACE_WHITE, 250U));
        assert(s3_calibration_add(&collector, IR_SURFACE_BLACK, 250U));
    }
    assert(!s3_calibration_finish(&collector, &active));
    assert(active.white_raw == 150U && active.black_raw == 280U);
    assert(s3_calibration_add(&collector, IR_SURFACE_WHITE, 300U));
    assert(!s3_calibration_finish(&collector, &active));
    collector.white.count = 65535U;
    assert(!s3_calibration_add(&collector, IR_SURFACE_WHITE, 300U));
}

static void test_line(void)
{
    s3_line_detector_t detector;
    s3_line_config_t cfg = s3_line_defaults();
    s3_steering_config_t steering = s3_steering_defaults();
    s3_line_observation_t line;
    s3_steering_request_t request;
    uint32_t t = 0U;
    unsigned int i, events;
    assert(s3_line_init(&detector, &cfg));
    for (i=0; i<4U; i++, t+=10000U) {
        line = s3_line_update(&detector,t,true,400U,400U);
        assert(line.reacquired_event == (i==3U));
    }
    assert(line.valid && line.present && line.error==0);
    request=s3_steering_compute(&steering,&line);
    assert(request.valid && request.left_mps==request.right_mps);
    line=s3_line_update(&detector,t,true,1000U,0U); t+=10000U;
    assert(line.error==-100);
    request=s3_steering_compute(&steering,&line);
    assert(request.valid && request.left_mps<request.right_mps);
    line=s3_line_update(&detector,t,true,0U,1000U); t+=10000U;
    request=s3_steering_compute(&steering,&line);
    assert(line.error==100 && request.left_mps>request.right_mps);
    steering.kp_mps=1.0f;
    request=s3_steering_compute(&steering,&line);
    assert(request.left_mps==steering.maximum_mps && request.right_mps==0.0f);
    events=0U;
    for(i=0;i<20U;i++,t+=10000U) {
        line=s3_line_update(&detector,t,true,0U,0U);
        if(line.lost_event) { events++; }
        assert(!s3_steering_compute(&steering,&line).valid);
    }
    assert(events==1U);
    for(i=0;i<4U;i++,t+=10000U) { line=s3_line_update(&detector,t,true,400U,400U); }
    assert(line.reacquired_event);
    /* A one-sample black spike does not emit a junction. */
    line=s3_line_update(&detector,t,true,1000U,1000U);t+=10000U;
    assert(!line.junction_event && !s3_steering_compute(&steering,&line).valid);
    line=s3_line_update(&detector,t,true,400U,400U);t+=10000U;
    events=0U;
    for(i=0;i<20U;i++,t+=10000U) {
        line=s3_line_update(&detector,t,true,1000U,1000U);
        if(line.junction_event) { events++; }
    }
    assert(events==1U);
    for(i=0;i<12U;i++,t+=10000U) { line=s3_line_update(&detector,t,true,400U,400U); }
    assert(!line.junction_candidate);
    for(i=0;i<4U;i++,t+=10000U) { line=s3_line_update(&detector,t,true,1000U,1000U); }
    assert(line.junction_event);
    line=s3_line_update(&detector,t,false,400U,400U);t+=10000U;
    assert(!line.valid && !s3_steering_compute(&steering,&line).valid);
    line=s3_line_update(&detector,t,true,400U,400U);
    line=s3_line_update(&detector,t+100000U,true,400U,400U);
    assert(!line.valid);
    /* Unsigned microsecond wrap remains continuous. */
    s3_line_reset(&detector);t=0xFFFFC000U;
    for(i=0;i<4U;i++,t+=10000U) { line=s3_line_update(&detector,t,true,400U,400U); }
    assert(line.valid && line.reacquired_event);
    steering.kp_mps=NAN;
    assert(!s3_steering_compute(&steering,&line).valid);
}

static const uint16_t test_letters[26]={
  0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,
  0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
  0x181,0x0C1,0x1C0,0x091,0x190,0x0D0};
static void make_runs(char letter,uint32_t *runs)
{
    unsigned int c,i;
    uint16_t patterns[3]={0x094,test_letters[(unsigned int)(letter-'A')],0x094};
    for(c=0;c<3U;c++) {
        for(i=0;i<9U;i++) { runs[c*10U+i]=(patterns[c]&(1U<<(8U-i)))?30000U:10000U; }
        if(c<2U) { runs[c*10U+9U]=10000U; }
    }
}
static unsigned int feed_duration(s3_barcode_decoder_t *d,uint32_t *t,bool black,
    uint32_t duration,s3_barcode_result_t *last)
{
    unsigned int count=0U;
    uint32_t elapsed;
    for(elapsed=0U;elapsed<duration;elapsed+=1000U) {
        s3_barcode_result_t result;
        s3_barcode_status_t status=s3_barcode_update(d,*t,true,black,&result);
        if(status==S3_BARCODE_DECODED) { count++;*last=result; }
        *t+=1000U;
    }
    return count;
}
static void test_barcode(void)
{
    s3_barcode_config_t cfg={3000U,3000U,1000000U,10000000U,10U};
    s3_barcode_result_t result={0};
    s3_barcode_decoder_t d;
    uint32_t runs[29],reverse[29];
    unsigned int c,i,k,count;
    const uint32_t *fixtures[4]={g_pdf_page_1,g_pdf_page_2,g_pdf_page_3,g_pdf_page_4};
    uint32_t t=0U;
    for(c=0U;c<26U;c++) {
        make_runs((char)('A'+c),runs);
        for(i=0;i<29U;i++) { reverse[i]=runs[28U-i]; }
        assert(s3_barcode_decode_runs(runs,29U,150000U,150000U,1000U,&cfg,&result));
        assert(result.letter==(char)('A'+c) && !result.reversed);
        assert(s3_barcode_decode_runs(reverse,29U,150000U,150000U,1000U,&cfg,&result));
        assert(result.letter==(char)('A'+c) && result.reversed);
        /* Exercise every symbol through timestamped transitions in both directions. */
        for (k=0U;k<2U;k++) {
            const uint32_t *trace = k ? reverse : runs;
            assert(s3_barcode_init(&d,&cfg));t=0xFFFF0000U;count=0U;
            count+=feed_duration(&d,&t,false,150000U,&result);
            for(i=0U;i<29U;i++) {
                count+=feed_duration(&d,&t,(i%2U)==0U,trace[i],&result);
            }
            count+=feed_duration(&d,&t,false,250000U,&result);
            assert(count==1U && result.letter==(char)('A'+c) && result.reversed==(k!=0U));
        }
        /* Scaled, jittered elements with locally consistent character speeds. */
        for(i=0U;i<29U;i++) {
            unsigned int character_index=i/10U;
            runs[i]=runs[i]*(80U+character_index*20U)/100U;
            runs[i]=runs[i]*((i%2U)?110U:90U)/100U;
        }
        assert(s3_barcode_decode_runs(runs,29U,200000U,200000U,1000U,&cfg,&result));
        assert(result.letter==(char)('A'+c));
    }
    for(c=0U;c<4U;c++) {
        for(i=0U;i<29U;i++) { runs[i]=fixtures[c][i]*500U;reverse[28U-i]=runs[i]; }
        assert(s3_barcode_decode_runs(runs,29U,600000U,600000U,1000U,&cfg,&result));
        assert(result.letter==((c<2U)?'A':'Z'));
        assert(s3_barcode_decode_runs(reverse,29U,600000U,600000U,1000U,&cfg,&result));
        assert(result.reversed && result.letter==((c<2U)?'A':'Z'));
        assert(!s3_barcode_decode_runs(runs,29U,1000U,600000U,1000U,&cfg,&result));
    }
    make_runs('A',runs);
    assert(!s3_barcode_decode_runs(runs,28U,150000U,150000U,1000U,&cfg,&result));
    assert(!s3_barcode_decode_runs(runs,30U,150000U,150000U,1000U,&cfg,&result));
    assert(!s3_barcode_decode_runs(runs,29U,150000U,100U,1000U,&cfg,&result));
    assert(!s3_barcode_decode_runs(runs,29U,150000U,150000U,10000U,&cfg,&result));
    runs[0]=30000U;
    assert(!s3_barcode_decode_runs(runs,29U,150000U,150000U,1000U,&cfg,&result));
    make_runs('A',runs);runs[20]=30000U; /* Invalid stop marker. */
    assert(!s3_barcode_decode_runs(runs,29U,150000U,150000U,1000U,&cfg,&result));
    make_runs('A',runs);runs[9]=90000U;
    assert(!s3_barcode_decode_runs(runs,29U,150000U,150000U,1000U,&cfg,&result));
    /* Sampled trace, duplicate suppression, then a fresh frame. */
    assert(s3_barcode_init(&d,&cfg));make_runs('Z',runs);count=0U;
    count+=feed_duration(&d,&t,false,150000U,&result);
    for(k=0U;k<2U;k++) {
        for(i=0U;i<29U;i++) { count+=feed_duration(&d,&t,(i%2U)==0U,runs[i],&result); }
        count+=feed_duration(&d,&t,false,250000U,&result);
        assert(result.letter=='Z' && count==k+1U);
    }
    /* A broken scan is reset; a full new quiet zone/frame can recover. */
    s3_barcode_reset(&d);t=0U;
    feed_duration(&d,&t,false,150000U,&result);
    feed_duration(&d,&t,true,10000U,&result);
    assert(s3_barcode_update(&d,t+100000U,true,false,&result)==S3_BARCODE_TIMING_ERROR);
    t+=101000U;count=feed_duration(&d,&t,false,150000U,&result);
    make_runs('A',runs);
    for(i=0;i<29U;i++) { count+=feed_duration(&d,&t,(i%2U)==0U,runs[i],&result); }
    count+=feed_duration(&d,&t,false,150000U,&result);
    assert(count==1U && result.letter=='A');
    assert(s3_barcode_update(&d,t,false,false,&result)==S3_BARCODE_REJECTED);
    /* A short trailing quiet zone cannot complete a frame. */
    s3_barcode_reset(&d);t=0U;count=feed_duration(&d,&t,false,150000U,&result);
    for(i=0U;i<29U;i++) { count+=feed_duration(&d,&t,(i%2U)==0U,runs[i],&result); }
    count+=feed_duration(&d,&t,false,50000U,&result);
    assert(count==0U && s3_barcode_update(&d,t,true,true,&result)==S3_BARCODE_REJECTED);
    /* Resolution gate also rejects widths with fewer than three samples. */
    assert(s3_barcode_decode_runs(runs,29U,150000U,150000U,2900U,&cfg,&result));
    for(i=0U;i<29U;i++) { runs[i]/=2U; }
    assert(!s3_barcode_decode_runs(runs,29U,150000U,150000U,2900U,&cfg,&result));
    /* Defensive overflow guard, including corrupted producer state. */
    s3_barcode_reset(&d);t=0U;
    feed_duration(&d,&t,false,150000U,&result);
    feed_duration(&d,&t,true,10000U,&result);
    d.count=S3_BARCODE_RUNS;
    assert(s3_barcode_update(&d,t,true,false,&result)==S3_BARCODE_OVERFLOW);
    /* Truncated frame timeout, not a fabricated character. */
    s3_barcode_reset(&d);t=0U;
    feed_duration(&d,&t,false,150000U,&result);
    feed_duration(&d,&t,true,10000U,&result);
    assert(feed_duration(&d,&t,false,1100000U,&result)==0U);
    assert(!d.collecting);
}

typedef struct {
    unsigned int barcodes,junctions,lost,acquired,faults;
    char letter;
    s3_nav_action_t action;
} simulated_consumer_t;
static void consume(simulated_consumer_t *consumer,const s3_navigation_output_t *out)
{
    if(out->events&S3_EVENT_BARCODE) { consumer->barcodes++;consumer->letter=out->barcode.letter;consumer->action=out->action; }
    if(out->events&S3_EVENT_JUNCTION) { consumer->junctions++; }
    if(out->events&S3_EVENT_LINE_LOST) { consumer->lost++; }
    if(out->events&S3_EVENT_LINE_REACQUIRED) { consumer->acquired++; }
    if(out->events&S3_EVENT_SENSOR_FAULT) { consumer->faults++; }
}
static s3_sensor_frame_t make_frame(uint32_t t,uint16_t barcode,uint16_t left,uint16_t right)
{
    s3_sensor_frame_t frame={0};
    unsigned int i;
    frame.timestamp_us=t;
    for(i=0U;i<S3_SENSOR_COUNT;i++) { frame.sensor[i]=(s3_raw_sample_t){t,0U,0,true,true}; }
    frame.sensor[0].raw=barcode;frame.sensor[1].raw=left;frame.sensor[2].raw=right;
    return frame;
}
static s3_navigation_output_t nav_feed(s3_navigation_t *nav,simulated_consumer_t *consumer,
    uint32_t *t,uint32_t duration,uint16_t barcode,uint16_t left,uint16_t right)
{
    uint32_t elapsed;
    s3_navigation_output_t out={0};
    for(elapsed=0U;elapsed<duration;elapsed+=1000U) {
        s3_sensor_frame_t frame=make_frame(*t,barcode,left,right);
        assert(s3_navigation_process(nav,&frame,&out));
        consume(consumer,&out);*t+=1000U;
    }
    return out;
}
static void nav_barcode(s3_navigation_t *nav,simulated_consumer_t *consumer,uint32_t *t,char letter)
{
    uint32_t runs[29];unsigned int i;
    make_runs(letter,runs);
    nav_feed(nav,consumer,t,150000U,150U,202U,202U);
    for(i=0U;i<29U;i++) { nav_feed(nav,consumer,t,runs[i],(i%2U)?150U:280U,202U,202U); }
    nav_feed(nav,consumer,t,150000U,150U,202U,202U);
}
static void test_navigation(void)
{
    s3_navigation_config_t cfg=s3_navigation_defaults();
    s3_navigation_t nav;
    s3_navigation_output_t out;
    s3_sensor_frame_t frame;
    simulated_consumer_t consumer={0};
    uint32_t t=0U;
    s3_calibration_collector_t collector;
    unsigned int i;
    assert(s3_navigation_init(&nav,&cfg));
    out=nav_feed(&nav,&consumer,&t,50000U,150U,280U,280U);
    assert(out.sensor[0].valid && !out.sensor[1].valid && out.fault_mask==0U);
    assert(!out.steering.valid && consumer.junctions==0U);
    cfg.acquisition.enabled_mask=7U;
    cfg.barcode.min_narrow_us=3000U;
    assert(s3_navigation_init(&nav,&cfg));consumer=(simulated_consumer_t){0};t=0U;
    out=nav_feed(&nav,&consumer,&t,50000U,150U,202U,202U);
    assert(out.steering.valid && consumer.acquired==1U);
    nav_barcode(&nav,&consumer,&t,'A');
    assert(consumer.barcodes==1U && consumer.letter=='A' && consumer.action==S3_NAV_LEFT);
    assert(consumer.junctions==0U); /* Barcode does not execute/declare a junction. */
    out=nav_feed(&nav,&consumer,&t,50000U,150U,280U,280U);
    assert(consumer.junctions==1U && !out.steering.valid);
    nav_feed(&nav,&consumer,&t,150000U,150U,202U,202U);
    out=nav_feed(&nav,&consumer,&t,120000U,150U,150U,150U);
    assert(consumer.lost==1U && !out.steering.valid);
    out=nav_feed(&nav,&consumer,&t,50000U,150U,202U,202U);
    assert(consumer.acquired==2U && out.steering.valid);
    frame=make_frame(t,150U,202U,202U);frame.sensor[1].valid=false;t+=1000U;
    assert(s3_navigation_process(&nav,&frame,&out));consume(&consumer,&out);
    assert(consumer.faults==1U && !out.steering.valid);
    frame.timestamp_us=t;for(i=0U;i<3U;i++) { frame.sensor[i].timestamp_us=t; }t+=1000U;
    assert(s3_navigation_process(&nav,&frame,&out));consume(&consumer,&out);
    assert(consumer.faults==1U);
    out=nav_feed(&nav,&consumer,&t,50000U,150U,202U,202U);
    assert(out.steering.valid && out.fault_mask==0U);
    frame=make_frame(t,150U,202U,202U);frame.sensor[2].timestamp_us=t-100000U;t+=1000U;
    assert(s3_navigation_process(&nav,&frame,&out));assert(out.fault_mask&S3_SENSOR_BIT(S3_SENSOR_RIGHT));
    /* Unmapped results remain visible; reconfiguration replaces the mapping. */
    assert(s3_navigation_init(&nav,&cfg));consumer=(simulated_consumer_t){0};
    nav_barcode(&nav,&consumer,&t,'Z');assert(consumer.letter=='Z' && consumer.action==S3_NAV_NONE);
    cfg.map[25]=S3_NAV_UTURN;
    assert(s3_navigation_init(&nav,&cfg));consumer=(simulated_consumer_t){0};
    nav_barcode(&nav,&consumer,&t,'Z');assert(consumer.action==S3_NAV_UTURN);
    assert(s3_navigation_set_mode(&nav,S3_MODE_SUPPRESSED));consumer=(simulated_consumer_t){0};
    nav_barcode(&nav,&consumer,&t,'A');
    out=nav_feed(&nav,&consumer,&t,50000U,150U,280U,280U);
    assert(out.sensor[0].valid && consumer.barcodes==0U && consumer.junctions==0U && !out.steering.valid);
    assert(s3_navigation_set_mode(&nav,S3_MODE_TRACK));
    nav_feed(&nav,&consumer,&t,50000U,150U,202U,202U);
    assert(consumer.barcodes==0U); /* Suppressed barcode is not replayed. */
    s3_calibration_begin(&collector);
    assert(!s3_navigation_apply_calibration(&nav,S3_SENSOR_LEFT,&collector));
    assert(nav.config.calibration[1].white_raw==150U);
    for(i=0U;i<10U;i++) {
        assert(s3_calibration_add(&collector,IR_SURFACE_WHITE,100U));
        assert(s3_calibration_add(&collector,IR_SURFACE_BLACK,3000U));
    }
    assert(s3_navigation_apply_calibration(&nav,S3_SENSOR_LEFT,&collector));
    assert(nav.config.calibration[1].black_raw==3000U && nav.config.calibration[0].black_raw==280U);
    cfg.map[0]=S3_NAV_ACTION_COUNT;assert(!s3_navigation_init(&nav,&cfg));
    assert(!s3_navigation_process(&nav,&frame,&out));
}
int main(void)
{
    test_acquisition();
    test_calibration();
    test_line();
    test_barcode();
    test_navigation();
    puts("PASS: acquisition, calibration, line detection, steering, Code 39 and simulated navigation");
    return 0;
}

#include "line_navigation.h"
s3_navigation_config_t s3_navigation_defaults(void)
{
    s3_navigation_config_t cfg={0};
    unsigned int i;
    cfg.acquisition=(s3_acquisition_config_t){{0U,1U,2U},1U};
    for(i=0U;i<S3_SENSOR_COUNT;i++) { cfg.calibration[i]=(ir_calibration_t){150U,280U,400U,600U}; }
    cfg.line=s3_line_defaults();cfg.steering=s3_steering_defaults();cfg.barcode=s3_barcode_defaults();
    cfg.map[0]=S3_NAV_LEFT;cfg.map[1]=S3_NAV_RIGHT;
    cfg.map[2]=S3_NAV_STRAIGHT;cfg.map[3]=S3_NAV_UTURN;
    cfg.max_sample_age_us=30000U;
    return cfg;
}
bool s3_navigation_init(s3_navigation_t *context, const s3_navigation_config_t *config)
{
    unsigned int i;
    if(context==0) { return false; }
    *context=(s3_navigation_t){0};
    if((config==0)||!s3_acquisition_config_valid(&config->acquisition)||
        !s3_steering_config_valid(&config->steering)||
        !s3_line_init(&context->line,&config->line)||
        !s3_barcode_init(&context->barcode,&config->barcode)||
        (config->max_sample_age_us==0U)||(config->max_sample_age_us>=0x80000000U)) { return false; }
    for(i=0U;i<S3_SENSOR_COUNT;i++) {
        if(!ir_reflectance_init(&context->sensor[i],&config->calibration[i])) { return false; }
    }
    for(i=0U;i<26U;i++) {
        if((unsigned int)config->map[i]>=S3_NAV_ACTION_COUNT) { return false; }
    }
    context->config=*config;context->mode=S3_MODE_TRACK;context->configured=true;
    return true;
}
bool s3_navigation_set_mode(s3_navigation_t *context, s3_processing_mode_t mode)
{
    if((context==0)||!context->configured||
        ((mode!=S3_MODE_TRACK)&&(mode!=S3_MODE_SUPPRESSED))) { return false; }
    if(context->mode!=mode) {
        s3_line_reset(&context->line);s3_barcode_reset(&context->barcode);
        context->mode=mode;
    }
    return true;
}
bool s3_navigation_apply_calibration(s3_navigation_t *context, s3_sensor_id_t id,
                                     const s3_calibration_collector_t *collector)
{
    ir_calibration_t candidate;
    if((context==0)||!context->configured||((unsigned int)id>=S3_SENSOR_COUNT)) { return false; }
    candidate=context->config.calibration[id];
    if(!s3_calibration_finish(collector,&candidate)) { return false; }
    context->config.calibration[id]=candidate;
    (void)ir_reflectance_init(&context->sensor[id],&candidate);
    s3_line_reset(&context->line);s3_barcode_reset(&context->barcode);
    return true;
}
bool s3_navigation_process(s3_navigation_t *context, const s3_sensor_frame_t *frame,
                            s3_navigation_output_t *output)
{
    unsigned int i;
    bool frame_time_bad, line_valid;
    if(output==0) { return false; }
    *output=(s3_navigation_output_t){0};
    if((context==0)||!context->configured||(frame==0)) { return false; }
    output->timestamp_us=frame->timestamp_us;
    output->sample_gap_us=context->seen_frame?(frame->timestamp_us-context->last_us):0U;
    frame_time_bad=context->seen_frame&&((output->sample_gap_us==0U)||
        (output->sample_gap_us>context->config.max_sample_age_us));
    context->last_us=frame->timestamp_us;context->seen_frame=true;
    for(i=0U;i<S3_SENSOR_COUNT;i++) {
        const s3_raw_sample_t *raw=&frame->sensor[i];
        uint8_t bit=(uint8_t)S3_SENSOR_BIT(i);
        bool enabled=(context->config.acquisition.enabled_mask&bit)!=0U;
        bool timing_bad=((uint32_t)(frame->timestamp_us-raw->timestamp_us)>context->config.max_sample_age_us);
        if((context->seen_sensors&bit)!=0U) {
            uint32_t gap=raw->timestamp_us-context->sensor_last_us[i];
            timing_bad=timing_bad||(gap==0U)||(gap>context->config.max_sample_age_us);
        }
        if(enabled&&raw->enabled&&raw->valid&&(raw->error==0)&&!timing_bad&&!frame_time_bad&&
            ir_reflectance_update(&context->sensor[i],raw->raw,&output->sensor[i].darkness)) {
            output->sensor[i].valid=true;
            output->sensor[i].surface=context->sensor[i].surface;
            context->seen_sensors|=bit;
            context->sensor_last_us[i]=raw->timestamp_us;
        } else {
            ir_reflectance_reset(&context->sensor[i]);
            context->seen_sensors&=(uint8_t)~bit;
            if(enabled) { output->fault_mask|=bit; }
        }
    }
    if((output->fault_mask & (uint8_t)~context->faults)!=0U) { output->events|=S3_EVENT_SENSOR_FAULT; }
    context->faults=output->fault_mask;
    line_valid=output->sensor[S3_SENSOR_LEFT].valid&&output->sensor[S3_SENSOR_RIGHT].valid;
    output->line=s3_line_update(&context->line,frame->timestamp_us,line_valid,
        output->sensor[S3_SENSOR_LEFT].darkness,output->sensor[S3_SENSOR_RIGHT].darkness);
    if((context->config.acquisition.enabled_mask&S3_SENSOR_BIT(S3_SENSOR_BARCODE))!=0U) {
        const s3_processed_sensor_t *barcode=&output->sensor[S3_SENSOR_BARCODE];
        output->barcode_status=s3_barcode_update(&context->barcode,
            frame->sensor[S3_SENSOR_BARCODE].timestamp_us,
            barcode->valid&&(barcode->surface!=IR_SURFACE_UNKNOWN),
            barcode->surface==IR_SURFACE_BLACK,&output->barcode);
    }
    if(context->mode==S3_MODE_TRACK) {
        if(output->fault_mask==0U) { output->steering=s3_steering_compute(&context->config.steering,&output->line); }
        if(output->line.lost_event) { output->events|=S3_EVENT_LINE_LOST; }
        if(output->line.reacquired_event) { output->events|=S3_EVENT_LINE_REACQUIRED; }
        if(output->line.junction_event) { output->events|=S3_EVENT_JUNCTION; }
        if(output->barcode_status==S3_BARCODE_DECODED) {
            output->events|=S3_EVENT_BARCODE;
            output->action=context->config.map[(unsigned int)(output->barcode.letter-'A')];
        }
    } else {
        output->line.lost_event=false;output->line.reacquired_event=false;output->line.junction_event=false;
    }
    return true;
}
const char *s3_surface_name(ir_surface_t surface)
{
    switch(surface) { case IR_SURFACE_WHITE:return "WHITE";case IR_SURFACE_BLACK:return "BLACK";default:return "UNKNOWN"; }
}
const char *s3_action_name(s3_nav_action_t action)
{
    switch(action) {
    case S3_NAV_LEFT:return "LEFT";case S3_NAV_RIGHT:return "RIGHT";
    case S3_NAV_STRAIGHT:return "STRAIGHT";case S3_NAV_UTURN:return "U-TURN";
    default:return "NONE";
    }
}

#ifndef BUDDY3_IR_SENSOR_H
#define BUDDY3_IR_SENSOR_H
#include "ir_acquisition.h"
/* Core-0 ADC owner only. Close/restart to reconfigure. */
int32_t ir_sensor_init(const s3_acquisition_config_t *config);
int32_t ir_sensor_read_frame(s3_sensor_frame_t *frame);
void ir_sensor_close(void);
#endif

/* Buddy 3: hardware-independent, per-sensor reflectance classification. */
#ifndef BUDDY3_IR_REFLECTANCE_H
#define BUDDY3_IR_REFLECTANCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IR_SURFACE_UNKNOWN = 0,
    IR_SURFACE_WHITE,
    IR_SURFACE_BLACK
} ir_surface_t;

typedef struct
{
    uint16_t white_raw;
    uint16_t black_raw;
    uint16_t white_limit; /* Normalised 0..1000: enter WHITE at/below this. */
    uint16_t black_limit; /* Enter BLACK at/above this; must exceed white_limit. */
} ir_calibration_t;

typedef struct
{
    ir_calibration_t calibration;
    ir_surface_t surface;
    bool configured;
} ir_reflectance_t;

/* One state object per sensor, owned by the sampling task. Either ADC polarity
 * is supported. Invalid calibration leaves the object unconfigured/UNKNOWN. */
bool ir_reflectance_init(ir_reflectance_t *p_sensor,
                        const ir_calibration_t *p_calibration);
void ir_reflectance_reset(ir_reflectance_t *p_sensor);

/* Returns false for invalid arguments/raw counts or unconfigured state.
 * Valid readings are clamped to 0 (white)..1000 (black). Inside the hysteresis
 * band, retain the previous surface; initially this is UNKNOWN. On invalid
 * raw input, reset the surface to UNKNOWN and leave p_darkness unchanged.
 * This cannot detect a disconnected wire or validate real track calibration. */
bool ir_reflectance_update(ir_reflectance_t *p_sensor, uint16_t raw,
                          uint16_t *p_darkness);

#endif /* BUDDY3_IR_REFLECTANCE_H */

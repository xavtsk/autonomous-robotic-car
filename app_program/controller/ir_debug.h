/* Optional observation-only IR preview. No ADC, navigation or motor calls.
 * Explicit 16-byte packet: version + 118 body bits, two zero padding bits.
 * Channel order is LEFT, RIGHT, BARCODE (not s3_sensor_id_t enum order).
 */
#ifndef CONTROLLER_IR_DEBUG_H
#define CONTROLLER_IR_DEBUG_H
#include <stdbool.h>
#include <stdint.h>
#ifndef APP_IR_DEBUG
#define APP_IR_DEBUG 0
#endif
#define IR_DEBUG_VERSION 1U
#define IR_DEBUG_PACKET_SIZE 16U
#define IR_DEBUG_PERIOD_US 50000U /* <= 20 Hz preview; processing stays 100 Hz */
typedef struct { uint8_t bytes[IR_DEBUG_PACKET_SIZE]; } ir_debug_packet_t;
typedef struct {
    uint32_t timestamp_us;
    uint16_t raw[3], darkness[3];
    uint8_t raw_valid, normalised_valid, surface[3], sequence;
} ir_debug_sample_t;
bool ir_debug_pack(const ir_debug_sample_t *sample, ir_debug_packet_t *packet);
bool ir_debug_unpack(const ir_debug_packet_t *packet, ir_debug_sample_t *sample);
bool ir_debug_due(uint32_t now, uint32_t *last, bool *seen);
#endif

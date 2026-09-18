/*
 * lwipopts.h - lwIP configuration for the robot car's telemetry link.
 *
 * lwIP is the TCP/IP stack bundled with the Pico SDK. It is configured by
 * #defines rather than by a settings file, and this header is where the SDK
 * looks for them.
 *
 * We start from the config the official pico_w examples use, then turn on
 * the MQTT client and size its buffers for our messages.
 */
#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

/* The pico-examples baseline: NO_SYS (no RTOS underneath lwIP), no sockets,
   sensible memory pool sizes for a Pico W. */
#include "lwipopts_examples_common.h"

/* ------------------------------------------------------------------------
 * MQTT
 * ------------------------------------------------------------------------ */

/*
 * Outgoing request slots. Each in-flight QoS 1 publish or subscribe occupies
 * one until the broker acknowledges it. Our telemetry is QoS 0 (fire and
 * forget, no slot needed); the slots are for the QoS 1 subscribe at startup
 * and the QoS 1 status messages.
 */
#define MQTT_REQ_MAX_IN_FLIGHT      8

/*
 * Largest payload we will ever send or receive in one go.
 *
 * Outgoing is bounded by TELE_PAYLOAD_MAX (224). Incoming has to hold a
 * barcode remap: 26 pairs of {"X":"STRAIGHT"} is about 340 bytes, so 512
 * covers the worst realistic command with room to spare.
 */
#define MQTT_OUTPUT_RINGBUF_SIZE    1024
#define MQTT_VAR_HEADER_BUFFER_LEN  512

/* Seconds of silence before the broker declares us dead and fires our Last
   Will. Short enough that a crash is noticed quickly during a 5-minute demo. */
#define MQTT_CONNECT_KEEPALIVE      30

#endif /* _LWIPOPTS_H */

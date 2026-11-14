/* Tribunal 1 application wire format. No RTOS, pointers or C struct layouts
 * cross the wire. See MQTT_PROTOCOL.md for bit order and field ranges. */
#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>

#define BUDDY1_WIRE_VERSION       3U
#define BUDDY1_PAYLOAD_MAX       14U /* version + 102-bit snapshot */
#define BUDDY1_HUMP_HEIGHT_MAX_MM 500U
#define BUDDY1_OBSTACLE_MIN_MM    30U
#define BUDDY1_OBSTACLE_MAX_MM    4000U
#define BUDDY1_OBSTACLE_WIDTH_MAX_MM 4095U
#define BUDDY1_MAP_BYTES          4U
#define BUDDY1_SPEED_MAX_MMS    510
#define BUDDY1_FAULT_STALL       (1U << 0)
#define BUDDY1_FAULT_ENCODER     (1U << 1)
#define BUDDY1_FAULT_IMU         (1U << 2)
#define BUDDY1_FAULT_NO_ECHO     (1U << 3)
#define BUDDY1_FAULT_WIFI        (1U << 4)
#define BUDDY1_FAULT_IR          (1U << 5)
#define BUDDY1_FAULT_BITS        6U
#define BUDDY1_FAULT_MASK        0x3fU
/* Route pose (live only, not in the snapshot): distance since START in cm,
 * sent modulo 4096 and unwrapped by the laptop; heading since START. */
#define BUDDY1_POSE_DISTANCE_BITS 12U
#define BUDDY1_POSE_DISTANCE_MOD  (1U << BUDDY1_POSE_DISTANCE_BITS)
/* Buddy 4 live terrain (not in the snapshot): pitch/roll in 0.1 degree,
 * -90.0..+90.0 (same 11-bit signed widths as Buddy 4's own packet), terrain
 * state, live climb height and this run's highest hump, 0..500 mm. */
#define BUDDY1_TILT_BITS          11U
#define BUDDY1_TILT_LIMIT_X10     900
#define BUDDY1_TERRAIN_STATES     8U  /* Buddy 4 terrain_state_t, 3 bits */
#define BUDDY1_TERRAIN_IMPACT     7U

enum {
    BUDDY1_TOPIC_MOTION = 0, BUDDY1_TOPIC_BARCODE, BUDDY1_TOPIC_HUMP,
    BUDDY1_TOPIC_HEADING, BUDDY1_TOPIC_OBSTACLE, BUDDY1_TOPIC_STATE,
    BUDDY1_TOPIC_FAULTS, BUDDY1_TOPIC_SNAPSHOT, BUDDY1_TOPIC_MAP_REPLY,
    BUDDY1_TOPIC_STATUS, BUDDY1_TOPIC_POSE, BUDDY1_TOPIC_TERRAIN,
    BUDDY1_TOPIC_AVOIDANCE, BUDDY1_TOPIC_SCAN, BUDDY1_TOPIC_COUNT
};
typedef enum {
    BUDDY1_ACTION_UNKNOWN = 0, BUDDY1_ACTION_FORWARD, BUDDY1_ACTION_LEFT,
    BUDDY1_ACTION_RIGHT, BUDDY1_ACTION_UTURN
} BUDDY1_BARCODE_ACTION;
typedef enum {
    BUDDY1_ROUTE_UNKNOWN = 0, BUDDY1_ROUTE_LEFT, BUDDY1_ROUTE_RIGHT
} BUDDY1_OBSTACLE_ROUTE;
typedef enum {
    BUDDY1_STATE_UNKNOWN = 0, BUDDY1_STATE_STOPPED,
    BUDDY1_STATE_LINE_FOLLOWING, BUDDY1_STATE_TURNING_LEFT,
    BUDDY1_STATE_TURNING_RIGHT, BUDDY1_STATE_UTURN,
    BUDDY1_STATE_AVOIDING_OBSTACLE, BUDDY1_STATE_COMPLETE, BUDDY1_STATE_ERROR
} BUDDY1_ROBOT_STATE;
enum {
    BUDDY1_REPLY_OK = 0, BUDDY1_REPLY_INVALID, BUDDY1_REPLY_BUSY,
    BUDDY1_REPLY_INTERNAL
};
typedef struct {
    uint8_t forward, left, right, uturn; /* decoded A-Z; never sent raw */
} BUDDY1_BARCODE_MAP;
/* Buddy 5 live status (car/telemetry/avoidance, 5 Hz). Unavailable = zero. */
#define BUDDY1_S5_STATE_MAX   6U  /* idle, wait-stop, scanning, moving, finish, complete, fault */
#define BUDDY1_S5_PASS_MAX    4U  /* approach, peek, peeking, step, extra */
#define BUDDY1_SERVO_MAX_DEG  180U
typedef struct {
    uint8_t range_valid;     /* front ultrasonic reading usable              */
    uint8_t no_echo;         /* confirmed no-echo (never with range_valid)    */
    uint16_t range_mm;       /* 30..4000 when valid, else 0                   */
    uint8_t servo_deg;       /* 1..180 last commanded, 90 = ahead; 0 unknown  */
    uint8_t state;           /* 0..6 Buddy 5 avoidance state                  */
    uint8_t fault;           /* 0..7 Buddy 5 fault reason                     */
    uint8_t route;           /* BUDDY1_OBSTACLE_ROUTE                          */
    uint8_t leg;             /* 0..15 bypass leg                               */
    uint8_t pass;            /* 0..4 leg-3 pass step                           */
    uint8_t swapped;         /* switched to the other side                    */
} BUDDY1_AVOIDANCE;
/* Buddy 5 obstacle profile (car/telemetry/scan, per finished scan or side
 * peek): FR6 position (edges, + = left of the car's centreline), width,
 * closest distance and left/right clearance. Invalid profile = zero values. */
#define BUDDY1_SCAN_EDGE_MAX_MM 4000
typedef struct {
    uint8_t valid;
    uint8_t scan_id;         /* low 8 bits of Buddy 5's scan id               */
    uint16_t closest_mm;     /* 30..4000                                      */
    uint16_t width_mm;       /* 0..4095                                       */
    uint16_t left_clear_mm, right_clear_mm; /* 0..4000                         */
    uint16_t forward_mm;     /* 0..4000, depth to the far face                */
    int16_t left_edge_mm, right_edge_mm;    /* -4000..4000                     */
} BUDDY1_SCAN;
/* One live Buddy 4 report. Unavailable values are sent as zero. */
typedef struct {
    uint8_t tilt_valid;      /* 0: pitch/roll unavailable                  */
    int16_t pitch_x10;       /* -900..900, + = nose up                      */
    int16_t roll_x10;        /* -900..900, + = right side up                */
    uint8_t state;           /* 0..7 terrain state; 7 = impact in the period */
    uint8_t on_hump;         /* a hump is being crossed now                 */
    uint16_t climb_mm;       /* 0..500, height so far on that hump          */
    uint8_t peak_valid;      /* a valid hump completed since START          */
    uint16_t peak_mm;        /* 0..500, highest of this run                 */
} BUDDY1_TERRAIN;

/* Temporary decoded view for callers/calculations. The shared cache is a
 * packed 14-byte packet, protected by the Buddy 1 mutex. */
typedef struct {
    uint8_t motion_valid;
    int16_t left_mms, right_mms;
    uint8_t barcode_valid, barcode_letter;
    BUDDY1_BARCODE_ACTION barcode_action;
    uint8_t hump_valid;
    uint16_t hump_count, hump_height_mm;
    uint8_t heading_valid;
    int16_t heading_deg;
    uint8_t obstacle_valid;
    uint16_t obstacle_distance_mm, obstacle_width_mm;
    BUDDY1_OBSTACLE_ROUTE obstacle_route;
    uint8_t robot_state_valid;
    BUDDY1_ROBOT_STATE robot_state;
    uint8_t faults_valid, faults;
    /* Live-only pose; never part of the snapshot packet. */
    uint8_t pose_valid;
    uint16_t pose_distance_cm;  /* 0..4095, distance since START modulo 4096 */
    int16_t pose_heading_deg;   /* 0..359, + = anticlockwise since START */
    /* Live-only Buddy 4 terrain; never part of the snapshot packet. */
    uint8_t terrain_valid;
    BUDDY1_TERRAIN terrain;
    /* Live-only Buddy 5 status and profile; never part of the snapshot. */
    uint8_t avoidance_valid;
    BUDDY1_AVOIDANCE avoidance;
    uint8_t scan_valid;
    BUDDY1_SCAN scan;
} BUDDY1_SNAPSHOT;

unsigned buddy1_payload_size(uint8_t topic);
unsigned buddy1_payload_bits(uint8_t topic);
const char *buddy1_topic_name(uint8_t topic);
/* Return packet length, or zero for invalid input/capacity. Byte 0 is version;
 * body bits are numbered from the least significant bit of byte 1. */
unsigned buddy1_encode(uint8_t topic, const BUDDY1_SNAPSHOT *value,
                       uint8_t *packet, unsigned capacity);
int buddy1_decode(uint8_t topic, const uint8_t *packet, unsigned length,
                  BUDDY1_SNAPSHOT *out);
int buddy1_payload_valid(uint8_t topic, const uint8_t *packet, unsigned length);
int buddy1_map_pack(const BUDDY1_BARCODE_MAP *map, uint8_t packet[BUDDY1_MAP_BYTES]);
int buddy1_map_unpack(const uint8_t *packet, unsigned length, BUDDY1_BARCODE_MAP *out);

#endif

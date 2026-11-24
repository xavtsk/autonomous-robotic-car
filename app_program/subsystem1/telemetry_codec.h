/** @file telemetry_codec.h
 * @brief Define bounded telemetry values, packets and codec interfaces.
 */
/* Tribunal 1 application wire format. No RTOS, pointers or C struct layouts
 * cross the wire. See MQTT_PROTOCOL.md for bit order and field ranges. */
#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>

#define BUDDY1_WIRE_VERSION 3U
#define BUDDY1_SCAN_FACE_VERSION 4U
#define BUDDY1_PAYLOAD_MAX                                                     \
    17U /* The version and 124-bit snapshot fit in 17 bytes. */
#define BUDDY1_HUMP_HEIGHT_MAX_MM 500U
#define BUDDY1_OBSTACLE_MIN_MM 30U
#define BUDDY1_OBSTACLE_MAX_MM 4000U
#define BUDDY1_OBSTACLE_WIDTH_MAX_MM 4095U
#define BUDDY1_MAP_BYTES 4U
#define BUDDY1_SPEED_MAX_MMS 510
#define BUDDY1_FAULT_STALL (1U << 0)
#define BUDDY1_FAULT_ENCODER (1U << 1)
#define BUDDY1_FAULT_IMU (1U << 2)
#define BUDDY1_FAULT_NO_ECHO (1U << 3)
#define BUDDY1_FAULT_WIFI (1U << 4)
#define BUDDY1_FAULT_IR (1U << 5)
#define BUDDY1_FAULT_BITS 6U
#define BUDDY1_FAULT_MASK 0x3fU
/* Both live pose and the snapshot carry distance and heading since START.
 * Distance is centimetres modulo 4096. The laptop unwraps the value. */
#define BUDDY1_POSE_DISTANCE_BITS 12U
#define BUDDY1_POSE_DISTANCE_MOD (1U << BUDDY1_POSE_DISTANCE_BITS)
/* The separate live terrain topic carries pitch and roll in tenths of a
 * degree, from -90.0 to +90.0. It also carries the terrain state, current
 * climb and run peak. Both heights range from 0 to 500 mm. */
#define BUDDY1_TILT_BITS 11U
#define BUDDY1_TILT_LIMIT_X10 900
#define BUDDY1_TERRAIN_STATES 8U /* Buddy 4 uses three bits. */
#define BUDDY1_TERRAIN_IMPACT 7U
/* 19 Nov 2026: Buddy 4 turn rate (compass, whole deg/s, same 10-bit signed
 * width as Buddy 4's own packet) and safety trip, previously USB-only. */
#define BUDDY1_TURN_BITS 10U
#define BUDDY1_TURN_LIMIT_DPS 360
#define BUDDY1_SAFETY_NONE 0U
#define BUDDY1_SAFETY_LIFTED 1U
#define BUDDY1_SAFETY_OVERTURNED 2U

typedef enum
{
    BUDDY1_TOPIC_MOTION = 0,
    BUDDY1_TOPIC_BARCODE,
    BUDDY1_TOPIC_HUMP,
    BUDDY1_TOPIC_HEADING,
    BUDDY1_TOPIC_OBSTACLE,
    BUDDY1_TOPIC_STATE,
    BUDDY1_TOPIC_FAULTS,
    BUDDY1_TOPIC_SNAPSHOT,
    BUDDY1_TOPIC_MAP_REPLY,
    BUDDY1_TOPIC_STATUS,
    BUDDY1_TOPIC_POSE,
    BUDDY1_TOPIC_TERRAIN,
    BUDDY1_TOPIC_AVOIDANCE,
    BUDDY1_TOPIC_SCAN,
    BUDDY1_TOPIC_SCAN_SAMPLE,
    BUDDY1_TOPIC_COUNT
} telemetry_codec_topic_t;

typedef enum
{
    BUDDY1_ACTION_UNKNOWN = 0,
    BUDDY1_ACTION_FORWARD,
    BUDDY1_ACTION_LEFT,
    BUDDY1_ACTION_RIGHT,
    BUDDY1_ACTION_UTURN
} telemetry_codec_action_t;

typedef enum
{
    BUDDY1_ROUTE_UNKNOWN = 0,
    BUDDY1_ROUTE_LEFT,
    BUDDY1_ROUTE_RIGHT
} telemetry_codec_route_t;

typedef enum
{
    BUDDY1_STATE_UNKNOWN = 0,
    BUDDY1_STATE_STOPPED,
    BUDDY1_STATE_LINE_FOLLOWING,
    BUDDY1_STATE_TURNING_LEFT,
    BUDDY1_STATE_TURNING_RIGHT,
    BUDDY1_STATE_UTURN,
    BUDDY1_STATE_AVOIDING_OBSTACLE,
    BUDDY1_STATE_COMPLETE,
    BUDDY1_STATE_ERROR
} telemetry_codec_state_t;

typedef enum
{
    BUDDY1_REPLY_OK = 0,
    BUDDY1_REPLY_INVALID,
    BUDDY1_REPLY_BUSY,
    BUDDY1_REPLY_INTERNAL
} telemetry_codec_reply_t;

typedef struct
{
    uint8_t forward;
    uint8_t left;
    uint8_t right;
    uint8_t uturn; /* The map uses encoded letters A through Z. */
} telemetry_codec_map_t;

/* The 5 Hz avoidance topic sends unavailable fields as zero. */
#define BUDDY1_S5_STATE_MAX                                                    \
    6U /* State values follow Buddy 5 from idle through fault. */
#define BUDDY1_S5_PASS_MAX 4U /* Pass steps end at extra. */
#define BUDDY1_SERVO_MAX_DEG 180U

typedef struct
{
    uint8_t  range_valid; /* The forward range is usable. */
    uint8_t  no_echo;     /* No echo implies an invalid range. */
    uint16_t range_mm;    /* Use 30 to 4000 mm if valid or zero. */
    uint8_t  servo_deg;   /* Use 1 to 180 degrees or zero if unknown. */
    uint8_t  state;       /* Buddy 5 state ranges from 0 to 6. */
    uint8_t  fault;       /* Buddy 5 reasons range from 0 to 7. */
    uint8_t  route;       /* telemetry_codec_route_t                          */
    uint8_t  leg;         /* The bypass leg ranges from 0 to 15. */
    uint8_t  pass;        /* Leg 3 has pass steps 0 through 4. */
    uint8_t  swapped;     /* The car changed sides. */
} telemetry_codec_avoidance_t;

/* The scan topic carries the finished sweep or side-peek profile. Positive
 * edge positions are left of the car's centreline. Width, closest distance
 * and both clearances are in millimetres. Invalid profiles use zero fields. */
#define BUDDY1_SCAN_EDGE_MAX_MM 4000

typedef struct
{
    uint8_t  valid;
    uint8_t  scan_id;    /* Use the low eight bits of the scan ID. */
    uint16_t closest_mm; /* 30..4000                                      */
    uint16_t width_mm;   /* 0..4095                                       */
    uint16_t left_clear_mm;
    uint16_t right_clear_mm; /* 0..4000 */
    uint16_t forward_mm;     /* The far face is 0 to 4000 mm away. */
    int16_t  left_edge_mm;
    int16_t  right_edge_mm;     /* -4000..4000                     */
    uint16_t face_width_min_mm; /* Bracketed face width, 0..4095 mm. */
    uint16_t face_width_max_mm; /* Both zero when the sweep lacks an edge. */
} telemetry_codec_scan_t;

/* One live Buddy 4 report. Unavailable values are sent as zero. */
typedef struct
{
    uint8_t  tilt_valid;       /* Zero marks unavailable pitch and roll. */
    int16_t  pitch_tenths_deg; /* Use -900 to 900. Positive raises the nose. */
    int16_t  roll_tenths_deg;  /* Use -900 to 900. Positive raises the right. */
    uint8_t  state;            /* State 7 reports an impact in the period. */
    uint8_t  on_hump;          /* The car is on a hump. */
    uint16_t climb_mm;         /* Climb reports 0 to 500 mm so far. */
    uint8_t  peak_valid;       /* A valid hump completed since START. */
    uint16_t peak_mm;          /* The run's peak is 0 to 500 mm. */
    uint8_t  turn_valid;       /* Zero marks an unavailable compass rate. */
    int16_t  turn_dps; /* Use -360 to 360 deg/s, positive anticlockwise. */
    uint8_t  safety;   /* Report no trip, lifted, or overturned. */
} telemetry_codec_terrain_t;

/* One Buddy 5 ultrasonic reading of a finished scan or side peek
 * (car/telemetry/scan_sample, sent one by one after the profile). */
#define BUDDY1_SCAN_SAMPLES_MAX 16U

typedef struct
{
    uint8_t  scan_id;     /* Use the same low byte as the scan topic. */
    uint8_t  peek;        /* Set for a single side-peek reading. */
    uint8_t  index;       /* Index from 0 to count - 1 in scan order. */
    uint8_t  count;       /* A scan contains one to 16 readings. */
    uint8_t  angle_deg;   /* Scan from 30 to 150 in 5 degree steps. */
    uint8_t  valid;       /* Zero marks no usable echo at this angle. */
    uint16_t distance_mm; /* Use 30 to 4000 mm if valid or zero. */
} telemetry_codec_scan_sample_t;

/* Temporary decoded view for callers/calculations. The shared cache is a
 * packed 17-byte packet, protected by the Buddy 1 mutex. */
typedef struct
{
    uint8_t                  motion_valid;
    int16_t                  left_mms;
    int16_t                  right_mms;
    uint8_t                  barcode_valid;
    uint8_t                  barcode_letter;
    telemetry_codec_action_t barcode_action;
    uint8_t                  hump_valid;
    uint16_t                 hump_count;
    uint16_t                 hump_height_mm;
    uint8_t                  heading_valid;
    int16_t                  heading_deg;
    uint8_t                  obstacle_valid;
    uint16_t                 obstacle_distance_mm;
    uint16_t                 obstacle_width_mm;
    telemetry_codec_route_t  obstacle_route;
    uint8_t                  robot_state_valid;
    telemetry_codec_state_t  robot_state;
    uint8_t                  faults_valid;
    uint8_t                  faults;
    /* Live-only pose; never part of the snapshot packet. */
    uint8_t  pose_valid;
    uint16_t pose_distance_cm; /* Wrap distance since START at 4096 cm. */
    int16_t  pose_heading_deg; /* Measure heading anticlockwise from START. */
    /* Live-only Buddy 4 terrain; never part of the snapshot packet. */
    uint8_t                   terrain_valid;
    telemetry_codec_terrain_t terrain;
    /* Live-only Buddy 5 status and profile; never part of the snapshot. */
    uint8_t                       avoidance_valid;
    telemetry_codec_avoidance_t   avoidance;
    uint8_t                       scan_valid;
    telemetry_codec_scan_t        scan;
    uint8_t                       scan_sample_valid;
    telemetry_codec_scan_sample_t scan_sample;
} telemetry_codec_snapshot_t;

/** @brief Return the version 3 wire length for a topic selector.
 * @param[in] topic Selector in 0..BUDDY1_TOPIC_COUNT-1.
 * @return Version byte plus rounded-up body bytes, or zero for an unknown
 * selector. MAP_REPLY has a size although buddy1_encode does not build it.
 */
unsigned telemetry_codec_payload_size (uint8_t topic);
/** @brief Return the encoded length for a topic and wire version.
 * @param[in] topic Telemetry layout selector.
 * @param[in] version Wire version byte. Version 4 extends only scan packets.
 * @return Total bytes including the version, or zero for an unknown pair.
 */
unsigned telemetry_codec_version_size (uint8_t topic, uint8_t version);
/** @brief Return the meaningful body width, excluding version and padding.
 * @param[in] topic Selector in 0..BUDDY1_TOPIC_COUNT-1.
 * @return Body bits for that layout, or zero for an unknown selector.
 */
unsigned telemetry_codec_payload_bits (uint8_t topic);
/** @brief Look up the MQTT topic string for a wire layout.
 * @param[in] topic Selector in 0..BUDDY1_TOPIC_COUNT-1.
 * @return Borrowed NUL-terminated string with program lifetime, or null for
 * an unknown selector. The caller must not modify or free the string.
 */
const char * telemetry_codec_topic_name (uint8_t topic);
/* Return packet length, or zero for invalid input/capacity. Byte 0 is version;
 * body bits are numbered from the least significant bit of byte 1. */
/** @brief Serialize the selected telemetry fields into a versioned packet.
 * @param[in] topic Known selector other than BUDDY1_TOPIC_MAP_REPLY.
 * @param[in] p_value Stable decoded fields in the units described below.
 * @param[out] p_packet Caller-owned storage of at least capacity bytes.
 * @param[in] capacity Writable byte count; at least
 * telemetry_codec_version_size(topic, version).
 * @return Packet length on success, or zero for nulls, unsupported topic,
 * insufficient capacity or invalid selected fields.
 * @details Null/topic/capacity failures leave the packet unchanged. After
 * those checks, the complete payload is cleared and its version written;
 * a later field failure can leave a partial packet. Bytes beyond the topic's
 * payload length are untouched. Treat a zero return as no usable packet.
 * @pre Input and packet storage do not overlap. The caller keeps input
 * stable and owns output exclusively; no locking, allocation or retention
 * of caller pointers is performed.
 * @par Layout and validity
 * Byte zero is BUDDY1_WIRE_VERSION for the base layout. A scan with
 * bracketed face edges uses BUDDY1_SCAN_FACE_VERSION. Fields are low bit first;
 * unused high bits are zero. Single-section topics encode their fields even
 * if the corresponding top-level validity flag is zero. SNAPSHOT selects the
 * original seven sections plus pose using nonzero validity flags; absent
 * sections have zero bits without validating their stale values. Version 3
 * includes pose in the snapshot despite retained earlier live-only notes.
 * Terrain, avoidance and scan reports are not snapshot sections. STATUS sets
 * the heartbeat and includes faults only when faults_valid is nonzero.
 * @par Numeric domains and precision
 * Motion accepts -510..510 mm/s and rounds to the nearest 2 mm/s, with ties
 * away from zero (at most 1 mm/s error). Other fields use exact integer units.
 * Barcode letters are A..Z and actions UNKNOWN..UTURN. Hump count is 16 bits
 * and height is 0..500 mm. Heading is 0..359 degrees. Obstacles use distance
 * 30..4000 mm, width 0..4095 mm and route UNKNOWN..RIGHT. Robot state is
 * STOPPED..ERROR; faults use only BUDDY1_FAULT_MASK. Pose uses 0..4095 cm and
 * heading 0..359 degrees; the caller performs distance wrapping beforehand.
 * Terrain accepts pitch/roll -900..900 in tenths of a degree, state 0..7,
 * climb/peak 0..500 mm, turn -360..360 degrees/s and safety NONE..OVERTURNED.
 * Unavailable tilt, climb, peak and turn components must have zero values.
 * Its validity flags are normalized from zero/nonzero.
 * Avoidance follows the ranges in telemetry_codec_avoidance_t: range_valid,
 * no_echo and swapped are 0/1; a valid range excludes no_echo, and an invalid
 * range must be zero. Scan geometry follows telemetry_codec_scan_t, with left
 * edge at least right edge when valid and all geometry zero when invalid; scan
 * ID remains unrestricted. A scan sample requires peek/valid 0/1, count 1..16,
 * index below count, angle 30..150 in 5-degree steps, and distance 30..4000 mm
 * when valid or zero otherwise. Values are rejected rather than saturated.
 */
unsigned buddy1_encode (uint8_t                            topic,
                        const telemetry_codec_snapshot_t * p_value,
                        uint8_t * p_packet, unsigned capacity);
/** @brief Decode one canonical telemetry payload into a fresh decoded view.
 * @param[in] topic Known selector other than BUDDY1_TOPIC_MAP_REPLY.
 * @param[in] p_packet Stable readable packet containing length bytes.
 * @param[in] length Exact telemetry_codec_version_size for the topic and
 * version byte.
 * @param[out] p_out Caller-owned snapshot; unchanged on every failure.
 * @return One after assigning the complete decoded view; zero for nulls,
 * unsupported topic, wrong length/version, nonzero padding or invalid fields.
 * @details A local zeroed result is populated before assignment. Fields not
 * present in the chosen topic remain zero, and decoded sections get validity
 * flags of one. Snapshot sections marked absent must contain canonical zeros.
 * STATUS requires heartbeat one and zero fault bits when faults are invalid.
 * Motion negative-zero codes are rejected; decoded speed is an even number
 * of mm/s. Other domains and units match buddy1_encode. No locking occurs
 * and no caller pointer is retained; the caller stabilizes input and owns
 * output.
 */
int buddy1_decode (uint8_t topic, const uint8_t * p_packet, unsigned length,
                   telemetry_codec_snapshot_t * p_out);
/** @brief Validate a payload without publishing decoded state.
 * @param[in] topic Selector to validate, including BUDDY1_TOPIC_MAP_REPLY.
 * @param[in] p_packet Stable readable packet, or null to return zero.
 * @param[in] length Exact packet byte count.
 * @return One for a valid packet, otherwise zero. MAP_REPLY specifically
 * requires two bytes: current version and reply code OK..INTERNAL; all other
 * topics use buddy1_decode. The input is never changed or retained.
 */
int telemetry_codec_payload_valid (uint8_t topic, const uint8_t * p_packet,
                                   unsigned length);
/** @brief Encode four distinct barcode letters for the route actions.
 * @param[in] p_map Stable mapping of forward, left, right and U-turn to A..Z.
 * @param[out] p_packet Writable four-byte packet; unchanged on failure.
 * @return One after writing the complete packet; zero for null pointers,
 * an out-of-range letter or any duplicate letter.
 * @details Byte zero is the version, followed by four five-bit A-based
 * codes in action order, low bit first. The final four padding bits are zero.
 * Validation completes before writes; no pointer is retained or lock taken.
 */
int buddy1_map_pack (const telemetry_codec_map_t * p_map,
                     uint8_t                       p_packet[BUDDY1_MAP_BYTES]);
/** @brief Decode and validate a complete four-action barcode map.
 * @param[in] p_packet Stable readable map packet containing length bytes.
 * @param[in] length Exactly BUDDY1_MAP_BYTES.
 * @param[out] p_out Caller-owned map; unchanged on failure.
 * @return One after assigning four distinct A..Z letters; zero for nulls,
 * wrong size/version, nonzero final padding, unused codes or duplicates.
 * @details A local decoded map is checked with buddy1_map_pack before commit.
 * The caller supplies synchronization; no caller pointer is retained.
 */
int telemetry_codec_map_unpack (const uint8_t * p_packet, unsigned length,
                                telemetry_codec_map_t * p_out);

#endif

/* End of file. */


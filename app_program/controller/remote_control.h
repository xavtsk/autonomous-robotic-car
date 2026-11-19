/** @file remote_control.h
 * @brief Define dashboard command, status and reply byte layouts and accessors.
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
/* Fixed byte layouts shared with the dashboard; see
 * docs/REMOTE_CONTROL_PROTOCOL.md. */
#include <stdint.h>
#include <stdbool.h>
#define REMOTE_VERSION 3U
#define REMOTE_REPLY_VERSION 4U
#define REMOTE_COMMAND_SIZE 14U
#define REMOTE_STATUS_SIZE 14U
#define REMOTE_REPLY_ACTION_BITS 2U
#define REMOTE_REPLY_RESULT_BITS 2U
#define REMOTE_REPLY_REASON_BITS 3U
#define REMOTE_REPLY_BITS 7U
#define REMOTE_REPLY_SIZE 10U
#define REMOTE_REPLY_ID_OFFSET 2U
#define REMOTE_REPLY_EPOCH_OFFSET 6U
#define REMOTE_REPLY_RING_SIZE 6U
#define REMOTE_MAX_AGE_US 2000000U

enum
{
    REMOTE_START = 1,
    REMOTE_STOP = 2,
    REMOTE_RESET = 3
};

enum
{
    REMOTE_QUEUED = 0,
    REMOTE_EXECUTED = 1,
    REMOTE_REJECTED = 2,
    REMOTE_CANCELLED = 3
};

enum
{
    REMOTE_NONE = 0,
    REMOTE_INVALID = 1,
    REMOTE_STALE = 2,
    REMOTE_REPLAY = 3,
    REMOTE_BUSY = 4,
    REMOTE_NOT_READY = 5,
    REMOTE_SUPERSEDED = 6,
    REMOTE_UNAVAILABLE = 7
};

enum
{
    REMOTE_AVAILABLE = 1,
    REMOTE_CAN_START = 2,
    REMOTE_CAN_RESET = 4,
    REMOTE_RUNNING = 8
};

/* Requests and status retain version 3. Only replies use the compact version
 * 4 layout: action bits 0..1, result 2..3, reason 4..6; bit 7 stays zero.
 * Request fields are assigned only after the wire action has been validated.
 * These native bit-fields stay in memory; they are never sent as wire bytes. */
typedef struct
{
    uint32_t id;
    uint32_t epoch;
    uint32_t time_us;
    unsigned action : REMOTE_REPLY_ACTION_BITS;
    bool valid : 1;
} remote_request_t;

static inline uint32_t remote_get32 (const uint8_t * p_p)
{
    return (uint32_t)p_p[0] | (uint32_t)p_p[1] << 8 | (uint32_t)p_p[2] << 16 |
           (uint32_t)p_p[3] << 24;
}

static inline void remote_put32 (uint8_t * p_p, uint32_t value_n)
{
    for (unsigned value_i = 0; value_i < 4; value_i++)
    {
        p_p[value_i] = (uint8_t)(value_n >> (8U * value_i));
    }
}

static inline bool remote_newer (uint32_t value_a, uint32_t value_b)
{
    return value_a != value_b && (uint32_t)(value_a - value_b) < 0x80000000U;
}

/* Network callbacks never operate motors. The sampling owner executes through
 * exactly the same drive/calibration/fault checks as physical controls. */
void s3_remote_link (uint32_t epoch, bool value_up);
void s3_remote_receive (const uint8_t * p_packet, unsigned length);
unsigned s3_remote_reply_take (uint8_t * p_packet, unsigned capacity);
unsigned s3_remote_status_take (uint8_t * p_packet, unsigned capacity);
uint32_t s3_remote_stop_sequence (void);
void s3_remote_physical_stop (bool held);
#endif

/* End of file. */


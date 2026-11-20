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

/** @brief Read an unsigned little-endian value without changing the buffer.
 * @param[in] p_p Non-NULL readable storage containing at least four bytes.
 * @return The full 32-bit value; no range or pointer validation is performed.
 */
static inline uint32_t remote_get32 (const uint8_t * p_p)
{
    return ((((uint32_t)p_p[0] | ((uint32_t)p_p[1] << 8)) |
             ((uint32_t)p_p[2] << 16)) |
            ((uint32_t)p_p[3] << 24));
}

/** @brief Write exactly four little-endian bytes without retaining a pointer.
 * @param[out] p_p Non-NULL writable storage containing at least four bytes.
 * @param value_n Full-range unsigned value to encode; no narrowing is applied.
 */
static inline void remote_put32 (uint8_t * p_p, uint32_t value_n)
{
    for (unsigned value_i = 0; value_i < 4; value_i++)
    {
        p_p[value_i] = (uint8_t)(value_n >> (8U * value_i));
    }
}

/** @brief Compare sequence numbers using the existing unsigned half-range rule.
 * @param value_a Candidate sequence number, including wrapped values.
 * @param value_b Previously consumed sequence number.
 * @return True for a nonzero forward difference below 2^31; equality and the
 * exact half-range difference return false. No state changes.
 */
static inline bool remote_newer (uint32_t value_a, uint32_t value_b)
{
    return ((value_a != value_b) &&
            ((uint32_t)(value_a - value_b) < 0x80000000U));
}

/* Network callbacks never operate motors. The sampling owner executes through
 * exactly the same drive/calibration/fault checks as physical controls. */
/** @brief Record link state and cancel applicable pending remote requests.
 * @param epoch Session identifier; an up event maps zero to one and resets the
 * consumed command ID. A down event does not assign a new epoch.
 * @param value_up True for an available remote link.
 * @note Task context: waits for the controller mutex. Unavailable objects or
 * lock failure leave state unchanged. Pending STOP survives link changes.
 * With APP_S3_MOTION disabled, this and the other remote setters are no-ops.
 */
void s3_remote_link (uint32_t epoch, bool value_up);
/** @brief Validate and enqueue a borrowed version-3 command.
 * @param[in] p_packet Readable command bytes for this call; NULL is ignored.
 * @param length Must equal REMOTE_COMMAND_SIZE. Bad version/action/length is
 * ignored without a reply. Storage is copied, not retained.
 * @note Task context; may wait for the controller mutex. Valid STOP takes
 * precedence and bypasses freshness/replay/busy checks. Other commands require
 * a current epoch, bounded microsecond age and a newer ID. Newer same-epoch
 * IDs are consumed even when rejected. Disabled integration ignores packets.
 * Network callbacks never drive motors directly.
 */
void s3_remote_receive (const uint8_t * p_packet, unsigned length);
/** @brief Remove one queued version-4 reply using a nonblocking mutex attempt.
 * @param[out] p_packet Writable buffer; NULL returns zero without consuming.
 * @param capacity Buffer size in bytes; at least REMOTE_REPLY_SIZE is required.
 * @return REMOTE_REPLY_SIZE on success, otherwise zero with output unchanged.
 * An empty queue, unavailable/locked controller or disabled integration
 * returns zero. The caller owns the output and no pointer is retained.
 */
unsigned s3_remote_reply_take (uint8_t * p_packet, unsigned capacity);
/** @brief Copy an eligible version-3 status and record its publication.
 * @param[out] p_packet Writable buffer; NULL returns zero.
 * @param capacity Buffer size in bytes; must be at least REMOTE_STATUS_SIZE.
 * @return REMOTE_STATUS_SIZE when connected and due, otherwise zero without
 * modifying output. The existing interval is 500000 us; capability flags are
 * cleared when the sampled state is older than 200000 us.
 * @note Uses a nonblocking mutex attempt. Failed locking, unavailable state
 * and disabled integration return zero. Wire bytes remain explicitly encoded.
 */
unsigned s3_remote_status_take (uint8_t * p_packet, unsigned capacity);
/** @brief Read the wrapping count of queued remote STOP requests.
 * @return Current 32-bit sequence, or zero for missing state, lock failure or
 * disabled integration. Task context; may wait for the controller mutex.
 */
uint32_t s3_remote_stop_sequence (void);
/** @brief Publish the physical STOP level used to inhibit remote START/RESET.
 * @param held True while physical STOP is held; does not itself drive motors.
 * @note Task context; may wait for the controller mutex. Missing state or lock
 * failure leaves the stored level unchanged. Disabled integration is a no-op.
 */
void s3_remote_physical_stop (bool held);
#endif

/* End of file. */


/** @file integration_codec.h
 * @brief Describe copied controller command and status packets.
 */
/* Explicit shared copies. Decoded calculation views never cross the mutex. */
#ifndef TEAM_INTEGRATION_CODEC_H
#define TEAM_INTEGRATION_CODEC_H
#include "s3_integration.h"
#include "../subsystem3/navigation_codec.h"

typedef struct
{
    uint8_t bytes[11];
} integration_codec_control_packet_t; /* The request occupies 83 bits. */

typedef struct
{
    navigation_codec_packet_t navigation; /* Use 479 bits in 60 bytes. */
    uint8_t feedback[22]; /* Use 176 bits and retain binary64 odometry. */
    uint8_t metadata[64]; /* Use 509 bits including run timing. */
} integration_codec_status_packet_t;

/** @brief Encode one validated control request into its canonical 11 bytes.
 * @param p_value Request to read; null is rejected. Sensor is validated only
 * for white/black/apply-calibration requests; mapping actions only for mapping.
 * @param p_out Destination packet, written only on success; null is rejected.
 * @return True for an encodable kind and applicable fields, otherwise false.
 * @note Unused sensor/mapping fields and reserved bits are encoded as zero.
 * The caller owns the destination; this function takes no lock and performs no
 * command submission or I/O.
 */
bool integration_codec_pack_cmd (const s3_integration_request_t *     p_value,
                                 integration_codec_control_packet_t * p_out);
/** @brief Decode a control packet only when its bytes are canonical.
 * @param p_packet Complete 11-byte packet to read; null is rejected.
 * @param p_out Decoded request, written only on success; null is rejected.
 * @return True when repacking reproduces every input byte, otherwise false.
 * @note Rejects nonzero reserved bits and unused fields, invalid kinds, and
 * invalid applicable sensor/mapping values. No command is executed or queued.
 */
bool integration_codec_unpack_cmd (
    const integration_codec_control_packet_t * p_packet,
    s3_integration_request_t *                 p_out);
/** @brief Encode a copied runtime status into the fixed 146-byte packet.
 * @param p_value Stable caller-owned status to read; null is rejected.
 * @param p_out Destination packet, written only on success; null is rejected.
 * @return False for out-of-range enums, sensor or calibration integer fields,
 * or navigation rejected by navigation_codec_pack; true after publishing a
 * copy.
 * @note Preserves binary64 odometry and binary32 calibration-average bits.
 * Those fields are not checked for finiteness; minimum/maximum ordering is not
 * checked here. The caller supplies a coherent status; no lock or I/O occurs.
 */
bool integration_codec_pack_status (const s3_integration_status_t *     p_value,
                                    integration_codec_status_packet_t * p_out);
/** @brief Decode a copied runtime packet after its structural checks pass.
 * @param p_packet Complete 146-byte packet to read; null is rejected.
 * @param p_out Decoded status, written only on success; null is rejected.
 * @return False for reserved metadata bits, invalid motion/drive enums or
 * sensor, or navigation rejected by navigation_codec_unpack; otherwise true.
 * @note Reconstructs signed ADC status without an out-of-range signed cast.
 * Floating representations are copied without finiteness validation, and no
 * runtime state is updated. The caller owns both buffers and synchronization.
 */
bool integration_codec_unpack_status (
    const integration_codec_status_packet_t * p_packet,
    s3_integration_status_t *                 p_out);
#endif

/* End of file. */


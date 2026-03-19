/*
 * emesh_frame.h — Wire frame header definition and encode/decode API.
 *
 * Frame layout (all multi-byte fields little-endian):
 *
 *  Byte   Field     Size  Description
 *  ────   ───────   ────  ──────────────────────────────────────────────────
 *   0     type        1   Packet type (EMESH_PACKET_TYPE_*)
 *   1     flags       1   Control flags (EMESH_FRAME_FLAG_*)
 *   2     ttl         1   Hop limit; decremented at each relay; drop when 0
 *   3–6   src_id      4   Sender node ID (little-endian uint32)
 *   7–10  dest_id     4   Destination ID; EMESH_DEST_BROADCAST = 0xFFFFFFFF
 *  11–12  seq         2   Per-sender sequence number (little-endian uint16)
 *  13–14  op          2   Op code: high byte = class, low byte = command
 *  15     length      1   Payload byte count that follows this header
 *  ────────────────────────────────────────────────────────────────────────
 *  Total: 16 bytes.  Max payload: 240 bytes (256-byte SX1276 FIFO − header).
 */

#ifndef EMESH_FRAME_H
#define EMESH_FRAME_H

#include <stdint.h>

#include "emesh_node_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  type;
    uint8_t  flags;
    uint8_t  ttl;
    uint32_t src_id;
    uint32_t dest_id;
    uint16_t seq;
    uint16_t op;
    uint8_t  length;
} emesh_frame_header_t;

#define EMESH_FRAME_HEADER_SIZE       16U   /* fixed header size in bytes      */
#define EMESH_FRAME_FIFO_SIZE        256U   /* SX1276 FIFO size in bytes       */
#define EMESH_FRAME_MAX_PAYLOAD      240U   /* FIFO_SIZE − HEADER_SIZE         */
#define EMESH_FRAME_DEFAULT_TTL        7U   /* default hop limit               */

/* ── flags byte ─────────────────────────────────────────────────────────── */
#define EMESH_FRAME_FLAG_ACK_REQUEST  0x01U /* sender expects an ACK           */
#define EMESH_FRAME_FLAG_ACK_PRESENT  0x02U /* piggybacked ACK in payload      */
#define EMESH_FRAME_FLAG_NAK          0x04U /* negative acknowledgement        */
#define EMESH_FRAME_FLAG_TIME_SYNC    0x08U /* requesting time synchronisation */
#define EMESH_FRAME_FLAG_REREGISTER   0x10U /* mobile node is re-registering   */
#define EMESH_FRAME_FLAG_BROADCAST    0x20U /* no ACK expected (broadcast)     */
#define EMESH_FRAME_FLAG_FRAGMENTED   0x40U /* payload carries fragment header */
#define EMESH_FRAME_FLAG_MORE_FRAGS   0x80U /* more fragments follow           */

/*
 * Encode a frame header into the first EMESH_FRAME_HEADER_SIZE bytes of
 * buffer.  buffer must point to at least EMESH_FRAME_HEADER_SIZE bytes.
 */
void emesh_frame_encode_header(const emesh_frame_header_t *header, uint8_t *buffer);

/*
 * Decode a frame header from the first EMESH_FRAME_HEADER_SIZE bytes of
 * buffer into header.
 */
void emesh_frame_decode_header(const uint8_t *buffer, emesh_frame_header_t *header);

/*
 * Return true if the frame is addressed to this node or is a broadcast.
 */
static inline bool emesh_frame_is_for_me(const emesh_frame_header_t *h, uint32_t my_id)
{
    return (h->dest_id == my_id) || (h->dest_id == EMESH_DEST_BROADCAST);
}

#ifdef __cplusplus
}
#endif

#endif /* EMESH_FRAME_H */

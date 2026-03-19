/*
 * retry.h — ACK/retry state machine for reliable unicast delivery.
 *
 * Usage:
 *   1. retry_send() — transmit a unicast frame and arm the retry context.
 *   2. retry_on_ack() — call when an ACK is received; clears the context.
 *   3. retry_tick() — call every loop iteration; retransmits if the ACK
 *      timer expires, up to RETRY_MAX_ATTEMPTS times.
 *
 * Broadcast frames (dest_id == EMESH_DEST_BROADCAST) bypass this module.
 *
 * Retry back-off: base_ms × 2^attempt, capped at RETRY_BACKOFF_MAX_MS.
 *
 * RAM: 4 slots × (16 + 128 + 16) bytes = 640 bytes.
 */

#ifndef RETRY_H
#define RETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "emesh_frame.h"
#include "csma_mac.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RETRY_TABLE_SIZE       4U
#define RETRY_MAX_ATTEMPTS     3U
#define RETRY_ACK_TIMEOUT_MS   500U
#define RETRY_BACKOFF_BASE_MS  200U
#define RETRY_BACKOFF_MAX_MS   8000U

typedef struct {
    bool     active;
    uint8_t  attempts;
    uint8_t  _pad[2];
    uint32_t ack_deadline_ms;    /* tick by which ACK must arrive      */
    emesh_frame_header_t header;
    uint8_t  payload[FRAME_QUEUE_MAX_PAYLOAD];
    uint8_t  payload_len;
} retry_entry_t;                 /* 16 + 128 + 12 = 156 bytes */

typedef struct {
    retry_entry_t slots[RETRY_TABLE_SIZE];
} retry_table_t;                 /* 624 bytes */

void retry_init(retry_table_t *tbl);

/*
 * Arm a retry slot for a unicast frame that has just been transmitted.
 * header.seq is used to match the incoming ACK.
 * Returns true if a slot was available.
 */
bool retry_arm(retry_table_t *tbl,
               const emesh_frame_header_t *header,
               const uint8_t *payload,
               uint8_t payload_len,
               uint32_t now_ms);

/*
 * Call when an ACK is received.  Clears the slot matching (src=ack_from,
 * seq=acked_seq) if one exists.
 * Returns true if a matching slot was found and cleared.
 */
bool retry_on_ack(retry_table_t *tbl, uint32_t ack_from, uint16_t acked_seq);

/*
 * Drive the retry state machine.  Retransmits any frame whose ACK deadline
 * has passed, or marks it as failed after RETRY_MAX_ATTEMPTS.
 *
 * mac is used for retransmission.
 * failed_dest is set to the dest_id of a failed frame (if any); 0 otherwise.
 */
void retry_tick(retry_table_t *tbl, csma_mac_t *mac,
                uint32_t now_ms, uint32_t *failed_dest);

#ifdef __cplusplus
}
#endif

#endif /* RETRY_H */

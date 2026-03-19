/*
 * frame_queue.h — Three-priority static TX queue.
 *
 * Priority levels:
 *   HIGH   — ACK, TIME_REQ/RESP, REG messages (4 slots)
 *   MEDIUM — telemetry, commands              (8 slots)
 *   LOW    — bulk / background traffic        (4 slots)
 *
 * The CSMA/CAD transmit loop always drains HIGH before MEDIUM before LOW.
 *
 * Payload is capped at FRAME_QUEUE_MAX_PAYLOAD bytes.  Larger frames must
 * use fragmentation (Phase 2).
 *
 * RAM: 16 entries × (16 + 128 + 8) bytes = 2 432 bytes total.
 */

#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#include "emesh_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_QUEUE_MAX_PAYLOAD   128U

#define FRAME_QUEUE_HIGH_SIZE       4U
#define FRAME_QUEUE_MED_SIZE        8U
#define FRAME_QUEUE_LOW_SIZE        4U
#define FRAME_QUEUE_TOTAL_SIZE  (FRAME_QUEUE_HIGH_SIZE + FRAME_QUEUE_MED_SIZE + FRAME_QUEUE_LOW_SIZE)

typedef enum {
    FRAME_PRIO_HIGH   = 0U,
    FRAME_PRIO_MEDIUM = 1U,
    FRAME_PRIO_LOW    = 2U,
    FRAME_PRIO_COUNT  = 3U,
} frame_prio_t;

typedef struct {
    emesh_frame_header_t header;
    uint8_t  payload[FRAME_QUEUE_MAX_PAYLOAD];
    uint8_t  payload_len;
    uint8_t  prio;          /* frame_prio_t */
    uint8_t  retry_count;
    uint8_t  _pad;
    uint32_t enqueue_ms;
    uint32_t next_tx_ms;    /* 0 = send immediately; >0 = retry after this tick */
} frame_queue_entry_t;      /* 16 + 128 + 8 = 152 bytes */

typedef struct {
    frame_queue_entry_t slots[FRAME_QUEUE_TOTAL_SIZE];
    uint8_t             used[FRAME_QUEUE_TOTAL_SIZE]; /* 1 = occupied */
    uint8_t             _pad[3];
} frame_queue_t;

void frame_queue_init(frame_queue_t *q);

/*
 * Enqueue a frame.  prio determines which pool it draws from.
 * Returns true on success; false if the priority pool is full.
 */
bool frame_queue_push(frame_queue_t *q,
                      const emesh_frame_header_t *header,
                      const uint8_t *payload,
                      uint8_t payload_len,
                      frame_prio_t prio,
                      uint32_t next_tx_ms);

/*
 * Peek at the highest-priority frame that is ready to transmit (next_tx_ms == 0
 * or next_tx_ms <= now_ms).  Returns a pointer to the slot (do not free it).
 * Returns NULL if the queue is empty or no frame is ready yet.
 */
frame_queue_entry_t *frame_queue_peek(frame_queue_t *q, uint32_t now_ms);

/*
 * Remove the entry pointed to by entry (obtained from frame_queue_peek).
 */
void frame_queue_remove(frame_queue_t *q, frame_queue_entry_t *entry);

/* Return true if the queue has at least one entry ready to transmit. */
bool frame_queue_has_ready(const frame_queue_t *q, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* FRAME_QUEUE_H */

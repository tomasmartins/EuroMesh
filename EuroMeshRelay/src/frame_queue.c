#include "frame_queue.h"

/* Slot index ranges per priority */
#define HIGH_START   0U
#define HIGH_END     FRAME_QUEUE_HIGH_SIZE
#define MED_START    HIGH_END
#define MED_END      (MED_START + FRAME_QUEUE_MED_SIZE)
#define LOW_START    MED_END
#define LOW_END      (LOW_START + FRAME_QUEUE_LOW_SIZE)

void frame_queue_init(frame_queue_t *q)
{
    uint8_t i;

    if (q == NULL) {
        return;
    }
    for (i = 0U; i < FRAME_QUEUE_TOTAL_SIZE; ++i) {
        q->used[i] = 0U;
    }
}

bool frame_queue_push(frame_queue_t *q,
                      const emesh_frame_header_t *header,
                      const uint8_t *payload,
                      uint8_t payload_len,
                      frame_prio_t prio,
                      uint32_t next_tx_ms)
{
    uint8_t start;
    uint8_t end;
    uint8_t len;
    uint8_t i;

    if (q == NULL || header == NULL || (payload == NULL && payload_len > 0U)) {
        return false;
    }
    if (payload_len > FRAME_QUEUE_MAX_PAYLOAD) {
        return false;
    }

    switch (prio) {
    case FRAME_PRIO_HIGH:
        start = HIGH_START; end = HIGH_END;
        break;
    case FRAME_PRIO_MEDIUM:
        start = MED_START;  end = MED_END;
        break;
    case FRAME_PRIO_LOW:
    default:
        start = LOW_START;  end = LOW_END;
        break;
    }

    for (i = start; i < end; ++i) {
        if (q->used[i] == 0U) {
            frame_queue_entry_t *e = &q->slots[i];
            e->header      = *header;
            len = payload_len;
            for (uint8_t j = 0U; j < len; ++j) {
                e->payload[j] = payload[j];
            }
            e->payload_len = len;
            e->prio        = (uint8_t)prio;
            e->retry_count = 0U;
            e->enqueue_ms  = 0U; /* caller can set via the returned pointer if needed */
            e->next_tx_ms  = next_tx_ms;
            q->used[i]     = 1U;
            return true;
        }
    }
    return false; /* pool full */
}

frame_queue_entry_t *frame_queue_peek(frame_queue_t *q, uint32_t now_ms)
{
    /* Scan HIGH first, then MED, then LOW; within a priority pick first ready. */
    static const uint8_t starts[FRAME_PRIO_COUNT] = { HIGH_START, MED_START, LOW_START };
    static const uint8_t ends[FRAME_PRIO_COUNT]   = { HIGH_END,   MED_END,   LOW_END   };
    uint8_t p;
    uint8_t i;

    if (q == NULL) {
        return NULL;
    }

    for (p = 0U; p < FRAME_PRIO_COUNT; ++p) {
        for (i = starts[p]; i < ends[p]; ++i) {
            if (q->used[i] == 0U) {
                continue;
            }
            /* Ready if next_tx_ms == 0 or time has passed (unsigned subtraction). */
            if (q->slots[i].next_tx_ms == 0U
                    || (now_ms - q->slots[i].next_tx_ms) < 0x80000000U) {
                return &q->slots[i];
            }
        }
    }
    return NULL;
}

void frame_queue_remove(frame_queue_t *q, frame_queue_entry_t *entry)
{
    uint8_t i;

    if (q == NULL || entry == NULL) {
        return;
    }
    for (i = 0U; i < FRAME_QUEUE_TOTAL_SIZE; ++i) {
        if (&q->slots[i] == entry) {
            q->used[i] = 0U;
            return;
        }
    }
}

bool frame_queue_has_ready(const frame_queue_t *q, uint32_t now_ms)
{
    uint8_t i;

    if (q == NULL) {
        return false;
    }
    for (i = 0U; i < FRAME_QUEUE_TOTAL_SIZE; ++i) {
        if (q->used[i] == 0U) {
            continue;
        }
        if (q->slots[i].next_tx_ms == 0U
                || (now_ms - q->slots[i].next_tx_ms) < 0x80000000U) {
            return true;
        }
    }
    return false;
}

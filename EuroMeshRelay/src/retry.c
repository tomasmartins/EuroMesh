#include "retry.h"
#include "emesh_node_caps.h"

void retry_init(retry_table_t *tbl)
{
    uint8_t i;

    if (tbl == NULL) {
        return;
    }
    for (i = 0U; i < RETRY_TABLE_SIZE; ++i) {
        tbl->slots[i].active = false;
    }
}

bool retry_arm(retry_table_t *tbl,
               const emesh_frame_header_t *header,
               const uint8_t *payload,
               uint8_t payload_len,
               uint32_t now_ms)
{
    uint8_t i;

    if (tbl == NULL || header == NULL) {
        return false;
    }
    if (payload_len > FRAME_QUEUE_MAX_PAYLOAD) {
        return false;
    }
    /* Broadcasts do not need retry tracking. */
    if (header->dest_id == EMESH_DEST_BROADCAST) {
        return true;
    }

    for (i = 0U; i < RETRY_TABLE_SIZE; ++i) {
        if (!tbl->slots[i].active) {
            retry_entry_t *s = &tbl->slots[i];
            s->active          = true;
            s->attempts        = 0U;
            s->header          = *header;
            s->payload_len     = payload_len;
            for (uint8_t j = 0U; j < payload_len; ++j) {
                s->payload[j] = payload[j];
            }
            s->ack_deadline_ms = now_ms + RETRY_ACK_TIMEOUT_MS;
            return true;
        }
    }
    return false; /* all slots occupied */
}

bool retry_on_ack(retry_table_t *tbl, uint32_t ack_from, uint16_t acked_seq)
{
    uint8_t i;

    if (tbl == NULL) {
        return false;
    }
    for (i = 0U; i < RETRY_TABLE_SIZE; ++i) {
        retry_entry_t *s = &tbl->slots[i];
        if (s->active
                && s->header.dest_id == ack_from
                && s->header.seq     == acked_seq) {
            s->active = false;
            return true;
        }
    }
    return false;
}

void retry_tick(retry_table_t *tbl, csma_mac_t *mac,
                uint32_t now_ms, uint32_t *failed_dest)
{
    uint8_t i;

    if (failed_dest != NULL) {
        *failed_dest = 0U;
    }
    if (tbl == NULL || mac == NULL) {
        return;
    }

    for (i = 0U; i < RETRY_TABLE_SIZE; ++i) {
        retry_entry_t *s = &tbl->slots[i];

        if (!s->active) {
            continue;
        }
        /* Deadline not yet reached — unsigned comparison handles wrap-around. */
        if ((now_ms - s->ack_deadline_ms) >= 0x80000000U) {
            continue;
        }

        /* ACK deadline expired. */
        if (s->attempts >= RETRY_MAX_ATTEMPTS) {
            /* Delivery failed — notify caller. */
            if (failed_dest != NULL) {
                *failed_dest = s->header.dest_id;
            }
            s->active = false;
            continue;
        }

        /* Retransmit. */
        s->attempts++;
        (void)csma_mac_send(mac, &s->header, s->payload, s->payload_len);

        /* Exponential back-off: base × 2^attempts, capped at max. */
        {
            uint32_t backoff = RETRY_BACKOFF_BASE_MS;
            uint8_t  shift   = s->attempts;
            while (shift > 0U && backoff < RETRY_BACKOFF_MAX_MS) {
                backoff <<= 1;
                shift--;
            }
            if (backoff > RETRY_BACKOFF_MAX_MS) {
                backoff = RETRY_BACKOFF_MAX_MS;
            }
            s->ack_deadline_ms = now_ms + backoff;
        }
    }
}

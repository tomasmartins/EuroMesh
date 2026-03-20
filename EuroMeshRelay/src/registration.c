#include "registration.h"
#include "emesh_packet_types.h"
#include "emesh_node_caps.h"
#include "beacon_packet.h"

#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Codec — SUBSCRIPTION request
 * ═══════════════════════════════════════════════════════════════════════════ */

uint8_t sub_request_encode(const sub_request_t *req, uint8_t *buf, uint8_t capacity)
{
    if (req == NULL || buf == NULL || capacity < SUB_REQUEST_PAYLOAD_SIZE) {
        return 0;
    }
    buf[0] = EMESH_PACKET_TYPE_SUBSCRIPTION;
    buf[1] = req->capability;
    buf[2] = req->requested_slots;
    buf[3] = (uint8_t)req->heard_rssi_dbm; /* int8 → uint8 two's complement */
    buf[4] = (uint8_t)(req->seq);
    buf[5] = (uint8_t)(req->seq >> 8);
    return SUB_REQUEST_PAYLOAD_SIZE;
}

bool sub_request_decode(const uint8_t *buf, uint8_t length, sub_request_t *out)
{
    if (buf == NULL || out == NULL || length < SUB_REQUEST_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_SUBSCRIPTION) {
        return false;
    }
    out->capability      = buf[1];
    out->requested_slots = buf[2];
    out->heard_rssi_dbm  = (int8_t)buf[3];
    out->seq             = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Codec — REG_RESPONSE
 * ═══════════════════════════════════════════════════════════════════════════ */

uint8_t reg_response_encode(const reg_response_t *resp, uint8_t *buf, uint8_t capacity)
{
    if (resp == NULL || buf == NULL || capacity < REG_RESPONSE_PAYLOAD_SIZE) {
        return 0;
    }
    buf[0] = EMESH_PACKET_TYPE_REG_RESPONSE;
    buf[1] = resp->status;
    buf[2] = (uint8_t)(resp->pan_id);
    buf[3] = (uint8_t)(resp->pan_id >> 8);
    buf[4] = resp->stratum;
    buf[5] = resp->nb_adv_interval_s;
    buf[6] = resp->beacon_slot;  /* TDMA beacon slot, 0xFF = none assigned  */
    buf[7] = 0x00U;              /* reserved                                */
    return REG_RESPONSE_PAYLOAD_SIZE;
}

bool reg_response_decode(const uint8_t *buf, uint8_t length, reg_response_t *out)
{
    if (buf == NULL || out == NULL || length < REG_RESPONSE_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_REG_RESPONSE) {
        return false;
    }
    out->status              = buf[1];
    out->pan_id              = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    out->stratum             = buf[4];
    out->nb_adv_interval_s   = buf[5];
    out->beacon_slot         = buf[6]; /* TDMA_NO_SLOT_ASSIGNED (0xFF) = none */
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Node-side state machine
 * ═══════════════════════════════════════════════════════════════════════════ */

void reg_ctx_init(reg_ctx_t *ctx, uint8_t capability)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->state      = REG_STATE_UNREGISTERED;
    ctx->capability = capability;
    ctx->best_rssi_dbm = -128;
}

void reg_ctx_tick(reg_ctx_t *ctx,
                  csma_mac_t *mac,
                  uint32_t    my_id,
                  bool        reg_window_open,
                  uint32_t    reg_slot_start_ms,
                  int8_t      best_beacon_rssi,
                  uint32_t    parent_id,
                  uint8_t     parent_stratum,
                  uint32_t    now_ms)
{
    sub_request_t        req;
    uint8_t              payload[SUB_REQUEST_PAYLOAD_SIZE];
    emesh_frame_header_t hdr;

    if (ctx == NULL || mac == NULL) {
        return;
    }

    /* Update the best-beacon RSSI hint regardless of state. */
    if (best_beacon_rssi > ctx->best_rssi_dbm) {
        ctx->best_rssi_dbm = best_beacon_rssi;
        ctx->parent_id     = parent_id;
        ctx->parent_stratum = parent_stratum;
    }

    /* Check if a PENDING request has timed out. */
    if (ctx->state == REG_STATE_PENDING || ctx->state == REG_STATE_REREGISTERING) {
        if ((now_ms - ctx->pending_deadline_ms) < 0x80000000U) {
            /* Deadline passed — increment attempt counter, revert to try again. */
            ctx->attempt_count++;
            if (ctx->attempt_count >= REG_MAX_ATTEMPTS) {
                /* Give up on this relay — start fresh. */
                ctx->state         = REG_STATE_UNREGISTERED;
                ctx->attempt_count = 0U;
                ctx->best_rssi_dbm = -128;
                ctx->parent_id     = 0U;
            } else {
                ctx->state = REG_STATE_UNREGISTERED;
            }
        } else {
            /* Still waiting for response — do nothing this frame. */
            return;
        }
    }

    if (ctx->state == REG_STATE_REGISTERED) {
        return; /* Nothing to do. */
    }

    /* UNREGISTERED: send a request if the registration window is open. */
    if (!reg_window_open || ctx->parent_id == 0U) {
        return;
    }

    req.capability      = ctx->capability;
    req.requested_slots = 0U;  /* CSMA/CAD — no slot needed */
    req.heard_rssi_dbm  = ctx->best_rssi_dbm;
    ctx->req_seq++;
    req.seq             = ctx->req_seq;

    memset(&hdr, 0, sizeof(hdr));
    hdr.type    = EMESH_PACKET_TYPE_SUBSCRIPTION;
    hdr.flags   = (ctx->state == REG_STATE_REREGISTERING)
                  ? EMESH_FRAME_FLAG_REREGISTER : 0U;
    hdr.ttl     = 1U;          /* registration is single-hop only */
    hdr.src_id  = my_id;
    hdr.dest_id = ctx->parent_id;
    hdr.seq     = ctx->req_seq;
    hdr.op      = EMESH_OP_MESH_REGISTER;
    hdr.length  = SUB_REQUEST_PAYLOAD_SIZE;

    if (sub_request_encode(&req, payload, sizeof(payload)) == 0U) {
        return;
    }

    /* Wait for the registration slot then transmit (CSMA/CAD). */
    (void)csma_mac_send_at(mac, &hdr, payload, SUB_REQUEST_PAYLOAD_SIZE,
                           reg_slot_start_ms);

    ctx->state               = REG_STATE_PENDING;
    ctx->last_attempt_ms     = now_ms;
    ctx->pending_deadline_ms = now_ms + REG_RESPONSE_WAIT_MS;
}

bool reg_ctx_handle_response(reg_ctx_t *ctx,
                             const emesh_frame_header_t *hdr,
                             const uint8_t *payload,
                             uint8_t payload_len,
                             uint32_t now_ms)
{
    reg_response_t resp;

    if (ctx == NULL || hdr == NULL || payload == NULL) {
        return false;
    }

    /* Only accept responses from our chosen parent. */
    if (hdr->src_id != ctx->parent_id) {
        return false;
    }
    if (ctx->state != REG_STATE_PENDING && ctx->state != REG_STATE_REREGISTERING) {
        return false;
    }
    if (!reg_response_decode(payload, payload_len, &resp)) {
        return false;
    }

    if (resp.status == REG_STATUS_OK) {
        ctx->state              = REG_STATE_REGISTERED;
        ctx->pan_id             = resp.pan_id;
        ctx->parent_stratum     = resp.stratum;
        ctx->nb_adv_interval_s  = resp.nb_adv_interval_s;
        ctx->attempt_count      = 0U;
        (void)now_ms;
        return true;
    }

    if (resp.status == REG_STATUS_FULL) {
        /* Relay is full — reset and try again next window. */
        ctx->state         = REG_STATE_UNREGISTERED;
        ctx->parent_id     = 0U;
        ctx->best_rssi_dbm = -128;
        ctx->attempt_count = 0U;
    } else {
        /* Rejected — increment attempts and retry. */
        ctx->state = REG_STATE_UNREGISTERED;
        ctx->attempt_count++;
    }
    return false;
}

void reg_ctx_reregister(reg_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    if (ctx->state == REG_STATE_REGISTERED) {
        ctx->state         = REG_STATE_REREGISTERING;
        ctx->attempt_count = 0U;
        ctx->best_rssi_dbm = -128; /* re-scan for best relay */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Relay / Gateway side
 * ═══════════════════════════════════════════════════════════════════════════ */

void reg_relay_init(reg_relay_t *relay, uint16_t pan_id,
                    uint8_t local_stratum, uint8_t nb_adv_interval_s,
                    uint8_t my_beacon_slot)
{
    if (relay == NULL) {
        return;
    }
    memset(relay, 0, sizeof(*relay));
    relay->pan_id            = pan_id;
    relay->local_stratum     = local_stratum;
    relay->nb_adv_interval_s = nb_adv_interval_s;
    relay->my_beacon_slot    = my_beacon_slot;
    /*
     * Downstream relays are allocated slots starting after this relay's own
     * slot.  If this relay is the gateway (slot 0), downstream relays start
     * at slot 1.
     */
    relay->next_relay_slot   = (uint8_t)(my_beacon_slot + 1U);
}

/* Find an existing entry index, or REG_MAX_NODES if not found. */
static uint8_t reg_relay_find(const reg_relay_t *relay, uint32_t node_id)
{
    uint8_t i;
    for (i = 0U; i < relay->count; ++i) {
        if (relay->nodes[i].node_id == node_id) {
            return i;
        }
    }
    return REG_MAX_NODES;
}

void reg_relay_handle_request(reg_relay_t *relay,
                              csma_mac_t  *mac,
                              uint32_t     my_id,
                              const emesh_frame_header_t *req_hdr,
                              const uint8_t *req_payload,
                              uint8_t        req_payload_len,
                              int16_t        req_rssi_dbm,
                              uint32_t       now_ms)
{
    sub_request_t        req;
    reg_response_t       resp;
    uint8_t              resp_payload[REG_RESPONSE_PAYLOAD_SIZE];
    emesh_frame_header_t resp_hdr;
    uint8_t              idx;
    static uint16_t      resp_seq = 0U;

    if (relay == NULL || mac == NULL || req_hdr == NULL || req_payload == NULL) {
        return;
    }
    if (!sub_request_decode(req_payload, req_payload_len, &req)) {
        return;
    }

    idx = reg_relay_find(relay, req_hdr->src_id);

    if (idx == REG_MAX_NODES) {
        /* New node */
        if (relay->count >= REG_MAX_NODES) {
            /* Table full */
            resp.status            = REG_STATUS_FULL;
            resp.pan_id            = relay->pan_id;
            resp.stratum           = relay->local_stratum;
            resp.nb_adv_interval_s = relay->nb_adv_interval_s;
            resp.beacon_slot       = TDMA_NO_SLOT_ASSIGNED;
        } else {
            reg_node_entry_t *e = &relay->nodes[relay->count];
            e->node_id        = req_hdr->src_id;
            e->capability     = req.capability;
            e->last_rssi_dbm  = req_rssi_dbm;
            e->last_seen_ms   = now_ms;
            e->registered_ms  = now_ms;

            /*
             * Assign a TDMA beacon slot if the registering node is a relay
             * (it needs to beacon).  Leaf nodes (EMESH_NODE_TIER_NODE) do not
             * transmit beacons and receive TDMA_NO_SLOT_ASSIGNED.
             *
             * The relay itself occupies its own slot (relay->my_beacon_slot).
             * Downstream relays receive sequentially allocated slots starting
             * from relay->next_relay_slot.
             */
            if (EMESH_NODE_TIER(req.capability) == EMESH_NODE_TIER_RELAY) {
                if (relay->next_relay_slot < TDMA_MAX_BEACON_SLOTS) {
                    e->beacon_slot = relay->next_relay_slot++;
                } else {
                    e->beacon_slot = TDMA_NO_SLOT_ASSIGNED; /* all slots used */
                }
            } else {
                e->beacon_slot = TDMA_NO_SLOT_ASSIGNED; /* leaf node — no beacon */
            }

            relay->count++;

            resp.status            = REG_STATUS_OK;
            resp.pan_id            = relay->pan_id;
            resp.stratum           = relay->local_stratum;
            resp.nb_adv_interval_s = relay->nb_adv_interval_s;
            resp.beacon_slot       = e->beacon_slot;
        }
    } else {
        /* Existing node — refresh entry, keep existing slot assignment. */
        relay->nodes[idx].capability    = req.capability;
        relay->nodes[idx].last_rssi_dbm = req_rssi_dbm;
        relay->nodes[idx].last_seen_ms  = now_ms;

        resp.status            = REG_STATUS_OK;
        resp.pan_id            = relay->pan_id;
        resp.stratum           = relay->local_stratum;
        resp.nb_adv_interval_s = relay->nb_adv_interval_s;
        resp.beacon_slot       = relay->nodes[idx].beacon_slot;
    }

    /* Build and transmit REG_RESPONSE (unicast back to requesting node). */
    if (reg_response_encode(&resp, resp_payload, sizeof(resp_payload)) == 0U) {
        return;
    }

    resp_seq++;
    memset(&resp_hdr, 0, sizeof(resp_hdr));
    resp_hdr.type    = EMESH_PACKET_TYPE_REG_RESPONSE;
    resp_hdr.flags   = 0U;
    resp_hdr.ttl     = 1U;
    resp_hdr.src_id  = my_id;
    resp_hdr.dest_id = req_hdr->src_id;
    resp_hdr.seq     = resp_seq;
    resp_hdr.op      = EMESH_OP_MESH_REGISTER;
    resp_hdr.length  = REG_RESPONSE_PAYLOAD_SIZE;

    (void)csma_mac_send(mac, &resp_hdr, resp_payload, REG_RESPONSE_PAYLOAD_SIZE);
}

void reg_relay_expire(reg_relay_t *relay, uint32_t now_ms, uint32_t max_age_ms)
{
    uint8_t i;

    if (relay == NULL) {
        return;
    }
    i = 0U;
    while (i < relay->count) {
        if ((now_ms - relay->nodes[i].last_seen_ms) > max_age_ms) {
            relay->count--;
            relay->nodes[i] = relay->nodes[relay->count];
            /* Re-check the swapped entry. */
        } else {
            i++;
        }
    }
}

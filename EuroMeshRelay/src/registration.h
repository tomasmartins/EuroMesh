/*
 * registration.h — Network registration protocol.
 *
 * Node side:
 *   reg_ctx_t tracks the per-node state machine:
 *     UNREGISTERED → PENDING → REGISTERED → REREGISTERING (mobile)
 *
 *   Call reg_ctx_tick() every super-frame beacon window.  When the node
 *   hears a beacon with net_flags BEACON_NET_FLAG_REG_OPEN, reg_ctx_tick()
 *   transmits a SUBSCRIPTION request in the registration slot and waits for
 *   a REG_RESPONSE.
 *
 * Relay / Gateway side:
 *   reg_relay_t holds the registered-node table and processes incoming
 *   SUBSCRIPTION requests.  On receipt it validates, assigns the node to
 *   the network, and unicasts a REG_RESPONSE back.
 *
 * ── SUBSCRIPTION request payload (6 bytes) ──────────────────────────────
 *   [type:1][capability:1][req_slots:1][heard_rssi:1][seq:2LE]
 *
 * ── REG_RESPONSE payload (8 bytes) ──────────────────────────────────────
 *   [type:1][status:1][pan_id:2LE][stratum:1][nb_adv_interval_s:1][_:2]
 */

#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <stdbool.h>
#include <stdint.h>

#include "csma_mac.h"
#include "emesh_frame.h"
#include "tdma.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Wire format sizes ───────────────────────────────────────────────────── */
#define SUB_REQUEST_PAYLOAD_SIZE   6U
#define REG_RESPONSE_PAYLOAD_SIZE  8U

/* ── REG_RESPONSE status codes ───────────────────────────────────────────── */
#define REG_STATUS_OK        0x00U
#define REG_STATUS_REJECTED  0x01U
#define REG_STATUS_FULL      0x02U

/* ── Retry config ────────────────────────────────────────────────────────── */
#define REG_MAX_ATTEMPTS      5U     /* before giving up on current relay   */
#define REG_RESPONSE_WAIT_MS  400U   /* time to listen for REG_RESPONSE     */

/* ── Sub-request codec structs ───────────────────────────────────────────── */
typedef struct {
    uint8_t  capability;      /* EMESH_NODE_TIER_* | EMESH_NODE_FLAG_*   */
    uint8_t  requested_slots; /* always 0 — CSMA/CAD needs no slot       */
    int8_t   heard_rssi_dbm;  /* RSSI of best beacon heard               */
    uint16_t seq;             /* request sequence number                  */
} sub_request_t;

typedef struct {
    uint8_t  status;
    uint16_t pan_id;
    uint8_t  stratum;
    uint8_t  nb_adv_interval_s; /* 0 = no advertisements                   */
    uint8_t  beacon_slot;       /* TDMA beacon slot assigned to this node;  */
                                /* TDMA_NO_SLOT_ASSIGNED (0xFF) if none     */
} reg_response_t;

uint8_t sub_request_encode(const sub_request_t *req, uint8_t *buf, uint8_t capacity);
bool    sub_request_decode(const uint8_t *buf, uint8_t length, sub_request_t *out);

uint8_t reg_response_encode(const reg_response_t *resp, uint8_t *buf, uint8_t capacity);
bool    reg_response_decode(const uint8_t *buf, uint8_t length, reg_response_t *out);

/* ═══════════════════════════════════════════════════════════════════════════
 * NODE SIDE
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    REG_STATE_UNREGISTERED  = 0,
    REG_STATE_PENDING,        /* request sent, awaiting REG_RESPONSE      */
    REG_STATE_REGISTERED,
    REG_STATE_REREGISTERING,  /* mobile node triggered re-registration    */
} reg_state_t;

typedef struct {
    reg_state_t state;
    uint8_t     capability;         /* our own tier + mobility flags       */
    uint8_t     attempt_count;
    uint8_t     _pad;
    uint16_t    pan_id;             /* assigned PAN ID                     */
    uint8_t     parent_stratum;     /* relay/GW stratum we registered to   */
    uint8_t     nb_adv_interval_s;
    uint32_t    parent_id;          /* relay/GW we registered with         */
    uint32_t    last_attempt_ms;    /* tick of last SUB_REQUEST TX         */
    uint32_t    pending_deadline_ms;/* tick after which PENDING → retry    */
    uint16_t    req_seq;            /* incremented on each request         */
    int8_t      best_rssi_dbm;      /* RSSI of best beacon heard           */
} reg_ctx_t;

void reg_ctx_init(reg_ctx_t *ctx, uint8_t capability);

/*
 * Drive the node-side registration state machine.
 * Call once per super-frame, after receiving (or missing) the beacon.
 *
 * reg_window_open: true if the beacon's net_flags had BEACON_NET_FLAG_REG_OPEN.
 * reg_slot_start_ms: absolute tick at which the registration slot begins.
 * best_beacon_rssi: RSSI of the beacon just received (or INT8_MIN if missed).
 * parent_id: src_id of the beacon sender.
 * parent_stratum: stratum field from the beacon.
 */
void reg_ctx_tick(reg_ctx_t *ctx,
                  csma_mac_t *mac,
                  uint32_t    my_id,
                  bool        reg_window_open,
                  uint32_t    reg_slot_start_ms,
                  int8_t      best_beacon_rssi,
                  uint32_t    parent_id,
                  uint8_t     parent_stratum,
                  uint32_t    now_ms);

/*
 * Call when a REG_RESPONSE frame is received addressed to this node.
 * Returns true if the response was accepted and state transitioned.
 */
bool reg_ctx_handle_response(reg_ctx_t *ctx,
                             const emesh_frame_header_t *hdr,
                             const uint8_t *payload,
                             uint8_t payload_len,
                             uint32_t now_ms);

/* Trigger re-registration (e.g., on significant node movement). */
void reg_ctx_reregister(reg_ctx_t *ctx);

static inline bool reg_ctx_is_registered(const reg_ctx_t *ctx)
{
    return (ctx != NULL) && (ctx->state == REG_STATE_REGISTERED);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RELAY / GATEWAY SIDE
 * ═══════════════════════════════════════════════════════════════════════════ */

#define REG_MAX_NODES  16U

typedef struct {
    uint32_t node_id;
    uint8_t  capability;
    uint8_t  beacon_slot;     /* TDMA slot assigned to this node             */
    uint8_t  _pad[2];
    int16_t  last_rssi_dbm;
    uint32_t last_seen_ms;
    uint32_t registered_ms;
} reg_node_entry_t;           /* 16 bytes × 16 = 256 bytes */

typedef struct {
    reg_node_entry_t nodes[REG_MAX_NODES];
    uint8_t          count;
    uint8_t          my_beacon_slot;     /* this relay's own TDMA slot       */
    uint8_t          next_relay_slot;    /* next slot to assign to a relay   */
    uint8_t          nb_adv_interval_s;
    uint16_t         pan_id;
    uint8_t          local_stratum;
    uint8_t          _pad;
} reg_relay_t;

void reg_relay_init(reg_relay_t *relay, uint16_t pan_id,
                    uint8_t local_stratum, uint8_t nb_adv_interval_s,
                    uint8_t my_beacon_slot);

/*
 * Process an incoming SUBSCRIPTION request.
 * If valid, inserts/updates the node registry, builds a REG_RESPONSE,
 * and unicasts it back via mac.
 * my_id is the relay's own src_id (used as frame header src).
 */
void reg_relay_handle_request(reg_relay_t *relay,
                              csma_mac_t  *mac,
                              uint32_t     my_id,
                              const emesh_frame_header_t *req_hdr,
                              const uint8_t *req_payload,
                              uint8_t        req_payload_len,
                              int16_t        req_rssi_dbm,
                              uint32_t       now_ms);

/*
 * Evict nodes that have not been seen for max_age_ms.
 * Call once per super-frame.
 */
void reg_relay_expire(reg_relay_t *relay, uint32_t now_ms, uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif

#endif /* REGISTRATION_H */

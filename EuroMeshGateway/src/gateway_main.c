/*
 * gateway_main.c — EuroMesh gateway application (Raspberry Pi + SX1301).
 *
 * Responsibilities
 * ────────────────
 * 1. Beacon TX on a UTC-aligned schedule.
 *    - Each gateway has a BEACON_SLOT (0, 1, 2, …).  Slot 0 fires at the
 *      start of each 60 s epoch boundary; slot N fires N×100 ms later.
 *    - This prevents two gateways that share LoRa coverage from transmitting
 *      at the same instant when both are NTP-disciplined.
 *    - Stratum: 0 (authoritative GPS/NTP reference).
 *    - Registration window open for the first REG_WINDOW_S of each period.
 *
 * 2. RX dispatch loop (runs between beacons):
 *    - SUBSCRIPTION  → accepted only during reg window; update node table,
 *                       send REG_RESPONSE.
 *    - TIME_REQ      → record T2, reply with TIME_RESP (T1=0, T2, T3).
 *    - BEACON        → if from another gateway, update the LoRa peer table
 *                       and detect beacon-slot collisions.
 *    - All others    → logged and discarded.
 *
 * 3. Node table expiry: nodes not seen for REG_NODE_EXPIRY_S are evicted.
 *
 * 4. Peer gateway table expiry: gateways silent for GW_PEER_TIMEOUT_S are
 *    removed.
 *
 * Beacon slot selection
 * ─────────────────────
 * Set the EUROMESH_BEACON_SLOT environment variable (0-7) before starting.
 * Default is 0.  When a collision with a peer is detected the gateway shifts
 * to the next free slot automatically and logs a warning.
 *
 * Time model
 * ──────────
 * The gateway is stratum 0 — it claims to BE the GPS source.  This is
 * correct when the RPi is GPS-disciplined (e.g., gpsd + chrony/PPS).  When
 * running with plain NTP (no PPS), the beacon is still stratum 0 but with
 * ~10 ms accuracy.
 *
 * SIGINT / SIGTERM
 * ────────────────
 * The main loop exits cleanly on SIGINT or SIGTERM, stopping the SX1301.
 */

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "beacon_packet.h"
#include "emesh_frame.h"
#include "emesh_node_caps.h"
#include "emesh_packet_types.h"
#include "gw_lora_peers.h"
#include "gw_registration.h"
#include "lgw_hal.h"
#include "time_sync.h"       /* TIME_SYNC_FLAG_* constants only */
#include "time_twoway.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Beacon period (seconds). */
#define BEACON_INTERVAL_S    60U

/*
 * Registration window: the beacon includes BEACON_NET_FLAG_REG_OPEN for the
 * first REG_WINDOW_S seconds of each beacon interval.  Relays and nodes
 * must transmit their SUBSCRIPTION request within this window.
 * SUBSCRIPTION frames received outside this window are discarded.
 */
#define REG_WINDOW_S         10U

/* Evict nodes silent for longer than this. */
#define REG_NODE_EXPIRY_S  3600U

/* Maximum registered nodes. */
#define REG_MAX_NODES        16U

/* Gateway PAN identifier. */
#define GATEWAY_PAN_ID     0x4555U  /* ASCII 'EU' */

/*
 * Neighbour advertisement interval advertised to registered nodes.
 * 0 = no advertisements (gateway does not send NB_ADV).
 */
#define GATEWAY_NB_ADV_S      0U

/* Frame TTL for gateway-originated frames. */
#define GATEWAY_TTL           7U

/* RX poll interval when idle (microseconds). */
#define RX_POLL_INTERVAL_US   5000U  /* 5 ms */

/* Capability advertised by this gateway. */
#define GATEWAY_CAPABILITY  (EMESH_NODE_TIER_GATEWAY | EMESH_NODE_FLAG_GPS)

/*
 * TDMA slot duration (ms).  Must match TDMA_SLOT_MS in the relay firmware
 * (EuroMeshRelay/src/tdma.h).  Each gateway occupies one beacon slot; slot 0
 * is the default.  Additional gateways should use slot 1, 2, etc.
 */
#define TDMA_SLOT_MS          100U

/* Maximum beacon slot index (0-based; slot 0 reserved for gateway 0). */
#define TDMA_MAX_BEACON_SLOTS   8U

/*
 * Collision window (ms): if a peer gateway's beacon is received within this
 * interval of our own beacon we consider the slots to be colliding.
 */
#define GW_COLLISION_WINDOW_MS  (TDMA_SLOT_MS / 2U)  /* 50 ms */

/* Peer gateway timeout: remove if not heard for this many seconds. */
#define GW_PEER_TIMEOUT_S     (BEACON_INTERVAL_S * 3U)  /* 3 missed beacons */

/* ═══════════════════════════════════════════════════════════════════════════
 * Node registry
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * TDMA slot allocation.
 * Gateway occupies its own beacon slot.  Registering relays are assigned
 * relay slots starting at TDMA_RELAY_SLOT_BASE.
 * Leaf nodes (EMESH_NODE_TIER_NODE) receive TDMA_NO_SLOT_ASSIGNED (0xFF).
 */
#define TDMA_NO_SLOT_ASSIGNED    0xFFU
#define TDMA_GATEWAY_SLOT        0U
#define TDMA_RELAY_SLOT_BASE     1U   /* relay slots start here */

typedef struct {
    uint32_t node_id;
    uint8_t  capability;
    uint8_t  beacon_slot;     /* TDMA beacon slot, 0xFF = none */
    uint8_t  hop_count;       /* 1 = direct, 2+ = via relay(s) */
    uint8_t  _pad;
    float    last_rssi_dbm;
    time_t   first_seen;
    time_t   last_seen;
    time_t   last_reg;        /* timestamp of last SUBSCRIPTION */
} gw_node_entry_t;

static gw_node_entry_t g_nodes[REG_MAX_NODES];
static uint8_t         g_node_count      = 0U;
/* Next relay slot to allocate (starts at TDMA_RELAY_SLOT_BASE). */
static uint8_t         g_next_relay_slot = TDMA_RELAY_SLOT_BASE;

/* ── Look-up ─────────────────────────────────────────────────────────────── */

static uint8_t node_find(uint32_t node_id)
{
    uint8_t i;
    for (i = 0U; i < g_node_count; i++) {
        if (g_nodes[i].node_id == node_id) {
            return i;
        }
    }
    return REG_MAX_NODES; /* not found */
}

/* ── Expiry ──────────────────────────────────────────────────────────────── */

static void node_expire(void)
{
    time_t now = time(NULL);
    uint8_t i  = 0U;

    while (i < g_node_count) {
        if ((now - g_nodes[i].last_seen) > (time_t)REG_NODE_EXPIRY_S) {
            printf("[GW] Evicted node 0x%08X (silent for %lds)\n",
                   g_nodes[i].node_id,
                   (long)(now - g_nodes[i].last_seen));
            g_node_count--;
            g_nodes[i] = g_nodes[g_node_count]; /* swap-remove */
            /* re-check swapped entry in same slot */
        } else {
            i++;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Beacon slot state
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * The active beacon slot for this gateway.  Initialised from the
 * EUROMESH_BEACON_SLOT environment variable (default 0).  May be bumped
 * automatically when a collision with a peer gateway is detected.
 */
static uint8_t g_my_beacon_slot = 0U;

/* UTC time (ms) of our last transmitted beacon. */
static uint64_t g_last_beacon_utc_ms = 0U;

/* ═══════════════════════════════════════════════════════════════════════════
 * Platform time helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/* UTC milliseconds since the Unix epoch (CLOCK_REALTIME). */
static uint64_t utc_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL
         + (uint64_t)(ts.tv_nsec / 1000000L);
}

/* Monotonic millisecond counter — for interval timing that must not jump. */
static uint64_t mono_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL
         + (uint64_t)(ts.tv_nsec / 1000000L);
}

/*
 * Compute the next UTC time (ms) at which this gateway should transmit its
 * beacon, given its assigned beacon slot.
 *
 * All gateways align to the same 60 s epoch boundaries derived from
 * CLOCK_REALTIME (NTP-disciplined wall clock).  Slot N fires N×TDMA_SLOT_MS
 * milliseconds after each boundary.  This guarantees non-overlapping
 * transmissions for gateways that share LoRa coverage, as long as their
 * clocks are within a few milliseconds of each other (typical for NTP).
 */
static uint64_t next_beacon_utc_ms(uint8_t slot)
{
    uint64_t now        = utc_now_ms();
    uint64_t period_ms  = (uint64_t)BEACON_INTERVAL_S * 1000ULL;
    uint64_t offset_ms  = (uint64_t)slot * (uint64_t)TDMA_SLOT_MS;

    /* Align to the current period boundary. */
    uint64_t boundary   = (now / period_ms) * period_ms;
    uint64_t candidate  = boundary + offset_ms;

    /* If the candidate is in the past (or right now), advance one period. */
    if (candidate <= now) {
        candidate += period_ms;
    }

    return candidate;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Sequence counters
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t g_beacon_seq   = 0U;
static uint16_t g_resp_seq     = 0U;

/* ═══════════════════════════════════════════════════════════════════════════
 * Beacon TX
 * ═══════════════════════════════════════════════════════════════════════════ */

static void send_beacon(uint32_t my_id, uint64_t boot_mono_ms)
{
    uint8_t              frame[EMESH_FRAME_HEADER_SIZE + BEACON_PAYLOAD_SIZE];
    emesh_frame_header_t hdr;
    beacon_payload_t     bp;
    uint64_t             now_ms_utc;
    uint64_t             now_ms_mono;

    memset(&hdr, 0, sizeof(hdr));
    memset(&bp,  0, sizeof(bp));

    now_ms_utc  = utc_now_ms();
    now_ms_mono = mono_now_ms();

    /*
     * The beacon always carries BEACON_NET_FLAG_REG_OPEN because it is only
     * ever transmitted at the start of the registration window.  Nodes know
     * the window duration (REG_WINDOW_S) and stop sending SUBSCRIPTION frames
     * after that.
     */
    bp.sync_flags   = TIME_SYNC_FLAG_UTC_VALID;
    bp.utc_epoch_ms = now_ms_utc;
    bp.pps_tick_ms  = 0U;
    bp.stratum      = EMESH_STRATUM_GPS;      /* 0 — authoritative             */
    bp.net_flags    = BEACON_NET_FLAG_REG_OPEN;

    bp.telem_flags       = BEACON_TELEM_FLAG_TX_PWR
                         | BEACON_TELEM_FLAG_NODES
                         | BEACON_TELEM_FLAG_UPTIME;
    bp.tx_power_dbm      = 14;
    bp.upstream_rssi_dbm = (int8_t)0x80;      /* no upstream (we are root)     */
    bp.node_count        = g_node_count;
    bp.uptime_s          = (uint32_t)((now_ms_mono - boot_mono_ms) / 1000ULL);

    hdr.type    = EMESH_PACKET_TYPE_BEACON;
    hdr.flags   = EMESH_FRAME_FLAG_BROADCAST;
    hdr.ttl     = GATEWAY_TTL;
    hdr.src_id  = my_id;
    hdr.dest_id = EMESH_DEST_BROADCAST;
    hdr.seq     = ++g_beacon_seq;
    hdr.op      = EMESH_OP_MESH_BEACON;
    hdr.length  = BEACON_PAYLOAD_SIZE;

    emesh_frame_encode_header(&hdr, frame);
    (void)beacon_payload_encode(&bp, frame + EMESH_FRAME_HEADER_SIZE,
                                BEACON_PAYLOAD_SIZE);

    if (lgw_hal_transmit(frame, (uint8_t)(EMESH_FRAME_HEADER_SIZE
                                          + BEACON_PAYLOAD_SIZE))) {
        g_last_beacon_utc_ms = now_ms_utc;
        printf("[GW] Beacon TX  seq=%-4u  slot=%u  utc=%llu ms  "
               "nodes=%u  uptime=%us  peers=%u\n",
               g_beacon_seq,
               g_my_beacon_slot,
               (unsigned long long)bp.utc_epoch_ms,
               g_node_count,
               bp.uptime_s,
               gw_lora_peers_count());
    } else {
        fprintf(stderr, "[GW] Beacon TX FAILED\n");
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SUBSCRIPTION handler
 * ═══════════════════════════════════════════════════════════════════════════ */

static void handle_subscription(uint32_t my_id,
                                 const emesh_frame_header_t *req_hdr,
                                 const uint8_t *payload,
                                 uint8_t payload_len,
                                 float rssi_dbm)
{
    sub_request_t        req;
    reg_response_t       resp;
    uint8_t              resp_payload[REG_RESPONSE_PAYLOAD_SIZE];
    uint8_t              resp_frame[EMESH_FRAME_HEADER_SIZE
                                    + REG_RESPONSE_PAYLOAD_SIZE];
    emesh_frame_header_t resp_hdr;
    uint8_t              idx;
    bool                 is_new;
    time_t               now = time(NULL);

    /*
     * Infer hop count from TTL decrement.
     *   GATEWAY_TTL = 7: node or relay sets TTL = 7 on first transmission.
     *   Each relay that forwards decrements TTL by 1.
     *   hop_count = GATEWAY_TTL - received_ttl  (minimum 0 = direct)
     *
     * Response TTL must be large enough to traverse the same number of hops
     * in the reverse direction: resp_ttl = hop_count + 1.
     */
    uint8_t hop_count = (req_hdr->ttl < (uint8_t)GATEWAY_TTL)
                        ? (uint8_t)(GATEWAY_TTL - req_hdr->ttl)
                        : 0U;
    uint8_t resp_ttl  = (uint8_t)(hop_count + 1U);

    if (!sub_request_decode(payload, payload_len, &req)) {
        fprintf(stderr, "[GW] Malformed SUBSCRIPTION from 0x%08X\n",
                req_hdr->src_id);
        return;
    }

    idx    = node_find(req_hdr->src_id);
    is_new = (idx == REG_MAX_NODES);

    memset(&resp, 0, sizeof(resp));
    resp.pan_id            = GATEWAY_PAN_ID;
    resp.stratum           = EMESH_STRATUM_GPS; /* relays become stratum 1   */
    resp.nb_adv_interval_s = GATEWAY_NB_ADV_S;

    if (is_new) {
        if (g_node_count >= REG_MAX_NODES) {
            resp.status      = REG_STATUS_FULL;
            resp.beacon_slot = TDMA_NO_SLOT_ASSIGNED;
            printf("[GW] SUBSCRIPTION from 0x%08X REJECTED (table full)\n",
                   req_hdr->src_id);
        } else {
            idx                        = g_node_count;
            g_nodes[idx].node_id       = req_hdr->src_id;
            g_nodes[idx].capability    = req.capability;
            g_nodes[idx].hop_count     = (uint8_t)(hop_count + 1U); /* total hops */
            g_nodes[idx].last_rssi_dbm = rssi_dbm;
            g_nodes[idx].first_seen    = now;
            g_nodes[idx].last_seen     = now;
            g_nodes[idx].last_reg      = now;

            /*
             * Assign a TDMA beacon slot to relay-tier nodes; leaf nodes do
             * not beacon and receive TDMA_NO_SLOT_ASSIGNED.
             */
            if ((req.capability & 0x03U) == (uint8_t)EMESH_NODE_TIER_RELAY) {
                if (g_next_relay_slot < TDMA_MAX_BEACON_SLOTS) {
                    g_nodes[idx].beacon_slot = g_next_relay_slot++;
                } else {
                    g_nodes[idx].beacon_slot = TDMA_NO_SLOT_ASSIGNED;
                }
            } else {
                g_nodes[idx].beacon_slot = TDMA_NO_SLOT_ASSIGNED;
            }

            g_node_count++;
            resp.status      = REG_STATUS_OK;
            resp.beacon_slot = g_nodes[idx].beacon_slot;
            printf("[GW] Registered   node 0x%08X  cap=0x%02X  "
                   "rssi=%.0f dBm  slot=%u  hops=%u  (total=%u)\n",
                   req_hdr->src_id, req.capability, rssi_dbm,
                   g_nodes[idx].beacon_slot, g_nodes[idx].hop_count,
                   g_node_count);
        }
    } else {
        g_nodes[idx].capability    = req.capability;
        g_nodes[idx].hop_count     = (uint8_t)(hop_count + 1U);
        g_nodes[idx].last_rssi_dbm = rssi_dbm;
        g_nodes[idx].last_seen     = now;
        g_nodes[idx].last_reg      = now;
        resp.status      = REG_STATUS_OK;
        resp.beacon_slot = g_nodes[idx].beacon_slot; /* keep existing slot  */
        printf("[GW] Re-registered node 0x%08X  cap=0x%02X  "
               "rssi=%.0f dBm  slot=%u  hops=%u\n",
               req_hdr->src_id, req.capability, rssi_dbm,
               g_nodes[idx].beacon_slot, g_nodes[idx].hop_count);
    }

    /* Build and send REG_RESPONSE. */
    if (reg_response_encode(&resp, resp_payload, sizeof(resp_payload)) == 0U) {
        return;
    }

    memset(&resp_hdr, 0, sizeof(resp_hdr));
    resp_hdr.type    = EMESH_PACKET_TYPE_REG_RESPONSE;
    resp_hdr.flags   = 0U;
    resp_hdr.ttl     = resp_ttl;     /* enough TTL to reach multi-hop nodes   */
    resp_hdr.src_id  = my_id;
    resp_hdr.dest_id = req_hdr->src_id;
    resp_hdr.seq     = ++g_resp_seq;
    resp_hdr.op      = EMESH_OP_MESH_REGISTER;
    resp_hdr.length  = REG_RESPONSE_PAYLOAD_SIZE;

    emesh_frame_encode_header(&resp_hdr, resp_frame);
    memcpy(resp_frame + EMESH_FRAME_HEADER_SIZE,
           resp_payload, REG_RESPONSE_PAYLOAD_SIZE);

    (void)lgw_hal_transmit(resp_frame, (uint8_t)(EMESH_FRAME_HEADER_SIZE
                                                  + REG_RESPONSE_PAYLOAD_SIZE));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TIME_REQ handler
 * ═══════════════════════════════════════════════════════════════════════════ */

static void handle_time_req(uint32_t my_id,
                             const emesh_frame_header_t *req_hdr,
                             const uint8_t *payload,
                             uint8_t payload_len,
                             uint64_t t2_ms)
{
    time_req_t           req;
    time_resp_t          resp;
    uint8_t              resp_payload[TIME_RESP_PAYLOAD_SIZE];
    uint8_t              resp_frame[EMESH_FRAME_HEADER_SIZE
                                    + TIME_RESP_PAYLOAD_SIZE];
    emesh_frame_header_t resp_hdr;

    if (!time_req_decode(payload, payload_len, &req)) {
        fprintf(stderr, "[GW] Malformed TIME_REQ from 0x%08X\n",
                req_hdr->src_id);
        return;
    }

    resp.req_seq = req.req_seq;
    resp.t1_ms   = 0U;         /* filled in by the requesting node             */
    resp.t2_ms   = t2_ms;
    resp.t3_ms   = utc_now_ms();

    if (time_resp_encode(&resp, resp_payload, sizeof(resp_payload)) == 0U) {
        return;
    }

    memset(&resp_hdr, 0, sizeof(resp_hdr));
    resp_hdr.type    = EMESH_PACKET_TYPE_TIME_RESP;
    resp_hdr.flags   = 0U;
    resp_hdr.ttl     = (uint8_t)((req_hdr->ttl < GATEWAY_TTL)
                                   ? (GATEWAY_TTL - req_hdr->ttl + 1U)
                                   : 1U);
    resp_hdr.src_id  = my_id;
    resp_hdr.dest_id = req_hdr->src_id;
    resp_hdr.seq     = ++g_resp_seq;
    resp_hdr.op      = EMESH_OP_MESH_TIME_RESP;
    resp_hdr.length  = TIME_RESP_PAYLOAD_SIZE;

    emesh_frame_encode_header(&resp_hdr, resp_frame);
    memcpy(resp_frame + EMESH_FRAME_HEADER_SIZE,
           resp_payload, TIME_RESP_PAYLOAD_SIZE);

    (void)lgw_hal_transmit(resp_frame, (uint8_t)(EMESH_FRAME_HEADER_SIZE
                                                  + TIME_RESP_PAYLOAD_SIZE));

    printf("[GW] TIME_RESP  → 0x%08X  t2=%llu ms  t3=%llu ms\n",
           req_hdr->src_id,
           (unsigned long long)resp.t2_ms,
           (unsigned long long)resp.t3_ms);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * DATA frame handler
 * ═══════════════════════════════════════════════════════════════════════════ */

static void handle_data(const emesh_frame_header_t *hdr,
                        const uint8_t *payload, uint8_t payload_len,
                        float rssi_dbm)
{
    uint8_t i;
    bool    is_text = true;

    printf("[GW] DATA from 0x%08X  rssi=%.0f dBm"
           "  op=0x%04X  seq=%u  len=%u\n",
           hdr->src_id, rssi_dbm, hdr->op, hdr->seq, payload_len);

    if (payload_len == 0U) {
        printf("[GW]   (empty payload)\n");
        return;
    }

    /* Detect printable ASCII (allow trailing NUL). */
    for (i = 0U; i < payload_len; i++) {
        uint8_t c = payload[i];
        if (c == 0U && i == (uint8_t)(payload_len - 1U)) break; /* trailing NUL */
        if (c < 0x20U || c > 0x7EU) { is_text = false; break; }
    }

    if (is_text) {
        /* Safe to print as string (NUL-terminate defensively). */
        char tmp[256];
        uint8_t copy_len = payload_len < 255U ? payload_len : 255U;
        memcpy(tmp, payload, copy_len);
        tmp[copy_len] = '\0';
        printf("[GW]   payload: \"%s\"\n", tmp);
    } else {
        printf("[GW]   payload hex:");
        for (i = 0U; i < payload_len; i++) printf(" %02X", payload[i]);
        printf("\n");
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RX dispatch
 * ═══════════════════════════════════════════════════════════════════════════ */

static void dispatch_frame(uint32_t my_id,
                            const uint8_t *raw, uint8_t raw_len,
                            const lgw_rx_meta_t *meta,
                            bool reg_window_open)
{
    emesh_frame_header_t hdr;
    const uint8_t       *payload;
    uint8_t              payload_len;
    uint64_t             t2_ms;

    if (raw_len < EMESH_FRAME_HEADER_SIZE) {
        return;
    }

    emesh_frame_decode_header(raw, &hdr);

    /*
     * Beacons from peer gateways are broadcast frames — process them even
     * though they are not addressed to us.  All other non-broadcast frames
     * that are not addressed to this gateway are ignored.
     */
    if (hdr.type != EMESH_PACKET_TYPE_BEACON
        && !emesh_frame_is_for_me(&hdr, my_id)) {
        return;
    }

    payload     = raw + EMESH_FRAME_HEADER_SIZE;
    payload_len = (uint8_t)(raw_len - EMESH_FRAME_HEADER_SIZE);

    if (payload_len < hdr.length) {
        return; /* truncated payload */
    }

    /* Capture T2 as close to RX as possible. */
    t2_ms = utc_now_ms();

    switch (hdr.type) {

    case EMESH_PACKET_TYPE_BEACON:
        {
            beacon_payload_t bp;
            bool is_gateway_peer = false;

            if (beacon_payload_decode(payload, payload_len, &bp)) {

                /*
                 * Determine if this beacon came from another gateway.
                 * The sender's tier is not directly in the beacon payload, but
                 * a gateway beacon is always:
                 *   - a broadcast frame
                 *   - with stratum 0 (EMESH_STRATUM_GPS) or low stratum
                 *   - and src_id != my_id
                 *
                 * A more reliable check uses the op-code class (EMESH_OP_MESH_BEACON),
                 * which every beacon carries regardless of tier.  We distinguish
                 * gateways from relays by stratum: stratum 0 is only claimed by
                 * gateways (relays always propagate a higher stratum).
                 */
                if (hdr.src_id != my_id && bp.stratum == EMESH_STRATUM_GPS) {
                    bool is_new = false;
                    gw_lora_peers_update(hdr.src_id,
                                         meta->rssi_dbm,
                                         meta->snr_db,
                                         bp.stratum,
                                         bp.node_count,
                                         t2_ms,
                                         &is_new);
                    is_gateway_peer = true;

                    if (is_new) {
                        printf("[GW] New peer GW 0x%08X  rssi=%.0f dBm  "
                               "snr=%.1f dB  nodes=%u  (total peers=%u)\n",
                               hdr.src_id, meta->rssi_dbm, meta->snr_db,
                               bp.node_count, gw_lora_peers_count());
                    } else {
                        printf("[GW] Peer GW    0x%08X  rssi=%.0f dBm  "
                               "snr=%.1f dB  nodes=%u\n",
                               hdr.src_id, meta->rssi_dbm, meta->snr_db,
                               bp.node_count);
                    }

                    /*
                     * Collision detection: if the peer's beacon arrived within
                     * GW_COLLISION_WINDOW_MS of our own last beacon, the two
                     * gateways are on the same slot.  Shift our slot up by one.
                     */
                    if (g_last_beacon_utc_ms > 0U
                        && gw_lora_peers_collision(g_last_beacon_utc_ms,
                                                    GW_COLLISION_WINDOW_MS)) {
                        uint8_t new_slot = (uint8_t)((g_my_beacon_slot + 1U)
                                                       % TDMA_MAX_BEACON_SLOTS);
                        printf("[GW] BEACON COLLISION with 0x%08X — shifting "
                               "from slot %u to slot %u\n",
                               hdr.src_id, g_my_beacon_slot, new_slot);
                        g_my_beacon_slot = new_slot;
                    }
                } else if (hdr.src_id != my_id) {
                    /* Relay beacon — informational only. */
                    printf("[GW] RX Beacon   from 0x%08X  rssi=%.0f dBm  "
                           "snr=%.1f dB  utc=%llu ms  stratum=%u  "
                           "nodes=%u  uptime=%us  reg_open=%d\n",
                           hdr.src_id, meta->rssi_dbm, meta->snr_db,
                           (unsigned long long)bp.utc_epoch_ms,
                           bp.stratum, bp.node_count, bp.uptime_s,
                           (bp.net_flags & BEACON_NET_FLAG_REG_OPEN) ? 1 : 0);
                }
                (void)is_gateway_peer;
            } else if (hdr.src_id != my_id) {
                printf("[GW] RX Beacon   from 0x%08X  rssi=%.0f dBm  "
                       "(malformed)\n", hdr.src_id, meta->rssi_dbm);
            }
        }
        break;

    case EMESH_PACKET_TYPE_SUBSCRIPTION:
        /*
         * Only accept registrations during the open window.
         * Frames arriving after the window closes are silently dropped —
         * the node will retry on the next beacon period.
         */
        if (!reg_window_open) {
            printf("[GW] SUBSCRIPTION from 0x%08X outside reg window — "
                   "ignored\n", hdr.src_id);
            break;
        }
        handle_subscription(my_id, &hdr, payload, hdr.length, meta->rssi_dbm);
        break;

    case EMESH_PACKET_TYPE_TIME_REQ:
        handle_time_req(my_id, &hdr, payload, hdr.length, t2_ms);
        break;

    case EMESH_PACKET_TYPE_DATA:
        handle_data(&hdr, payload, hdr.length, meta->rssi_dbm);
        break;

    case EMESH_PACKET_TYPE_ACK:
    case EMESH_PACKET_TYPE_NEIGHBOUR_ADV:
    case EMESH_PACKET_TYPE_REG_RESPONSE:
    case EMESH_PACKET_TYPE_TIME_RESP:
     /* Not handled at the gateway level — log only. */
    printf("[GW] RX type=0x%02X from 0x%08X  rssi=%.0f dBm (unhandled)\n", hdr.type, hdr.src_id, meta->rssi_dbm);
        break;

    default:
        printf("[GW] RX unknown type=0x%02X from 0x%08X  len=%u\n",
               hdr.type, hdr.src_id, hdr.length);
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Signal handling
 * ═══════════════════════════════════════════════════════════════════════════ */

static volatile int g_running = 1;

static void sig_handler(int signum)
{
    (void)signum;
    g_running = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Entry point
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char *argv[])
{
    uint32_t      my_id;
    uint64_t      next_beacon_ms;  /* UTC ms of next scheduled beacon          */
    uint64_t      reg_close_utc_ms; /* UTC ms when reg window closes           */
    uint64_t      last_expiry_ms;  /* monotonic ms of last periodic expiry     */
    uint64_t      last_peer_log_ms;/* monotonic ms of last peer table printout */
    uint8_t       rx_buf[256];
    uint8_t       rx_len;
    lgw_rx_meta_t rx_meta;
    bool          reg_window_open;
    const char   *slot_env;

    (void)argc;
    (void)argv;

    /* ── Signal handlers ─────────────────────────────────────────────────── */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── Beacon slot ─────────────────────────────────────────────────────── */
    slot_env = getenv("EUROMESH_BEACON_SLOT");
    if (slot_env != NULL) {
        int slot_val = atoi(slot_env);
        if (slot_val >= 0 && slot_val < (int)TDMA_MAX_BEACON_SLOTS) {
            g_my_beacon_slot = (uint8_t)slot_val;
        } else {
            fprintf(stderr, "[GW] Invalid EUROMESH_BEACON_SLOT=%s — using 0\n",
                    slot_env);
        }
    }

    /* ── Node ID ─────────────────────────────────────────────────────────── */
    my_id = emesh_get_node_id();
    printf("[GW] EuroMesh Gateway  id=0x%08X  cap=0x%02X  beacon_slot=%u\n",
           my_id, GATEWAY_CAPABILITY, g_my_beacon_slot);

    /* ── Peer table ──────────────────────────────────────────────────────── */
    gw_lora_peers_init();

    /* ── SX1301 init ─────────────────────────────────────────────────────── */
    if (!lgw_hal_init()) {
        fprintf(stderr, "[GW] SX1301 init failed — exiting\n");
        return EXIT_FAILURE;
    }

    /* ── Main loop ───────────────────────────────────────────────────────── */
    /*
     * Beacon timing is UTC-aligned:
     *   next_beacon_ms = next multiple of BEACON_INTERVAL_S seconds in UTC
     *                    + g_my_beacon_slot * TDMA_SLOT_MS
     *
     * Setting next_beacon_ms = 0 forces an immediate beacon on the first
     * iteration, then subsequent beacons align to epoch boundaries.
     */
    next_beacon_ms     = 0U;
    reg_close_utc_ms   = 0U;
    last_expiry_ms     = mono_now_ms();
    last_peer_log_ms   = mono_now_ms();
    uint64_t boot_mono_ms = mono_now_ms();

    while (g_running) {
        uint64_t now_utc  = utc_now_ms();
        uint64_t now_mono = mono_now_ms();

        /* ── Beacon window (UTC-aligned slot) ────────────────────────────── */
        if (now_utc >= next_beacon_ms) {
            node_expire();
            send_beacon(my_id, boot_mono_ms);

            /* Open the registration window. */
            reg_close_utc_ms = now_utc + (uint64_t)REG_WINDOW_S * 1000ULL;

            /* Schedule the next beacon at the proper UTC-aligned slot. */
            next_beacon_ms = next_beacon_utc_ms(g_my_beacon_slot);

            last_expiry_ms = now_mono;
        }

        /* Registration window state. */
        reg_window_open = (now_utc < reg_close_utc_ms);

        /* ── RX poll ─────────────────────────────────────────────────────── */
        rx_len = 0U;
        if (lgw_hal_receive(rx_buf, (uint8_t)sizeof(rx_buf),
                            &rx_len, &rx_meta)) {
            dispatch_frame(my_id, rx_buf, rx_len, &rx_meta, reg_window_open);
        } else {
            (void)usleep(RX_POLL_INTERVAL_US);
        }

        /* ── Periodic expiry (every minute) ──────────────────────────────── */
        if ((now_mono - last_expiry_ms) >= 60000U) {
            node_expire();
            gw_lora_peers_expire((time_t)GW_PEER_TIMEOUT_S);
            last_expiry_ms = now_mono;
        }

        /* ── Periodic peer table log (every 5 minutes) ───────────────────── */
        if ((now_mono - last_peer_log_ms) >= 300000U) {
            gw_lora_peers_print();
            last_peer_log_ms = now_mono;
        }
    }

    /* ── Shutdown ────────────────────────────────────────────────────────── */
    printf("\n[GW] Shutting down...\n");
    gw_lora_peers_print();
    lgw_hal_stop();
    return EXIT_SUCCESS;
}

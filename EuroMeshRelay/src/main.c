/*
 * main.c — EuroMesh Relay firmware entry point.
 *
 * Super-frame structure (TDMA_SUPERFRAME_PERIOD_MS = 60 s):
 *
 *   ┌──────────────┬──────────────┬──────────────────────────────────────────┐
 *   │  Slot 0      │  Slot 1      │   Inter-frame (remainder of super-frame) │
 *   │  BEACON      │  REGISTRATION│   CSMA/CAD — no schedule                 │
 *   │  500 ms      │  500 ms      │                                          │
 *   └──────────────┴──────────────┴──────────────────────────────────────────┘
 *
 * Slot 0: This relay broadcasts a beacon carrying UTC, PPS, stratum and a
 *         net_flags byte that signals whether the registration window is open.
 *
 * Slot 1: Unregistered downstream nodes contend via CSMA/CAD.  The relay
 *         opens a receive window (nodes_relay_open_reg_window) and unicasts
 *         a REG_RESPONSE to each valid SUBSCRIPTION request.
 *         Simultaneously, if this relay is unregistered upstream (e.g., the
 *         gateway has not yet been heard), reg_ctx_tick() fires a SUBSCRIPTION
 *         toward the best known gateway.
 *
 * Inter-frame: continuous CSMA/CAD receive loop dispatching:
 *   BEACON       → time_sync_handle_beacon + nb_table_update
 *   REG_RESPONSE → reg_ctx_handle_response (upstream registration)
 *   SUBSCRIPTION → reg_relay_handle_request (late arrivals outside window)
 *   ACK          → retry_on_ack
 *   TIME_REQ     → build + unicast TIME_RESP (relay acts as time server)
 *   TIME_RESP    → time_twoway_compute + time_sync_apply_twoway
 *   DATA         → dedup + ACK + forward / application dispatch
 *   NEIGHBOUR_ADV→ (reserved for future parsing)
 */

#include "stm32f4xx_hal.h"

#include "ack_packet.h"
#include "beacon_packet.h"
#include "csma_mac.h"
#include "dedup.h"
#include "emesh_frame.h"
#include "emesh_node_caps.h"
#include "emesh_packet_types.h"
#include "frame_queue.h"
#include "neighbour_table.h"
#include "nodes.h"
#include "registration.h"
#include "retry.h"
#include "sx1276.h"
#include "tdma.h"
#include "time_sync.h"
#include "time_twoway.h"

#include <string.h>

/* ── Hardware pin mapping ─────────────────────────────────────────────────── */
#define SX1276_NSS_PORT   GPIOB
#define SX1276_NSS_PIN    GPIO_PIN_6
#define SX1276_RST_PORT   GPIOB
#define SX1276_RST_PIN    GPIO_PIN_7
#define SX1276_DIO0_PORT  GPIOB
#define SX1276_DIO0_PIN   GPIO_PIN_8

/* ── Role configuration ───────────────────────────────────────────────────── */
/*
 * Adjust MY_CAPABILITY for the hardware variant:
 *   EMESH_NODE_TIER_RELAY                           — relay, no GPS
 *   EMESH_NODE_TIER_RELAY   | EMESH_NODE_FLAG_GPS   — relay with GPS (default)
 *   EMESH_NODE_TIER_GATEWAY | EMESH_NODE_FLAG_GPS   — full gateway
 */
#define MY_CAPABILITY  (EMESH_NODE_TIER_RELAY | EMESH_NODE_FLAG_GPS)

/* ── Relay operational mode ───────────────────────────────────────────────── */
/*
 * Three-state machine governing how the relay sources its clock and whether
 * it treats itself as the network root:
 *
 *   SEARCHING  — Boot state.  The relay listens for a gateway beacon for up to
 *                STANDALONE_SEARCH_TIMEOUT_MS before declaring itself root.
 *                If a gateway is heard and registration completes within the
 *                window, the relay jumps directly to SYNCED.
 *
 *   STANDALONE — No gateway was found (or the gateway has gone silent).  The
 *                relay self-seeds its clock from the monotonic tick (stratum 1)
 *                and beacons as the network root so that downstream nodes
 *                always have a valid time reference.
 *
 *   SYNCED     — An upstream gateway is present and registered.  The relay
 *                uses the gateway's time (stratum = gateway_stratum + 1).
 *                If no upstream beacon is heard for GATEWAY_LOST_TIMEOUT_MS
 *                the relay reverts to STANDALONE to keep the network alive.
 */
typedef enum {
    RELAY_MODE_SEARCHING  = 0,
    RELAY_MODE_STANDALONE = 1,
    RELAY_MODE_SYNCED     = 2,
} relay_mode_t;

/*
 * Wait two super-frames (120 s) before promoting to standalone root.
 * This gives the gateway time to transmit at least two beacons on boot.
 */
#define STANDALONE_SEARCH_TIMEOUT_MS  (2U * TDMA_SUPERFRAME_PERIOD_MS)   /* 120 s */

/*
 * Declare the gateway lost after three consecutive missed super-frames (180 s).
 * Two missed beacons are tolerated before falling back to standalone.
 */
#define GATEWAY_LOST_TIMEOUT_MS       (3U * TDMA_SUPERFRAME_PERIOD_MS)   /* 180 s */

/* Age-out periods — expressed as multiples of the super-frame period. */
#define NODE_MAX_AGE_MS  (3U * TDMA_SUPERFRAME_PERIOD_MS)   /* 3 min */
#define NB_MAX_AGE_MS    (5U * TDMA_SUPERFRAME_PERIOD_MS)   /* 5 min */

/*
 * Guard time (ms) before the beacon slot: stop inter-frame TX this early to
 * ensure the beacon goes out on time.
 */
#define BEACON_GUARD_MS  100U

/* Sentinel: no pending upstream TIME_REQ. */
#define TIME_REQ_SEQ_NONE  0U

/* ── Static peripheral handles ────────────────────────────────────────────── */
static SPI_HandleTypeDef hspi1;

/* ── Static mesh state ────────────────────────────────────────────────────── */
static sx1276_t      g_radio;
static csma_mac_t    g_mac;
static time_sync_t   g_time_sync;

/* Relay-side: downstream node registry. */
static reg_relay_t   g_relay;

/* Node-side: upstream registration state (this relay → gateway). */
static reg_ctx_t     g_upstream_ctx;

/* Protocol support. */
static dedup_table_t g_dedup;
static nb_table_t    g_nb_table;
static retry_table_t g_retry;

/* Pending two-way time-sync exchange. */
static uint16_t g_time_req_seq   = TIME_REQ_SEQ_NONE;
static uint64_t g_time_req_t1_ms = 0U;   /* our UTC when TIME_REQ was sent */

/* Global outbound sequence counter. */
static uint16_t g_tx_seq = 0U;

/* ── Relay mode state ─────────────────────────────────────────────────────── */
static relay_mode_t g_relay_mode             = RELAY_MODE_SEARCHING;
static uint32_t     g_boot_tick_ms           = 0U;
static uint32_t     g_last_upstream_heard_ms = 0U;

/* ── Forward declarations ─────────────────────────────────────────────────── */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void Error_Handler(void);

static void relay_mode_update(uint32_t now_ms);
static void handle_rx_frame(uint32_t my_id,
                            const emesh_frame_header_t *hdr,
                            const uint8_t *payload, uint8_t payload_len,
                            const sx1276_rx_metadata_t *meta,
                            uint32_t now_ms);
static bool try_receive_frame(uint32_t my_id, uint32_t now_ms);
static void try_send_time_req(uint32_t my_id, uint32_t now_ms);

/* ── Internal helpers ─────────────────────────────────────────────────────── */

/* Block until target_ms (wrap-around-safe via signed comparison). */
static void wait_until_ms(uint32_t target_ms)
{
    while ((int32_t)(target_ms - HAL_GetTick()) > 0) {
        HAL_Delay(1);
    }
}

/*
 * Broadcast a beacon frame carrying the current time-sync state.
 * reg_window_open is forwarded in net_flags so that nodes know they can
 * submit a SUBSCRIPTION request in the following slot.
 */
static void build_and_send_beacon(uint32_t my_id, bool reg_window_open)
{
    beacon_payload_t     beacon;
    uint8_t              payload[BEACON_PAYLOAD_SIZE];
    uint8_t              payload_len;
    emesh_frame_header_t hdr;

    beacon_payload_from_sync(&g_time_sync, HAL_GetTick(), reg_window_open, &beacon);
    payload_len = beacon_payload_encode(&beacon, payload, sizeof(payload));
    if (payload_len == 0U) {
        return;
    }

    memset(&hdr, 0, sizeof(hdr));
    g_tx_seq++;
    hdr.type    = EMESH_PACKET_TYPE_BEACON;
    hdr.flags   = EMESH_FRAME_FLAG_BROADCAST;
    hdr.ttl     = 1U;
    hdr.src_id  = my_id;
    hdr.dest_id = EMESH_DEST_BROADCAST;
    hdr.seq     = g_tx_seq;
    hdr.op      = EMESH_OP_MESH_BEACON;
    hdr.length  = payload_len;

    (void)csma_mac_send(&g_mac, &hdr, payload, payload_len);
}

/*
 * Build and unicast a TIME_RESP in reply to an incoming TIME_REQ.
 * t2 and t3 are our UTC clock readings at RX and TX respectively.
 * The node's T1 is stored locally by the node; we carry T1=0 and the node
 * substitutes its own stored value when it processes the response.
 */
static void handle_time_req(uint32_t my_id,
                            const emesh_frame_header_t *req_hdr,
                            const uint8_t *req_payload,
                            uint8_t req_payload_len)
{
    time_req_t           req;
    time_resp_t          resp;
    uint8_t              resp_payload[TIME_RESP_PAYLOAD_SIZE];
    emesh_frame_header_t resp_hdr;
    uint8_t              resp_len;

    if (!time_req_decode(req_payload, req_payload_len, &req)) {
        return;
    }
    /* Only answer if we have a synchronised clock. */
    if (!g_time_sync.utc_valid) {
        return;
    }

    resp.req_seq = req.req_seq;
    resp.t1_ms   = 0U;                               /* filled in by node */
    resp.t2_ms   = time_sync_now_utc_ms(&g_time_sync); /* UTC at ~RX      */
    resp.t3_ms   = time_sync_now_utc_ms(&g_time_sync); /* UTC at ~TX      */

    resp_len = time_resp_encode(&resp, resp_payload, sizeof(resp_payload));
    if (resp_len == 0U) {
        return;
    }

    memset(&resp_hdr, 0, sizeof(resp_hdr));
    g_tx_seq++;
    resp_hdr.type    = EMESH_PACKET_TYPE_TIME_RESP;
    resp_hdr.flags   = 0U;
    resp_hdr.ttl     = 1U;
    resp_hdr.src_id  = my_id;
    resp_hdr.dest_id = req_hdr->src_id;
    resp_hdr.seq     = g_tx_seq;
    resp_hdr.op      = EMESH_OP_MESH_TIME_RESP;
    resp_hdr.length  = resp_len;

    (void)csma_mac_send(&g_mac, &resp_hdr, resp_payload, resp_len);
}

/*
 * Send a TIME_REQ toward the best upstream relay/gateway if a two-way
 * resync is due and no exchange is already in flight.
 */
static void try_send_time_req(uint32_t my_id, uint32_t now_ms)
{
    const nb_entry_t    *best;
    time_req_t           req;
    uint8_t              payload[TIME_REQ_PAYLOAD_SIZE];
    uint8_t              payload_len;
    emesh_frame_header_t hdr;

    if (g_time_req_seq != TIME_REQ_SEQ_NONE) {
        return; /* exchange already in progress */
    }
    if (!time_sync_needs_resync(&g_time_sync, now_ms)) {
        return;
    }

    best = nb_table_best(&g_nb_table, EMESH_NODE_TIER_RELAY);
    if (best == NULL) {
        return; /* no known upstream neighbour */
    }

    g_tx_seq++;
    req.req_seq = g_tx_seq;
    payload_len = time_req_encode(&req, payload, sizeof(payload));
    if (payload_len == 0U) {
        return;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.type    = EMESH_PACKET_TYPE_TIME_REQ;
    hdr.flags   = 0U;
    hdr.ttl     = 1U;
    hdr.src_id  = my_id;
    hdr.dest_id = best->node_id;
    hdr.seq     = g_tx_seq;
    hdr.op      = EMESH_OP_MESH_TIME_REQ;
    hdr.length  = payload_len;

    if (csma_mac_send(&g_mac, &hdr, payload, payload_len) == HAL_OK) {
        g_time_req_seq   = g_tx_seq;
        g_time_req_t1_ms = time_sync_now_utc_ms(&g_time_sync);
    }
}

/*
 * Drive the relay operational mode state machine.
 * Call once at the top of every super-frame iteration.
 */
static void relay_mode_update(uint32_t now_ms)
{
    switch (g_relay_mode) {

    case RELAY_MODE_SEARCHING:
        /*
         * Fast-path: if a gateway was already heard and registration
         * completed inside the search window, go straight to SYNCED.
         */
        if (reg_ctx_is_registered(&g_upstream_ctx) && g_time_sync.utc_valid) {
            g_relay_mode             = RELAY_MODE_SYNCED;
            g_last_upstream_heard_ms = now_ms;
            break;
        }
        /*
         * Search timeout: no gateway heard within the window.
         * Self-seed the clock from the monotonic tick so downstream nodes
         * receive a valid (though not UTC-accurate) time reference.
         * Using EMESH_STRATUM_GPS (0) as the source stratum causes
         * time_sync_apply_sample to set our stratum to 1.
         */
        if ((now_ms - g_boot_tick_ms) >= STANDALONE_SEARCH_TIMEOUT_MS) {
            time_sync_apply_sample(&g_time_sync,
                                   (uint64_t)now_ms, /* uptime-based epoch */
                                   0U,               /* no PPS             */
                                   EMESH_STRATUM_GPS, /* src 0 → stratum 1 */
                                   now_ms);
            g_relay_mode = RELAY_MODE_STANDALONE;
        }
        break;

    case RELAY_MODE_STANDALONE:
        /*
         * Gateway appeared and upstream registration completed.
         * Transition to SYNCED; the time_sync is already updated because
         * time_sync_handle_beacon accepted the lower-stratum beacon.
         */
        if (reg_ctx_is_registered(&g_upstream_ctx)) {
            g_relay_mode             = RELAY_MODE_SYNCED;
            g_last_upstream_heard_ms = now_ms;
        }
        break;

    case RELAY_MODE_SYNCED:
        /*
         * Gateway has gone silent for too long.  Revert to STANDALONE so
         * downstream nodes continue receiving valid beacons.  The clock
         * remains valid from the last gateway sync; drift will increase
         * gradually until a gateway re-appears.
         */
        if ((now_ms - g_last_upstream_heard_ms) >= GATEWAY_LOST_TIMEOUT_MS) {
            g_relay_mode = RELAY_MODE_STANDALONE;
        }
        break;
    }
}

/*
 * Dispatch a decoded inbound frame to the correct protocol handler.
 */
static void handle_rx_frame(uint32_t my_id,
                            const emesh_frame_header_t *hdr,
                            const uint8_t *payload, uint8_t payload_len,
                            const sx1276_rx_metadata_t *meta,
                            uint32_t now_ms)
{
    /* ── Passive neighbour update — every received frame ─────────────────── */
    nb_table_update(&g_nb_table,
                    hdr->src_id,
                    meta->rssi_dbm,
                    meta->snr_db,
                    0U,                   /* tier refined below per packet type */
                    EMESH_STRATUM_UNKNOWN,
                    now_ms);

    /* ── Deduplication — skip for beacons and ACKs (always process) ──────── */
    if (hdr->type != EMESH_PACKET_TYPE_BEACON &&
        hdr->type != EMESH_PACKET_TYPE_ACK) {
        if (dedup_is_duplicate(&g_dedup, hdr->src_id, hdr->seq, now_ms)) {
            return;
        }
    }

    switch (hdr->type) {

    /* ── BEACON ────────────────────────────────────────────────────────────── */
    case EMESH_PACKET_TYPE_BEACON: {
        bool             sync_req = (hdr->flags & EMESH_FRAME_FLAG_TIME_SYNC) != 0U;
        beacon_payload_t bp;

        /* payload[0] = type byte; time_sync_handle_beacon wants payload[1..]. */
        if (payload_len > 1U) {
            time_sync_handle_beacon(&g_time_sync, &payload[1],
                                    (uint8_t)(payload_len - 1U), sync_req);
        }

        /* Refine the neighbour entry with stratum from the decoded beacon. */
        if (beacon_payload_decode(payload, payload_len, &bp)) {
            nb_table_update(&g_nb_table,
                            hdr->src_id,
                            meta->rssi_dbm,
                            meta->snr_db,
                            EMESH_NODE_TIER_RELAY, /* beacon senders are >= relay */
                            bp.stratum,
                            now_ms);

            /*
             * Track the last time we heard from a higher-quality (lower-
             * stratum) upstream source.  Used by relay_mode_update() to
             * detect gateway loss in RELAY_MODE_SYNCED.
             */
            if (bp.stratum < g_time_sync.stratum) {
                g_last_upstream_heard_ms = now_ms;
            }

            /*
             * Feed the beacon into the upstream registration state machine so
             * that reg_ctx_tick() knows the best parent RSSI and stratum.
             * reg_slot_start_ms = 0 here because we are not in the reg slot;
             * the actual slot start is supplied in the super-frame loop below.
             */
            reg_ctx_tick(&g_upstream_ctx, &g_mac, my_id,
                         (bp.net_flags & BEACON_NET_FLAG_REG_OPEN) != 0U,
                         0U,
                         meta->rssi_dbm,
                         hdr->src_id,
                         bp.stratum,
                         now_ms);
        }
        break;
    }

    /* ── ACK ──────────────────────────────────────────────────────────────── */
    case EMESH_PACKET_TYPE_ACK: {
        uint16_t acked_seq;
        if (!emesh_frame_is_for_me(hdr, my_id)) {
            break;
        }
        /* acked_seq lives at payload[1..2] (after the type byte). */
        if (payload_len < 3U) {
            break;
        }
        acked_seq = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
        (void)retry_on_ack(&g_retry, hdr->src_id, acked_seq);
        break;
    }

    /* ── REG_RESPONSE (upstream relay accepted our subscription) ─────────── */
    case EMESH_PACKET_TYPE_REG_RESPONSE:
        if (emesh_frame_is_for_me(hdr, my_id)) {
            (void)reg_ctx_handle_response(&g_upstream_ctx, hdr,
                                          payload, payload_len, now_ms);
        }
        break;

    /* ── SUBSCRIPTION (downstream node registering, arrived outside window) ─ */
    case EMESH_PACKET_TYPE_SUBSCRIPTION:
        reg_relay_handle_request(&g_relay, &g_mac, my_id,
                                 hdr, payload, payload_len,
                                 meta->rssi_dbm, now_ms);
        break;

    /* ── TIME_REQ (node wants two-way sync; we are the time server) ─────── */
    case EMESH_PACKET_TYPE_TIME_REQ:
        if (emesh_frame_is_for_me(hdr, my_id)) {
            handle_time_req(my_id, hdr, payload, payload_len);
        }
        break;

    /* ── TIME_RESP (upstream replied to our TIME_REQ) ───────────────────── */
    case EMESH_PACKET_TYPE_TIME_RESP: {
        time_resp_t          resp;
        time_twoway_result_t result;
        uint64_t             t4_ms;
        uint8_t              src_stratum;

        if (!emesh_frame_is_for_me(hdr, my_id)) {
            break;
        }
        if (!time_resp_decode(payload, payload_len, &resp)) {
            break;
        }
        if (resp.req_seq != g_time_req_seq) {
            break; /* stale or spurious response */
        }

        t4_ms = time_sync_now_utc_ms(&g_time_sync);

        /* Restore the node's own T1 (the relay leaves this zero). */
        resp.t1_ms = g_time_req_t1_ms;

        time_twoway_compute(&resp, t4_ms, &result);

        /* Apply: our stratum is parent_stratum + 1; clamp to UNKNOWN. */
        src_stratum = g_time_sync.stratum > 0U
                      ? (uint8_t)(g_time_sync.stratum - 1U)
                      : 0U;
        time_sync_apply_twoway(&g_time_sync,
                               result.offset_ms, result.rtt_ms,
                               src_stratum, now_ms);

        g_time_req_seq   = TIME_REQ_SEQ_NONE;
        g_time_req_t1_ms = 0U;
        break;
    }

    /* ── DATA (application payload) ──────────────────────────────────────── */
    case EMESH_PACKET_TYPE_DATA: {
        if (!emesh_frame_is_for_me(hdr, my_id)) {
            /* Forward with decremented TTL if the frame has hops remaining. */
            if (hdr->ttl > 1U) {
                emesh_frame_header_t fwd = *hdr;
                fwd.ttl--;
                (void)csma_mac_send(&g_mac, &fwd, payload, payload_len);
            }
            break;
        }

        /* Send ACK if requested. */
        if ((hdr->flags & EMESH_FRAME_FLAG_ACK_REQUEST) != 0U) {
            emesh_ack_packet_t   ack_pkt  = {0};
            uint8_t              ack_buf[8];
            size_t               ack_len  = 0U;
            emesh_frame_header_t ack_hdr  = {0};

            ack_pkt.acked_seq = hdr->seq;
            if (emesh_ack_encode(&ack_pkt, ack_buf, sizeof(ack_buf), &ack_len)) {
                g_tx_seq++;
                ack_hdr.type    = EMESH_PACKET_TYPE_ACK;
                ack_hdr.flags   = 0U;
                ack_hdr.ttl     = 1U;
                ack_hdr.src_id  = my_id;
                ack_hdr.dest_id = hdr->src_id;
                ack_hdr.seq     = g_tx_seq;
                ack_hdr.op      = EMESH_OP(EMESH_OP_CLASS_MESH, 0x00U);
                ack_hdr.length  = (uint8_t)ack_len;
                (void)csma_mac_send(&g_mac, &ack_hdr, ack_buf, (uint8_t)ack_len);
            }
        }

        /* Application payload available in payload[0..payload_len-1].
         * Extend here with application-specific dispatch. */
        break;
    }

    /* ── NEIGHBOUR_ADV (peer advertises its link-quality view) ──────────── */
    case EMESH_PACKET_TYPE_NEIGHBOUR_ADV:
        /* The sender is already captured via the passive update at entry.
         * Full advertisement parsing (updating our table with their view)
         * can be added when neighbour routing is implemented. */
        break;

    default:
        break;
    }
}

/*
 * Attempt to receive one frame.
 * Returns true if a frame was received and dispatched, false otherwise.
 */
static bool try_receive_frame(uint32_t my_id, uint32_t now_ms)
{
    uint8_t              rx_buf[EMESH_FRAME_FIFO_SIZE];
    uint8_t              rx_len = 0U;
    sx1276_rx_metadata_t meta   = {0};
    emesh_frame_header_t hdr;
    uint8_t              payload_len;

    if (sx1276_receive_bytes(&g_radio, rx_buf, sizeof(rx_buf),
                             &rx_len, &meta) != HAL_OK) {
        return false;
    }
    if (!meta.crc_ok || rx_len < EMESH_FRAME_HEADER_SIZE) {
        return false;
    }

    emesh_frame_decode_header(rx_buf, &hdr);
    payload_len = (uint8_t)(rx_len - EMESH_FRAME_HEADER_SIZE);

    handle_rx_frame(my_id, &hdr,
                    &rx_buf[EMESH_FRAME_HEADER_SIZE], payload_len,
                    &meta, now_ms);
    return true;
}

/* ── Entry point ──────────────────────────────────────────────────────────── */

int main(void)
{
    uint32_t my_id;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    /* Derive a unique 32-bit node ID from the STM32 UID registers. */
    my_id = emesh_get_node_id();

    /* ── Radio ─────────────────────────────────────────────────────────────── */
    g_radio.hspi       = &hspi1;
    g_radio.nss_port   = SX1276_NSS_PORT;
    g_radio.nss_pin    = SX1276_NSS_PIN;
    g_radio.reset_port = SX1276_RST_PORT;
    g_radio.reset_pin  = SX1276_RST_PIN;
    g_radio.dio0_port  = SX1276_DIO0_PORT;
    g_radio.dio0_pin   = SX1276_DIO0_PIN;

    sx1276_init(&g_radio);
    {
        sx1276_lora_config_t radio_cfg = {
            .frequency_hz         = 869525000U, /* 869.525 MHz EU ISM       */
            .bandwidth_bits       = 0x02U,      /* 125 kHz                  */
            .spreading_factor     = 0x07U,      /* SF7                      */
            .coding_rate_bits     = 0x01U,      /* CR 4/5                   */
            .tx_power_dbm         = 15,
            .implicit_header_mode = false,
            .use_pa_boost         = true,
        };
        sx1276_configure_lora(&g_radio, &radio_cfg);
    }

    /* ── MAC ───────────────────────────────────────────────────────────────── */
    csma_mac_init(&g_mac, &g_radio,
                  /* backoff_min_ms = */ 20U,
                  /* backoff_max_ms = */ 200U,
                  /* max_attempts   = */ 5U);

    /* ── Protocol modules ─────────────────────────────────────────────────── */
    time_sync_init(&g_time_sync, TIME_SYNC_TIMEOUT_MS, TIME_SYNC_MAX_SKEW_MS);

    reg_relay_init(&g_relay,
                   /* pan_id          = */ (uint16_t)(my_id & 0xFFFFU),
                   /* local_stratum   = */ EMESH_STRATUM_UNKNOWN,
                   /* nb_adv_interval = */ 30U);

    reg_ctx_init(&g_upstream_ctx, MY_CAPABILITY);

    dedup_init(&g_dedup);
    nb_table_init(&g_nb_table);
    retry_init(&g_retry);

    g_boot_tick_ms = HAL_GetTick();

    /* ═══════════════════════════════════════════════════════════════════════
     * Super-frame loop
     * ═══════════════════════════════════════════════════════════════════════ */
    while (1) {
        uint32_t now_ms;
        uint32_t beacon_start_ms;
        uint32_t reg_slot_start_ms;
        uint32_t next_beacon_ms;
        uint32_t failed_dest;

        now_ms          = HAL_GetTick();
        beacon_start_ms = tdma_next_beacon_ms(now_ms);

        /* ── 0. Update relay operational mode ────────────────────────────── */
        relay_mode_update(now_ms);

        /* ── 1. Wait for the beacon slot ──────────────────────────────────── */
        wait_until_ms(beacon_start_ms);

        /* ── 2. Broadcast beacon ──────────────────────────────────────────── */
        build_and_send_beacon(my_id, /* reg_window_open = */ true);

        /* ── 3. Drive upstream registration ──────────────────────────────── */
        /*
         * If this relay has not yet registered with a gateway, send a
         * SUBSCRIPTION at the start of the registration slot.
         * We pass the best upstream neighbour's RSSI/stratum from the
         * neighbour table so the state machine can pick the best parent.
         */
        reg_slot_start_ms = tdma_reg_window_start_ms(beacon_start_ms);
        {
            const nb_entry_t *best_upstream;
            int8_t            best_rssi    = -128;
            uint32_t          parent_id    = 0U;
            uint8_t           parent_strat = EMESH_STRATUM_UNKNOWN;

            best_upstream = nb_table_best(&g_nb_table, EMESH_NODE_TIER_RELAY);
            if (best_upstream != NULL) {
                /* Clamp rssi_dbm to int8_t range. */
                best_rssi    = (best_upstream->rssi_dbm >= -128)
                               ? (int8_t)best_upstream->rssi_dbm
                               : (int8_t)(-128);
                parent_id    = best_upstream->node_id;
                parent_strat = best_upstream->stratum;
            }

            reg_ctx_tick(&g_upstream_ctx, &g_mac, my_id,
                         /* reg_window_open  = */ true,
                         reg_slot_start_ms,
                         best_rssi,
                         parent_id,
                         parent_strat,
                         HAL_GetTick());
        }

        /* ── 4. Open the registration window for downstream nodes ─────────── */
        wait_until_ms(reg_slot_start_ms);
        nodes_relay_open_reg_window(&g_relay, &g_mac, &g_radio,
                                    my_id, TDMA_REG_SLOT_MS);

        /* ── 5. Per-super-frame maintenance ─────────────────────────────────
         *
         * Expire stale node and neighbour entries.
         * Propagate our current stratum into the relay registry so that
         * REG_RESPONSEs carry an up-to-date stratum.
         */
        now_ms = HAL_GetTick();
        nodes_relay_expire(&g_relay, now_ms, NODE_MAX_AGE_MS);
        nb_table_expire(&g_nb_table, now_ms, NB_MAX_AGE_MS);

        if (g_time_sync.utc_valid) {
            g_relay.local_stratum = g_time_sync.stratum;
        }

        /* ── 6. Inter-frame: CSMA/CAD RX until the next beacon slot ─────────
         *
         * Leave BEACON_GUARD_MS of headroom so the next iteration's
         * build_and_send_beacon() fires on time.
         */
        next_beacon_ms = tdma_next_beacon_ms(HAL_GetTick());

        while ((int32_t)((next_beacon_ms - BEACON_GUARD_MS) - HAL_GetTick()) > 0) {
            now_ms = HAL_GetTick();

            /* Drive the ACK/retry state machine. */
            failed_dest = 0U;
            retry_tick(&g_retry, &g_mac, now_ms, &failed_dest);
            if (failed_dest != 0U) {
                /* Delivery failed — caller can hook application logic here
                 * (e.g., evict the unreachable neighbour, raise an alarm). */
                (void)failed_dest;
            }

            /* Initiate two-way clock sync if due. */
            try_send_time_req(my_id, now_ms);

            /* Receive and dispatch one frame (non-blocking). */
            (void)try_receive_frame(my_id, now_ms);
        }
    }

    /* Unreachable. */
}

/* ── STM32 HAL initialisation ─────────────────────────────────────────────── */

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    osc_init.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc_init.HSIState            = RCC_HSI_ON;
    osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc_init.PLL.PLLState        = RCC_PLL_ON;
    osc_init.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc_init.PLL.PLLM            = 16;
    osc_init.PLL.PLLN            = 336;
    osc_init.PLL.PLLP            = RCC_PLLP_DIV4;
    osc_init.PLL.PLLQ            = 7;
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        Error_Handler();
    }

    clk_init.ClockType      = RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1
                            | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SPI1: PA5 = SCK, PA6 = MISO, PA7 = MOSI */
    gpio_init.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_NOPULL;
    gpio_init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* NSS and RESET: output push-pull */
    gpio_init.Pin   = SX1276_NSS_PIN | SX1276_RST_PIN;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = 0U;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* DIO0: input (PacketDone / CAD-done interrupt line) */
    gpio_init.Pin  = SX1276_DIO0_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* Deassert NSS and RESET at startup. */
    HAL_GPIO_WritePin(SX1276_NSS_PORT,  SX1276_NSS_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(SX1276_RST_PORT,  SX1276_RST_PIN,  GPIO_PIN_SET);
}

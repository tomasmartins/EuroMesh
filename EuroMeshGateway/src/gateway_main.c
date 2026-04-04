/*
 * gateway_main.c — EuroMesh gateway application (Raspberry Pi + SX1301).
 *
 * Responsibilities
 * ────────────────
 * 1. Beacon TX every BEACON_INTERVAL_S seconds.
 *    - Source: CLOCK_REALTIME (NTP or GPS-disciplined).
 *    - Stratum: 0 (authoritative GPS/NTP reference).
 *    - Registration window open for the first REG_WINDOW_S of each period.
 *
 * 2. RX dispatch loop (runs between beacons):
 *    - SUBSCRIPTION  → update node table, send REG_RESPONSE.
 *    - TIME_REQ      → record T2, reply with TIME_RESP (T1=0, T2, T3).
 *    - All others    → logged and discarded.
 *
 * 3. Node table expiry: nodes not seen for REG_NODE_EXPIRY_S are evicted.
 *
 * Time model
 * ──────────
 * The gateway is stratum 0 — it claims to BE the GPS source.  This is
 * correct when the RPi is GPS-disciplined (e.g., gpsd + chrony/PPS).  When
 * running with plain NTP (no PPS), the beacon is still stratum 0 but with
 * ~10 ms accuracy.  For the hardware integration test either source is
 * sufficient; the relay simply syncs from the gateway beacon and becomes
 * stratum 1.
 *
 * The pps_tick_ms field in the beacon is set to 0 and the TIME_SYNC_FLAG_PPS
 * bit is NOT set unless a PPS source is detected (future extension).
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
#include "gw_registration.h"
#include "lgw_hal.h"
#include "time_sync.h"       /* TIME_SYNC_FLAG_* constants only */
#include "time_twoway.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Beacon period: one per minute to give relays ample time to receive. */
#define BEACON_INTERVAL_S    60U

/*
 * Registration window: the beacon includes BEACON_NET_FLAG_REG_OPEN for the
 * first REG_WINDOW_S seconds of each beacon interval.  Relays and nodes
 * must transmit their SUBSCRIPTION request within this window.
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

/* ═══════════════════════════════════════════════════════════════════════════
 * Node registry
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * TDMA slot allocation.
 * Gateway occupies slot 0.  Registering relays are assigned slots 1, 2, ...
 * Leaf nodes (EMESH_NODE_TIER_NODE) receive TDMA_NO_SLOT_ASSIGNED (0xFF).
 */
#define TDMA_NO_SLOT_ASSIGNED   0xFFU
#define TDMA_GATEWAY_SLOT       0U
#define TDMA_MAX_BEACON_SLOTS   8U

typedef struct {
    uint32_t node_id;
    uint8_t  capability;
    uint8_t  beacon_slot;     /* TDMA beacon slot, 0xFF = none */
    float    last_rssi_dbm;
    time_t   first_seen;
    time_t   last_seen;
} gw_node_entry_t;

static gw_node_entry_t g_nodes[REG_MAX_NODES];
static uint8_t         g_node_count      = 0U;
/* Next relay slot to allocate (starts at 1 — slot 0 is the gateway). */
static uint8_t         g_next_relay_slot = 1U;

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

/* Monotonic millisecond counter — for interval timing. */
static uint64_t mono_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL
         + (uint64_t)(ts.tv_nsec / 1000000L);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Sequence counters
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t g_beacon_seq   = 0U;
static uint16_t g_resp_seq     = 0U;

/* ═══════════════════════════════════════════════════════════════════════════
 * Beacon TX
 * ═══════════════════════════════════════════════════════════════════════════ */

static void send_beacon(uint32_t my_id, bool reg_window_open,
                        uint64_t boot_utc_ms)
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
     * Build the beacon payload.
     * The gateway IS stratum 0 (GPS/NTP authority).
     * pps_tick_ms = 0 and PPS_VALID not set until a hardware PPS is wired.
     */
    bp.sync_flags   = TIME_SYNC_FLAG_UTC_VALID;
    bp.utc_epoch_ms = now_ms_utc;
    bp.pps_tick_ms  = 0U;
    bp.stratum      = EMESH_STRATUM_GPS;   /* 0 — authoritative             */
    bp.net_flags    = reg_window_open ? BEACON_NET_FLAG_REG_OPEN : 0U;

    /* Telemetry: gateway has no upstream, reports registered node count. */
    bp.telem_flags       = BEACON_TELEM_FLAG_TX_PWR
                         | BEACON_TELEM_FLAG_NODES
                         | BEACON_TELEM_FLAG_UPTIME;
    bp.tx_power_dbm      = 14;                  /* SX1301 TX power           */
    bp.upstream_rssi_dbm = (int8_t)0x80;        /* no upstream (we are root) */
    bp.node_count        = g_node_count;
    bp.uptime_s          = (uint32_t)((now_ms_mono - boot_utc_ms) / 1000ULL);

    /* Frame header. */
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
        printf("[GW] Beacon TX  seq=%-4u  utc=%llu ms  "
               "reg_open=%d  nodes=%u  uptime=%us\n",
               g_beacon_seq,
               (unsigned long long)bp.utc_epoch_ms,
               reg_window_open ? 1 : 0,
               g_node_count,
               bp.uptime_s);
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
            g_nodes[idx].last_rssi_dbm = rssi_dbm;
            g_nodes[idx].first_seen    = now;
            g_nodes[idx].last_seen     = now;

            /*
             * Assign a TDMA beacon slot to relays; leaf nodes do not beacon.
             * Gateway slot 0 is reserved for this gateway.
             */
            if ((req.capability & 0x03U) == 0x01U) { /* EMESH_NODE_TIER_RELAY */
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
                   "rssi=%.0f dBm  slot=%u  (total=%u)\n",
                   req_hdr->src_id, req.capability, rssi_dbm,
                   g_nodes[idx].beacon_slot, g_node_count);
        }
    } else {
        g_nodes[idx].capability    = req.capability;
        g_nodes[idx].last_rssi_dbm = rssi_dbm;
        g_nodes[idx].last_seen     = now;
        resp.status      = REG_STATUS_OK;
        resp.beacon_slot = g_nodes[idx].beacon_slot;  /* keep existing slot  */
        printf("[GW] Re-registered node 0x%08X  cap=0x%02X  "
               "rssi=%.0f dBm  slot=%u\n",
               req_hdr->src_id, req.capability, rssi_dbm,
               g_nodes[idx].beacon_slot);
    }

    /* Build REG_RESPONSE frame. */
    if (reg_response_encode(&resp, resp_payload, sizeof(resp_payload)) == 0U) {
        return;
    }

    memset(&resp_hdr, 0, sizeof(resp_hdr));
    resp_hdr.type    = EMESH_PACKET_TYPE_REG_RESPONSE;
    resp_hdr.flags   = 0U;
    resp_hdr.ttl     = 1U;       /* registration is single-hop            */
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
                             uint64_t t2_ms)  /* UTC at approximately RX    */
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

    /*
     * NTP-style two-way exchange:
     *   T1 = node's UTC at TIME_REQ TX — stored by node; echoed as 0 here.
     *   T2 = gateway UTC at TIME_REQ RX — captured by the caller.
     *   T3 = gateway UTC at TIME_RESP TX — captured just before sending.
     *
     * The node fills in T1 from its own state when it receives this response.
     */
    resp.req_seq = req.req_seq;
    resp.t1_ms   = 0U;        /* filled in by the requesting node            */
    resp.t2_ms   = t2_ms;
    resp.t3_ms   = utc_now_ms(); /* UTC at ~ TX (encode + transmit overhead) */

    if (time_resp_encode(&resp, resp_payload, sizeof(resp_payload)) == 0U) {
        return;
    }

    memset(&resp_hdr, 0, sizeof(resp_hdr));
    resp_hdr.type    = EMESH_PACKET_TYPE_TIME_RESP;
    resp_hdr.flags   = 0U;
    resp_hdr.ttl     = 1U;
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
 * RX dispatch
 * ═══════════════════════════════════════════════════════════════════════════ */

static void dispatch_frame(uint32_t my_id,
                            const uint8_t *raw, uint8_t raw_len,
                            const lgw_rx_meta_t *meta)
{
    emesh_frame_header_t hdr;
    const uint8_t       *payload;
    uint8_t              payload_len;
    uint64_t             t2_ms;

    if (raw_len < EMESH_FRAME_HEADER_SIZE) {
        return; /* too short to contain a valid header */
    }

    emesh_frame_decode_header(raw, &hdr);

    /* Only process frames addressed to us or broadcast. */
    if (!emesh_frame_is_for_me(&hdr, my_id)) {
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
            if (beacon_payload_decode(payload, payload_len, &bp)) {
                printf("[GW] RX Beacon   from 0x%08X  rssi=%.0f dBm  snr=%.1f dB  "
                       "utc=%llu ms  stratum=%u  nodes=%u  uptime=%us  "
                       "reg_open=%d\n",
                       hdr.src_id, meta->rssi_dbm, meta->snr_db,
                       (unsigned long long)bp.utc_epoch_ms,
                       bp.stratum, bp.node_count, bp.uptime_s,
                       (bp.net_flags & BEACON_NET_FLAG_REG_OPEN) ? 1 : 0);
            } else {
                printf("[GW] RX Beacon   from 0x%08X  rssi=%.0f dBm  (malformed)\n",
                       hdr.src_id, meta->rssi_dbm);
            }
        }
        break;

    case EMESH_PACKET_TYPE_SUBSCRIPTION:
        handle_subscription(my_id, &hdr, payload, hdr.length,
                             meta->rssi_dbm);
        break;

    case EMESH_PACKET_TYPE_TIME_REQ:
        handle_time_req(my_id, &hdr, payload, hdr.length, t2_ms);
        break;

    case EMESH_PACKET_TYPE_ACK:
    case EMESH_PACKET_TYPE_DATA:
    case EMESH_PACKET_TYPE_NEIGHBOUR_ADV:
    case EMESH_PACKET_TYPE_REG_RESPONSE:
    case EMESH_PACKET_TYPE_TIME_RESP:
        /* Not handled at the gateway level — log only. */
        printf("[GW] RX type=0x%02X from 0x%08X (unhandled)\n",
               hdr.type, hdr.src_id);
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
    uint64_t      last_beacon_ms;
    uint64_t      last_expiry_ms;
    uint64_t      now_ms;
    uint8_t       rx_buf[256];
    uint8_t       rx_len;
    lgw_rx_meta_t rx_meta;
    bool          reg_window_open;

    (void)argc;
    (void)argv;

    /* ── Signal handlers ─────────────────────────────────────────────────── */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── Node ID ─────────────────────────────────────────────────────────── */
    my_id = emesh_get_node_id();
    printf("[GW] EuroMesh Gateway  id=0x%08X  cap=0x%02X\n",
           my_id, GATEWAY_CAPABILITY);

    /* ── SX1301 init ─────────────────────────────────────────────────────── */
    if (!lgw_hal_init()) {
        fprintf(stderr, "[GW] SX1301 init failed — exiting\n");
        return EXIT_FAILURE;
    }

    /* ── Main loop ───────────────────────────────────────────────────────── */
    /*
     * The gateway occupies TDMA slot 0.  The 60 s super-frame is driven by
     * wall-clock time (CLOCK_MONOTONIC) to stay consistent across reboots.
     * Beacon slot timing: the gateway fires at the start of each 60 s window.
     * Registered relays beacon in their allocated slots (slot 1, 2, ...) in
     * the same super-frame; the gateway does not enforce relay slot timing —
     * it relies on the relay firmware respecting its assigned slot offset.
     */
    last_beacon_ms = 0U;          /* force immediate beacon on first loop     */
    last_expiry_ms = mono_now_ms();
    uint64_t boot_mono_ms = mono_now_ms();

    while (g_running) {
        now_ms = mono_now_ms();

        /* ── Beacon window (TDMA slot 0) ─────────────────────────────────── */
        if ((now_ms - last_beacon_ms) >= (uint64_t)BEACON_INTERVAL_S * 1000U) {
            node_expire();
            last_beacon_ms = now_ms;
            send_beacon(my_id, /*reg_window_open=*/true, boot_mono_ms);
        }

        /* Registration window is open for the first REG_WINDOW_S after TX. */
        reg_window_open = (now_ms - last_beacon_ms)
                          < (uint64_t)REG_WINDOW_S * 1000U;

        /* ── RX poll ─────────────────────────────────────────────────────── */
        rx_len = 0U;
        if (lgw_hal_receive(rx_buf, (uint8_t)sizeof(rx_buf),
                            &rx_len, &rx_meta)) {
            dispatch_frame(my_id, rx_buf, rx_len, &rx_meta);
        } else {
            /* Nothing received — yield the CPU briefly. */
            (void)usleep(RX_POLL_INTERVAL_US);
        }

        /* ── Periodic expiry (every minute) ──────────────────────────────── */
        if ((now_ms - last_expiry_ms) >= 60000U) {
            node_expire();
            last_expiry_ms = now_ms;
        }
    }

    /* ── Shutdown ────────────────────────────────────────────────────────── */
    printf("\n[GW] Shutting down...\n");
    lgw_hal_stop();
    return EXIT_SUCCESS;
}

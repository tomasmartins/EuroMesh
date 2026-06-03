/*
 * gw_upstream.c — Gateway upstream registration client (state machine).
 */

#include <stdio.h>
#include <string.h>

#include "gw_upstream.h"
#include "emesh_frame.h"
#include "emesh_node_caps.h"
#include "emesh_packet_types.h"
#include "gw_registration.h"

/* TTL for SUBSCRIPTION frames sent upstream. */
#define GW_UP_SUB_TTL  7U

/* ── Module state ────────────────────────────────────────────────────────── */

static gw_upstream_state_t g_state        = GW_UP_SCANNING;
static uint32_t            g_my_id        = 0U;
static uint8_t             g_capability   = 0U;

/* Scan / retry timing (monotonic ms). */
static uint64_t g_scan_start_ms  = 0U;
static uint64_t g_reg_sent_ms    = 0U;
static uint8_t  g_retry_count    = 0U;

/* Best upstream candidate discovered during scanning. */
static uint32_t g_upstream_id      = 0U;
static uint8_t  g_upstream_stratum = 0U;   /* stratum reported by upstream */
static float    g_upstream_rssi    = -999.0f;
static uint64_t g_upstream_last_ms = 0U;   /* last beacon monotonic time  */

/* Registration result. */
static uint8_t  g_assigned_slot  = 0xFFU;  /* slot from REG_RESPONSE       */
static uint8_t  g_my_stratum     = 0xFFU;  /* stratum we will advertise    */

/* TX pending flag — set by on_beacon or retry logic, cleared by sub_sent. */
static bool g_needs_tx = false;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void enter_scanning(uint64_t now_ms)
{
    g_state         = GW_UP_SCANNING;
    g_scan_start_ms = now_ms;
    g_upstream_id   = 0U;
    g_retry_count   = 0U;
    g_needs_tx      = false;
    g_my_stratum    = EMESH_STRATUM_UNKNOWN;
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void gw_upstream_init(uint32_t my_id, uint8_t capability, uint64_t now_ms)
{
    g_my_id      = my_id;
    g_capability = capability;
    enter_scanning(now_ms);
    printf("[UP] Scanning for upstream gateway (timeout %u s)...\n",
           GW_UPSTREAM_SCAN_TIMEOUT_MS / 1000U);
}

/* ── Event inputs ────────────────────────────────────────────────────────── */

void gw_upstream_on_beacon(uint32_t src_id,
                            uint8_t  stratum,
                            float    rssi_dbm,
                            bool     reg_open,
                            uint64_t now_ms)
{
    /*
     * Only stratum-0 gateways qualify as upstream candidates.
     * Relay beacons (stratum >= 1) are ignored by this module.
     */
    if (stratum != EMESH_STRATUM_GPS) {
        return;
    }

    if (g_state == GW_UP_SCANNING) {
        /*
         * Accept the first stratum-0 gateway heard, or a better one
         * (higher RSSI) if we have already recorded a candidate.
         */
        if (g_upstream_id == 0U || rssi_dbm > g_upstream_rssi) {
            if (g_upstream_id != src_id) {
                printf("[UP] Candidate upstream GW 0x%08X  rssi=%.0f dBm\n",
                       src_id, (double)rssi_dbm);
            }
            g_upstream_id      = src_id;
            g_upstream_stratum = stratum;
            g_upstream_rssi    = rssi_dbm;
        }
        g_upstream_last_ms = now_ms;

        if (reg_open && g_upstream_id == src_id) {
            printf("[UP] Upstream GW 0x%08X has reg window open — arming SUBSCRIPTION\n",
                   src_id);
            g_needs_tx = true;
        }

    } else if (g_state == GW_UP_REGISTERING) {
        /* Keep tracking the candidate while waiting for a response. */
        if (src_id == g_upstream_id) {
            g_upstream_rssi    = rssi_dbm;
            g_upstream_last_ms = now_ms;
            /* Re-arm TX if the registration window re-opened (e.g. new epoch). */
            if (reg_open && !g_needs_tx) {
                g_needs_tx = true;
            }
        }

    } else if (g_state == GW_UP_REGISTERED) {
        if (src_id == g_upstream_id) {
            g_upstream_rssi    = rssi_dbm;
            g_upstream_last_ms = now_ms;
        }

    } else if (g_state == GW_UP_STANDALONE) {
        /*
         * Lowest-node-ID-wins root election: if a stratum-0 gateway with
         * a lower ID appears and has its registration window open, yield to
         * it.  This handles the case where we went standalone before the
         * true root appeared on the network.
         */
        if (stratum == EMESH_STRATUM_GPS && src_id < g_my_id && reg_open) {
            printf("[UP] Lower-ID gateway 0x%08X seen — yielding root to it\n",
                   src_id);
            enter_scanning(now_ms);
            g_upstream_id      = src_id;
            g_upstream_stratum = stratum;
            g_upstream_rssi    = rssi_dbm;
            g_upstream_last_ms = now_ms;
            g_needs_tx         = true;
        }
    }
}

void gw_upstream_on_reg_response(uint8_t  status,
                                  uint8_t  slot,
                                  uint8_t  upstream_stratum,
                                  uint64_t now_ms)
{
    if (g_state != GW_UP_REGISTERING) {
        return; /* stale or unexpected */
    }

    if (status == REG_STATUS_OK) {
        g_state            = GW_UP_REGISTERED;
        g_assigned_slot    = slot;
        g_my_stratum       = (upstream_stratum < 254U)
                             ? (uint8_t)(upstream_stratum + 1U)
                             : 254U;
        g_upstream_last_ms = now_ms;
        printf("[UP] Registered with upstream GW 0x%08X  "
               "slot=%u  our_stratum=%u\n",
               g_upstream_id, g_assigned_slot, g_my_stratum);
    } else {
        printf("[UP] SUBSCRIPTION rejected by 0x%08X (status=0x%02X) — "
               "returning to scan\n", g_upstream_id, status);
        enter_scanning(now_ms);
    }
}

/* ── Periodic tick ───────────────────────────────────────────────────────── */

bool gw_upstream_tick(uint64_t now_ms)
{
    switch (g_state) {

    case GW_UP_SCANNING:
        if (g_needs_tx) {
            return true;
        }
        if ((now_ms - g_scan_start_ms) >= GW_UPSTREAM_SCAN_TIMEOUT_MS) {
            printf("[UP] Scan timeout — no upstream gateway found, "
                   "operating standalone (stratum 0)\n");
            g_state      = GW_UP_STANDALONE;
            g_my_stratum = EMESH_STRATUM_GPS;
        }
        break;

    case GW_UP_REGISTERING:
        if (g_needs_tx) {
            return true;
        }
        if ((now_ms - g_reg_sent_ms) >= GW_UPSTREAM_REG_TIMEOUT_MS) {
            if (g_retry_count < GW_UPSTREAM_MAX_RETRIES) {
                g_retry_count++;
                printf("[UP] REG_RESPONSE timeout — retry %u/%u\n",
                       g_retry_count, GW_UPSTREAM_MAX_RETRIES);
                g_needs_tx = true;
                return true;
            } else {
                printf("[UP] SUBSCRIPTION failed after %u retries — "
                       "returning to scan\n", GW_UPSTREAM_MAX_RETRIES);
                enter_scanning(now_ms);
            }
        }
        break;

    case GW_UP_REGISTERED:
        if ((now_ms - g_upstream_last_ms) >= GW_UPSTREAM_PEER_TIMEOUT_MS) {
            printf("[UP] Upstream GW 0x%08X lost (no beacon for %u s) — "
                   "returning to scan\n",
                   g_upstream_id,
                   GW_UPSTREAM_PEER_TIMEOUT_MS / 1000U);
            enter_scanning(now_ms);
        }
        break;

    case GW_UP_STANDALONE:
        /*
         * Not truly terminal: if a gateway with a lower node ID appears
         * it takes priority as the root (lowest ID wins).  We yield by
         * re-entering SCANNING.  See also gw_upstream_on_beacon().
         */
        break;
    }

    return false;
}

void gw_upstream_sub_sent(uint64_t now_ms)
{
    g_reg_sent_ms = now_ms;
    g_needs_tx    = false;

    if (g_state == GW_UP_SCANNING) {
        g_state = GW_UP_REGISTERING;
        printf("[UP] SUBSCRIPTION sent to 0x%08X  "
               "rssi=%.0f dBm  retry=%u\n",
               g_upstream_id, (double)g_upstream_rssi, g_retry_count);
    }
}

/* ── TX frame builder ────────────────────────────────────────────────────── */

uint8_t gw_upstream_sub_frame(uint8_t *buf, uint8_t capacity, uint16_t seq)
{
    emesh_frame_header_t hdr;
    uint8_t             *payload;
    uint8_t              total = (uint8_t)(EMESH_FRAME_HEADER_SIZE
                                           + SUB_REQUEST_PAYLOAD_SIZE);

    if (buf == NULL || capacity < total || g_upstream_id == 0U) {
        return 0U;
    }

    memset(&hdr, 0, sizeof(hdr));
    hdr.type    = EMESH_PACKET_TYPE_SUBSCRIPTION;
    hdr.flags   = 0U;
    hdr.ttl     = GW_UP_SUB_TTL;
    hdr.src_id  = g_my_id;
    hdr.dest_id = g_upstream_id;
    hdr.seq     = seq;
    hdr.op      = EMESH_OP_MESH_REGISTER;
    hdr.length  = SUB_REQUEST_PAYLOAD_SIZE;

    emesh_frame_encode_header(&hdr, buf);

    payload    = buf + EMESH_FRAME_HEADER_SIZE;
    payload[0] = EMESH_PACKET_TYPE_SUBSCRIPTION;
    payload[1] = g_capability;
    payload[2] = 0U;                            /* requested_slots (CSMA = 0)  */
    payload[3] = (uint8_t)(int8_t)g_upstream_rssi; /* heard_rssi               */
    payload[4] = (uint8_t)(seq & 0xFFU);
    payload[5] = (uint8_t)(seq >> 8);

    return total;
}

/* ── State queries ───────────────────────────────────────────────────────── */

gw_upstream_state_t gw_upstream_state(void) { return g_state; }

const char *gw_upstream_state_name(void)
{
    switch (g_state) {
    case GW_UP_SCANNING:     return "SCANNING";
    case GW_UP_REGISTERING:  return "REGISTERING";
    case GW_UP_REGISTERED:   return "REGISTERED";
    case GW_UP_STANDALONE:   return "STANDALONE";
    default:                 return "UNKNOWN";
    }
}

bool gw_upstream_is_ready(void)
{
    return g_state == GW_UP_REGISTERED || g_state == GW_UP_STANDALONE;
}

uint8_t gw_upstream_stratum(void)
{
    return g_my_stratum; /* EMESH_STRATUM_GPS (0) if standalone,
                            upstream+1 if registered,
                            EMESH_STRATUM_UNKNOWN (255) while scanning */
}

uint8_t gw_upstream_slot(void)
{
    return (g_state == GW_UP_REGISTERED) ? g_assigned_slot : 0xFFU;
}

uint32_t gw_upstream_peer_id(void)
{
    return g_upstream_id;
}

float gw_upstream_peer_rssi(void)
{
    return g_upstream_rssi;
}

/*
 * gw_lora_peers.c — Peer gateway table (discovered via LoRa beacons).
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "gw_lora_peers.h"

/* ── Private state ────────────────────────────────────────────────────────── */

static gw_lora_peer_t g_peers[GW_LORA_PEERS_MAX];
static uint8_t        g_count = 0U;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/* Return the index of a peer by node_id, or GW_LORA_PEERS_MAX if not found. */
static uint8_t peer_find(uint32_t gw_id)
{
    uint8_t i;
    for (i = 0U; i < g_count; i++) {
        if (g_peers[i].gw_id == gw_id) {
            return i;
        }
    }
    return GW_LORA_PEERS_MAX;
}

/*
 * Find the index of the entry with the oldest last_seen timestamp.
 * Used when the table is full and a new peer must be accommodated.
 */
static uint8_t peer_oldest(void)
{
    uint8_t oldest_idx  = 0U;
    time_t  oldest_time = g_peers[0].last_seen;
    uint8_t i;

    for (i = 1U; i < g_count; i++) {
        if (g_peers[i].last_seen < oldest_time) {
            oldest_time = g_peers[i].last_seen;
            oldest_idx  = i;
        }
    }
    return oldest_idx;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void gw_lora_peers_init(void)
{
    memset(g_peers, 0, sizeof(g_peers));
    g_count = 0U;
}

void gw_lora_peers_update(uint32_t gw_id,
                           float    rssi_dbm,
                           float    snr_db,
                           uint8_t  stratum,
                           uint8_t  node_count,
                           uint64_t rx_utc_ms,
                           bool    *is_new_out)
{
    uint8_t idx   = peer_find(gw_id);
    time_t  now   = time(NULL);
    bool    is_new = (idx == GW_LORA_PEERS_MAX);

    if (is_new_out != NULL) {
        *is_new_out = is_new;
    }

    if (is_new) {
        if (g_count < GW_LORA_PEERS_MAX) {
            idx = g_count;
            g_count++;
        } else {
            /* Table full — replace the oldest entry. */
            idx = peer_oldest();
            printf("[GW-PEERS] Table full — evicting 0x%08X to make room for "
                   "0x%08X\n", g_peers[idx].gw_id, gw_id);
        }
        g_peers[idx].gw_id      = gw_id;
        g_peers[idx].first_seen = now;
    }

    g_peers[idx].rssi_dbm   = rssi_dbm;
    g_peers[idx].snr_db     = snr_db;
    g_peers[idx].stratum    = stratum;
    g_peers[idx].node_count = node_count;
    g_peers[idx].last_seen  = now;
    g_peers[idx].rx_utc_ms  = rx_utc_ms;
}

uint8_t gw_lora_peers_count(void)
{
    return g_count;
}

const gw_lora_peer_t *gw_lora_peers_get(uint8_t idx)
{
    if (idx >= g_count) {
        return NULL;
    }
    return &g_peers[idx];
}

bool gw_lora_peers_collision(uint64_t our_beacon_utc_ms, uint32_t delta_ms)
{
    uint8_t i;
    for (i = 0U; i < g_count; i++) {
        uint64_t peer_rx = g_peers[i].rx_utc_ms;
        /*
         * A collision is inferred when a peer's beacon was received within
         * delta_ms of our own last beacon transmission.  Because we observe
         * the peer's beacon *after* it was sent (RF propagation + processing
         * latency is negligible at LoRa distances), if rx_utc_ms falls within
         * [our_beacon_utc_ms - delta_ms, our_beacon_utc_ms + delta_ms] it is
         * very likely the peer fired at approximately the same time as us.
         */
        uint64_t lo = (our_beacon_utc_ms > (uint64_t)delta_ms)
                      ? (our_beacon_utc_ms - (uint64_t)delta_ms)
                      : 0U;
        uint64_t hi = our_beacon_utc_ms + (uint64_t)delta_ms;
        if (peer_rx >= lo && peer_rx <= hi) {
            return true;
        }
    }
    return false;
}

uint8_t gw_lora_peers_expire(time_t timeout_s)
{
    time_t  now     = time(NULL);
    uint8_t removed = 0U;
    uint8_t i       = 0U;

    while (i < g_count) {
        if ((now - g_peers[i].last_seen) > timeout_s) {
            printf("[GW-PEERS] Peer 0x%08X expired (silent for %lds)\n",
                   g_peers[i].gw_id,
                   (long)(now - g_peers[i].last_seen));
            g_count--;
            g_peers[i] = g_peers[g_count]; /* swap-remove */
            removed++;
            /* Re-check the swapped-in entry at the same index. */
        } else {
            i++;
        }
    }
    return removed;
}

void gw_lora_peers_print(void)
{
    uint8_t i;
    if (g_count == 0U) {
        printf("[GW-PEERS] No peer gateways heard\n");
        return;
    }
    printf("[GW-PEERS] %u peer gateway(s) in range:\n", g_count);
    for (i = 0U; i < g_count; i++) {
        printf("[GW-PEERS]   [%u] 0x%08X  rssi=%.0f dBm  snr=%.1f dB  "
               "stratum=%u  nodes=%u  last_seen=%lds ago\n",
               i,
               g_peers[i].gw_id,
               g_peers[i].rssi_dbm,
               g_peers[i].snr_db,
               g_peers[i].stratum,
               g_peers[i].node_count,
               (long)(time(NULL) - g_peers[i].last_seen));
    }
}

/*
 * gw_lora_peers.h — Table of peer gateways heard via LoRa beacons.
 *
 * When the gateway receives a beacon from another GATEWAY-tier node it updates
 * this table.  The table is used to:
 *   1. Log the mesh topology (which gateways are in range).
 *   2. Detect beacon-slot collisions so the local gateway can shift its own
 *      beacon slot when another gateway is transmitting at the same time.
 *
 * All state is in RAM; there is no persistence (a missed beacon from a peer
 * during a restart is harmless — the peer will be re-discovered on the next
 * super-frame).
 *
 * Thread-safety: single-threaded use only (main gateway loop).
 */

#ifndef GW_LORA_PEERS_H
#define GW_LORA_PEERS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum number of peer gateways tracked simultaneously. */
#define GW_LORA_PEERS_MAX  8U

/*
 * Entry for one peer gateway heard via LoRa.
 *
 * rx_utc_ms: UTC timestamp (ms) of the most recently received beacon from
 *            this peer.  Used together with the local gateway's own beacon
 *            schedule to infer whether they collide.
 */
typedef struct {
    uint32_t gw_id;         /* peer's 32-bit node ID                          */
    float    rssi_dbm;      /* RSSI of the last received beacon                */
    float    snr_db;        /* SNR of the last received beacon                 */
    uint8_t  stratum;       /* stratum reported in the peer's beacon           */
    uint8_t  node_count;    /* registered-node count from the peer's beacon    */
    time_t   first_seen;    /* wall-clock time of first reception              */
    time_t   last_seen;     /* wall-clock time of last reception               */
    uint64_t rx_utc_ms;     /* UTC ms of last beacon (for collision detection) */
} gw_lora_peer_t;

/* ── Lifecycle ────────────────────────────────────────────────────────────── */

void gw_lora_peers_init(void);

/* ── Update ───────────────────────────────────────────────────────────────── */

/*
 * Record a beacon received from peer gateway gw_id.
 * Creates a new entry if not already known (evicts oldest if the table is
 * full).  is_new_out is set to true when a previously unseen gateway is added.
 */
void gw_lora_peers_update(uint32_t gw_id,
                           float    rssi_dbm,
                           float    snr_db,
                           uint8_t  stratum,
                           uint8_t  node_count,
                           uint64_t rx_utc_ms,
                           bool    *is_new_out);

/* ── Query ────────────────────────────────────────────────────────────────── */

uint8_t                gw_lora_peers_count(void);
const gw_lora_peer_t  *gw_lora_peers_get(uint8_t idx);

/*
 * Return true if a peer gateway sent its beacon within delta_ms milliseconds
 * of our_beacon_utc_ms.  Used to detect a collision on the same TDMA slot.
 */
bool gw_lora_peers_collision(uint64_t our_beacon_utc_ms, uint32_t delta_ms);

/* ── Expiry ───────────────────────────────────────────────────────────────── */

/*
 * Remove peers not heard within timeout_s seconds.
 * Returns the number of entries evicted.
 */
uint8_t gw_lora_peers_expire(time_t timeout_s);

/* ── Logging ──────────────────────────────────────────────────────────────── */

/* Print the current peer table to stdout. */
void gw_lora_peers_print(void);

#ifdef __cplusplus
}
#endif

#endif /* GW_LORA_PEERS_H */

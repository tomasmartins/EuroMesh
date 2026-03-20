/*
 * beacon_packet.h — Beacon payload encode / decode.
 *
 * Every relay and gateway transmits a beacon on its assigned TDMA slot.
 * Leaf nodes (EMESH_NODE_TIER_NODE) do NOT transmit beacons.
 *
 * Beacon payload wire format (24 bytes, immediately after the 16-byte header):
 *
 *  Off  Size  Field
 *  ───  ────  ──────────────────────────────────────────────────────────────
 *                   ── Sync block (16 bytes) ──
 *   0    1    type          = EMESH_PACKET_TYPE_BEACON (0x01)
 *   1    1    sync_flags    TIME_SYNC_FLAG_* bitmask
 *   2    8    utc_epoch_ms  milliseconds since Unix epoch, little-endian
 *  10    4    pps_tick_ms   HAL_GetTick() value at last PPS pulse, LE
 *  14    1    stratum       0 = GPS authority, 255 = unknown
 *  15    1    net_flags     bit 0: registration window open
 *                   ── Telemetry block (8 bytes) ──
 *  16    1    telem_flags   BEACON_TELEM_FLAG_* bitmask
 *  17    1    tx_power_dbm  TX power at time of this beacon (int8_t)
 *  18    1    upstream_rssi RSSI of best upstream beacon received (int8_t)
 *                           0x80 (INT8_MIN) if no upstream (e.g., gateway)
 *  19    1    node_count    number of directly registered downstream nodes
 *  20    4    uptime_s      node uptime in seconds, little-endian uint32
 *
 * Total payload: 24 bytes.  Full wire frame: 16 (header) + 24 = 40 bytes.
 *
 * Receivers must accept beacons with length >= BEACON_PAYLOAD_SIZE_MIN (16)
 * for backward compatibility; the telemetry block is optional.
 */

#ifndef BEACON_PACKET_H
#define BEACON_PACKET_H

#include <stdbool.h>
#include <stdint.h>

#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Payload sizes ───────────────────────────────────────────────────────── */
#define BEACON_PAYLOAD_SIZE_MIN    16U  /* sync block only (backwards compat) */
#define BEACON_PAYLOAD_SIZE        24U  /* sync + telemetry (current version) */

/* ── net_flags bits ──────────────────────────────────────────────────────── */
#define BEACON_NET_FLAG_REG_OPEN   0x01U  /* registration window is open       */

/* ── telem_flags bits ────────────────────────────────────────────────────── */
#define BEACON_TELEM_FLAG_TX_PWR   0x01U  /* tx_power_dbm field is valid       */
#define BEACON_TELEM_FLAG_RSSI     0x02U  /* upstream_rssi field is valid      */
#define BEACON_TELEM_FLAG_NODES    0x04U  /* node_count field is valid         */
#define BEACON_TELEM_FLAG_UPTIME   0x08U  /* uptime_s field is valid           */

/* ── Payload struct ──────────────────────────────────────────────────────── */
typedef struct {
    /* ── Sync block ─────────────────────────────────────────────────────── */
    uint8_t  sync_flags;          /* TIME_SYNC_FLAG_*                        */
    uint64_t utc_epoch_ms;        /* UTC at the moment of encoding           */
    uint32_t pps_tick_ms;         /* HAL_GetTick() at last PPS edge          */
    uint8_t  stratum;             /* sender's stratum level                  */
    uint8_t  net_flags;           /* network control flags                   */

    /* ── Telemetry block ─────────────────────────────────────────────────── */
    uint8_t  telem_flags;         /* BEACON_TELEM_FLAG_* validity bitmask    */
    int8_t   tx_power_dbm;        /* TX power (dBm) used for this beacon     */
    int8_t   upstream_rssi_dbm;   /* RSSI of best upstream beacon received;  */
                                  /* INT8_MIN (0x80) if no upstream          */
    uint8_t  node_count;          /* registered downstream nodes             */
    uint32_t uptime_s;            /* node uptime in seconds                  */
} beacon_payload_t;

/* ── Codec ───────────────────────────────────────────────────────────────── */

/*
 * Encode a beacon payload into buf[0..BEACON_PAYLOAD_SIZE-1].
 * Returns BEACON_PAYLOAD_SIZE on success, 0 if buf is NULL or too small.
 * capacity must be >= BEACON_PAYLOAD_SIZE (24).
 */
uint8_t beacon_payload_encode(const beacon_payload_t *beacon,
                              uint8_t *buf, uint8_t capacity);

/*
 * Decode a beacon payload from buf (length bytes) into *out.
 * Accepts length >= BEACON_PAYLOAD_SIZE_MIN (16); fills telemetry fields
 * with zero / invalid defaults if the telemetry block is absent.
 * Returns true on success; false if the buffer is too short or type mismatch.
 */
bool beacon_payload_decode(const uint8_t *buf, uint8_t length,
                           beacon_payload_t *out);

/*
 * Build a beacon_payload_t from the current time_sync state.
 * now_tick_ms should be HAL_GetTick() at the moment of building.
 * Telemetry fields are zeroed; caller should populate them before encoding.
 */
void beacon_payload_from_sync(const time_sync_t *sync,
                              uint32_t now_tick_ms,
                              bool reg_window_open,
                              beacon_payload_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_PACKET_H */

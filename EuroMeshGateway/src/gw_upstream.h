/*
 * gw_upstream.h — Gateway upstream registration client.
 *
 * On boot, the gateway enters SCANNING state and listens for beacons from a
 * parent gateway (stratum 0).  When a beacon with the registration window open
 * is heard, it sends a SUBSCRIPTION and waits for a REG_RESPONSE.  On
 * success it enters REGISTERED state with an assigned beacon slot and stratum.
 *
 * If no upstream gateway beacon is heard within GW_UPSTREAM_SCAN_TIMEOUT_MS,
 * the gateway transitions to STANDALONE and operates as a root (stratum 0).
 *
 * State diagram:
 *
 *   SCANNING ──(beacon stratum-0 + reg_open)──► REGISTERING
 *   SCANNING ──(scan timeout)─────────────────► STANDALONE
 *   REGISTERING ──(REG_RESPONSE OK)───────────► REGISTERED
 *   REGISTERING ──(retries exhausted)─────────► SCANNING
 *   REGISTERED ──(upstream lost)──────────────► SCANNING
 *
 * All timestamps passed to this module must be monotonic (ms) to avoid
 * sensitivity to NTP jumps.
 */

#ifndef GW_UPSTREAM_H
#define GW_UPSTREAM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Tuning constants ────────────────────────────────────────────────────── */

/* Scan for an upstream gateway for this long before going standalone. */
#define GW_UPSTREAM_SCAN_TIMEOUT_MS     90000U  /* 90 s (1.5 × 60 s beacon) */

/* Timeout waiting for REG_RESPONSE after sending SUBSCRIPTION. */
#define GW_UPSTREAM_REG_TIMEOUT_MS       5000U  /* 5 s per retry            */

/* Maximum SUBSCRIPTION retries before reverting to SCANNING. */
#define GW_UPSTREAM_MAX_RETRIES             3U

/* Upstream gateway considered lost if no beacon for this long. */
#define GW_UPSTREAM_PEER_TIMEOUT_MS    180000U  /* 3 × 60 s beacon period   */

/* ── State ───────────────────────────────────────────────────────────────── */

typedef enum {
    GW_UP_SCANNING,     /* Listening for an upstream gateway beacon          */
    GW_UP_REGISTERING,  /* SUBSCRIPTION sent; awaiting REG_RESPONSE          */
    GW_UP_REGISTERED,   /* Registered; slot and stratum assigned by upstream */
    GW_UP_STANDALONE,   /* No upstream found; operating as root (stratum 0)  */
} gw_upstream_state_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/*
 * Initialise the state machine.  Call once at startup, before the main loop.
 *   my_id:      this gateway's 32-bit node ID
 *   capability: EMESH_NODE_TIER_GATEWAY | EMESH_NODE_FLAG_*
 *   now_ms:     current monotonic timestamp (ms)
 */
void gw_upstream_init(uint32_t my_id, uint8_t capability, uint64_t now_ms);

/* ── Event inputs ────────────────────────────────────────────────────────── */

/*
 * Feed a received beacon into the state machine.
 * Only stratum-0 beacons are accepted as upstream candidates.
 *   src_id:   beacon sender node ID
 *   stratum:  stratum field from the beacon payload
 *   rssi_dbm: receive RSSI
 *   reg_open: true if BEACON_NET_FLAG_REG_OPEN is set
 *   now_ms:   current monotonic timestamp (ms)
 */
void gw_upstream_on_beacon(uint32_t src_id,
                            uint8_t  stratum,
                            float    rssi_dbm,
                            bool     reg_open,
                            uint64_t now_ms);

/*
 * Feed a REG_RESPONSE addressed to this gateway into the state machine.
 *   status:            REG_STATUS_OK, _REJECTED, or _FULL
 *   slot:              beacon slot assigned by the upstream gateway
 *   upstream_stratum:  stratum field from the REG_RESPONSE payload
 *   now_ms:            current monotonic timestamp (ms)
 */
void gw_upstream_on_reg_response(uint8_t  status,
                                  uint8_t  slot,
                                  uint8_t  upstream_stratum,
                                  uint64_t now_ms);

/* ── Periodic tick ───────────────────────────────────────────────────────── */

/*
 * Call each main-loop iteration to drive timeouts and retries.
 * Returns true when a SUBSCRIPTION frame should be transmitted.
 * Caller must then call gw_upstream_sub_frame() to build the bytes, transmit
 * them, and call gw_upstream_sub_sent(now_ms) to record the send time.
 */
bool gw_upstream_tick(uint64_t now_ms);

/* Record that the SUBSCRIPTION was successfully transmitted. */
void gw_upstream_sub_sent(uint64_t now_ms);

/* ── TX frame builder ────────────────────────────────────────────────────── */

/*
 * Build the complete SUBSCRIPTION frame (16-byte header + 6-byte payload).
 * buf must be at least 22 bytes.  seq is the caller-managed frame sequence
 * number (already incremented).
 * Returns the total frame length (22), or 0 on error.
 */
uint8_t gw_upstream_sub_frame(uint8_t *buf, uint8_t capacity, uint16_t seq);

/* ── State queries ───────────────────────────────────────────────────────── */

gw_upstream_state_t gw_upstream_state(void);
const char         *gw_upstream_state_name(void);

/* True when the gateway is ready to begin beaconing (REGISTERED or STANDALONE). */
bool gw_upstream_is_ready(void);

/*
 * Stratum to advertise in outgoing beacons:
 *   REGISTERED  → upstream_stratum + 1
 *   STANDALONE  → EMESH_STRATUM_GPS (0)
 *   Other       → EMESH_STRATUM_UNKNOWN (255)
 */
uint8_t gw_upstream_stratum(void);

/*
 * Beacon slot assigned by the upstream gateway.
 * Returns 0xFF if not in REGISTERED state.
 */
uint8_t gw_upstream_slot(void);

/* Node ID of the upstream gateway (0 if STANDALONE or not registered). */
uint32_t gw_upstream_peer_id(void);

/* RSSI of the last beacon received from the upstream gateway (dBm). */
float gw_upstream_peer_rssi(void);

#ifdef __cplusplus
}
#endif

#endif /* GW_UPSTREAM_H */

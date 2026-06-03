/*
 * location_adv.h — Location advertisement packet codec.
 *
 * LOCATION_ADV is broadcast by any node that knows its position — typically
 * a gateway with a configured or GPS-derived location, or a node with an
 * on-board GPS.  Neighbouring gateways receive it and store the sender's
 * position in their peer / node tables.
 *
 * Wire format (14 bytes, follows the 16-byte frame header):
 *
 *  Off  Size  Field
 *  ───  ────  ──────────────────────────────────────────────────────────────
 *   0    1    type      = EMESH_PACKET_TYPE_LOCATION_ADV (0x09)
 *   1    1    flags     bit 0: GPS_VALID — live GPS fix (not manual config)
 *                       bit 1: ALT_VALID — altitude field is meaningful
 *   2    4    lat_udeg  int32 LE — latitude  in microdegrees (deg × 1 000 000)
 *   6    4    lon_udeg  int32 LE — longitude in microdegrees (deg × 1 000 000)
 *  10    4    alt_cm    int32 LE — altitude in centimetres above WGS-84
 *
 * Range:
 *   lat_udeg: ±90 000 000   (±90°, fits comfortably in int32)
 *   lon_udeg: ±180 000 000  (±180°, fits comfortably in int32)
 *   alt_cm:   ±2 147 483 km (more than enough)
 *
 * ── RSSI-based node location inference ───────────────────────────────────────
 *
 * One of the EuroMesh design goals is to infer the position of nodes that have
 * no GPS using RSSI measurements from multiple gateways (trilateration).
 *
 * Model:
 *   1. Each gateway records the RSSI of every received frame per node_id.
 *   2. RSSI → distance via the log-distance path-loss model:
 *        d = d0 × 10^((RSSI(d0) − RSSI_rx) / (10 × n))
 *      where d0 = 1 m reference distance, n = path-loss exponent (≈2–3 urban).
 *   3. With ≥3 gateways providing (position, distance) pairs, the node
 *      position is found as the least-squares intersection of the distance
 *      circles (2-D) or spheres (3-D).
 *   4. Confidence increases with more gateways and decreasing RSSI variance.
 *
 * Current implementation:
 *   - Each gateway records RSSI per node locally in gw_node_entry_t.
 *   - Inter-gateway RSSI sharing (needed for trilateration) is a future
 *     feature; it will use either IP backhaul or a new RSSI_REPORT LoRa frame.
 */

#ifndef LOCATION_ADV_H
#define LOCATION_ADV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Payload size ────────────────────────────────────────────────────────── */
#define LOCATION_ADV_PAYLOAD_SIZE   14U

/* ── flags byte ──────────────────────────────────────────────────────────── */
#define LOCATION_ADV_FLAG_GPS_VALID  0x01U  /* live GPS fix (not manual)    */
#define LOCATION_ADV_FLAG_ALT_VALID  0x02U  /* altitude field is valid      */

/* ── Payload struct ──────────────────────────────────────────────────────── */
typedef struct {
    bool   gps_valid;  /* true = live GPS fix; false = manually configured   */
    bool   alt_valid;  /* true = alt_m is meaningful                         */
    double lat_deg;    /* latitude  in decimal degrees (−90 … +90)           */
    double lon_deg;    /* longitude in decimal degrees (−180 … +180)         */
    float  alt_m;      /* altitude in metres above WGS-84 ellipsoid          */
} location_adv_t;

/* ── Codec ───────────────────────────────────────────────────────────────── */

/*
 * Encode a location_adv_t into buf[0..LOCATION_ADV_PAYLOAD_SIZE-1].
 * Returns LOCATION_ADV_PAYLOAD_SIZE on success, 0 on error.
 * capacity must be >= LOCATION_ADV_PAYLOAD_SIZE (14).
 */
uint8_t location_adv_encode(const location_adv_t *loc,
                             uint8_t *buf, uint8_t capacity);

/*
 * Decode a LOCATION_ADV payload from buf (length bytes) into *out.
 * Returns true on success; false if buf is NULL, too short, or type mismatch.
 */
bool location_adv_decode(const uint8_t *buf, uint8_t length,
                          location_adv_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LOCATION_ADV_H */

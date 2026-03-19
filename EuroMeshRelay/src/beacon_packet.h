/*
 * beacon_packet.h — Beacon payload encode / decode.
 *
 * Beacon payload wire format (16 bytes, immediately after the 16-byte header):
 *
 *  Off  Size  Field
 *  ───  ────  ──────────────────────────────────────────────────────────────
 *   0    1    type          = EMESH_PACKET_TYPE_BEACON (0x01)
 *   1    1    sync_flags    TIME_SYNC_FLAG_* bitmask
 *   2    8    utc_epoch_ms  milliseconds since Unix epoch, little-endian
 *  10    4    pps_tick_ms   HAL_GetTick() value at last PPS pulse, LE
 *  14    1    stratum       0 = GPS authority, 255 = unknown
 *  15    1    net_flags     bit 0: registration window open
 *
 * Total payload: 16 bytes.  Full wire frame: 16 (header) + 16 = 32 bytes.
 */

#ifndef BEACON_PACKET_H
#define BEACON_PACKET_H

#include <stdbool.h>
#include <stdint.h>

#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BEACON_PAYLOAD_SIZE         16U

/* net_flags bits */
#define BEACON_NET_FLAG_REG_OPEN    0x01U  /* registration window is open */

typedef struct {
    uint8_t  sync_flags;      /* TIME_SYNC_FLAG_* */
    uint64_t utc_epoch_ms;    /* UTC at the moment of encoding */
    uint32_t pps_tick_ms;     /* HAL_GetTick() at last PPS edge */
    uint8_t  stratum;         /* sender's stratum level */
    uint8_t  net_flags;       /* network control flags */
} beacon_payload_t;

/*
 * Encode a beacon payload into buf[0..BEACON_PAYLOAD_SIZE-1].
 * Uses sync to derive current UTC and PPS.
 * Returns BEACON_PAYLOAD_SIZE on success, 0 if buf is NULL or too small.
 */
uint8_t beacon_payload_encode(const beacon_payload_t *beacon,
                              uint8_t *buf, uint8_t capacity);

/*
 * Decode a beacon payload from buf (length bytes) into *out.
 * Returns true on success; false if the buffer is too short or type mismatch.
 */
bool beacon_payload_decode(const uint8_t *buf, uint8_t length,
                           beacon_payload_t *out);

/*
 * Build a beacon_payload_t from the current time_sync state.
 * now_tick_ms should be HAL_GetTick() at the moment of building.
 */
void beacon_payload_from_sync(const time_sync_t *sync,
                              uint32_t now_tick_ms,
                              bool reg_window_open,
                              beacon_payload_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BEACON_PACKET_H */

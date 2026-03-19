/*
 * neighbour_adv.h — Neighbour Advertisement packet codec.
 *
 * NEIGHBOUR_ADV payload wire format:
 *
 *  Off  Size  Field
 *  ───  ────  ──────────────────────────────────────────────────────────────
 *   0    1    type          = EMESH_PACKET_TYPE_NEIGHBOUR_ADV (0x05)
 *   1    1    entry_count   number of entries N (0 ≤ N ≤ NB_ADV_MAX_ENTRIES)
 *
 *  Per entry (8 bytes each):
 *   +0   4    node_id       little-endian uint32
 *   +4   1    rssi_dbm      int8 cast to uint8 (two's complement)
 *   +5   1    snr_db        int8 cast to uint8
 *   +6   1    tier          EMESH_NODE_TIER_*
 *   +7   1    stratum
 *
 * Maximum entries: 8.
 * Maximum payload: 2 + 8×8 = 66 bytes.
 * Full wire frame:  16 (header) + 66 = 82 bytes  ← well within 256-byte FIFO.
 */

#ifndef NEIGHBOUR_ADV_H
#define NEIGHBOUR_ADV_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NB_ADV_MAX_ENTRIES   8U
#define NB_ADV_ENTRY_SIZE    8U
#define NB_ADV_HEADER_SIZE   2U
#define NB_ADV_MAX_PAYLOAD   (NB_ADV_HEADER_SIZE + NB_ADV_MAX_ENTRIES * NB_ADV_ENTRY_SIZE)

typedef struct {
    uint32_t node_id;
    int8_t   rssi_dbm;
    int8_t   snr_db;
    uint8_t  tier;
    uint8_t  stratum;
} nb_adv_entry_t;

typedef struct {
    uint8_t        entry_count;
    nb_adv_entry_t entries[NB_ADV_MAX_ENTRIES];
} nb_adv_payload_t;

/*
 * Encode a NEIGHBOUR_ADV payload into buf.
 * Returns bytes written (NB_ADV_HEADER_SIZE + entry_count*NB_ADV_ENTRY_SIZE),
 * or 0 on error.
 */
uint8_t nb_adv_encode(const nb_adv_payload_t *adv,
                      uint8_t *buf, uint8_t capacity);

/*
 * Decode a NEIGHBOUR_ADV payload from buf (length bytes) into *out.
 * Returns true on success.
 */
bool nb_adv_decode(const uint8_t *buf, uint8_t length, nb_adv_payload_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NEIGHBOUR_ADV_H */

/*
 * neighbour_table.h — Passive neighbour discovery and link-quality table.
 *
 * The table is updated by overhearing any received frame: src_id + RSSI/SNR
 * from sx1276_rx_metadata_t are recorded without any dedicated hello packet.
 *
 * Eviction policy (when full): replace the entry with the lowest rssi_dbm.
 * Stale entries (age > max_age_ms) are removed by nb_table_expire().
 *
 * RAM: 8 entries × 20 bytes = 160 bytes + 4 bytes overhead = 164 bytes.
 */

#ifndef NEIGHBOUR_TABLE_H
#define NEIGHBOUR_TABLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NB_TABLE_MAX_ENTRIES  8U

typedef struct {
    uint32_t node_id;         /* sender's src_id                         */
    int16_t  rssi_dbm;        /* last measured RSSI                      */
    int8_t   snr_db;          /* last measured SNR                       */
    uint8_t  tier;            /* EMESH_NODE_TIER_*                       */
    uint8_t  stratum;         /* sender's clock stratum                  */
    uint8_t  _pad[1];
    uint32_t last_heard_ms;   /* HAL_GetTick() at most recent reception  */
    uint32_t first_heard_ms;  /* HAL_GetTick() at first reception        */
} nb_entry_t;                 /* 20 bytes */

typedef struct {
    nb_entry_t entries[NB_TABLE_MAX_ENTRIES];  /* 160 bytes */
    uint8_t    count;
    uint8_t    _pad[3];
} nb_table_t;                 /* 164 bytes */

void nb_table_init(nb_table_t *tbl);

/*
 * Insert or update the entry for node_id.
 * If the table is full and the new RSSI is better than the worst entry,
 * that entry is replaced.
 */
void nb_table_update(nb_table_t *tbl,
                     uint32_t node_id,
                     int16_t  rssi_dbm,
                     int8_t   snr_db,
                     uint8_t  tier,
                     uint8_t  stratum,
                     uint32_t now_ms);

/* Return a pointer to the entry for node_id, or NULL if not found. */
const nb_entry_t *nb_table_find(const nb_table_t *tbl, uint32_t node_id);

/*
 * Remove entries older than max_age_ms.
 * Call once per super-frame.
 */
void nb_table_expire(nb_table_t *tbl, uint32_t now_ms, uint32_t max_age_ms);

/*
 * Return the entry with the best (highest) RSSI that has tier >= min_tier,
 * or NULL if no such entry exists.
 * Used to find the best relay/gateway for registration or routing.
 */
const nb_entry_t *nb_table_best(const nb_table_t *tbl, uint8_t min_tier);

#ifdef __cplusplus
}
#endif

#endif /* NEIGHBOUR_TABLE_H */

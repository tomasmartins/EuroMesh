#include "neighbour_table.h"
#include "emesh_node_caps.h"

void nb_table_init(nb_table_t *tbl)
{
    uint8_t i;

    if (tbl == NULL) {
        return;
    }
    for (i = 0U; i < NB_TABLE_MAX_ENTRIES; ++i) {
        tbl->entries[i].node_id = 0U;
    }
    tbl->count = 0U;
}

void nb_table_update(nb_table_t *tbl,
                     uint32_t node_id,
                     int16_t  rssi_dbm,
                     int8_t   snr_db,
                     uint8_t  tier,
                     uint8_t  stratum,
                     uint32_t now_ms)
{
    uint8_t i;
    uint8_t worst_idx = 0U;
    int16_t worst_rssi;

    if (tbl == NULL || node_id == 0U) {
        return;
    }

    /* Update existing entry if present. */
    for (i = 0U; i < tbl->count; ++i) {
        if (tbl->entries[i].node_id == node_id) {
            tbl->entries[i].rssi_dbm      = rssi_dbm;
            tbl->entries[i].snr_db        = snr_db;
            tbl->entries[i].tier          = tier;
            tbl->entries[i].stratum       = stratum;
            tbl->entries[i].last_heard_ms = now_ms;
            return;
        }
    }

    /* New entry: insert if there is space. */
    if (tbl->count < NB_TABLE_MAX_ENTRIES) {
        nb_entry_t *e       = &tbl->entries[tbl->count];
        e->node_id          = node_id;
        e->rssi_dbm         = rssi_dbm;
        e->snr_db           = snr_db;
        e->tier             = tier;
        e->stratum          = stratum;
        e->last_heard_ms    = now_ms;
        e->first_heard_ms   = now_ms;
        tbl->count++;
        return;
    }

    /* Table full: evict the entry with the worst (lowest) RSSI if the new
     * node is better — keeps the strongest-link set in memory. */
    worst_rssi = tbl->entries[0].rssi_dbm;
    for (i = 1U; i < NB_TABLE_MAX_ENTRIES; ++i) {
        if (tbl->entries[i].rssi_dbm < worst_rssi) {
            worst_rssi = tbl->entries[i].rssi_dbm;
            worst_idx  = i;
        }
    }

    if (rssi_dbm > worst_rssi) {
        nb_entry_t *e       = &tbl->entries[worst_idx];
        e->node_id          = node_id;
        e->rssi_dbm         = rssi_dbm;
        e->snr_db           = snr_db;
        e->tier             = tier;
        e->stratum          = stratum;
        e->last_heard_ms    = now_ms;
        e->first_heard_ms   = now_ms;
    }
}

const nb_entry_t *nb_table_find(const nb_table_t *tbl, uint32_t node_id)
{
    uint8_t i;

    if (tbl == NULL || node_id == 0U) {
        return NULL;
    }
    for (i = 0U; i < tbl->count; ++i) {
        if (tbl->entries[i].node_id == node_id) {
            return &tbl->entries[i];
        }
    }
    return NULL;
}

void nb_table_expire(nb_table_t *tbl, uint32_t now_ms, uint32_t max_age_ms)
{
    uint8_t i;

    if (tbl == NULL) {
        return;
    }
    i = 0U;
    while (i < tbl->count) {
        /* Unsigned subtraction handles HAL_GetTick() wrap-around. */
        if ((now_ms - tbl->entries[i].last_heard_ms) > max_age_ms) {
            /* Swap with the last entry to fill the gap. */
            tbl->count--;
            tbl->entries[i] = tbl->entries[tbl->count];
            /* Do not increment i — re-check the swapped entry. */
        } else {
            i++;
        }
    }
}

const nb_entry_t *nb_table_best(const nb_table_t *tbl, uint8_t min_tier)
{
    uint8_t           i;
    const nb_entry_t *best = NULL;

    if (tbl == NULL) {
        return NULL;
    }
    for (i = 0U; i < tbl->count; ++i) {
        const nb_entry_t *e = &tbl->entries[i];
        if (EMESH_NODE_TIER(e->tier) < min_tier) {
            continue;
        }
        if (best == NULL || e->rssi_dbm > best->rssi_dbm) {
            best = e;
        }
    }
    return best;
}

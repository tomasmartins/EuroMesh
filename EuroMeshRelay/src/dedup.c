#include "dedup.h"

void dedup_init(dedup_table_t *tbl)
{
    uint8_t i;

    if (tbl == NULL) {
        return;
    }
    for (i = 0U; i < DEDUP_TABLE_SIZE; ++i) {
        tbl->entries[i].src_id = 0U;
        tbl->entries[i].count  = 0U;
        tbl->entries[i].head   = 0U;
    }
    tbl->count = 0U;
}

/* Find an existing entry for src_id; return its index or DEDUP_TABLE_SIZE. */
static uint8_t dedup_find(const dedup_table_t *tbl, uint32_t src_id)
{
    uint8_t i;

    for (i = 0U; i < tbl->count; ++i) {
        if (tbl->entries[i].src_id == src_id) {
            return i;
        }
    }
    return DEDUP_TABLE_SIZE;
}

/* Find the index of the entry that has not been seen for the longest time. */
static uint8_t dedup_oldest(const dedup_table_t *tbl, uint32_t now_ms)
{
    uint8_t  oldest_idx  = 0U;
    uint32_t oldest_age  = 0U;
    uint8_t  i;

    for (i = 0U; i < tbl->count; ++i) {
        uint32_t age = now_ms - tbl->entries[i].last_seen_ms;
        if (age >= oldest_age) {
            oldest_age = age;
            oldest_idx = i;
        }
    }
    return oldest_idx;
}

bool dedup_is_duplicate(dedup_table_t *tbl, uint32_t src_id,
                        uint16_t seq, uint32_t now_ms)
{
    uint8_t       idx;
    dedup_entry_t *entry;
    uint8_t        i;

    if (tbl == NULL || src_id == 0U) {
        return false;
    }

    idx = dedup_find(tbl, src_id);

    if (idx == DEDUP_TABLE_SIZE) {
        /* Unknown source — allocate a slot. */
        if (tbl->count < DEDUP_TABLE_SIZE) {
            idx = tbl->count;
            tbl->count++;
        } else {
            /* Evict the oldest source. */
            idx = dedup_oldest(tbl, now_ms);
        }
        entry              = &tbl->entries[idx];
        entry->src_id      = src_id;
        entry->head        = 0U;
        entry->count       = 0U;
        entry->last_seen_ms = now_ms;
    } else {
        entry = &tbl->entries[idx];
    }

    /* Check if seq is already in the ring buffer. */
    for (i = 0U; i < entry->count; ++i) {
        if (entry->seen[i] == seq) {
            entry->last_seen_ms = now_ms;
            return true; /* duplicate */
        }
    }

    /* Not seen — record it. */
    entry->seen[entry->head] = seq;
    entry->head = (uint8_t)((entry->head + 1U) % DEDUP_WINDOW);
    if (entry->count < DEDUP_WINDOW) {
        entry->count++;
    }
    entry->last_seen_ms = now_ms;
    return false;
}

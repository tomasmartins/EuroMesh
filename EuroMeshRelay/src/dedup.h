/*
 * dedup.h — Per-source frame deduplication via sequence-number tracking.
 *
 * Each unique src_id gets a slot in a fixed table.  A frame is a duplicate
 * if its (src_id, seq) pair has been seen within the last DEDUP_WINDOW
 * sequence numbers from that source.
 *
 * RAM: 16 entries × 12 bytes = 192 bytes + 4 bytes overhead = 196 bytes.
 */

#ifndef DEDUP_H
#define DEDUP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DEDUP_TABLE_SIZE   16U  /* max unique sources tracked              */
#define DEDUP_WINDOW        8U  /* seq numbers remembered per source       */
#define DEDUP_MAX_AGE_MS  120000U /* 2 min: evict source if silent        */

typedef struct {
    uint32_t src_id;
    uint16_t seen[DEDUP_WINDOW]; /* ring buffer of recently seen seq nums  */
    uint8_t  head;               /* next write position in ring            */
    uint8_t  count;              /* how many entries are valid (0..WINDOW) */
    uint32_t last_seen_ms;
} dedup_entry_t;                 /* 12 bytes (with 2 bytes natural padding -> 16) */

typedef struct {
    dedup_entry_t entries[DEDUP_TABLE_SIZE];
    uint8_t       count;
    uint8_t       _pad[3];
} dedup_table_t;

void dedup_init(dedup_table_t *tbl);

/*
 * Check whether (src_id, seq) is a duplicate.
 * If not a duplicate, record it so future copies are rejected.
 * Evicts the oldest (least recently active) entry when the table is full.
 * Returns true if the frame is a duplicate and should be dropped.
 */
bool dedup_is_duplicate(dedup_table_t *tbl, uint32_t src_id,
                        uint16_t seq, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* DEDUP_H */

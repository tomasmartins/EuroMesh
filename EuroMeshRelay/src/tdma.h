/*
 * tdma.h — TDMA super-frame definitions.
 *
 * Super-frame layout (60 s period)
 * ─────────────────────────────────
 *
 *   Slot 0   BEACON — Gateway          │ 500 ms
 *   Slot 1   BEACON — Relay 1          │ 500 ms   ← assigned at registration
 *   Slot 2   BEACON — Relay 2          │ 500 ms
 *   ...
 *   Slot N-1 BEACON — Relay N-1        │ 500 ms
 *   Slot N   REGISTRATION window       │ 500 ms
 *   ─────────────────────────────────────
 *   Remaining (~55 s)  CSMA/CAD period — any registered node may transmit.
 *
 * Rules
 * ─────
 *   • Every relay and gateway transmits exactly one beacon per super-frame on
 *     its allocated slot.
 *   • Leaf nodes (EMESH_NODE_TIER_NODE) do NOT transmit beacons.
 *   • Nodes with EMESH_NODE_FLAG_FORWARD may retransmit data frames for
 *     neighbours that are out of direct range (multi-hop forwarding).
 *   • Slot 0 is always the gateway.  Relays are assigned slots 1..N-1 during
 *     the REG_RESPONSE handshake.
 *   • An unregistered relay uses slot 0 (standalone root) until it receives a
 *     REG_RESPONSE with a different slot assignment.
 *
 * Maximum beacon nodes:
 *   TDMA_MAX_BEACON_SLOTS includes the gateway (slot 0), leaving
 *   TDMA_MAX_BEACON_SLOTS-1 slots for relays.  Increasing this value
 *   shifts the registration window and the CSMA start later.
 */

#ifndef TDMA_H
#define TDMA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Super-frame timing ──────────────────────────────────────────────────── */
/*
 * Each beaconing node (gateway or relay) occupies one 100 ms slot within the
 * 1000 ms beacon window:
 *
 *   Slot 0   (  0– 100 ms) Gateway beacon
 *   Slot 1   (100– 200 ms) Relay 1 beacon
 *   ...
 *   Slot 9   (900–1000 ms) Relay 9 beacon
 *   Reg window (1000–1500 ms) SUBSCRIPTION contention
 *   CSMA period (~58.5 s)
 */
#define TDMA_SLOT_MS               100U    /* per-node beacon slot duration     */
#define TDMA_BEACON_WINDOW_MS     1000U    /* total duration of all beacon slots */
#define TDMA_REG_SLOT_MS           500U    /* registration window duration      */
#define TDMA_SUPERFRAME_PERIOD_MS  60000U  /* 60 s between super-frame starts   */

/* Number of beacon slots derived from the beacon window and slot size. */
#define TDMA_MAX_BEACON_SLOTS  (TDMA_BEACON_WINDOW_MS / TDMA_SLOT_MS)  /* 10 */

/*
 * Registration slot index: immediately after the last beacon slot.
 * Offset into the super-frame: TDMA_MAX_BEACON_SLOTS × TDMA_SLOT_MS
 *                            = 10 × 100 = 1000 ms = TDMA_BEACON_WINDOW_MS.
 */
#define TDMA_REG_SLOT_INDEX        TDMA_MAX_BEACON_SLOTS

/* Sentinel — node has not yet been assigned a slot. */
#define TDMA_NO_SLOT_ASSIGNED      0xFFU

/* Gateway always occupies slot 0. */
#define TDMA_GATEWAY_SLOT_INDEX    0U

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * Return the HAL_GetTick() (or mono_ms on Linux) value at which the next
 * super-frame starts (i.e., the start of the next slot-0 window).
 */
uint32_t tdma_next_frame_start_ms(uint32_t now_ms);

/*
 * Return the absolute tick at which slot_index starts within the super-frame
 * that begins at frame_start_ms.
 *
 *   slot 0 → frame_start_ms
 *   slot 1 → frame_start_ms + 500
 *   slot N → frame_start_ms + N × 500
 */
uint32_t tdma_slot_start_ms(uint32_t frame_start_ms, uint8_t slot_index);

/*
 * Convenience: return the start tick of the NEXT occurrence of slot_index,
 * given the current tick now_ms.
 */
uint32_t tdma_next_slot_ms(uint32_t now_ms, uint8_t slot_index);

/*
 * Return the start tick of the registration window that follows all beacon
 * slots in the super-frame beginning at frame_start_ms.
 */
uint32_t tdma_reg_window_start_ms(uint32_t frame_start_ms);

/*
 * Return the end tick of the registration window.
 */
uint32_t tdma_reg_window_end_ms(uint32_t frame_start_ms);

/*
 * Return true if now_ms falls within the registration window of the
 * super-frame whose slot-0 starts at frame_start_ms.
 */
bool tdma_is_in_reg_window(uint32_t frame_start_ms, uint32_t now_ms);

/* ── Legacy shim ─────────────────────────────────────────────────────────── */
/*
 * Equivalent to tdma_next_slot_ms(now_ms, TDMA_GATEWAY_SLOT_INDEX).
 * Kept for callers that have not yet migrated to the slot-aware API.
 */
static inline uint32_t tdma_next_beacon_ms(uint32_t now_ms)
{
    return tdma_next_slot_ms(now_ms, TDMA_GATEWAY_SLOT_INDEX);
}

#ifdef __cplusplus
}
#endif

#endif /* TDMA_H */

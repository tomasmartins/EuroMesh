/*
 * tdma.h — TDMA super-frame definitions.
 *
 * After registration, nodes communicate via CSMA/CAD-LBT on the shared
 * channel.  TDMA is used ONLY for:
 *
 *   Slot 0  BEACON      — Gateway/Relay transmits; all nodes receive.
 *   Slot 1  REGISTRATION — Unregistered nodes contend (CSMA/CA).
 *
 * Between super-frames the channel is unscheduled: any registered node
 * may transmit at any time using CAD-based listen-before-talk.
 */

#ifndef TDMA_H
#define TDMA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Super-frame timing ──────────────────────────────────────────────────── */
#define TDMA_BEACON_SLOT_MS        500U   /* beacon slot duration              */
#define TDMA_REG_SLOT_MS           500U   /* registration window duration      */
#define TDMA_SUPERFRAME_PERIOD_MS  60000U /* 60 s between beacon slots         */

/* ── Slot indices ────────────────────────────────────────────────────────── */
#define TDMA_BEACON_SLOT_INDEX     0U
#define TDMA_REG_SLOT_INDEX        1U
#define TDMA_FRAME_SLOT_COUNT      2U

/* ── Sentinel ────────────────────────────────────────────────────────────── */
#define TDMA_NO_SLOT_ASSIGNED      0xFFU

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * Return the HAL_GetTick() value at which the next beacon slot starts,
 * given the current tick now_ms.
 */
uint32_t tdma_next_beacon_ms(uint32_t now_ms);

/*
 * Return the start tick of the registration window that follows the
 * beacon slot beginning at beacon_start_ms.
 */
uint32_t tdma_reg_window_start_ms(uint32_t beacon_start_ms);

/*
 * Return the end tick of the registration window.
 */
uint32_t tdma_reg_window_end_ms(uint32_t beacon_start_ms);

/*
 * Return true if now_ms falls within the registration window of the
 * super-frame whose beacon slot starts at beacon_start_ms.
 */
bool tdma_is_in_reg_window(uint32_t beacon_start_ms, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* TDMA_H */

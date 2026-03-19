/*
 * emesh_node_caps.h — Node tier, mobility, and identity definitions.
 *
 * The capability byte encodes both tier and mobility in a single uint8_t:
 *   bits [1:0]  tier   (EMESH_NODE_TIER_*)
 *   bit  [6]    GPS    node has a GPS / PPS reference
 *   bit  [7]    MOBILE node may move; re-registration is expected on movement
 *   bits [5:2]  reserved, must be zero
 */

#ifndef EMESH_NODE_CAPS_H
#define EMESH_NODE_CAPS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Tier values (bits [1:0] of capability byte) ─────────────────────────── */
#define EMESH_NODE_TIER_NODE     0x00U  /* leaf: sensor / actuator            */
#define EMESH_NODE_TIER_RELAY    0x01U  /* mid-tier forwarder                 */
#define EMESH_NODE_TIER_GATEWAY  0x02U  /* backbone / internet uplink         */

/* ── Capability flags (OR'd into the capability byte) ────────────────────── */
#define EMESH_NODE_FLAG_GPS      0x40U  /* node has GPS / PPS reference       */
#define EMESH_NODE_FLAG_MOBILE   0x80U  /* may move; triggers re-registration */

/* ── Helper macros ───────────────────────────────────────────────────────── */
#define EMESH_NODE_TIER(cap)       ((uint8_t)((cap) & 0x03U))
#define EMESH_NODE_IS_MOBILE(cap)  (((cap) & EMESH_NODE_FLAG_MOBILE) != 0U)
#define EMESH_NODE_HAS_GPS(cap)    (((cap) & EMESH_NODE_FLAG_GPS)    != 0U)

/* ── Stratum constants ───────────────────────────────────────────────────── */
#define EMESH_STRATUM_GPS         0U    /* GPS-disciplined clock (authoritative) */
#define EMESH_STRATUM_UNKNOWN   255U    /* clock not yet synchronised            */

/* ── Broadcast destination ───────────────────────────────────────────────── */
#define EMESH_DEST_BROADCAST  0xFFFFFFFFU

/* ── STM32F4 unique device ID ────────────────────────────────────────────── */
/*
 * The STM32F4 provides a 96-bit unique device ID at 0x1FFF7A10.
 * XOR of the three 32-bit words yields a compact 32-bit node ID.
 * Collision probability across ≤32 nodes is negligible.
 */
#define EMESH_STM32_UID_BASE  0x1FFF7A10U

static inline uint32_t emesh_get_node_id(void)
{
    const volatile uint32_t *uid = (const volatile uint32_t *)EMESH_STM32_UID_BASE;
    return uid[0] ^ uid[1] ^ uid[2];
}

#ifdef __cplusplus
}
#endif

#endif /* EMESH_NODE_CAPS_H */

/*
 * emesh_node_caps.h — Node tier, mobility, and identity definitions.
 *                     Linux / Raspberry Pi version (no STM32 HAL).
 *
 * Protocol constants are identical to the relay version.
 * emesh_get_node_id() reads the RPi CPU serial from /proc/cpuinfo and folds
 * it to 32 bits.  Falls back to the process PID for development environments
 * that do not expose a CPU serial (e.g., x86 build machines).
 */

#ifndef EMESH_NODE_CAPS_H
#define EMESH_NODE_CAPS_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Tier values (bits [1:0] of capability byte) ─────────────────────────── */
#define EMESH_NODE_TIER_NODE     0x00U  /* leaf: sensor / actuator            */
#define EMESH_NODE_TIER_RELAY    0x01U  /* mid-tier forwarder                 */
#define EMESH_NODE_TIER_GATEWAY  0x02U  /* backbone / internet uplink         */

/* ── Capability flags ────────────────────────────────────────────────────── */
#define EMESH_NODE_FLAG_FORWARD  0x04U  /* node may forward data frames       */
#define EMESH_NODE_FLAG_GPS      0x40U  /* node has GPS / PPS reference       */
#define EMESH_NODE_FLAG_MOBILE   0x80U  /* may move; triggers re-registration */

/* ── Helper macros ───────────────────────────────────────────────────────── */
#define EMESH_NODE_TIER(cap)          ((uint8_t)((cap) & 0x03U))
#define EMESH_NODE_IS_MOBILE(cap)     (((cap) & EMESH_NODE_FLAG_MOBILE)  != 0U)
#define EMESH_NODE_HAS_GPS(cap)       (((cap) & EMESH_NODE_FLAG_GPS)     != 0U)
#define EMESH_NODE_CAN_FORWARD(cap)   (((cap) & EMESH_NODE_FLAG_FORWARD) != 0U)

/* ── Stratum constants ───────────────────────────────────────────────────── */
#define EMESH_STRATUM_GPS         0U    /* GPS-disciplined clock (authoritative) */
#define EMESH_STRATUM_UNKNOWN   255U    /* clock not yet synchronised            */

/* ── Broadcast destination ───────────────────────────────────────────────── */
#define EMESH_DEST_BROADCAST  0xFFFFFFFFU

/* ── Node ID (Raspberry Pi CPU serial, folded to 32 bits) ────────────────── */
/*
 * Reads "Serial : <hex>" from /proc/cpuinfo.
 * Folds the 64-bit serial to 32 bits via XOR.
 * Returns (uint32_t)getpid() as a fallback on non-RPi hosts.
 */
static inline uint32_t emesh_get_node_id(void)
{
    FILE    *f;
    char     line[256];
    uint64_t serial = 0U;

    f = fopen("/proc/cpuinfo", "r");
    if (f != NULL) {
        while (fgets(line, (int)sizeof(line), f) != NULL) {
            if (strncmp(line, "Serial", 6) == 0) {
                const char *colon = strchr(line, ':');
                if (colon != NULL) {
                    serial = (uint64_t)strtoull(colon + 2, NULL, 16);
                }
                break;
            }
        }
        fclose(f);
    }

    if (serial == 0U) {
        /* Fallback for non-RPi hosts (development / CI). */
        serial = (uint64_t)(unsigned)getpid();
    }

    return (uint32_t)(serial ^ (serial >> 32));
}

#ifdef __cplusplus
}
#endif

#endif /* EMESH_NODE_CAPS_H */

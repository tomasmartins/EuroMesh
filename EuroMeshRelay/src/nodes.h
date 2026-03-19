/*
 * nodes.h — Relay-side registration window management.
 *
 * During the registration slot of each super-frame the relay opens a receive
 * window and processes incoming SUBSCRIPTION requests.  Each valid frame is
 * handed to reg_relay_handle_request() which inserts/updates the node table
 * and unicasts a REG_RESPONSE back.
 *
 * Node-side subscription logic lives in registration.h (reg_ctx_*).
 */

#ifndef NODES_H
#define NODES_H

#include <stdint.h>

#include "csma_mac.h"
#include "registration.h"
#include "sx1276.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Open a registration receive window of window_ms milliseconds.
 *
 * Continuously receives frames until the window expires.  Each frame whose
 * type is EMESH_PACKET_TYPE_SUBSCRIPTION is forwarded to
 * reg_relay_handle_request().  Non-subscription frames (e.g., stray data)
 * are silently discarded.
 *
 * my_id    : relay's own 32-bit node ID (used as src_id in REG_RESPONSE).
 * window_ms: duration of the registration slot (TDMA_REG_SLOT_MS).
 */
void nodes_relay_open_reg_window(reg_relay_t *relay,
                                 csma_mac_t  *mac,
                                 sx1276_t    *radio,
                                 uint32_t     my_id,
                                 uint32_t     window_ms);

/*
 * Evict stale node table entries.
 * Thin wrapper around reg_relay_expire(); call once per super-frame.
 *
 * max_age_ms: entries not seen within this period are removed.
 */
void nodes_relay_expire(reg_relay_t *relay, uint32_t now_ms, uint32_t max_age_ms);

/*
 * Return the number of nodes currently registered with this relay.
 */
static inline uint8_t nodes_relay_count(const reg_relay_t *relay)
{
    return (relay != NULL) ? relay->count : 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* NODES_H */

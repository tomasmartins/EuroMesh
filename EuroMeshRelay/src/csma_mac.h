/*
 * csma_mac.h — CSMA/CA MAC with CAD-based Listen Before Talk.
 *
 * Before every transmission the radio runs a Channel Activity Detection
 * scan (SX1276 CAD mode, ~2 ms at SF7).  If a preamble is detected the
 * sender backs off for a jittered interval and retries up to max_attempts.
 *
 * Used for:
 *   - Contention-based registration slot (TDMA slot 1)
 *   - All post-registration data transmissions (pure CSMA/CAD, no TDMA)
 */

#ifndef CSMA_MAC_H
#define CSMA_MAC_H

#include <stdbool.h>

#include "emesh_frame.h"
#include "sx1276.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sx1276_t *radio;
    uint32_t  backoff_min_ms;
    uint32_t  backoff_max_ms;
    uint8_t   max_attempts;
    uint32_t  prng_state;   /* LCG state for collision-avoidance jitter */
} csma_mac_t;

/*
 * Initialise the MAC context.
 * prng is seeded from HAL_GetTick() XOR the mac pointer address.
 */
void csma_mac_init(csma_mac_t *mac, sx1276_t *radio,
                   uint32_t backoff_min_ms, uint32_t backoff_max_ms,
                   uint8_t max_attempts);

/*
 * Perform a single CAD scan.
 * Returns true if the channel is clear (no preamble detected).
 */
bool csma_mac_cad_clear(csma_mac_t *mac);

/*
 * Build a complete wire frame (header + payload) and transmit it using
 * CAD-based LBT.  payload[0] must be the EMESH packet type byte.
 * Returns HAL_TIMEOUT if max_attempts are exhausted without a clear channel.
 */
HAL_StatusTypeDef csma_mac_send(csma_mac_t *mac,
                                const emesh_frame_header_t *header,
                                const uint8_t *payload,
                                uint8_t payload_length);

/*
 * Wait until slot_start_ms (wrapping-safe), then call csma_mac_send.
 * Used exclusively for the TDMA registration slot.
 */
HAL_StatusTypeDef csma_mac_send_at(csma_mac_t *mac,
                                   const emesh_frame_header_t *header,
                                   const uint8_t *payload,
                                   uint8_t payload_length,
                                   uint32_t slot_start_ms);

#ifdef __cplusplus
}
#endif

#endif /* CSMA_MAC_H */

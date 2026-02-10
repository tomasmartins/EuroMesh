#ifndef CSMA_MAC_H
#define CSMA_MAC_H

#include <stdbool.h>
#include "sx1276.h"
#include "emesh_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sx1276_t *radio;
    uint8_t rssi_threshold;
    uint32_t backoff_min_ms;
    uint32_t backoff_max_ms;
    uint8_t max_attempts;
} csma_mac_t;

void csma_mac_init(csma_mac_t *mac, sx1276_t *radio, uint8_t rssi_threshold, uint32_t backoff_min_ms, uint32_t backoff_max_ms, uint8_t max_attempts);
bool csma_mac_is_channel_clear(csma_mac_t *mac);
/* payload[0] must contain the EMESH packet type. */
HAL_StatusTypeDef csma_mac_send(csma_mac_t *mac, const emesh_frame_header_t *header, const uint8_t *payload, uint8_t payload_length);
HAL_StatusTypeDef csma_mac_send_tdma(csma_mac_t *mac, const emesh_frame_header_t *header, const uint8_t *payload, uint8_t payload_length, uint32_t frame_start_ms, uint32_t slot_length_ms, uint8_t slot_index);

#ifdef __cplusplus
}
#endif

#endif

#ifndef NODES_H
#define NODES_H

#include <stdbool.h>
#include <stdint.h>

#include "sx1276.h"
#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*nodes_packet_handler_t)(time_sync_t *sync,
                                       const sx1276_packet_header_t *header,
                                       const uint8_t *payload,
                                       uint8_t payload_length);

uint8_t nodes_build_subscription_payload(uint8_t *payload, uint8_t capacity);
void nodes_open_subscription_window(time_sync_t *sync,
                                    sx1276_t *radio,
                                    uint32_t window_ms,
                                    nodes_packet_handler_t packet_handler);
void nodes_handle_subscription_packet(const sx1276_packet_header_t *header,
                                      const uint8_t *payload,
                                      uint8_t payload_length);

#ifdef __cplusplus
}
#endif

#endif

#ifndef ACK_PACKET_H
#define ACK_PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "emesh_packet_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EMESH_ACK_FLAG_RX_TICK_PRESENT  0x01
#define EMESH_ACK_FLAG_UTC_PRESENT      0x02
#define EMESH_ACK_FLAG_PPS_TICK_PRESENT 0x04
#define EMESH_ACK_FLAG_NAK              0x08
#define EMESH_ACK_FLAG_ACK_OF_ACK       0x10

typedef struct {
    uint8_t flags;
    uint16_t acked_seq;
    uint32_t rx_tick_ms;
    uint32_t utc_epoch_s;
    uint16_t utc_epoch_ms;
    uint32_t pps_tick_ms;
} emesh_ack_packet_t;

size_t emesh_ack_payload_size(const emesh_ack_packet_t *ack);
bool emesh_ack_encode(const emesh_ack_packet_t *ack, uint8_t *buffer, size_t buffer_size, size_t *encoded_size);
bool emesh_ack_decode(emesh_ack_packet_t *ack, const uint8_t *buffer, size_t buffer_size, size_t *decoded_size);

#ifdef __cplusplus
}
#endif

#endif

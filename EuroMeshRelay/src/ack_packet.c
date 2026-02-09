#include "ack_packet.h"

static void write_u16_le(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFu);
    buffer[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void write_u32_le(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFu);
    buffer[1] = (uint8_t)((value >> 8) & 0xFFu);
    buffer[2] = (uint8_t)((value >> 16) & 0xFFu);
    buffer[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint16_t read_u16_le(const uint8_t *buffer)
{
    return (uint16_t)(buffer[0]) | ((uint16_t)(buffer[1]) << 8);
}

static uint32_t read_u32_le(const uint8_t *buffer)
{
    return (uint32_t)(buffer[0]) | ((uint32_t)(buffer[1]) << 8) | ((uint32_t)(buffer[2]) << 16)
           | ((uint32_t)(buffer[3]) << 24);
}

size_t emesh_ack_payload_size(const emesh_ack_packet_t *ack)
{
    size_t size = 1 + sizeof(uint16_t);

    if (ack == NULL) {
        return 0;
    }

    if ((ack->flags & EMESH_ACK_FLAG_RX_TICK_PRESENT) != 0u) {
        size += sizeof(uint32_t);
    }

    if ((ack->flags & EMESH_ACK_FLAG_UTC_PRESENT) != 0u) {
        size += sizeof(uint32_t) + sizeof(uint16_t);
    }

    if ((ack->flags & EMESH_ACK_FLAG_PPS_TICK_PRESENT) != 0u) {
        size += sizeof(uint32_t);
    }

    return size;
}

bool emesh_ack_encode(const emesh_ack_packet_t *ack, uint8_t *buffer, size_t buffer_size, size_t *encoded_size)
{
    size_t offset = 0;
    size_t required_size = 0;

    if (ack == NULL || buffer == NULL) {
        return false;
    }

    required_size = emesh_ack_payload_size(ack);
    if (required_size == 0 || buffer_size < required_size) {
        return false;
    }

    buffer[offset++] = ack->flags;
    write_u16_le(&buffer[offset], ack->acked_seq);
    offset += sizeof(uint16_t);

    if ((ack->flags & EMESH_ACK_FLAG_RX_TICK_PRESENT) != 0u) {
        write_u32_le(&buffer[offset], ack->rx_tick_ms);
        offset += sizeof(uint32_t);
    }

    if ((ack->flags & EMESH_ACK_FLAG_UTC_PRESENT) != 0u) {
        write_u32_le(&buffer[offset], ack->utc_epoch_s);
        offset += sizeof(uint32_t);
        write_u16_le(&buffer[offset], ack->utc_epoch_ms);
        offset += sizeof(uint16_t);
    }

    if ((ack->flags & EMESH_ACK_FLAG_PPS_TICK_PRESENT) != 0u) {
        write_u32_le(&buffer[offset], ack->pps_tick_ms);
        offset += sizeof(uint32_t);
    }

    if (encoded_size != NULL) {
        *encoded_size = offset;
    }

    return true;
}

bool emesh_ack_decode(emesh_ack_packet_t *ack, const uint8_t *buffer, size_t buffer_size, size_t *decoded_size)
{
    size_t offset = 0;
    size_t minimum_size = 1 + sizeof(uint16_t);

    if (ack == NULL || buffer == NULL || buffer_size < minimum_size) {
        return false;
    }

    ack->flags = buffer[offset++];
    ack->acked_seq = read_u16_le(&buffer[offset]);
    offset += sizeof(uint16_t);

    if ((ack->flags & EMESH_ACK_FLAG_RX_TICK_PRESENT) != 0u) {
        if (buffer_size < offset + sizeof(uint32_t)) {
            return false;
        }
        ack->rx_tick_ms = read_u32_le(&buffer[offset]);
        offset += sizeof(uint32_t);
    } else {
        ack->rx_tick_ms = 0u;
    }

    if ((ack->flags & EMESH_ACK_FLAG_UTC_PRESENT) != 0u) {
        if (buffer_size < offset + sizeof(uint32_t) + sizeof(uint16_t)) {
            return false;
        }
        ack->utc_epoch_s = read_u32_le(&buffer[offset]);
        offset += sizeof(uint32_t);
        ack->utc_epoch_ms = read_u16_le(&buffer[offset]);
        offset += sizeof(uint16_t);
    } else {
        ack->utc_epoch_s = 0u;
        ack->utc_epoch_ms = 0u;
    }

    if ((ack->flags & EMESH_ACK_FLAG_PPS_TICK_PRESENT) != 0u) {
        if (buffer_size < offset + sizeof(uint32_t)) {
            return false;
        }
        ack->pps_tick_ms = read_u32_le(&buffer[offset]);
        offset += sizeof(uint32_t);
    } else {
        ack->pps_tick_ms = 0u;
    }

    if (decoded_size != NULL) {
        *decoded_size = offset;
    }

    return true;
}

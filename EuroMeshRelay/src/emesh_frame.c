#include "emesh_frame.h"

void emesh_frame_encode_header(const emesh_frame_header_t *header, uint8_t *buffer)
{
    if (header == 0 || buffer == 0) {
        return;
    }

    buffer[0] = header->type;
    buffer[1] = header->flags;
    buffer[2] = header->ttl;
    buffer[3] = (uint8_t)(header->src_id >> 24);
    buffer[4] = (uint8_t)(header->src_id >> 16);
    buffer[5] = (uint8_t)(header->src_id >> 8);
    buffer[6] = (uint8_t)(header->src_id);
    buffer[7] = (uint8_t)(header->dest_id >> 24);
    buffer[8] = (uint8_t)(header->dest_id >> 16);
    buffer[9] = (uint8_t)(header->dest_id >> 8);
    buffer[10] = (uint8_t)(header->dest_id);
    buffer[11] = (uint8_t)(header->seq >> 8);
    buffer[12] = (uint8_t)(header->seq);
}

void emesh_frame_decode_header(const uint8_t *buffer, emesh_frame_header_t *header)
{
    if (header == 0 || buffer == 0) {
        return;
    }

    header->type = buffer[0];
    header->flags = buffer[1];
    header->ttl = buffer[2];
    header->src_id = ((uint32_t)buffer[3] << 24)
        | ((uint32_t)buffer[4] << 16)
        | ((uint32_t)buffer[5] << 8)
        | ((uint32_t)buffer[6]);
    header->dest_id = ((uint32_t)buffer[7] << 24)
        | ((uint32_t)buffer[8] << 16)
        | ((uint32_t)buffer[9] << 8)
        | ((uint32_t)buffer[10]);
    header->seq = (uint16_t)(((uint16_t)buffer[11] << 8) | buffer[12]);
}

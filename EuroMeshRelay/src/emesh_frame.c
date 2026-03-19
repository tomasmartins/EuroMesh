#include "emesh_frame.h"

void emesh_frame_encode_header(const emesh_frame_header_t *header, uint8_t *buffer)
{
    if (header == NULL || buffer == NULL) {
        return;
    }

    buffer[0]  = header->type;
    buffer[1]  = header->flags;
    buffer[2]  = header->ttl;

    /* src_id — little-endian */
    buffer[3]  = (uint8_t)(header->src_id);
    buffer[4]  = (uint8_t)(header->src_id >> 8);
    buffer[5]  = (uint8_t)(header->src_id >> 16);
    buffer[6]  = (uint8_t)(header->src_id >> 24);

    /* dest_id — little-endian */
    buffer[7]  = (uint8_t)(header->dest_id);
    buffer[8]  = (uint8_t)(header->dest_id >> 8);
    buffer[9]  = (uint8_t)(header->dest_id >> 16);
    buffer[10] = (uint8_t)(header->dest_id >> 24);

    /* seq — little-endian */
    buffer[11] = (uint8_t)(header->seq);
    buffer[12] = (uint8_t)(header->seq >> 8);

    /* op — little-endian (low byte = command, high byte = class) */
    buffer[13] = (uint8_t)(header->op);
    buffer[14] = (uint8_t)(header->op >> 8);

    /* payload length */
    buffer[15] = header->length;
}

void emesh_frame_decode_header(const uint8_t *buffer, emesh_frame_header_t *header)
{
    if (header == NULL || buffer == NULL) {
        return;
    }

    header->type   = buffer[0];
    header->flags  = buffer[1];
    header->ttl    = buffer[2];

    /* src_id — little-endian */
    header->src_id = (uint32_t) buffer[3]
                   | ((uint32_t)buffer[4]  << 8)
                   | ((uint32_t)buffer[5]  << 16)
                   | ((uint32_t)buffer[6]  << 24);

    /* dest_id — little-endian */
    header->dest_id = (uint32_t) buffer[7]
                    | ((uint32_t)buffer[8]  << 8)
                    | ((uint32_t)buffer[9]  << 16)
                    | ((uint32_t)buffer[10] << 24);

    /* seq — little-endian */
    header->seq = (uint16_t) buffer[11]
                | ((uint16_t)buffer[12] << 8);

    /* op — little-endian */
    header->op  = (uint16_t) buffer[13]
                | ((uint16_t)buffer[14] << 8);

    header->length = buffer[15];
}

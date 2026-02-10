#ifndef EMESH_FRAME_H
#define EMESH_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint8_t ttl;
    uint32_t src_id;
    uint32_t dest_id;
    uint16_t seq;
} emesh_frame_header_t;

#define EMESH_FRAME_HEADER_SIZE                13U
#define EMESH_FRAME_FLAG_ACK_REQUEST           0x01U
#define EMESH_FRAME_FLAG_ACK_PRESENT           0x02U
#define EMESH_FRAME_FLAG_NAK                   0x04U
#define EMESH_FRAME_FLAG_TIME_SYNC_REQUEST     0x08U

void emesh_frame_encode_header(const emesh_frame_header_t *header, uint8_t *buffer);
void emesh_frame_decode_header(const uint8_t *buffer, emesh_frame_header_t *header);

#ifdef __cplusplus
}
#endif

#endif

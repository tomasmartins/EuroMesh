#ifndef EMESH_PACKET_TYPES_H
#define EMESH_PACKET_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * All packet payloads begin with a leading type byte.
 */
#define EMESH_PACKET_TYPE_BEACON       0x01
#define EMESH_PACKET_TYPE_ACK          0x02
#define EMESH_PACKET_TYPE_SUBSCRIPTION 0x03
#define EMESH_PACKET_TYPE_PAYLOAD      0x04

#ifdef __cplusplus
}
#endif

#endif

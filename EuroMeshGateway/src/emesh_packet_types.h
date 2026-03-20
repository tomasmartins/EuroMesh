/*
 * emesh_packet_types.h — Packet type constants and op-class definitions.
 *
 * Every wire frame payload begins with a leading type byte matching the
 * frame header's type field.  The op field (header bytes 13–14) further
 * qualifies data frames with an application-layer operation code.
 */

#ifndef EMESH_PACKET_TYPES_H
#define EMESH_PACKET_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Packet types ────────────────────────────────────────────────────────── */
#define EMESH_PACKET_TYPE_BEACON        0x01U  /* GW/relay time + network info  */
#define EMESH_PACKET_TYPE_ACK           0x02U  /* acknowledgement / NAK         */
#define EMESH_PACKET_TYPE_SUBSCRIPTION  0x03U  /* registration request          */
#define EMESH_PACKET_TYPE_DATA          0x04U  /* application data payload      */
#define EMESH_PACKET_TYPE_NEIGHBOUR_ADV 0x05U  /* neighbour table advertisement */
#define EMESH_PACKET_TYPE_REG_RESPONSE  0x06U  /* registration response         */
#define EMESH_PACKET_TYPE_TIME_REQ      0x07U  /* two-way time sync request     */
#define EMESH_PACKET_TYPE_TIME_RESP     0x08U  /* two-way time sync response    */

/* ── Op-field class codes (high byte of the 16-bit op field) ─────────────── */
#define EMESH_OP_CLASS_MESH        0x00U  /* mesh control (reg, time, routing) */
#define EMESH_OP_CLASS_TELEMETRY   0x01U  /* sensor / telemetry data           */
#define EMESH_OP_CLASS_COMMAND     0x02U  /* actuation / remote command        */
#define EMESH_OP_CLASS_CUSTOM      0xFFU  /* application-defined               */

/* Build a 16-bit op value from class + command bytes. */
#define EMESH_OP(cls, cmd) \
    ((uint16_t)(((uint16_t)(uint8_t)(cls) << 8) | (uint8_t)(cmd)))

/* ── Well-known op codes (EMESH_OP_CLASS_MESH) ───────────────────────────── */
#define EMESH_OP_MESH_BEACON      EMESH_OP(EMESH_OP_CLASS_MESH, 0x01U)
#define EMESH_OP_MESH_REGISTER    EMESH_OP(EMESH_OP_CLASS_MESH, 0x02U)
#define EMESH_OP_MESH_TIME_REQ    EMESH_OP(EMESH_OP_CLASS_MESH, 0x03U)
#define EMESH_OP_MESH_TIME_RESP   EMESH_OP(EMESH_OP_CLASS_MESH, 0x04U)
#define EMESH_OP_MESH_NB_ADV      EMESH_OP(EMESH_OP_CLASS_MESH, 0x05U)

#ifdef __cplusplus
}
#endif

#endif /* EMESH_PACKET_TYPES_H */

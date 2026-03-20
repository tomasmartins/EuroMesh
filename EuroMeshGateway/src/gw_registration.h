/*
 * gw_registration.h — Registration packet codec for the gateway.
 *
 * Contains only the wire-format encode/decode functions and the data structs
 * needed by the gateway to process SUBSCRIPTION requests and send
 * REG_RESPONSE replies.  No csma_mac or STM32 HAL dependency.
 *
 * Wire formats (identical to relay/registration.h):
 *
 *   SUBSCRIPTION payload (6 bytes):
 *     [type:1][capability:1][req_slots:1][heard_rssi:1][seq:2LE]
 *
 *   REG_RESPONSE payload (8 bytes):
 *     [type:1][status:1][pan_id:2LE][stratum:1][nb_adv_interval_s:1][_:2]
 */

#ifndef GW_REGISTRATION_H
#define GW_REGISTRATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Payload sizes ───────────────────────────────────────────────────────── */
#define SUB_REQUEST_PAYLOAD_SIZE   6U
#define REG_RESPONSE_PAYLOAD_SIZE  8U

/* ── REG_RESPONSE status codes ───────────────────────────────────────────── */
#define REG_STATUS_OK        0x00U
#define REG_STATUS_REJECTED  0x01U
#define REG_STATUS_FULL      0x02U

/* ── Structs ─────────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t  capability;        /* EMESH_NODE_TIER_* | EMESH_NODE_FLAG_*   */
    uint8_t  requested_slots;   /* always 0 — CSMA/CAD needs no slots      */
    int8_t   heard_rssi_dbm;    /* RSSI of best beacon heard               */
    uint16_t seq;               /* request sequence number                  */
} sub_request_t;

typedef struct {
    uint8_t  status;
    uint16_t pan_id;
    uint8_t  stratum;
    uint8_t  nb_adv_interval_s;
    uint8_t  beacon_slot;   /* TDMA beacon slot assigned, 0xFF = none */
} reg_response_t;

/* TDMA_NO_SLOT_ASSIGNED — used when a leaf node (not a relay) registers. */
#define TDMA_NO_SLOT_ASSIGNED  0xFFU

/* ── Codec ───────────────────────────────────────────────────────────────── */

bool    sub_request_decode(const uint8_t *buf, uint8_t length, sub_request_t *out);
uint8_t reg_response_encode(const reg_response_t *resp, uint8_t *buf, uint8_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* GW_REGISTRATION_H */

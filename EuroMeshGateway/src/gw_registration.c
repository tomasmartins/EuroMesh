/*
 * gw_registration.c — Registration packet codec (gateway side).
 * Identical wire format to the relay registration.c codec functions;
 * the csma_mac and STM32-dependent relay-side state machine is omitted.
 */

#include "gw_registration.h"
#include "emesh_packet_types.h"

bool sub_request_decode(const uint8_t *buf, uint8_t length, sub_request_t *out)
{
    if (buf == NULL || out == NULL || length < SUB_REQUEST_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_SUBSCRIPTION) {
        return false;
    }
    out->capability      = buf[1];
    out->requested_slots = buf[2];
    out->heard_rssi_dbm  = (int8_t)buf[3];
    out->seq             = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    return true;
}

uint8_t reg_response_encode(const reg_response_t *resp,
                             uint8_t *buf, uint8_t capacity)
{
    if (resp == NULL || buf == NULL || capacity < REG_RESPONSE_PAYLOAD_SIZE) {
        return 0U;
    }
    buf[0] = EMESH_PACKET_TYPE_REG_RESPONSE;
    buf[1] = resp->status;
    buf[2] = (uint8_t)(resp->pan_id);
    buf[3] = (uint8_t)(resp->pan_id >> 8);
    buf[4] = resp->stratum;
    buf[5] = resp->nb_adv_interval_s;
    buf[6] = resp->beacon_slot;   /* TDMA slot: 0xFF = no beacon needed */
    buf[7] = 0x00U;               /* reserved                           */
    return REG_RESPONSE_PAYLOAD_SIZE;
}

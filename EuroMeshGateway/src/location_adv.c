/*
 * location_adv.c — Location advertisement packet codec.
 */

#include <stddef.h>
#include "location_adv.h"
#include "emesh_packet_types.h"

uint8_t location_adv_encode(const location_adv_t *loc,
                             uint8_t *buf, uint8_t capacity)
{
    int32_t lat_udeg;
    int32_t lon_udeg;
    int32_t alt_cm;

    if (loc == NULL || buf == NULL || capacity < LOCATION_ADV_PAYLOAD_SIZE) {
        return 0U;
    }

    lat_udeg = (int32_t)(loc->lat_deg * 1e6);
    lon_udeg = (int32_t)(loc->lon_deg * 1e6);
    alt_cm   = loc->alt_valid ? (int32_t)((double)loc->alt_m * 100.0) : 0;

    buf[0]  = EMESH_PACKET_TYPE_LOCATION_ADV;
    buf[1]  = (uint8_t)((loc->gps_valid ? LOCATION_ADV_FLAG_GPS_VALID : 0U)
                       | (loc->alt_valid ? LOCATION_ADV_FLAG_ALT_VALID : 0U));

    /* lat_udeg little-endian */
    buf[2]  = (uint8_t)( (uint32_t)lat_udeg        & 0xFFU);
    buf[3]  = (uint8_t)(((uint32_t)lat_udeg >>  8U) & 0xFFU);
    buf[4]  = (uint8_t)(((uint32_t)lat_udeg >> 16U) & 0xFFU);
    buf[5]  = (uint8_t)(((uint32_t)lat_udeg >> 24U) & 0xFFU);

    /* lon_udeg little-endian */
    buf[6]  = (uint8_t)( (uint32_t)lon_udeg        & 0xFFU);
    buf[7]  = (uint8_t)(((uint32_t)lon_udeg >>  8U) & 0xFFU);
    buf[8]  = (uint8_t)(((uint32_t)lon_udeg >> 16U) & 0xFFU);
    buf[9]  = (uint8_t)(((uint32_t)lon_udeg >> 24U) & 0xFFU);

    /* alt_cm little-endian */
    buf[10] = (uint8_t)( (uint32_t)alt_cm        & 0xFFU);
    buf[11] = (uint8_t)(((uint32_t)alt_cm >>  8U) & 0xFFU);
    buf[12] = (uint8_t)(((uint32_t)alt_cm >> 16U) & 0xFFU);
    buf[13] = (uint8_t)(((uint32_t)alt_cm >> 24U) & 0xFFU);

    return LOCATION_ADV_PAYLOAD_SIZE;
}

bool location_adv_decode(const uint8_t *buf, uint8_t length,
                          location_adv_t *out)
{
    int32_t lat_udeg;
    int32_t lon_udeg;
    int32_t alt_cm;

    if (buf == NULL || out == NULL || length < LOCATION_ADV_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_LOCATION_ADV) {
        return false;
    }

    out->gps_valid = (buf[1] & LOCATION_ADV_FLAG_GPS_VALID) != 0U;
    out->alt_valid = (buf[1] & LOCATION_ADV_FLAG_ALT_VALID) != 0U;

    lat_udeg = (int32_t)((uint32_t)buf[2]
                       | ((uint32_t)buf[3] <<  8U)
                       | ((uint32_t)buf[4] << 16U)
                       | ((uint32_t)buf[5] << 24U));

    lon_udeg = (int32_t)((uint32_t)buf[6]
                       | ((uint32_t)buf[7] <<  8U)
                       | ((uint32_t)buf[8] << 16U)
                       | ((uint32_t)buf[9] << 24U));

    alt_cm   = (int32_t)((uint32_t)buf[10]
                       | ((uint32_t)buf[11] <<  8U)
                       | ((uint32_t)buf[12] << 16U)
                       | ((uint32_t)buf[13] << 24U));

    out->lat_deg = (double)lat_udeg / 1e6;
    out->lon_deg = (double)lon_udeg / 1e6;
    out->alt_m   = out->alt_valid ? (float)((double)alt_cm / 100.0) : 0.0f;

    return true;
}

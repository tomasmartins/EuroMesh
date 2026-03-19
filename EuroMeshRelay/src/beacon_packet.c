#include "beacon_packet.h"
#include "emesh_packet_types.h"

uint8_t beacon_payload_encode(const beacon_payload_t *beacon,
                              uint8_t *buf, uint8_t capacity)
{
    if (beacon == NULL || buf == NULL || capacity < BEACON_PAYLOAD_SIZE) {
        return 0;
    }

    buf[0]  = EMESH_PACKET_TYPE_BEACON;
    buf[1]  = beacon->sync_flags;

    /* utc_epoch_ms — little-endian uint64 */
    buf[2]  = (uint8_t)(beacon->utc_epoch_ms);
    buf[3]  = (uint8_t)(beacon->utc_epoch_ms >> 8);
    buf[4]  = (uint8_t)(beacon->utc_epoch_ms >> 16);
    buf[5]  = (uint8_t)(beacon->utc_epoch_ms >> 24);
    buf[6]  = (uint8_t)(beacon->utc_epoch_ms >> 32);
    buf[7]  = (uint8_t)(beacon->utc_epoch_ms >> 40);
    buf[8]  = (uint8_t)(beacon->utc_epoch_ms >> 48);
    buf[9]  = (uint8_t)(beacon->utc_epoch_ms >> 56);

    /* pps_tick_ms — little-endian uint32 */
    buf[10] = (uint8_t)(beacon->pps_tick_ms);
    buf[11] = (uint8_t)(beacon->pps_tick_ms >> 8);
    buf[12] = (uint8_t)(beacon->pps_tick_ms >> 16);
    buf[13] = (uint8_t)(beacon->pps_tick_ms >> 24);

    buf[14] = beacon->stratum;
    buf[15] = beacon->net_flags;

    return BEACON_PAYLOAD_SIZE;
}

bool beacon_payload_decode(const uint8_t *buf, uint8_t length,
                           beacon_payload_t *out)
{
    if (buf == NULL || out == NULL || length < BEACON_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_BEACON) {
        return false;
    }

    out->sync_flags = buf[1];

    out->utc_epoch_ms = (uint64_t) buf[2]
                      | ((uint64_t)buf[3]  << 8)
                      | ((uint64_t)buf[4]  << 16)
                      | ((uint64_t)buf[5]  << 24)
                      | ((uint64_t)buf[6]  << 32)
                      | ((uint64_t)buf[7]  << 40)
                      | ((uint64_t)buf[8]  << 48)
                      | ((uint64_t)buf[9]  << 56);

    out->pps_tick_ms = (uint32_t) buf[10]
                     | ((uint32_t)buf[11] << 8)
                     | ((uint32_t)buf[12] << 16)
                     | ((uint32_t)buf[13] << 24);

    out->stratum   = buf[14];
    out->net_flags = buf[15];

    return true;
}

void beacon_payload_from_sync(const time_sync_t *sync,
                              uint32_t now_tick_ms,
                              bool reg_window_open,
                              beacon_payload_t *out)
{
    if (out == NULL) {
        return;
    }

    if (sync != NULL && sync->utc_valid) {
        uint32_t elapsed      = now_tick_ms - sync->last_update_tick_ms;
        out->utc_epoch_ms     = sync->last_utc_epoch_ms + (uint64_t)elapsed;
        out->sync_flags       = TIME_SYNC_FLAG_UTC_VALID;

        if (sync->pps_valid) {
            out->sync_flags  |= TIME_SYNC_FLAG_PPS_VALID;
            out->pps_tick_ms  = sync->last_pps_tick_ms;
        } else {
            out->pps_tick_ms  = 0U;
        }
        out->stratum = sync->stratum;
    } else {
        /* No valid time — broadcast presence only; receivers will not sync. */
        out->utc_epoch_ms = 0U;
        out->pps_tick_ms  = 0U;
        out->sync_flags   = 0x00U;
        out->stratum      = 255U; /* EMESH_STRATUM_UNKNOWN */
    }

    out->net_flags = reg_window_open ? BEACON_NET_FLAG_REG_OPEN : 0x00U;
}

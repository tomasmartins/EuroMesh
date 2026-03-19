#include "neighbour_adv.h"
#include "emesh_packet_types.h"

uint8_t nb_adv_encode(const nb_adv_payload_t *adv,
                      uint8_t *buf, uint8_t capacity)
{
    uint8_t n;
    uint8_t needed;

    if (adv == NULL || buf == NULL) {
        return 0;
    }

    n = adv->entry_count;
    if (n > NB_ADV_MAX_ENTRIES) {
        n = NB_ADV_MAX_ENTRIES;
    }

    needed = (uint8_t)(NB_ADV_HEADER_SIZE + (uint8_t)(n * NB_ADV_ENTRY_SIZE));
    if (capacity < needed) {
        return 0;
    }

    buf[0] = EMESH_PACKET_TYPE_NEIGHBOUR_ADV;
    buf[1] = n;

    for (uint8_t i = 0U; i < n; ++i) {
        uint8_t *e  = &buf[NB_ADV_HEADER_SIZE + i * NB_ADV_ENTRY_SIZE];
        uint32_t id = adv->entries[i].node_id;

        e[0] = (uint8_t)(id);
        e[1] = (uint8_t)(id >> 8);
        e[2] = (uint8_t)(id >> 16);
        e[3] = (uint8_t)(id >> 24);
        e[4] = (uint8_t)adv->entries[i].rssi_dbm; /* int8 → uint8 two's complement */
        e[5] = (uint8_t)adv->entries[i].snr_db;
        e[6] = adv->entries[i].tier;
        e[7] = adv->entries[i].stratum;
    }

    return needed;
}

bool nb_adv_decode(const uint8_t *buf, uint8_t length, nb_adv_payload_t *out)
{
    uint8_t n;

    if (buf == NULL || out == NULL || length < NB_ADV_HEADER_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_NEIGHBOUR_ADV) {
        return false;
    }

    n = buf[1];
    if (n > NB_ADV_MAX_ENTRIES) {
        return false;
    }
    if (length < (uint8_t)(NB_ADV_HEADER_SIZE + n * NB_ADV_ENTRY_SIZE)) {
        return false;
    }

    out->entry_count = n;
    for (uint8_t i = 0U; i < n; ++i) {
        const uint8_t *e = &buf[NB_ADV_HEADER_SIZE + i * NB_ADV_ENTRY_SIZE];

        out->entries[i].node_id   = (uint32_t) e[0]
                                  | ((uint32_t)e[1] << 8)
                                  | ((uint32_t)e[2] << 16)
                                  | ((uint32_t)e[3] << 24);
        out->entries[i].rssi_dbm  = (int8_t)e[4];
        out->entries[i].snr_db    = (int8_t)e[5];
        out->entries[i].tier      = e[6];
        out->entries[i].stratum   = e[7];
    }

    return true;
}

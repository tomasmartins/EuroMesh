#include "nodes.h"

#include "emesh_frame.h"
#include "emesh_packet_types.h"
#include "stm32f4xx_hal.h"

#include <string.h>

void nodes_relay_open_reg_window(reg_relay_t *relay,
                                 csma_mac_t  *mac,
                                 sx1276_t    *radio,
                                 uint32_t     my_id,
                                 uint32_t     window_ms)
{
    uint32_t             start_ms;
    uint8_t              rx_frame[EMESH_FRAME_FIFO_SIZE];
    uint8_t              rx_len;
    sx1276_rx_metadata_t meta;
    emesh_frame_header_t hdr;
    uint8_t              payload_len;

    if (relay == NULL || mac == NULL || radio == NULL || window_ms == 0U) {
        return;
    }

    start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < window_ms) {
        memset(&meta, 0, sizeof(meta));
        rx_len = 0U;

        if (sx1276_receive_bytes(radio, rx_frame, sizeof(rx_frame),
                                 &rx_len, &meta) != HAL_OK) {
            continue;
        }

        /* Drop frames with CRC errors or that are shorter than the header. */
        if (!meta.crc_ok || rx_len < EMESH_FRAME_HEADER_SIZE) {
            continue;
        }

        emesh_frame_decode_header(rx_frame, &hdr);

        /* Only process SUBSCRIPTION requests during this window. */
        if (hdr.type != EMESH_PACKET_TYPE_SUBSCRIPTION) {
            continue;
        }

        payload_len = (uint8_t)(rx_len - EMESH_FRAME_HEADER_SIZE);

        reg_relay_handle_request(relay, mac, my_id,
                                 &hdr,
                                 &rx_frame[EMESH_FRAME_HEADER_SIZE],
                                 payload_len,
                                 meta.rssi_dbm,
                                 HAL_GetTick());
    }
}

void nodes_relay_expire(reg_relay_t *relay, uint32_t now_ms, uint32_t max_age_ms)
{
    reg_relay_expire(relay, now_ms, max_age_ms);
}

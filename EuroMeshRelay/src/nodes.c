#include "nodes.h"

#include "emesh_packet_types.h"
#include "stm32f4xx_hal.h"

static bool nodes_is_unique_node(uint32_t node_id)
{
    (void)node_id;
    return true;
}

static bool nodes_is_location_valid(const uint8_t *payload, uint8_t length)
{
    (void)payload;
    (void)length;
    /* TODO: Validate location against last known fix (e.g., reject >100 km movement in 1 second). */
    return true;
}

uint8_t nodes_build_subscription_payload(uint8_t *payload, uint8_t capacity)
{
    if (payload == NULL || capacity < 2U) {
        return 0;
    }
    payload[0] = EMESH_PACKET_TYPE_SUBSCRIPTION;
    payload[1] = 0x01;
    return 2U;
}

void nodes_open_subscription_window(time_sync_t *sync,
                                    sx1276_t *radio,
                                    uint32_t window_ms,
                                    nodes_packet_handler_t packet_handler)
{
    uint32_t start_ms = HAL_GetTick();
    sx1276_packet_header_t rx_header = {0};
    uint8_t rx_payload[64] = {0};
    uint8_t rx_payload_length = 0;

    if (radio == NULL || packet_handler == NULL) {
        return;
    }

    while ((HAL_GetTick() - start_ms) < window_ms) {
        if (sx1276_receive_packet(radio, &rx_header, rx_payload, sizeof(rx_payload), &rx_payload_length) == HAL_OK) {
            packet_handler(sync, &rx_header, rx_payload, rx_payload_length);
        } else {
            HAL_Delay(1);
        }
    }
}

void nodes_handle_subscription_packet(const sx1276_packet_header_t *header,
                                      const uint8_t *payload,
                                      uint8_t payload_length)
{
    if (header == NULL || payload == NULL || payload_length <= 1U) {
        return;
    }
    const uint8_t *subscription_payload = &payload[1];
    uint8_t subscription_length = (uint8_t)(payload_length - 1U);
    if (!nodes_is_unique_node(header->src_id)) {
        return;
    }
    if (!nodes_is_location_valid(subscription_payload, subscription_length)) {
        return;
    }
    /* TODO: Assign TDMA slot once uniqueness and location validity are confirmed. */
}

#include "csma_mac.h"

static uint8_t csma_mac_read_rssi(const csma_mac_t *mac)
{
    return sx1276_read_reg(mac->radio, SX1276_REG_RSSI_VALUE);
}

static uint32_t csma_mac_next_backoff_ms(const csma_mac_t *mac)
{
    uint32_t min_ms = mac->backoff_min_ms;
    uint32_t max_ms = mac->backoff_max_ms;
    uint32_t span = 0;

    if (max_ms < min_ms) {
        max_ms = min_ms;
    }
    span = max_ms - min_ms + 1U;
    if (span == 0U) {
        return min_ms;
    }
    return min_ms + (HAL_GetTick() % span);
}

void csma_mac_init(csma_mac_t *mac, sx1276_t *radio, uint8_t rssi_threshold, uint32_t backoff_min_ms, uint32_t backoff_max_ms, uint8_t max_attempts)
{
    if (mac == NULL) {
        return;
    }
    mac->radio = radio;
    mac->rssi_threshold = rssi_threshold;
    mac->backoff_min_ms = backoff_min_ms;
    mac->backoff_max_ms = backoff_max_ms;
    mac->max_attempts = max_attempts;
}

bool csma_mac_is_channel_clear(csma_mac_t *mac)
{
    uint8_t rssi = 0;

    if (mac == NULL || mac->radio == NULL) {
        return false;
    }
    sx1276_write_reg(mac->radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_RX_CONTINUOUS_MODE);
    HAL_Delay(1);
    rssi = csma_mac_read_rssi(mac);
    return rssi < mac->rssi_threshold;
}

HAL_StatusTypeDef csma_mac_send(csma_mac_t *mac, const sx1276_packet_header_t *header, const uint8_t *payload, uint8_t payload_length)
{
    uint8_t attempts = 0;

    if (mac == NULL || mac->radio == NULL) {
        return HAL_ERROR;
    }

    while (attempts < mac->max_attempts) {
        if (csma_mac_is_channel_clear(mac)) {
            return sx1276_send_packet(mac->radio, header, payload, payload_length);
        }
        HAL_Delay(csma_mac_next_backoff_ms(mac));
        attempts++;
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef csma_mac_send_tdma(csma_mac_t *mac, const sx1276_packet_header_t *header, const uint8_t *payload, uint8_t payload_length, uint32_t frame_start_ms, uint32_t slot_length_ms, uint8_t slot_index)
{
    uint32_t slot_offset_ms = (uint32_t)slot_index * slot_length_ms;
    uint32_t slot_start_ms = frame_start_ms + slot_offset_ms;
    uint32_t now = HAL_GetTick();

    if (mac == NULL || mac->radio == NULL || slot_length_ms == 0U) {
        return HAL_ERROR;
    }

    if (now > slot_start_ms) {
        return HAL_TIMEOUT;
    }

    while (HAL_GetTick() < slot_start_ms) {
        HAL_Delay(1);
    }

    return sx1276_send_packet(mac->radio, header, payload, payload_length);
}

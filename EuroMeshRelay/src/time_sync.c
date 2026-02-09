#include "time_sync.h"
#include "stm32f4xx_hal.h"

#include <string.h>

static uint32_t time_sync_read_le_u32(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0])
        | ((uint32_t)buffer[1] << 8)
        | ((uint32_t)buffer[2] << 16)
        | ((uint32_t)buffer[3] << 24);
}

static uint64_t time_sync_read_le_u64(const uint8_t *buffer)
{
    return ((uint64_t)buffer[0])
        | ((uint64_t)buffer[1] << 8)
        | ((uint64_t)buffer[2] << 16)
        | ((uint64_t)buffer[3] << 24)
        | ((uint64_t)buffer[4] << 32)
        | ((uint64_t)buffer[5] << 40)
        | ((uint64_t)buffer[6] << 48)
        | ((uint64_t)buffer[7] << 56);
}

static uint32_t time_sync_abs_delta_ms(uint64_t utc_epoch_ms, uint32_t local_tick_ms, int64_t utc_offset_ms)
{
    int64_t local_time_ms = (int64_t)local_tick_ms + utc_offset_ms;
    int64_t delta_ms = (int64_t)utc_epoch_ms - local_time_ms;

    if (delta_ms < 0) {
        delta_ms = -delta_ms;
    }

    if (delta_ms > (int64_t)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)delta_ms;
}

void time_sync_init(time_sync_t *sync, uint32_t sync_timeout_ms, uint32_t max_clock_skew_ms)
{
    if (sync == NULL) {
        return;
    }

    memset(sync, 0, sizeof(*sync));
    sync->sync_timeout_ms = sync_timeout_ms;
    sync->max_clock_skew_ms = max_clock_skew_ms;
}

bool time_sync_should_accept_sample(const time_sync_t *sync,
                                    bool sync_requested,
                                    uint32_t local_tick_ms,
                                    uint64_t received_utc_ms)
{
    uint32_t skew_ms = 0;

    if (sync == NULL) {
        return false;
    }

    if (sync_requested || !sync->utc_valid) {
        return true;
    }

    if (sync->sync_timeout_ms > 0U) {
        uint32_t elapsed_ms = local_tick_ms - sync->last_update_tick_ms;
        if (elapsed_ms >= sync->sync_timeout_ms) {
            return true;
        }
    }

    if (sync->max_clock_skew_ms > 0U && received_utc_ms > 0U) {
        skew_ms = time_sync_abs_delta_ms(received_utc_ms, local_tick_ms, sync->utc_offset_ms);
        if (skew_ms >= sync->max_clock_skew_ms) {
            return true;
        }
    }

    return false;
}

void time_sync_handle_ntp_sample(time_sync_t *sync,
                                 uint64_t utc_epoch_ms,
                                 uint32_t pps_tick_ms,
                                 int64_t utc_offset_ms,
                                 uint32_t update_tick_ms)
{
    if (sync == NULL) {
        return;
    }

    sync->utc_valid = true;
    sync->pps_valid = true;
    sync->utc_offset_ms = utc_offset_ms;
    sync->last_utc_epoch_ms = utc_epoch_ms;
    sync->last_pps_tick_ms = pps_tick_ms;
    sync->last_update_tick_ms = update_tick_ms;
}

void time_sync_handle_payload(time_sync_t *sync,
                              const uint8_t *payload,
                              uint8_t length,
                              bool sync_requested)
{
    const uint8_t minimum_length = TIME_SYNC_FLAGS_SIZE_BYTES + TIME_SYNC_UTC_SIZE_BYTES + TIME_SYNC_PPS_SIZE_BYTES;
    uint32_t local_tick_ms = 0;
    uint8_t sync_flags = 0;
    uint64_t received_utc_ms = 0;
    uint32_t received_pps_tick_ms = 0;
    uint32_t received_rx_tick_ms = 0;
    bool has_rx_tick = false;
    int64_t utc_offset_ms = 0;

    if (sync == NULL || payload == NULL) {
        return;
    }

    if (length < minimum_length) {
        return;
    }

    sync_flags = payload[0];
    if ((sync_flags & TIME_SYNC_FLAG_UTC_VALID) == 0U || (sync_flags & TIME_SYNC_FLAG_PPS_VALID) == 0U) {
        return;
    }

    received_utc_ms = time_sync_read_le_u64(&payload[TIME_SYNC_FLAGS_SIZE_BYTES]);
    received_pps_tick_ms = time_sync_read_le_u32(&payload[TIME_SYNC_FLAGS_SIZE_BYTES + TIME_SYNC_UTC_SIZE_BYTES]);
    local_tick_ms = HAL_GetTick();

    if (!time_sync_should_accept_sample(sync, sync_requested, local_tick_ms, received_utc_ms)) {
        return;
    }

    if (length >= (minimum_length + TIME_SYNC_RX_TICK_SIZE_BYTES)) {
        received_rx_tick_ms = time_sync_read_le_u32(&payload[TIME_SYNC_FLAGS_SIZE_BYTES + TIME_SYNC_UTC_SIZE_BYTES + TIME_SYNC_PPS_SIZE_BYTES]);
        has_rx_tick = (sync_flags & TIME_SYNC_FLAG_RX_TICK_VALID) != 0U;
    }

    if (has_rx_tick) {
        utc_offset_ms = (int64_t)received_utc_ms - (int64_t)received_rx_tick_ms;
    } else {
        utc_offset_ms = (int64_t)received_utc_ms - (int64_t)local_tick_ms;
    }

    time_sync_handle_ntp_sample(sync, received_utc_ms, received_pps_tick_ms, utc_offset_ms, local_tick_ms);
}

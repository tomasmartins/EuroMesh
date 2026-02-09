#include "time_sync.h"

#include <string.h>

void time_sync_init(time_sync_t *sync)
{
    if (sync == NULL) {
        return;
    }
    sync->hrtc = hrtc;
    sync->offset_ms = 0;
    sync->last_rtc_ms = 0;
    sync->last_tick_ms = HAL_GetTick();
}

uint32_t time_sync_now_ms(const time_sync_t *sync)
{
    if (sync == NULL) {
        return HAL_GetTick();
    }
    return (uint32_t)((int32_t)HAL_GetTick() + sync->offset_ms);
}

HAL_StatusTypeDef time_sync_handle_ntp_sample(time_sync_t *sync, uint32_t t1_ms, uint32_t t2_ms, uint32_t t3_ms, uint32_t t4_ms)
{
    int32_t offset = 0;

    if (sync == NULL) {
        return HAL_ERROR;
    }

    offset = (int32_t)(((int32_t)(t2_ms - t1_ms) + (int32_t)(t3_ms - t4_ms)) / 2);
    sync->offset_ms = offset;
    sync->last_tick_ms = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef time_sync_handle_pps(time_sync_t *sync, uint32_t pps_tick_ms)
{
    uint32_t rtc_ms = 0;

    if (sync == NULL) {
        return HAL_ERROR;
    }
    if (time_sync_read_rtc_ms(sync, &rtc_ms) != HAL_OK) {
        return HAL_ERROR;
    }

    sync->offset_ms = (int32_t)rtc_ms - (int32_t)pps_tick_ms;
    sync->last_rtc_ms = rtc_ms;
    sync->last_tick_ms = pps_tick_ms;
    return HAL_OK;

    memset(sync, 0, sizeof(*sync));
}

void time_sync_handle_ntp_sample(time_sync_t *sync,
                                 uint64_t utc_epoch_ms,
                                 uint32_t pps_tick_ms,
                                 int64_t local_offset_ms,
                                 uint32_t update_tick_ms)
{
    if (sync == NULL) {
        return;
    }

    sync->utc_valid = true;
    sync->pps_valid = true;
    sync->utc_offset_ms = local_offset_ms;
    sync->last_utc_epoch_ms = utc_epoch_ms;
    sync->last_pps_tick_ms = pps_tick_ms;
    sync->last_update_tick_ms = update_tick_ms;
}

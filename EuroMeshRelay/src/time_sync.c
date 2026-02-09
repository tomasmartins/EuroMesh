#include "time_sync.h"

static HAL_StatusTypeDef time_sync_read_rtc_ms(time_sync_t *sync, uint32_t *rtc_ms)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    uint32_t subseconds = 0;

    if (sync == NULL || sync->hrtc == NULL || rtc_ms == NULL) {
        return HAL_ERROR;
    }

    if (HAL_RTC_GetTime(sync->hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_RTC_GetDate(sync->hrtc, &date, RTC_FORMAT_BIN) != HAL_OK) {
        return HAL_ERROR;
    }

    subseconds = (sync->hrtc->Instance->SSR & 0xFFFFU);
    *rtc_ms = (uint32_t)time.Hours * 3600000U
        + (uint32_t)time.Minutes * 60000U
        + (uint32_t)time.Seconds * 1000U
        + (1000U - ((subseconds * 1000U) / (sync->hrtc->Init.SynchPrediv + 1U)));

    return HAL_OK;
}

void time_sync_init(time_sync_t *sync, RTC_HandleTypeDef *hrtc)
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
}

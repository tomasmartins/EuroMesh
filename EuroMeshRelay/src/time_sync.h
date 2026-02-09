#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    RTC_HandleTypeDef *hrtc;
    int32_t offset_ms;
    uint32_t last_rtc_ms;
    uint32_t last_tick_ms;
} time_sync_t;

void time_sync_init(time_sync_t *sync, RTC_HandleTypeDef *hrtc);
uint32_t time_sync_now_ms(const time_sync_t *sync);
HAL_StatusTypeDef time_sync_handle_ntp_sample(time_sync_t *sync, uint32_t t1_ms, uint32_t t2_ms, uint32_t t3_ms, uint32_t t4_ms);
HAL_StatusTypeDef time_sync_handle_pps(time_sync_t *sync, uint32_t pps_tick_ms);

#ifdef __cplusplus
}
#endif

#endif

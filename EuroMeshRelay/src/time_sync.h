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
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool utc_valid;
    bool pps_valid;
    int64_t utc_offset_ms;
    uint64_t last_utc_epoch_ms;
    uint32_t last_pps_tick_ms;
    uint32_t last_update_tick_ms;
} time_sync_t;

void time_sync_init(time_sync_t *sync);
void time_sync_handle_ntp_sample(time_sync_t *sync,
                                 uint64_t utc_epoch_ms,
                                 uint32_t pps_tick_ms,
                                 int64_t local_offset_ms,
                                 uint32_t update_tick_ms);

#endif

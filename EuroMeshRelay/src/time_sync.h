#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool utc_valid;
    bool pps_valid;
    int64_t utc_offset_ms;
    uint64_t last_utc_epoch_ms;
    uint32_t last_pps_tick_ms;
    uint32_t last_update_tick_ms;
    uint32_t sync_timeout_ms;
    uint32_t max_clock_skew_ms;
} time_sync_t;

void time_sync_init(time_sync_t *sync, uint32_t sync_timeout_ms, uint32_t max_clock_skew_ms);
bool time_sync_should_accept_sample(const time_sync_t *sync,
                                    bool sync_requested,
                                    uint32_t local_tick_ms,
                                    uint64_t received_utc_ms);
void time_sync_handle_ntp_sample(time_sync_t *sync,
                                 uint64_t utc_epoch_ms,
                                 uint32_t pps_tick_ms,
                                 int64_t utc_offset_ms,
                                 uint32_t update_tick_ms);

#ifdef __cplusplus
}
#endif

#endif

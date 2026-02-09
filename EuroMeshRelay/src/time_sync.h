#ifndef TIME_SYNC_H
#define TIME_SYNC_H

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

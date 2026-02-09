#include "time_sync.h"

#include <string.h>

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

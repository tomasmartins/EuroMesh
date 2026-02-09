#include "time_sync.h"

#include <string.h>

void time_sync_init(time_sync_t *sync)
{
    if (sync == NULL) {
        return;
    }

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

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

#define TIME_SYNC_FLAG_UTC_VALID     0x01
#define TIME_SYNC_FLAG_PPS_VALID     0x02
#define TIME_SYNC_FLAG_RX_TICK_VALID 0x04
#define TIME_SYNC_FLAGS_SIZE_BYTES   1U
#define TIME_SYNC_UTC_SIZE_BYTES     8U
#define TIME_SYNC_PPS_SIZE_BYTES     4U
#define TIME_SYNC_RX_TICK_SIZE_BYTES 4U

#define TIME_SYNC_TIMEOUT_MS  30000U
#define TIME_SYNC_MAX_SKEW_MS 2000U

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

void time_sync_handle_payload(time_sync_t *sync,
                              const uint8_t *payload,
                              uint8_t length,
                              bool sync_requested);

#ifdef __cplusplus
}
#endif

#endif

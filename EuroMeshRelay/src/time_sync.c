#include "time_sync.h"
#include "emesh_node_caps.h"
#include "stm32f4xx_hal.h"

#include <string.h>

/* ── Internal helpers ────────────────────────────────────────────────────── */

static uint32_t ts_read_le_u32(const uint8_t *b)
{
    return (uint32_t) b[0]
         | ((uint32_t)b[1] << 8)
         | ((uint32_t)b[2] << 16)
         | ((uint32_t)b[3] << 24);
}

static uint64_t ts_read_le_u64(const uint8_t *b)
{
    return (uint64_t) b[0]
         | ((uint64_t)b[1] << 8)
         | ((uint64_t)b[2] << 16)
         | ((uint64_t)b[3] << 24)
         | ((uint64_t)b[4] << 32)
         | ((uint64_t)b[5] << 40)
         | ((uint64_t)b[6] << 48)
         | ((uint64_t)b[7] << 56);
}

static uint32_t ts_abs_delta_ms(uint64_t received_utc_ms,
                                uint32_t local_tick_ms,
                                int64_t  utc_offset_ms)
{
    int64_t local_utc = (int64_t)local_tick_ms + utc_offset_ms;
    int64_t delta     = (int64_t)received_utc_ms - local_utc;

    if (delta < 0) { delta = -delta; }
    if (delta > (int64_t)UINT32_MAX) { return UINT32_MAX; }
    return (uint32_t)delta;
}

/* Update the linear drift estimate given a new sync point. */
static void ts_update_drift(time_sync_t *sync,
                            uint64_t utc_epoch_ms,
                            uint32_t update_tick_ms)
{
    uint32_t elapsed_ticks;
    int64_t  old_offset;
    int64_t  new_offset;
    int64_t  drift;

    if (!sync->utc_valid || sync->drift_ref_tick_ms == 0U) {
        /* First sample — just record the reference. */
        sync->drift_ref_tick_ms = update_tick_ms;
        sync->drift_ref_utc_ms  = utc_epoch_ms;
        sync->drift_ppm         = 0;
        return;
    }

    elapsed_ticks = update_tick_ms - sync->drift_ref_tick_ms;
    if (elapsed_ticks < 1000U) {
        return; /* too short for a stable estimate */
    }

    old_offset = (int64_t)sync->drift_ref_utc_ms - (int64_t)sync->drift_ref_tick_ms;
    new_offset = (int64_t)utc_epoch_ms            - (int64_t)update_tick_ms;
    drift      = (new_offset - old_offset) * 1000000LL / (int64_t)elapsed_ticks;

    /* Clamp to ±500 ppm — anything beyond that is likely a step correction. */
    if (drift >  500LL) { drift =  500LL; }
    if (drift < -500LL) { drift = -500LL; }
    sync->drift_ppm = (int32_t)drift;

    sync->drift_ref_tick_ms = update_tick_ms;
    sync->drift_ref_utc_ms  = utc_epoch_ms;
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void time_sync_init(time_sync_t *sync,
                    uint32_t sync_timeout_ms,
                    uint32_t max_clock_skew_ms)
{
    if (sync == NULL) {
        return;
    }
    memset(sync, 0, sizeof(*sync));
    sync->sync_timeout_ms  = sync_timeout_ms;
    sync->max_clock_skew_ms = max_clock_skew_ms;
    sync->resync_interval_ms = TIME_SYNC_RESYNC_TIGHT_MS;
    sync->stratum            = EMESH_STRATUM_UNKNOWN;
}

/* ── Query ───────────────────────────────────────────────────────────────── */

uint64_t time_sync_now_utc_ms(const time_sync_t *sync)
{
    uint32_t now_tick;
    uint32_t elapsed;
    int64_t  drift_correction;

    if (sync == NULL || !sync->utc_valid) {
        return 0U;
    }

    now_tick = HAL_GetTick();
    elapsed  = now_tick - sync->last_update_tick_ms;

    /* Apply linear drift correction: correction_ms = elapsed_ms × ppm / 1e6 */
    drift_correction = ((int64_t)elapsed * sync->drift_ppm) / 1000000LL;

    return sync->last_utc_epoch_ms + (uint64_t)elapsed + (uint64_t)drift_correction;
}

bool time_sync_needs_resync(const time_sync_t *sync, uint32_t now_ms)
{
    if (sync == NULL || !sync->utc_valid) {
        return true;
    }
    return (now_ms - sync->last_twoway_ms) >= sync->resync_interval_ms;
}

/* ── Accept logic ────────────────────────────────────────────────────────── */

bool time_sync_should_accept(const time_sync_t *sync,
                             uint8_t  src_stratum,
                             bool     sync_requested,
                             uint32_t local_tick_ms,
                             uint64_t received_utc_ms)
{
    uint32_t elapsed_ms;
    uint32_t skew_ms;

    if (sync == NULL) {
        return false;
    }

    /* Always accept if we have no valid time yet. */
    if (!sync->utc_valid) {
        return true;
    }

    /* Always accept from a better (lower-numbered) stratum source. */
    if (src_stratum < sync->stratum) {
        return true;
    }

    /* Explicit request from application. */
    if (sync_requested) {
        return true;
    }

    /* Accept if our current sync is stale. */
    if (sync->sync_timeout_ms > 0U) {
        elapsed_ms = local_tick_ms - sync->last_update_tick_ms;
        if (elapsed_ms >= sync->sync_timeout_ms) {
            return true;
        }
    }

    /* Accept if the incoming time deviates beyond our tolerance. */
    if (sync->max_clock_skew_ms > 0U && received_utc_ms > 0U) {
        skew_ms = ts_abs_delta_ms(received_utc_ms, local_tick_ms,
                                  sync->utc_offset_ms);
        if (skew_ms >= sync->max_clock_skew_ms) {
            return true;
        }
    }

    return false;
}

/* ── Legacy shim ─────────────────────────────────────────────────────────── */

bool time_sync_should_accept_sample(const time_sync_t *sync,
                                    bool sync_requested,
                                    uint32_t local_tick_ms,
                                    uint64_t received_utc_ms)
{
    uint8_t stratum = (sync != NULL) ? sync->stratum : EMESH_STRATUM_UNKNOWN;
    return time_sync_should_accept(sync, stratum, sync_requested,
                                   local_tick_ms, received_utc_ms);
}

/* ── Update ──────────────────────────────────────────────────────────────── */

void time_sync_apply_sample(time_sync_t *sync,
                            uint64_t utc_epoch_ms,
                            uint32_t pps_tick_ms,
                            uint8_t  src_stratum,
                            uint32_t update_tick_ms)
{
    if (sync == NULL) {
        return;
    }

    ts_update_drift(sync, utc_epoch_ms, update_tick_ms);

    sync->utc_valid          = true;
    sync->pps_valid          = (pps_tick_ms != 0U);
    sync->last_utc_epoch_ms  = utc_epoch_ms;
    sync->last_pps_tick_ms   = pps_tick_ms;
    sync->last_update_tick_ms = update_tick_ms;
    sync->utc_offset_ms      = (int64_t)utc_epoch_ms - (int64_t)update_tick_ms;

    /* Our stratum is one hop further than the source. */
    if (src_stratum < (EMESH_STRATUM_UNKNOWN - 1U)) {
        sync->stratum = src_stratum + 1U;
    } else {
        sync->stratum = EMESH_STRATUM_UNKNOWN;
    }
}

void time_sync_apply_twoway(time_sync_t *sync,
                            int64_t  offset_ms,
                            int32_t  rtt_ms,
                            uint8_t  src_stratum,
                            uint32_t now_ms)
{
    uint64_t corrected_utc;

    if (sync == NULL) {
        return;
    }

    corrected_utc = (uint64_t)((int64_t)time_sync_now_utc_ms(sync) + offset_ms);

    sync->last_rtt_ms   = rtt_ms;
    sync->last_twoway_ms = now_ms;

    time_sync_apply_sample(sync, corrected_utc, sync->last_pps_tick_ms,
                           src_stratum, now_ms);
}

/* ── Legacy shim ─────────────────────────────────────────────────────────── */

void time_sync_handle_ntp_sample(time_sync_t *sync,
                                 uint64_t utc_epoch_ms,
                                 uint32_t pps_tick_ms,
                                 int64_t  utc_offset_ms,
                                 uint32_t update_tick_ms)
{
    uint8_t stratum = (sync != NULL) ? sync->stratum : EMESH_STRATUM_UNKNOWN;

    (void)utc_offset_ms; /* recomputed internally by time_sync_apply_sample */
    time_sync_apply_sample(sync, utc_epoch_ms, pps_tick_ms, stratum, update_tick_ms);
}

/* ── Beacon handler ──────────────────────────────────────────────────────── */

void time_sync_handle_beacon(time_sync_t *sync,
                             const uint8_t *payload_after_type,
                             uint8_t length,
                             bool sync_requested)
{
    /*
     * Expected layout of payload_after_type (15 bytes):
     *   [sync_flags:1][utc_ms:8][pps_tick:4][stratum:1][net_flags:1]
     * The type byte has already been consumed by the caller.
     */
    const uint8_t MIN_LEN = TIME_SYNC_FLAGS_SIZE_BYTES
                          + TIME_SYNC_UTC_SIZE_BYTES
                          + TIME_SYNC_PPS_SIZE_BYTES;
    uint8_t  sync_flags;
    uint64_t received_utc_ms;
    uint32_t received_pps_ms;
    uint8_t  src_stratum;
    uint32_t local_tick_ms;

    if (sync == NULL || payload_after_type == NULL || length < MIN_LEN) {
        return;
    }

    sync_flags = payload_after_type[0];
    if ((sync_flags & TIME_SYNC_FLAG_UTC_VALID) == 0U) {
        return; /* sender has no valid time — ignore */
    }

    received_utc_ms = ts_read_le_u64(&payload_after_type[1]);
    received_pps_ms = ts_read_le_u32(&payload_after_type[9]);
    src_stratum     = (length >= MIN_LEN + 1U) ? payload_after_type[MIN_LEN] : EMESH_STRATUM_UNKNOWN;

    local_tick_ms = HAL_GetTick();

    if (!time_sync_should_accept(sync, src_stratum, sync_requested,
                                 local_tick_ms, received_utc_ms)) {
        return;
    }

    time_sync_apply_sample(sync, received_utc_ms, received_pps_ms,
                           src_stratum, local_tick_ms);
}

/*
 * time_sync.h — Clock synchronisation state and API.
 *
 * Tier roles:
 *   Gateway  — GPS PPS-disciplined (stratum 0); authoritative time server.
 *   Relay    — Syncs from gateway beacon; propagates time (stratum = parent+1).
 *   Node     — Keeps local RTC; syncs from beacon or two-way TIME_REQ/RESP.
 *
 * Resync guidance:
 *   Tight timing (±5 ms slots)   → resync every 5–15 minutes
 *   Rough alignment (±1 s)       → resync every 1–6 hours
 *
 * Two-way sync: node sends TIME_REQ; relay echoes T1/T2/T3 in TIME_RESP;
 * node computes offset = ((T2−T1)+(T3−T4))/2 via time_twoway_compute().
 */

#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Payload flag bits (used in beacon and time_sync_handle_beacon) ──────── */
#define TIME_SYNC_FLAG_UTC_VALID      0x01U
#define TIME_SYNC_FLAG_PPS_VALID      0x02U
#define TIME_SYNC_FLAG_RX_TICK_VALID  0x04U

/* Internal payload field sizes (used by time_sync_handle_beacon). */
#define TIME_SYNC_FLAGS_SIZE_BYTES    1U
#define TIME_SYNC_UTC_SIZE_BYTES      8U
#define TIME_SYNC_PPS_SIZE_BYTES      4U
#define TIME_SYNC_RX_TICK_SIZE_BYTES  4U

/* Default resync intervals */
#define TIME_SYNC_TIMEOUT_MS       30000U   /* 30 s — accept next beacon     */
#define TIME_SYNC_MAX_SKEW_MS       2000U   /* 2 s  — hard re-accept limit   */
#define TIME_SYNC_RESYNC_TIGHT_MS  300000U  /* 5 min — tight-slot nodes      */
#define TIME_SYNC_RESYNC_LOOSE_MS 3600000U  /* 1 h  — rough-time nodes       */

typedef struct {
    /* ── Validity ──────────────────────────────────────────────────────── */
    bool     utc_valid;
    bool     pps_valid;
    uint8_t  stratum;           /* 0=GPS authority, 255=unknown            */
    uint8_t  _pad0;

    /* ── Last accepted sample ───────────────────────────────────────────── */
    uint64_t last_utc_epoch_ms; /* UTC at last_update_tick_ms              */
    uint32_t last_pps_tick_ms;  /* HAL_GetTick() at last PPS edge          */
    uint32_t last_update_tick_ms;

    /* ── Derived offset (HAL_GetTick() + utc_offset_ms ≈ UTC) ──────────── */
    int64_t  utc_offset_ms;

    /* ── Linear drift estimate ──────────────────────────────────────────── */
    int32_t  drift_ppm;         /* clamped to ±500 ppm                     */
    uint32_t drift_ref_tick_ms;
    uint64_t drift_ref_utc_ms;

    /* ── Two-way sync state ─────────────────────────────────────────────── */
    uint32_t last_twoway_ms;    /* HAL_GetTick() of last completed exchange */
    int32_t  last_rtt_ms;       /* most recent measured RTT                 */

    /* ── Config ─────────────────────────────────────────────────────────── */
    uint32_t sync_timeout_ms;
    uint32_t max_clock_skew_ms;
    uint32_t resync_interval_ms; /* how often to request two-way sync       */
} time_sync_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void time_sync_init(time_sync_t *sync,
                    uint32_t sync_timeout_ms,
                    uint32_t max_clock_skew_ms);

/* ── Query ───────────────────────────────────────────────────────────────── */

/*
 * Return the best estimate of the current UTC in milliseconds, compensating
 * for measured drift.  Returns 0 if UTC is not yet valid.
 */
uint64_t time_sync_now_utc_ms(const time_sync_t *sync);

/*
 * Return true if a two-way sync is due (last_twoway_ms is older than
 * resync_interval_ms).
 */
bool time_sync_needs_resync(const time_sync_t *sync, uint32_t now_ms);

/* ── Accept logic ────────────────────────────────────────────────────────── */

/*
 * Stratum-aware accept decision:
 *   - Always accept if UTC not yet valid.
 *   - Always accept if src_stratum < sync->stratum (better source).
 *   - Otherwise apply skew / timeout guards.
 */
bool time_sync_should_accept(const time_sync_t *sync,
                             uint8_t  src_stratum,
                             bool     sync_requested,
                             uint32_t local_tick_ms,
                             uint64_t received_utc_ms);

/* ── Update ──────────────────────────────────────────────────────────────── */

/*
 * Apply a new time sample.  Computes drift if a previous sample exists.
 * Sets sync->stratum = src_stratum + 1 (we are one hop from the source).
 * Gateways call this with src_stratum = EMESH_STRATUM_GPS and src_stratum=0
 * to mark themselves as authoritative.
 */
void time_sync_apply_sample(time_sync_t *sync,
                            uint64_t utc_epoch_ms,
                            uint32_t pps_tick_ms,
                            uint8_t  src_stratum,
                            uint32_t update_tick_ms);

/*
 * Apply the result of a two-way exchange.
 * offset_ms and rtt_ms come from time_twoway_compute().
 */
void time_sync_apply_twoway(time_sync_t *sync,
                            int64_t  offset_ms,
                            int32_t  rtt_ms,
                            uint8_t  src_stratum,
                            uint32_t now_ms);

/*
 * Parse a beacon payload (starting at the byte AFTER the type byte) and
 * apply the time sample if the stratum-aware accept logic allows it.
 * Replaces the old time_sync_handle_payload.
 */
void time_sync_handle_beacon(time_sync_t *sync,
                             const uint8_t *payload_after_type,
                             uint8_t length,
                             bool sync_requested);

/* ── Legacy shim (kept for any existing callers) ─────────────────────────── */
void time_sync_handle_ntp_sample(time_sync_t *sync,
                                 uint64_t utc_epoch_ms,
                                 uint32_t pps_tick_ms,
                                 int64_t  utc_offset_ms,
                                 uint32_t update_tick_ms);

bool time_sync_should_accept_sample(const time_sync_t *sync,
                                    bool sync_requested,
                                    uint32_t local_tick_ms,
                                    uint64_t received_utc_ms);

#ifdef __cplusplus
}
#endif

#endif /* TIME_SYNC_H */

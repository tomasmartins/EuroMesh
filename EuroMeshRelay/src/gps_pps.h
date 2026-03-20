/*
 * gps_pps.h — GPS NMEA + PPS time source driver.
 *
 * Hardware connections (configurable via macros below):
 *   GPS UART RX — USART3 on PC11 (AF7)   — NMEA sentences at 9600 baud
 *   PPS input   — PA0 (EXTI0, rising edge) — 1 Hz pulse at top of second
 *
 * Operation:
 *   NMEA $GPRMC sentences are received over USART3.  The PPS edge fires at
 *   the exact UTC second boundary.  When both a valid $GPRMC sentence and a
 *   recent PPS edge are available, the module is considered locked and can
 *   provide a precise UTC timestamp anchored to the PPS tick.
 *
 * Lock condition:
 *   gps_pps_is_locked() returns true when:
 *     1. The most recent $GPRMC sentence had status = 'A' (active).
 *     2. A PPS edge was seen within GPS_PPS_MAX_AGE_MS of now.
 *
 * Usage:
 *   1. Call gps_pps_init() once after HAL_Init().
 *   2. The driver feeds itself via ISR (USART3 + EXTI0) — no polling needed.
 *   3. Each super-frame, call gps_pps_apply_to_sync() to push a fresh GPS
 *      sample into the time_sync module when locked.
 */

#ifndef GPS_PPS_H
#define GPS_PPS_H

#include <stdbool.h>
#include <stdint.h>

#include "time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Hardware pin configuration ───────────────────────────────────────────── */
/*
 * GPS UART: USART3, 9600 baud.
 *   Only RX is needed (we do not send commands to the GPS module).
 *   PC11 = USART3_RX, AF7.
 */
#define GPS_UART_INSTANCE     USART3
#define GPS_UART_BAUD         9600U
#define GPS_UART_RX_PORT      GPIOC
#define GPS_UART_RX_PIN       GPIO_PIN_11
#define GPS_UART_RX_AF        GPIO_AF7_USART3
#define GPS_UART_IRQn         USART3_IRQn

/*
 * PPS input: PA0, EXTI line 0, rising edge.
 *   The 1 Hz PPS pulse arrives at the exact UTC second boundary.
 */
#define GPS_PPS_PORT          GPIOA
#define GPS_PPS_PIN           GPIO_PIN_0
#define GPS_PPS_EXTI_IRQn     EXTI0_IRQn

/* ── Timing constants ─────────────────────────────────────────────────────── */
/* Maximum age of a PPS edge for the module to be considered locked (ms). */
#define GPS_PPS_MAX_AGE_MS    2000U

/* NMEA sentence buffer size (longest practical sentence < 90 bytes). */
#define GPS_NMEA_BUF_SIZE     100U

/* ── State struct ─────────────────────────────────────────────────────────── */
typedef struct {
    /* ── NMEA parser state ───────────────────────────────────────────────── */
    char     nmea_buf[GPS_NMEA_BUF_SIZE];
    uint8_t  nmea_len;
    bool     nmea_overflow;

    /* ── Last good fix ───────────────────────────────────────────────────── */
    bool     fix_valid;           /* true if last $GPRMC status was 'A'     */
    uint64_t fix_utc_epoch_s;     /* UTC epoch seconds at the last fix      */

    /* ── PPS tracking ────────────────────────────────────────────────────── */
    bool     pps_seen;            /* at least one PPS edge received          */
    uint32_t pps_tick_ms;         /* HAL_GetTick() at the last PPS edge     */

    /* ── HAL handle (owned by this module) ──────────────────────────────── */
    UART_HandleTypeDef huart;
    uint8_t            rx_byte;   /* single-byte DMA/IT target              */
} gps_pps_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/*
 * Initialise USART3 (RX only, 9600 baud) and the PPS EXTI input.
 * Clocks must already be enabled (call after SystemClock_Config).
 */
void gps_pps_init(gps_pps_t *gps);

/* ── ISR entry points (call from the relevant IRQ handlers) ──────────────── */

/*
 * Feed one byte from the USART3 RX ISR.
 * The driver re-arms the single-byte interrupt automatically.
 */
void gps_pps_feed_byte(gps_pps_t *gps, uint8_t byte);

/*
 * Record the HAL_GetTick() value at which the PPS rising edge fired.
 * Call from the EXTI0 IRQ handler.
 */
void gps_pps_on_pps_edge(gps_pps_t *gps, uint32_t tick_ms);

/* ── Query ───────────────────────────────────────────────────────────────── */

/*
 * Returns true if a valid fix and a recent PPS edge are both available.
 * now_ms should be HAL_GetTick().
 */
bool gps_pps_is_locked(const gps_pps_t *gps, uint32_t now_ms);

/*
 * Apply the current GPS time sample to a time_sync_t.
 * Does nothing if not locked.  Should be called once per super-frame.
 * Passes src_stratum = EMESH_STRATUM_GPS so that the relay's sync->stratum
 * is set to 1 (GPS-disciplined relay, one hop from the GPS source).
 */
void gps_pps_apply_to_sync(gps_pps_t *gps,
                            time_sync_t *sync,
                            uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* GPS_PPS_H */

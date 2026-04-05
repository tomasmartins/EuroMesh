/*
 * lgw_hal.h — Thin wrapper around the Semtech libloragw SX1301 HAL.
 *
 * Exposes only the three primitives the gateway needs:
 *   lgw_hal_init()      — configure and start the concentrator.
 *   lgw_hal_receive()   — poll for one received LoRa frame (non-blocking).
 *   lgw_hal_transmit()  — send a raw LoRa frame immediately.
 *
 * Hardware assumptions:
 *   Concentrator: iC880A or RAK831 (SX1301 + SX1257 RF front-end).
 *   SPI:          /dev/spidev0.0  (or as mapped by libloragw board driver).
 *   Frequency:    869.525 MHz  (EU ISM band, matching the relay).
 *   Modulation:   SF7, BW125 kHz, CR 4/5.
 *   IQ polarity:  non-inverted (invert_pol = false) — must match the relay
 *                 SX1276 which uses the default non-inverted RX/TX IQ mode.
 *
 * Clock source:
 *   RF chain 1 provides the reference clock (CLKSRC = 1), which is the
 *   standard configuration for iC880A / RAK831 boards.
 *
 * Receive:
 *   The multi-SF channel (IF0) is configured with DR_LORA_MULTI so the
 *   concentrator can receive SF7–SF12.  In practice, relays transmit SF7.
 */

#ifndef LGW_HAL_H
#define LGW_HAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Per-frame metadata returned by lgw_hal_receive() ───────────────────── */
typedef struct {
    float    rssi_dbm;    /* average RSSI of the received frame             */
    float    snr_db;      /* average SNR                                    */
    uint32_t timestamp_us;/* concentrator internal counter at RX (microsec) */
} lgw_rx_meta_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/*
 * Initialise and start the SX1301 concentrator.
 * Returns true on success; false on any libloragw error.
 * Must be called once before lgw_hal_receive() or lgw_hal_transmit().
 */
bool lgw_hal_init(void);

/*
 * Stop the concentrator and release resources.
 * Safe to call even if lgw_hal_init() was not called or failed.
 */
void lgw_hal_stop(void);

/* ── Frame I/O ───────────────────────────────────────────────────────────── */

/*
 * Poll for one received frame (non-blocking).
 *
 * Fills buf[0..(*len_out - 1)] with the raw frame bytes and populates *meta.
 * Returns true if a valid LoRa frame was received, false if the channel is
 * empty or an error occurred.
 *
 * buf_capacity must be at least 256 bytes (maximum LoRa payload for SX1301).
 */
bool lgw_hal_receive(uint8_t *buf, uint16_t buf_capacity,
                     uint8_t *len_out, lgw_rx_meta_t *meta);

/*
 * Transmit len bytes from buf immediately (IMMEDIATE TX mode).
 * Returns true if lgw_send() accepted the packet, false on error.
 */
bool lgw_hal_transmit(const uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* LGW_HAL_H */

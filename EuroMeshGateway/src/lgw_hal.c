/*
 * lgw_hal.c — Semtech libloragw SX1301 HAL wrapper.
 *
 * Concentrator board: iC880A or RAK831 (SX1301 + SX1257).
 * Channel plan: single channel, 869.525 MHz, SF7-12 multi-SF RX, SF7 TX.
 * IQ polarity: non-inverted on both RX and TX (matches SX1276 relay default).
 *
 * libloragw API version: v1.5.x (Semtech legacy concentrator library).
 * Install: git clone https://github.com/Lora-net/lora_gateway && make install
 */

#include "lgw_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Semtech libloragw public header. */
#include "loragw_hal.h"

/* ── Configuration constants ─────────────────────────────────────────────── */

#define LGW_FREQ_HZ           869525000U    /* 869.525 MHz EU ISM            */
#define LGW_RF_POWER_DBM      14            /* 14 dBm — well within duty cap */
#define LGW_PREAMBLE_SYMS     8U            /* standard LoRa preamble length */
#define LGW_TX_RF_CHAIN       0U            /* RF chain used for TX          */

/*
 * RSSI calibration offset for SX1257 + iC880A / RAK831.
 * Adjust per board if measured RSSI differs from expected.
 */
#define LGW_RSSI_OFFSET       -166.0f

/* ── Module state ────────────────────────────────────────────────────────── */

static bool g_started = false;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

bool lgw_hal_init(void)
{
    struct lgw_board_conf_s boardconf;
    struct lgw_conf_rxrf_s  rfconf;
    struct lgw_conf_rxif_s  ifconf;
    int                     rc;

    if (g_started) {
        return true; /* already initialised */
    }

    /* ── Board configuration ─────────────────────────────────────────────── */
    memset(&boardconf, 0, sizeof(boardconf));
    boardconf.lorawan_public = false; /* EuroMesh, not LoRaWAN */
    boardconf.clksrc         = 1U;    /* RF chain 1 is the clock source      */

    rc = lgw_board_setconf(boardconf);
    if (rc != LGW_HAL_SUCCESS) {
        fprintf(stderr, "[lgw_hal] lgw_board_setconf failed (%d)\n", rc);
        return false;
    }

    /* ── RF chain 0: 869.525 MHz, TX enabled ────────────────────────────── */
    memset(&rfconf, 0, sizeof(rfconf));
    rfconf.enable      = true;
    rfconf.freq_hz     = LGW_FREQ_HZ;
    rfconf.rssi_offset = LGW_RSSI_OFFSET;
    rfconf.type        = LGW_RADIO_TYPE_SX1257;
    rfconf.tx_enable   = true;

    rc = lgw_rxrf_setconf(0, rfconf);
    if (rc != LGW_HAL_SUCCESS) {
        fprintf(stderr, "[lgw_hal] lgw_rxrf_setconf(0) failed (%d)\n", rc);
        return false;
    }

    /* ── RF chain 1: same frequency, clock source only, TX disabled ──────── */
    memset(&rfconf, 0, sizeof(rfconf));
    rfconf.enable      = true;
    rfconf.freq_hz     = LGW_FREQ_HZ;
    rfconf.rssi_offset = LGW_RSSI_OFFSET;
    rfconf.type        = LGW_RADIO_TYPE_SX1257;
    rfconf.tx_enable   = false;

    rc = lgw_rxrf_setconf(1, rfconf);
    if (rc != LGW_HAL_SUCCESS) {
        fprintf(stderr, "[lgw_hal] lgw_rxrf_setconf(1) failed (%d)\n", rc);
        return false;
    }

    /* ── IF channel 0: 0 Hz offset from RF0, multi-SF LoRa (SF7-SF12) ───── */
    /*
     * DR_LORA_MULTI allows the multi-SF demodulator to receive any spreading
     * factor.  The relay transmits SF7; other spreading factors are silently
     * discarded at the frame dispatch layer.
     */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.enable    = true;
    ifconf.rf_chain  = 0U;
    ifconf.freq_hz   = 0;          /* on-centre: 869.525 MHz exactly        */
    ifconf.bandwidth = BW_125KHZ;
    ifconf.datarate  = DR_LORA_MULTI; /* accept SF7–SF12                    */

    rc = lgw_rxif_setconf(0, ifconf);
    if (rc != LGW_HAL_SUCCESS) {
        fprintf(stderr, "[lgw_hal] lgw_rxif_setconf(0) failed (%d)\n", rc);
        return false;
    }

    /* ── Start the concentrator ──────────────────────────────────────────── */
    rc = lgw_start();
    if (rc != LGW_HAL_SUCCESS) {
        fprintf(stderr, "[lgw_hal] lgw_start() failed (%d)\n", rc);
        return false;
    }

    g_started = true;
    printf("[lgw_hal] SX1301 started: %.3f MHz SF7-12 BW125\n",
           (double)LGW_FREQ_HZ / 1e6);
    return true;
}

void lgw_hal_stop(void)
{
    if (g_started) {
        (void)lgw_stop();
        g_started = false;
    }
}

/* ── Frame I/O ───────────────────────────────────────────────────────────── */

bool lgw_hal_receive(uint8_t *buf, uint8_t buf_capacity,
                     uint8_t *len_out, lgw_rx_meta_t *meta)
{
    struct lgw_pkt_rx_s rxpkt[4]; /* fetch up to 4 queued packets per call  */
    int nb_pkt;
    int i;

    if (!g_started || buf == NULL || len_out == NULL || meta == NULL) {
        return false;
    }

    nb_pkt = lgw_receive((int)(sizeof(rxpkt) / sizeof(rxpkt[0])), rxpkt);
    if (nb_pkt == LGW_HAL_ERROR) {
        fprintf(stderr, "[lgw_hal] lgw_receive() error\n");
        return false;
    }

    for (i = 0; i < nb_pkt; i++) {
        const struct lgw_pkt_rx_s *p = &rxpkt[i];

        /* Accept only LoRa packets with a good CRC. */
        if (p->modulation != MOD_LORA) {
            continue;
        }
        if (p->status != STAT_CRC_OK) {
            continue;
        }
        /* Accept only SF7 (the relay's SF). */
        if (p->datarate != DR_LORA_SF7) {
            continue;
        }
        if ((uint16_t)p->size > (uint16_t)buf_capacity) {
            fprintf(stderr, "[lgw_hal] frame too large (%u > %u)\n",
                    (unsigned)p->size, (unsigned)buf_capacity);
            continue;
        }

        memcpy(buf, p->payload, p->size);
        *len_out          = (uint8_t)p->size;
        meta->rssi_dbm    = p->rssi;
        meta->snr_db      = p->snr;
        meta->timestamp_us = p->count_us;
        return true; /* return the first valid frame; caller loops for more */
    }

    return false; /* nothing received */
}

bool lgw_hal_transmit(const uint8_t *buf, uint8_t len)
{
    struct lgw_pkt_tx_s txpkt;
    int rc;

    if (!g_started || buf == NULL || len == 0U) {
        return false;
    }

    memset(&txpkt, 0, sizeof(txpkt));
    txpkt.freq_hz    = LGW_FREQ_HZ;
    txpkt.tx_mode    = IMMEDIATE;         /* send as soon as possible        */
    txpkt.rf_chain   = LGW_TX_RF_CHAIN;
    txpkt.rf_power   = LGW_RF_POWER_DBM;
    txpkt.modulation = MOD_LORA;
    txpkt.bandwidth  = BW_125KHZ;
    txpkt.datarate   = DR_LORA_SF7;
    txpkt.coderate   = CR_LORA_4_5;
    /*
     * invert_pol = false: non-inverted IQ.
     * The relay SX1276 uses the default non-inverted IQ mode, so gateway TX
     * must also be non-inverted for the relay to receive correctly.
     */
    txpkt.invert_pol = false;
    txpkt.preamble   = LGW_PREAMBLE_SYMS;
    txpkt.no_crc     = false;
    txpkt.no_header  = false;
    txpkt.size       = (uint16_t)len;
    memcpy(txpkt.payload, buf, len);

    rc = lgw_send(txpkt);
    if (rc != LGW_HAL_SUCCESS) {
        fprintf(stderr, "[lgw_hal] lgw_send() failed (%d)\n", rc);
        return false;
    }

    /*
     * Wait for the TX to complete before returning.
     * lgw_send() returns immediately; poll lgw_status() for TX_FREE.
     */
    {
        uint8_t tx_status = TX_STATUS_UNKNOWN;
        int     retries   = 0;
        do {
            (void)lgw_status(TX_STATUS, &tx_status);
            if (tx_status == TX_FREE) {
                break;
            }
            /* ~1 ms sleep between polls (usleep is POSIX). */
            (void)usleep(1000);
            retries++;
        } while (retries < 3000); /* 3 s hard limit */
    }

    return true;
}

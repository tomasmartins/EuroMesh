/*
 * test_sx1301.c — EuroMeshGateway SX1301 hardware connectivity test.
 *
 * Performs the GPIO reset, initialises the concentrator via lgw_hal, then
 * listens for 10 seconds and reports any received LoRa frames.
 * Exits 0 on success (concentrator started), 1 on failure.
 *
 * Usage:
 *   sudo ./test_sx1301 [rx_seconds]
 *
 * Environment overrides (same as lgw_hal):
 *   LGW_RESET_CHIP  — libgpiod chip name  (default: gpiochip4)
 *   LGW_RESET_LINE  — libgpiod line number (default: 21 = PIN_15)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "../src/lgw_hal.h"

#define DEFAULT_RX_SECONDS  10

int main(int argc, char *argv[])
{
    int      rx_seconds = DEFAULT_RX_SECONDS;
    int      rx_count   = 0;
    uint8_t  buf[256];
    uint8_t  len;
    lgw_rx_meta_t meta;
    time_t   deadline;

    if (argc >= 2) {
        rx_seconds = atoi(argv[1]);
        if (rx_seconds < 0) { rx_seconds = 0; }
    }

    printf("[test_sx1301] Init SX1301...\n");
    if (!lgw_hal_init()) {
        fprintf(stderr, "[test_sx1301] FAIL: lgw_hal_init() returned false\n");
        return 1;
    }
    printf("[test_sx1301] OK: concentrator started\n");

    if (rx_seconds > 0) {
        printf("[test_sx1301] Listening for LoRa frames for %d second(s)...\n",
               rx_seconds);
        deadline = time(NULL) + rx_seconds;

        while (time(NULL) < deadline) {
            len = 0;
            if (lgw_hal_receive(buf, (uint8_t)sizeof(buf), &len, &meta)) {
                printf("[test_sx1301] RX  len=%-3u  rssi=%.0f dBm  snr=%.1f dB  "
                       "ts=%u us\n",
                       len, meta.rssi_dbm, meta.snr_db, meta.timestamp_us);
                rx_count++;
            } else {
                usleep(5000);
            }
        }

        printf("[test_sx1301] Received %d frame(s) in %d second(s)\n",
               rx_count, rx_seconds);
    }

    lgw_hal_stop();
    printf("[test_sx1301] Done\n");
    return 0;
}

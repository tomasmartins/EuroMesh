/*
 * sx1276.h — SX1276 LoRa radio driver interface.
 *
 * Radio settings for EuroMesh: SF7, 125 kHz BW, 869.525 MHz, PA_BOOST.
 * SX1276 internal FIFO: 256 bytes (shared TX/RX base addresses).
 */

#ifndef SX1276_H
#define SX1276_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Hardware handle ─────────────────────────────────────────────────────── */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t      nss_pin;
    GPIO_TypeDef *reset_port;
    uint16_t      reset_pin;
    GPIO_TypeDef *dio0_port;
    uint16_t      dio0_pin;
} sx1276_t;

/* ── Register map ────────────────────────────────────────────────────────── */
#define SX1276_REG_FIFO                0x00U
#define SX1276_REG_OP_MODE             0x01U
#define SX1276_REG_FRF_MSB             0x06U
#define SX1276_REG_FRF_MID             0x07U
#define SX1276_REG_FRF_LSB             0x08U
#define SX1276_REG_PA_CONFIG           0x09U
#define SX1276_REG_LNA                 0x0CU
#define SX1276_REG_FIFO_ADDR_PTR       0x0DU
#define SX1276_REG_FIFO_TX_BASE_ADDR   0x0EU
#define SX1276_REG_FIFO_RX_BASE_ADDR   0x0FU
#define SX1276_REG_FIFO_RX_CURRENT     0x10U
#define SX1276_REG_IRQ_FLAGS           0x12U
#define SX1276_REG_RX_NB_BYTES         0x13U
#define SX1276_REG_PKT_SNR_VALUE       0x19U
#define SX1276_REG_PKT_RSSI_VALUE      0x1AU
#define SX1276_REG_RSSI_VALUE          0x1BU
#define SX1276_REG_MODEM_CONFIG_1      0x1DU
#define SX1276_REG_MODEM_CONFIG_2      0x1EU
#define SX1276_REG_SYMB_TIMEOUT_LSB    0x1FU
#define SX1276_REG_PREAMBLE_MSB        0x20U
#define SX1276_REG_PREAMBLE_LSB        0x21U
#define SX1276_REG_PAYLOAD_LENGTH      0x22U
#define SX1276_REG_MODEM_CONFIG_3      0x26U
#define SX1276_REG_DIO_MAPPING_1       0x40U

/* ── OpMode constants ────────────────────────────────────────────────────── */
#define SX1276_LONG_RANGE_MODE         0x80U
#define SX1276_SLEEP_MODE              0x00U
#define SX1276_STDBY_MODE              0x01U
#define SX1276_TX_MODE                 0x03U
#define SX1276_RX_CONTINUOUS_MODE      0x05U
#define SX1276_CAD_MODE                0x07U  /* Channel Activity Detection     */

/* ── IRQ flag bits (RegIrqFlags) ─────────────────────────────────────────── */
#define SX1276_IRQ_CAD_DETECTED        0x01U  /* preamble detected during CAD  */
#define SX1276_IRQ_CAD_DONE            0x04U  /* CAD scan complete             */
#define SX1276_IRQ_TX_DONE             0x08U
#define SX1276_IRQ_PAYLOAD_CRC_ERROR   0x20U
#define SX1276_IRQ_RX_DONE             0x40U

/* ── Configuration structs ───────────────────────────────────────────────── */
typedef struct {
    uint32_t frequency_hz;
    uint8_t  bandwidth_bits;    /* 4-bit: BW[3:0] in RegModemConfig1      */
    uint8_t  spreading_factor;  /* 4-bit: SF6–SF12                        */
    uint8_t  coding_rate_bits;  /* 3-bit: CR4/5=1 … CR4/8=4              */
    int8_t   tx_power_dbm;
    bool     implicit_header_mode;
    bool     use_pa_boost;      /* true → PA_BOOST (≤17 dBm); false → RFO */
} sx1276_lora_config_t;

typedef struct {
    int16_t  rssi_dbm;
    int8_t   snr_db;
    uint32_t timestamp_ms;
    bool     crc_ok;
    bool     implicit_header_mode;
    uint8_t  irq_flags;
} sx1276_rx_metadata_t;

/* ── Public API ──────────────────────────────────────────────────────────── */
void              sx1276_init(sx1276_t *radio);
void              sx1276_reset(sx1276_t *radio);
uint8_t           sx1276_read_reg(sx1276_t *radio, uint8_t addr);
void              sx1276_write_reg(sx1276_t *radio, uint8_t addr, uint8_t value);
void              sx1276_configure_lora(sx1276_t *radio, const sx1276_lora_config_t *config);

HAL_StatusTypeDef sx1276_send_bytes(sx1276_t *radio, const uint8_t *data,
                                    uint8_t data_length, uint32_t timeout_ms);

HAL_StatusTypeDef sx1276_receive_bytes(sx1276_t *radio, uint8_t *buffer,
                                       uint8_t buffer_capacity,
                                       uint8_t *received_length,
                                       sx1276_rx_metadata_t *metadata);

/*
 * sx1276_cad — Perform one Channel Activity Detection scan.
 *
 * Sets *channel_active = true if a LoRa preamble was detected.
 * At SF7/125kHz the scan takes approximately 2 ms.
 * Returns HAL_TIMEOUT if the CAD_DONE flag does not appear within timeout_ms.
 */
HAL_StatusTypeDef sx1276_cad(sx1276_t *radio, bool *channel_active,
                              uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* SX1276_H */

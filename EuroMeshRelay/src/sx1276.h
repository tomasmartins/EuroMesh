#ifndef SX1276_H
#define SX1276_H

#include <stdbool.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *dio0_port;
    uint16_t dio0_pin;
} sx1276_t;

#define SX1276_REG_FIFO                0x00
#define SX1276_REG_OP_MODE             0x01
#define SX1276_REG_FIFO_ADDR_PTR       0x0D
#define SX1276_REG_FIFO_TX_BASE_ADDR   0x0E
#define SX1276_REG_FIFO_RX_BASE_ADDR   0x0F
#define SX1276_REG_FIFO_RX_CURRENT     0x10
#define SX1276_REG_FRF_MSB             0x06
#define SX1276_REG_FRF_MID             0x07
#define SX1276_REG_FRF_LSB             0x08
#define SX1276_REG_PA_CONFIG           0x09
#define SX1276_REG_MODEM_CONFIG_1      0x1D
#define SX1276_REG_MODEM_CONFIG_2      0x1E
#define SX1276_REG_SYMB_TIMEOUT_LSB    0x1F
#define SX1276_REG_PREAMBLE_MSB        0x20
#define SX1276_REG_PREAMBLE_LSB        0x21
#define SX1276_REG_MODEM_CONFIG_3      0x26
#define SX1276_REG_PAYLOAD_LENGTH      0x22
#define SX1276_REG_RX_NB_BYTES         0x13
#define SX1276_REG_IRQ_FLAGS           0x12
#define SX1276_REG_PKT_SNR_VALUE       0x19
#define SX1276_REG_PKT_RSSI_VALUE      0x1A
#define SX1276_REG_RSSI_VALUE          0x1B
#define SX1276_REG_DIO_MAPPING_1       0x40

#define SX1276_LONG_RANGE_MODE         0x80
#define SX1276_SLEEP_MODE              0x00
#define SX1276_STDBY_MODE              0x01
#define SX1276_TX_MODE                 0x03
#define SX1276_RX_CONTINUOUS_MODE      0x05

#define SX1276_IRQ_TX_DONE             0x08
#define SX1276_IRQ_RX_DONE             0x40
#define SX1276_IRQ_PAYLOAD_CRC_ERROR   0x20

typedef struct {
    uint32_t frequency_hz;
    uint8_t bandwidth_bits;
    uint8_t spreading_factor;
    uint8_t coding_rate_bits;
    int8_t tx_power_dbm;
    bool implicit_header_mode;
} sx1276_lora_config_t;

typedef struct {
    int16_t rssi_dbm;
    int8_t snr_db;
    uint32_t timestamp_ms;
    bool crc_ok;
    bool implicit_header_mode;
    uint8_t irq_flags;
} sx1276_rx_metadata_t;

void sx1276_init(sx1276_t *radio);
void sx1276_reset(sx1276_t *radio);
uint8_t sx1276_read_reg(sx1276_t *radio, uint8_t addr);
void sx1276_write_reg(sx1276_t *radio, uint8_t addr, uint8_t value);
void sx1276_read_burst(sx1276_t *radio, uint8_t addr, uint8_t *buffer, uint8_t length);
void sx1276_write_burst(sx1276_t *radio, uint8_t addr, const uint8_t *buffer, uint8_t length);
void sx1276_set_frequency(sx1276_t *radio, uint32_t frequency_hz);
void sx1276_configure_lora(sx1276_t *radio, const sx1276_lora_config_t *config);
void sx1276_set_irq_mapping(sx1276_t *radio, uint8_t dio0_mapping_bits);
HAL_StatusTypeDef sx1276_send_bytes(sx1276_t *radio, const uint8_t *data, uint8_t data_length, uint32_t timeout_ms);
HAL_StatusTypeDef sx1276_receive_bytes(sx1276_t *radio, uint8_t *buffer, uint8_t buffer_capacity, uint8_t *received_length, sx1276_rx_metadata_t *metadata);

#ifdef __cplusplus
}
#endif

#endif

#ifndef SX1276_H
#define SX1276_H

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

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint8_t ttl;
    uint32_t src_id;
    uint32_t dest_id;
    uint16_t seq;
    uint16_t op;
    uint8_t length;
} sx1276_packet_header_t;

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

#define SX1276_LONG_RANGE_MODE         0x80
#define SX1276_SLEEP_MODE              0x00
#define SX1276_STDBY_MODE              0x01
#define SX1276_TX_MODE                 0x03
#define SX1276_RX_CONTINUOUS_MODE      0x05

#define SX1276_IRQ_TX_DONE             0x08
#define SX1276_IRQ_RX_DONE             0x40

void sx1276_init(sx1276_t *radio);
void sx1276_reset(sx1276_t *radio);
uint8_t sx1276_read_reg(sx1276_t *radio, uint8_t addr);
void sx1276_write_reg(sx1276_t *radio, uint8_t addr, uint8_t value);
void sx1276_read_burst(sx1276_t *radio, uint8_t addr, uint8_t *buffer, uint8_t length);
void sx1276_write_burst(sx1276_t *radio, uint8_t addr, const uint8_t *buffer, uint8_t length);
void sx1276_set_frequency(sx1276_t *radio, uint32_t frequency_hz);
void sx1276_configure_lora(sx1276_t *radio, uint32_t frequency_hz, uint8_t bandwidth_bits, uint8_t spreading_factor);
HAL_StatusTypeDef sx1276_send_packet(sx1276_t *radio, const sx1276_packet_header_t *header, const uint8_t *payload);
HAL_StatusTypeDef sx1276_receive_packet(sx1276_t *radio, sx1276_packet_header_t *header, uint8_t *payload, uint8_t payload_capacity, uint8_t *payload_length);

#ifdef __cplusplus
}
#endif

#endif

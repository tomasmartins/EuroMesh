#include "sx1276.h"

static void sx1276_select(sx1276_t *radio)
{
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
}

static void sx1276_deselect(sx1276_t *radio)
{
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
}

static void sx1276_read_burst(sx1276_t *radio, uint8_t addr, uint8_t *buffer, uint8_t length)
{
    uint8_t tx = addr & 0x7FU;

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(radio->hspi, buffer, length, HAL_MAX_DELAY);
    sx1276_deselect(radio);
}

static void sx1276_write_burst(sx1276_t *radio, uint8_t addr, const uint8_t *buffer, uint8_t length)
{
    uint8_t tx = addr | 0x80U;

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, &tx, 1, HAL_MAX_DELAY);
    /* HAL_SPI_Transmit takes a non-const pointer; the buffer is not modified. */
    HAL_SPI_Transmit(radio->hspi, (uint8_t *)(uintptr_t)buffer, length, HAL_MAX_DELAY);
    sx1276_deselect(radio);
}

static void sx1276_set_frequency(sx1276_t *radio, uint32_t frequency_hz)
{
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000U;

    sx1276_write_reg(radio, SX1276_REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1276_write_reg(radio, SX1276_REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1276_write_reg(radio, SX1276_REG_FRF_LSB, (uint8_t)(frf));
}

static void sx1276_set_irq_mapping(sx1276_t *radio, uint8_t dio0_mapping_bits)
{
    uint8_t value = (uint8_t)((dio0_mapping_bits & 0x03U) << 6);

    sx1276_write_reg(radio, SX1276_REG_DIO_MAPPING_1, value);
}

static uint8_t sx1276_pa_config(bool use_pa_boost, int8_t tx_power_dbm)
{
    if (use_pa_boost) {
        /* PA_BOOST path: Pout = 17 - (15 - OutputPower), range 2–17 dBm. */
        int8_t clamped = tx_power_dbm;
        if (clamped < 2)  { clamped = 2;  }
        if (clamped > 17) { clamped = 17; }
        return (uint8_t)(0x80U | (uint8_t)(clamped - 2));
    } else {
        /* RFO path: MaxPower=7 → Pmax=15 dBm, Pout = OutputPower, range 0–14 dBm. */
        int8_t clamped = tx_power_dbm;
        if (clamped < 0)  { clamped = 0;  }
        if (clamped > 14) { clamped = 14; }
        return (uint8_t)(0x70U | (uint8_t)clamped);
    }
}

void sx1276_init(sx1276_t *radio)
{
    sx1276_reset(radio);
    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_SLEEP_MODE);
    HAL_Delay(5);
    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_STDBY_MODE);
    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, 0xFF);
    sx1276_write_reg(radio, SX1276_REG_PREAMBLE_MSB, 0x00);
    sx1276_write_reg(radio, SX1276_REG_PREAMBLE_LSB, 0x08);
    sx1276_set_irq_mapping(radio, 0x00);
}

void sx1276_reset(sx1276_t *radio)
{
    HAL_GPIO_WritePin(radio->reset_port, radio->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(radio->reset_port, radio->reset_pin, GPIO_PIN_SET);
    HAL_Delay(5);
}

uint8_t sx1276_read_reg(sx1276_t *radio, uint8_t addr)
{
    uint8_t value = 0;
    uint8_t tx = addr & 0x7FU;

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(radio->hspi, &value, 1, HAL_MAX_DELAY);
    sx1276_deselect(radio);

    return value;
}

void sx1276_write_reg(sx1276_t *radio, uint8_t addr, uint8_t value)
{
    uint8_t buffer[2] = { (uint8_t)(addr | 0x80U), value };

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, buffer, sizeof(buffer), HAL_MAX_DELAY);
    sx1276_deselect(radio);
}

void sx1276_configure_lora(sx1276_t *radio, const sx1276_lora_config_t *config)
{
    uint8_t modem_config_1;
    uint8_t modem_config_2;
    uint8_t modem_config_3 = 0x04U; /* AGC on, low data rate optimize off */

    if (radio == NULL || config == NULL) {
        return;
    }

    /* Mask each field to its defined bit-width before shifting. */
    modem_config_1 = (uint8_t)(((config->bandwidth_bits   & 0x0FU) << 4)
                              | ((config->coding_rate_bits & 0x07U) << 1));
    if (config->implicit_header_mode) {
        modem_config_1 |= 0x01U;
    }
    modem_config_2 = (uint8_t)(((config->spreading_factor & 0x0FU) << 4) | 0x04U);

    sx1276_set_frequency(radio, config->frequency_hz);
    sx1276_write_reg(radio, SX1276_REG_MODEM_CONFIG_1, modem_config_1);
    sx1276_write_reg(radio, SX1276_REG_MODEM_CONFIG_2, modem_config_2);
    sx1276_write_reg(radio, SX1276_REG_MODEM_CONFIG_3, modem_config_3);
    sx1276_write_reg(radio, SX1276_REG_SYMB_TIMEOUT_LSB, 0x08);
    sx1276_write_reg(radio, SX1276_REG_PA_CONFIG,
                     sx1276_pa_config(config->use_pa_boost, config->tx_power_dbm));
}

HAL_StatusTypeDef sx1276_send_bytes(sx1276_t *radio, const uint8_t *data, uint8_t data_length, uint32_t timeout_ms)
{
    uint32_t start;

    if (radio == NULL || data == NULL || data_length == 0U) {
        return HAL_ERROR;
    }

    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_STDBY_MODE);
    sx1276_write_reg(radio, SX1276_REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1276_write_reg(radio, SX1276_REG_FIFO_ADDR_PTR, 0x00);
    sx1276_write_burst(radio, SX1276_REG_FIFO, data, data_length);
    sx1276_write_reg(radio, SX1276_REG_PAYLOAD_LENGTH, data_length);
    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, SX1276_IRQ_TX_DONE);
    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_TX_MODE);

    start = HAL_GetTick();
    while ((sx1276_read_reg(radio, SX1276_REG_IRQ_FLAGS) & SX1276_IRQ_TX_DONE) == 0U) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, SX1276_IRQ_TX_DONE);
    return HAL_OK;
}

HAL_StatusTypeDef sx1276_receive_bytes(sx1276_t *radio, uint8_t *buffer, uint8_t buffer_capacity,
                                       uint8_t *received_length, sx1276_rx_metadata_t *metadata)
{
    uint8_t irq_flags;
    uint8_t available;

    if (radio == NULL || buffer == NULL || received_length == NULL) {
        return HAL_ERROR;
    }

    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_RX_CONTINUOUS_MODE);
    irq_flags = sx1276_read_reg(radio, SX1276_REG_IRQ_FLAGS);
    if ((irq_flags & SX1276_IRQ_RX_DONE) == 0U) {
        return HAL_BUSY;
    }

    sx1276_write_reg(radio, SX1276_REG_FIFO_ADDR_PTR,
                     sx1276_read_reg(radio, SX1276_REG_FIFO_RX_CURRENT));
    available = sx1276_read_reg(radio, SX1276_REG_RX_NB_BYTES);
    if (available > buffer_capacity) {
        sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, irq_flags);
        return HAL_ERROR;
    }

    if (available > 0U) {
        sx1276_read_burst(radio, SX1276_REG_FIFO, buffer, available);
    }
    *received_length = available;

    if (metadata != NULL) {
        int8_t  raw_snr      = (int8_t)sx1276_read_reg(radio, SX1276_REG_PKT_SNR_VALUE);
        int16_t packet_rssi  = (int16_t)sx1276_read_reg(radio, SX1276_REG_PKT_RSSI_VALUE);
        bool implicit_header = (sx1276_read_reg(radio, SX1276_REG_MODEM_CONFIG_1) & 0x01U) != 0U;
        /* Correction per datasheet: -157 dBm with LNA boost active, -164 dBm without. */
        bool lna_boost = (sx1276_read_reg(radio, SX1276_REG_LNA) & 0x03U) != 0U;

        metadata->snr_db              = (int8_t)((int16_t)raw_snr / 4);
        metadata->rssi_dbm            = (int16_t)(packet_rssi - (lna_boost ? 157 : 164));
        metadata->timestamp_ms        = HAL_GetTick();
        metadata->crc_ok              = (irq_flags & SX1276_IRQ_PAYLOAD_CRC_ERROR) == 0U;
        metadata->implicit_header_mode = implicit_header;
        metadata->irq_flags           = irq_flags;
    }

    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, irq_flags);
    return HAL_OK;
}

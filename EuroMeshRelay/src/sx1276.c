#include "sx1276.h"

static void sx1276_select(sx1276_t *radio)
{
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_RESET);
}

static void sx1276_deselect(sx1276_t *radio)
{
    HAL_GPIO_WritePin(radio->nss_port, radio->nss_pin, GPIO_PIN_SET);
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
    sx1276_write_reg(radio, SX1276_REG_PA_CONFIG, 0x8F);
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
    uint8_t tx = addr & 0x7F;

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(radio->hspi, &value, 1, HAL_MAX_DELAY);
    sx1276_deselect(radio);

    return value;
}

void sx1276_write_reg(sx1276_t *radio, uint8_t addr, uint8_t value)
{
    uint8_t buffer[2] = { (uint8_t)(addr | 0x80), value };

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, buffer, sizeof(buffer), HAL_MAX_DELAY);
    sx1276_deselect(radio);
}

void sx1276_read_burst(sx1276_t *radio, uint8_t addr, uint8_t *buffer, uint8_t length)
{
    uint8_t tx = addr & 0x7F;

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(radio->hspi, buffer, length, HAL_MAX_DELAY);
    sx1276_deselect(radio);
}

void sx1276_write_burst(sx1276_t *radio, uint8_t addr, const uint8_t *buffer, uint8_t length)
{
    uint8_t tx = addr | 0x80;

    sx1276_select(radio);
    HAL_SPI_Transmit(radio->hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(radio->hspi, (uint8_t *)buffer, length, HAL_MAX_DELAY);
    sx1276_deselect(radio);
}

void sx1276_set_frequency(sx1276_t *radio, uint32_t frequency_hz)
{
    uint64_t frf = ((uint64_t)frequency_hz << 19) / 32000000U;

    sx1276_write_reg(radio, SX1276_REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1276_write_reg(radio, SX1276_REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1276_write_reg(radio, SX1276_REG_FRF_LSB, (uint8_t)(frf));
}

void sx1276_configure_lora(sx1276_t *radio, uint32_t frequency_hz, uint8_t bandwidth_bits, uint8_t spreading_factor)
{
    uint8_t modem_config_1 = (uint8_t)((bandwidth_bits << 4) | (0x01 << 1));
    uint8_t modem_config_2 = (uint8_t)((spreading_factor << 4) | 0x04);
    uint8_t modem_config_3 = 0x04;

    sx1276_set_frequency(radio, frequency_hz);
    sx1276_write_reg(radio, SX1276_REG_MODEM_CONFIG_1, modem_config_1);
    sx1276_write_reg(radio, SX1276_REG_MODEM_CONFIG_2, modem_config_2);
    sx1276_write_reg(radio, SX1276_REG_MODEM_CONFIG_3, modem_config_3);
    sx1276_write_reg(radio, SX1276_REG_SYMB_TIMEOUT_LSB, 0x08);
}

HAL_StatusTypeDef sx1276_send_packet(sx1276_t *radio, const sx1276_packet_header_t *header, const uint8_t *payload, uint8_t payload_length)
{
    uint8_t header_buffer[SX1276_PACKET_HEADER_SIZE] = {0};
    uint8_t total_length = payload_length + (uint8_t)sizeof(header_buffer);

    header_buffer[0] = header->type;
    header_buffer[1] = header->flags;
    header_buffer[2] = header->ttl;
    header_buffer[3] = (uint8_t)(header->src_id >> 24);
    header_buffer[4] = (uint8_t)(header->src_id >> 16);
    header_buffer[5] = (uint8_t)(header->src_id >> 8);
    header_buffer[6] = (uint8_t)(header->src_id);
    header_buffer[7] = (uint8_t)(header->dest_id >> 24);
    header_buffer[8] = (uint8_t)(header->dest_id >> 16);
    header_buffer[9] = (uint8_t)(header->dest_id >> 8);
    header_buffer[10] = (uint8_t)(header->dest_id);
    header_buffer[11] = (uint8_t)(header->seq >> 8);
    header_buffer[12] = (uint8_t)(header->seq);

    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_STDBY_MODE);
    sx1276_write_reg(radio, SX1276_REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1276_write_reg(radio, SX1276_REG_FIFO_ADDR_PTR, 0x00);
    sx1276_write_burst(radio, SX1276_REG_FIFO, header_buffer, sizeof(header_buffer));
    if (payload_length > 0U) {
        sx1276_write_burst(radio, SX1276_REG_FIFO, payload, payload_length);
    }
    sx1276_write_reg(radio, SX1276_REG_PAYLOAD_LENGTH, total_length);
    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, SX1276_IRQ_TX_DONE);
    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_TX_MODE);

    uint32_t start = HAL_GetTick();
    while ((sx1276_read_reg(radio, SX1276_REG_IRQ_FLAGS) & SX1276_IRQ_TX_DONE) == 0U) {
        if ((HAL_GetTick() - start) > 2000U) {
            return HAL_TIMEOUT;
        }
    }
    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, SX1276_IRQ_TX_DONE);
    return HAL_OK;
}

HAL_StatusTypeDef sx1276_receive_packet(sx1276_t *radio, sx1276_packet_header_t *header, uint8_t *payload, uint8_t payload_capacity, uint8_t *payload_length)
{
    uint8_t header_buffer[SX1276_PACKET_HEADER_SIZE] = {0};
    uint8_t available = 0;
    uint8_t payload_size = 0;

    sx1276_write_reg(radio, SX1276_REG_OP_MODE, SX1276_LONG_RANGE_MODE | SX1276_RX_CONTINUOUS_MODE);
    if ((sx1276_read_reg(radio, SX1276_REG_IRQ_FLAGS) & SX1276_IRQ_RX_DONE) == 0U) {
        return HAL_BUSY;
    }

    sx1276_write_reg(radio, SX1276_REG_IRQ_FLAGS, SX1276_IRQ_RX_DONE);
    sx1276_write_reg(radio, SX1276_REG_FIFO_ADDR_PTR, sx1276_read_reg(radio, SX1276_REG_FIFO_RX_CURRENT));
    available = sx1276_read_reg(radio, SX1276_REG_RX_NB_BYTES);
    if (available < sizeof(header_buffer)) {
        return HAL_ERROR;
    }

    sx1276_read_burst(radio, SX1276_REG_FIFO, header_buffer, sizeof(header_buffer));
    header->type = header_buffer[0];
    header->flags = header_buffer[1];
    header->ttl = header_buffer[2];
    header->src_id = ((uint32_t)header_buffer[3] << 24)
        | ((uint32_t)header_buffer[4] << 16)
        | ((uint32_t)header_buffer[5] << 8)
        | ((uint32_t)header_buffer[6]);
    header->dest_id = ((uint32_t)header_buffer[7] << 24)
        | ((uint32_t)header_buffer[8] << 16)
        | ((uint32_t)header_buffer[9] << 8)
        | ((uint32_t)header_buffer[10]);
    header->seq = (uint16_t)((header_buffer[11] << 8) | header_buffer[12]);
    payload_size = (uint8_t)(available - (uint8_t)sizeof(header_buffer));

    if (payload_size > payload_capacity) {
        return HAL_ERROR;
    }

    if (payload_size > 0U) {
        sx1276_read_burst(radio, SX1276_REG_FIFO, payload, payload_size);
    }
    if (payload_length != NULL) {
        *payload_length = payload_size;
    }
    return HAL_OK;
}

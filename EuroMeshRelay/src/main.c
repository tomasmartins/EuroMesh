#include "stm32f4xx_hal.h"
#include "sx1276.h"
#include "time_sync.h"

#define SX1276_NSS_PORT GPIOB
#define SX1276_NSS_PIN  GPIO_PIN_6
#define SX1276_RST_PORT GPIOB
#define SX1276_RST_PIN  GPIO_PIN_7
#define SX1276_DIO0_PORT GPIOB
#define SX1276_DIO0_PIN  GPIO_PIN_8

static SPI_HandleTypeDef hspi1;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

#define PACKET_TYPE_BEACON 0x01
#define PACKET_TYPE_ACK    0x02

#define TIME_SYNC_FLAG_UTC_VALID     0x01
#define TIME_SYNC_FLAG_PPS_VALID     0x02
#define TIME_SYNC_FLAG_RX_TICK_VALID 0x04

static uint32_t read_le_u32(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0])
        | ((uint32_t)buffer[1] << 8)
        | ((uint32_t)buffer[2] << 16)
        | ((uint32_t)buffer[3] << 24);
}

static uint64_t read_le_u64(const uint8_t *buffer)
{
    return ((uint64_t)buffer[0])
        | ((uint64_t)buffer[1] << 8)
        | ((uint64_t)buffer[2] << 16)
        | ((uint64_t)buffer[3] << 24)
        | ((uint64_t)buffer[4] << 32)
        | ((uint64_t)buffer[5] << 40)
        | ((uint64_t)buffer[6] << 48)
        | ((uint64_t)buffer[7] << 56);
}

static void handle_time_sync_payload(time_sync_t *sync,
                                     const uint8_t *payload,
                                     uint8_t length,
                                     bool sync_requested)
{
    const uint8_t minimum_length = 1 + 8 + 4;
    uint32_t local_now_ms = HAL_GetTick();
    uint8_t flags = 0;
    uint64_t utc_epoch_ms = 0;
    uint32_t pps_tick_ms = 0;
    uint32_t rx_tick_ms = 0;
    bool rx_tick_valid = false;
    int64_t local_offset_ms = 0;

    if (length < minimum_length) {
        return;
    }

    flags = payload[0];
    if ((flags & TIME_SYNC_FLAG_UTC_VALID) == 0U || (flags & TIME_SYNC_FLAG_PPS_VALID) == 0U) {
        return;
    }

    utc_epoch_ms = read_le_u64(&payload[1]);
    pps_tick_ms = read_le_u32(&payload[9]);

    if (!time_sync_should_accept_sample(sync, sync_requested, local_now_ms, utc_epoch_ms)) {
        return;
    }

    if (length >= (minimum_length + 4)) {
        rx_tick_ms = read_le_u32(&payload[13]);
        rx_tick_valid = (flags & TIME_SYNC_FLAG_RX_TICK_VALID) != 0U;
    }

    if (rx_tick_valid) {
        local_offset_ms = (int64_t)utc_epoch_ms - (int64_t)rx_tick_ms;
    } else {
        local_offset_ms = (int64_t)utc_epoch_ms - (int64_t)local_now_ms;
    }

    time_sync_handle_ntp_sample(sync, utc_epoch_ms, pps_tick_ms, local_offset_ms, local_now_ms);
}

static void handle_received_packet(time_sync_t *sync,
                                   const sx1276_packet_header_t *header,
                                   const uint8_t *payload,
                                   uint8_t payload_length)
{
    bool sync_requested = (header->flags & SX1276_PACKET_FLAG_TIME_SYNC_REQUEST) != 0U;

    if (header->type == PACKET_TYPE_BEACON || header->type == PACKET_TYPE_ACK) {
        handle_time_sync_payload(sync, payload, payload_length, sync_requested);
    }
}

int main(void)
{
    sx1276_t radio = {
        .hspi = &hspi1,
        .nss_port = SX1276_NSS_PORT,
        .nss_pin = SX1276_NSS_PIN,
        .reset_port = SX1276_RST_PORT,
        .reset_pin = SX1276_RST_PIN,
        .dio0_port = SX1276_DIO0_PORT,
        .dio0_pin = SX1276_DIO0_PIN,
    };

    time_sync_t time_sync;
    uint8_t payload[64] = {0};
    sx1276_packet_header_t header = {0};

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    sx1276_init(&radio);
    sx1276_configure_lora(&radio, 869525000U, 0x07, 0x07);
    time_sync_init(&time_sync, 300000U, 5000U);

    while (1) {
        uint8_t payload_length = 0;

        if (sx1276_receive_packet(&radio, &header, payload, sizeof(payload), &payload_length) == HAL_OK) {
            handle_received_packet(&time_sync, &header, payload, payload_length);
        }

        HAL_Delay(10);
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc_init.HSIState = RCC_HSI_ON;
    osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc_init.PLL.PLLM = 16;
    osc_init.PLL.PLLN = 336;
    osc_init.PLL.PLLP = RCC_PLLP_DIV4;
    osc_init.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        while (1) {
        }
    }

    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2) != HAL_OK) {
        while (1) {
        }
    }
}

static void MX_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        while (1) {
        }
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_init.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    gpio_init.Pin = SX1276_NSS_PIN | SX1276_RST_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    gpio_init.Pin = SX1276_DIO0_PIN;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    HAL_GPIO_WritePin(SX1276_NSS_PORT, SX1276_NSS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SX1276_RST_PORT, SX1276_RST_PIN, GPIO_PIN_SET);
}

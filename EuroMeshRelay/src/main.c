#include "stm32f4xx_hal.h"
#include "csma_mac.h"
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
#define PACKET_TYPE_SUBSCRIPTION 0x03

#define TDMA_SLOT_LENGTH_MS          200U
#define TDMA_BEACON_SLOT_INDEX       0U
#define TDMA_SUBSCRIPTION_SLOT_INDEX 1U
#define TDMA_FRAME_SLOT_COUNT        2U
#define TDMA_FRAME_LENGTH_MS (TDMA_SLOT_LENGTH_MS * TDMA_FRAME_SLOT_COUNT)

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

static void handle_received_packet(time_sync_t *sync,
                                   const sx1276_packet_header_t *header,
                                   const uint8_t *payload,
                                   uint8_t payload_length);

static uint32_t tdma_frame_start_ms(uint32_t now_ms)
{
    uint32_t frames_elapsed = now_ms / TDMA_FRAME_LENGTH_MS;

    return (frames_elapsed + 1U) * TDMA_FRAME_LENGTH_MS;
}

static void wait_until_ms(uint32_t target_ms)
{
    while (HAL_GetTick() < target_ms) {
        HAL_Delay(1);
    }
}

static uint8_t build_beacon_payload(uint8_t *payload, uint8_t capacity)
{
    if (capacity < 4U) {
        return 0;
    }
    uint32_t now_ms = HAL_GetTick();
    payload[0] = (uint8_t)now_ms;
    payload[1] = (uint8_t)(now_ms >> 8);
    payload[2] = (uint8_t)(now_ms >> 16);
    payload[3] = (uint8_t)(now_ms >> 24);
    return 4U;
}

static uint8_t build_subscription_payload(uint8_t *payload, uint8_t capacity)
{
    if (capacity < 1U) {
        return 0;
    }
    payload[0] = 0x01;
    return 1U;
}

static void open_subscription_window(time_sync_t *sync, sx1276_t *radio, uint32_t window_ms)
{
    uint32_t start_ms = HAL_GetTick();
    sx1276_packet_header_t rx_header = {0};
    uint8_t rx_payload[64] = {0};
    uint8_t rx_payload_length = 0;

    while ((HAL_GetTick() - start_ms) < window_ms) {
        if (sx1276_receive_packet(radio, &rx_header, rx_payload, sizeof(rx_payload), &rx_payload_length) == HAL_OK) {
            handle_received_packet(sync, &rx_header, rx_payload, rx_payload_length);
        } else {
            HAL_Delay(1);
        }
    }
}

static void handle_time_sync_payload(time_sync_t *sync, const uint8_t *payload, uint8_t length)
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
    if (header->type == PACKET_TYPE_BEACON || header->type == PACKET_TYPE_ACK) {
        handle_time_sync_payload(sync, payload, payload_length);
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

    csma_mac_t mac = {0};
    time_sync_t time_sync;
    uint8_t payload[64] = {0};
    sx1276_packet_header_t header = {0};

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    sx1276_init(&radio);
    sx1276_configure_lora(&radio, 869525000U, 0x07, 0x07);
    time_sync_init(&time_sync);
    csma_mac_init(&mac, &radio, 150U, 5U, 20U, 3U);

    while (1) {
        uint8_t payload_length = 0;
        uint32_t frame_start_ms = tdma_frame_start_ms(HAL_GetTick());
        uint32_t subscription_slot_start_ms = frame_start_ms + (TDMA_SUBSCRIPTION_SLOT_INDEX * TDMA_SLOT_LENGTH_MS);

        payload_length = build_beacon_payload(payload, sizeof(payload));
        header.type = PACKET_TYPE_BEACON;
        header.seq++;
        (void)csma_mac_send_tdma(&mac, &header, payload, payload_length, frame_start_ms, TDMA_SLOT_LENGTH_MS, TDMA_BEACON_SLOT_INDEX);

        payload_length = build_subscription_payload(payload, sizeof(payload));
        header.type = PACKET_TYPE_SUBSCRIPTION;
        header.seq++;
        if (payload_length > 0U) {
            (void)csma_mac_send_tdma(&mac, &header, payload, payload_length, frame_start_ms, TDMA_SLOT_LENGTH_MS, TDMA_SUBSCRIPTION_SLOT_INDEX);
        } else {
            wait_until_ms(subscription_slot_start_ms);
            open_subscription_window(&time_sync, &radio, TDMA_SLOT_LENGTH_MS);
        }

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

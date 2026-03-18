#include "stm32f4xx_hal.h"
#include "csma_mac.h"
#include "sx1276.h"
#include "emesh_frame.h"
#include "tdma.h"
#include "time_sync.h"
#include "nodes.h"
#include "emesh_packet_types.h"

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
static void Error_Handler(void);

static void handle_received_packet(time_sync_t *sync,
                                   const emesh_frame_header_t *header,
                                   const uint8_t *payload,
                                   uint8_t payload_length);

/* Wait until target_ms, handling HAL_GetTick() wrap-around via signed comparison. */
static void wait_until_ms(uint32_t target_ms)
{
    while ((int32_t)(target_ms - HAL_GetTick()) > 0) {
        HAL_Delay(1);
    }
}

/*
 * Build a beacon payload carrying time sync data when UTC is valid.
 *
 * Wire format (starting at payload[0]):
 *   [type:1][flags:1][utc_epoch_ms:8][pps_tick_ms:4]  = 14 bytes total
 *
 * Receivers pass payload[1..] to time_sync_handle_payload, which expects:
 *   [flags:1][utc_epoch_ms:8][pps_tick_ms:4]
 */
static uint8_t build_beacon_payload(const time_sync_t *sync, uint8_t *payload, uint8_t capacity)
{
    const uint8_t BEACON_SIZE = 14U;

    if (payload == NULL || capacity < BEACON_SIZE) {
        return 0;
    }

    payload[0] = EMESH_PACKET_TYPE_BEACON;

    if (sync != NULL && sync->utc_valid && sync->pps_valid) {
        uint32_t now_tick = HAL_GetTick();
        uint32_t elapsed  = now_tick - sync->last_update_tick_ms;
        uint64_t utc_ms   = sync->last_utc_epoch_ms + (uint64_t)elapsed;
        uint32_t pps      = sync->last_pps_tick_ms;

        payload[1]  = TIME_SYNC_FLAG_UTC_VALID | TIME_SYNC_FLAG_PPS_VALID;
        payload[2]  = (uint8_t)(utc_ms);
        payload[3]  = (uint8_t)(utc_ms >> 8);
        payload[4]  = (uint8_t)(utc_ms >> 16);
        payload[5]  = (uint8_t)(utc_ms >> 24);
        payload[6]  = (uint8_t)(utc_ms >> 32);
        payload[7]  = (uint8_t)(utc_ms >> 40);
        payload[8]  = (uint8_t)(utc_ms >> 48);
        payload[9]  = (uint8_t)(utc_ms >> 56);
        payload[10] = (uint8_t)(pps);
        payload[11] = (uint8_t)(pps >> 8);
        payload[12] = (uint8_t)(pps >> 16);
        payload[13] = (uint8_t)(pps >> 24);
    } else {
        /* No valid time — broadcast presence only; flags=0 causes receivers to skip sync. */
        payload[1] = 0x00U;
        for (uint8_t i = 2U; i < BEACON_SIZE; ++i) {
            payload[i] = 0x00U;
        }
    }

    return BEACON_SIZE;
}

static void handle_received_packet(time_sync_t *sync,
                                   const emesh_frame_header_t *header,
                                   const uint8_t *payload,
                                   uint8_t payload_length)
{
    uint8_t packet_type;
    bool sync_requested;

    if (payload_length == 0U) {
        return;
    }
    packet_type    = payload[0];
    sync_requested = (header->flags & EMESH_FRAME_FLAG_TIME_SYNC_REQUEST) != 0U;

    if (header->type != packet_type) {
        return;
    }

    if (packet_type == EMESH_PACKET_TYPE_BEACON) {
        /*
         * Beacons carry time sync data in the format expected by time_sync_handle_payload.
         * ACK packets use a different wire encoding and are not routed here.
         */
        time_sync_handle_payload(sync, &payload[1], payload_length - 1U, sync_requested);
    } else if (packet_type == EMESH_PACKET_TYPE_SUBSCRIPTION) {
        nodes_handle_subscription_packet(header, payload, payload_length);
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
    emesh_frame_header_t header = {0};

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    sx1276_init(&radio);
    {
        sx1276_lora_config_t radio_config = {
            .frequency_hz      = 869525000U,
            .bandwidth_bits    = 0x07U,
            .spreading_factor  = 0x07U,
            .coding_rate_bits  = 0x01U,
            .tx_power_dbm      = 15,
            .implicit_header_mode = false,
            .use_pa_boost      = true,
        };
        sx1276_configure_lora(&radio, &radio_config);
    }

    time_sync_init(&time_sync, TIME_SYNC_TIMEOUT_MS, TIME_SYNC_MAX_SKEW_MS);
    csma_mac_init(&mac, &radio, 150U, 5U, 20U, 3U);

    while (1) {
        uint8_t payload_length;
        uint32_t frame_start_ms          = tdma_next_frame_start_ms(HAL_GetTick());
        uint32_t subscription_slot_start = frame_start_ms
                                         + TDMA_SUBSCRIPTION_SLOT_INDEX * TDMA_SLOT_LENGTH_MS;

        /* Slot 0: transmit beacon with current time sync state. */
        payload_length = build_beacon_payload(&time_sync, payload, sizeof(payload));
        header.type = EMESH_PACKET_TYPE_BEACON;
        header.seq++;
        (void)csma_mac_send_tdma(&mac, &header, payload, payload_length,
                                 frame_start_ms, TDMA_SLOT_LENGTH_MS, TDMA_BEACON_SLOT_INDEX);

        /* Slot 1: optionally send a subscription request, then listen for incoming ones. */
        payload_length = nodes_build_subscription_payload(payload, sizeof(payload));
        header.type = EMESH_PACKET_TYPE_SUBSCRIPTION;
        header.seq++;
        if (payload_length > 0U) {
            (void)csma_mac_send_tdma(&mac, &header, payload, payload_length,
                                     frame_start_ms, TDMA_SLOT_LENGTH_MS, TDMA_SUBSCRIPTION_SLOT_INDEX);
        } else {
            wait_until_ms(subscription_slot_start);
        }

        /* Open a subscription window for the remainder of the subscription slot.
         * Unsigned subtraction (now - slot_start) correctly handles tick wrap-around. */
        {
            uint32_t elapsed   = HAL_GetTick() - subscription_slot_start;
            uint32_t remaining = (elapsed < TDMA_SLOT_LENGTH_MS)
                                 ? (TDMA_SLOT_LENGTH_MS - elapsed) : 0U;
            if (remaining > 0U) {
                nodes_open_subscription_window(&time_sync, &radio, remaining,
                                               handle_received_packet);
            }
        }

        /* Poll for any frame received outside of the scheduled TDMA slots. */
        {
            uint8_t rx_frame[96] = {0};
            uint8_t rx_frame_length = 0;
            sx1276_rx_metadata_t rx_metadata = {0};

            if (sx1276_receive_bytes(&radio, rx_frame, sizeof(rx_frame),
                                     &rx_frame_length, &rx_metadata) == HAL_OK
                && rx_metadata.crc_ok
                && rx_frame_length >= EMESH_FRAME_HEADER_SIZE) {
                emesh_frame_decode_header(rx_frame, &header);
                payload_length = (uint8_t)(rx_frame_length - EMESH_FRAME_HEADER_SIZE);
                if (payload_length > (uint8_t)sizeof(payload)) {
                    payload_length = (uint8_t)sizeof(payload);
                }
                for (uint8_t i = 0; i < payload_length; ++i) {
                    payload[i] = rx_frame[EMESH_FRAME_HEADER_SIZE + i];
                }
                handle_received_packet(&time_sync, &header, payload, payload_length);
            }
        }

        HAL_Delay(10);
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1) {
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
        Error_Handler();
    }

    clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
    clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
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
        Error_Handler();
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

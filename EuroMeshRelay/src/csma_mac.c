#include "csma_mac.h"

/* ── PRNG (Numerical Recipes LCG) ────────────────────────────────────────── */

static uint32_t csma_prng_next(csma_mac_t *mac)
{
    mac->prng_state = mac->prng_state * 1664525U + 1013904223U;
    return mac->prng_state;
}

static uint32_t csma_next_backoff_ms(csma_mac_t *mac)
{
    uint32_t min_ms = mac->backoff_min_ms;
    uint32_t max_ms = mac->backoff_max_ms;
    uint32_t span;

    if (max_ms < min_ms) {
        max_ms = min_ms;
    }
    span = max_ms - min_ms + 1U;
    return min_ms + (csma_prng_next(mac) % span);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void csma_mac_init(csma_mac_t *mac, sx1276_t *radio,
                   uint32_t backoff_min_ms, uint32_t backoff_max_ms,
                   uint8_t max_attempts)
{
    if (mac == NULL) {
        return;
    }
    mac->radio          = radio;
    mac->backoff_min_ms = backoff_min_ms;
    mac->backoff_max_ms = backoff_max_ms;
    mac->max_attempts   = max_attempts;
    /* Seed PRNG from current tick XOR stack address for per-boot variation. */
    mac->prng_state     = HAL_GetTick() ^ (uint32_t)(uintptr_t)mac;
}

bool csma_mac_cad_clear(csma_mac_t *mac)
{
    bool channel_active = true;

    if (mac == NULL || mac->radio == NULL) {
        return false;
    }
    if (sx1276_cad(mac->radio, &channel_active, 10U) != HAL_OK) {
        /* CAD timed out — assume busy to be safe */
        return false;
    }
    return !channel_active;
}

HAL_StatusTypeDef csma_mac_send(csma_mac_t *mac,
                                const emesh_frame_header_t *header,
                                const uint8_t *payload,
                                uint8_t payload_length)
{
    uint8_t  frame[EMESH_FRAME_FIFO_SIZE];
    uint8_t  frame_length;
    uint8_t  attempt = 0;

    if (mac == NULL || mac->radio == NULL || header == NULL
            || payload == NULL || payload_length == 0U) {
        return HAL_ERROR;
    }
    if ((uint16_t)payload_length + EMESH_FRAME_HEADER_SIZE > EMESH_FRAME_FIFO_SIZE) {
        return HAL_ERROR;
    }

    emesh_frame_encode_header(header, frame);
    for (uint8_t i = 0U; i < payload_length; ++i) {
        frame[EMESH_FRAME_HEADER_SIZE + i] = payload[i];
    }
    frame_length = (uint8_t)(EMESH_FRAME_HEADER_SIZE + payload_length);

    while (attempt < mac->max_attempts) {
        if (csma_mac_cad_clear(mac)) {
            return sx1276_send_bytes(mac->radio, frame, frame_length, 2000U);
        }
        HAL_Delay(csma_next_backoff_ms(mac));
        attempt++;
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef csma_mac_send_at(csma_mac_t *mac,
                                   const emesh_frame_header_t *header,
                                   const uint8_t *payload,
                                   uint8_t payload_length,
                                   uint32_t slot_start_ms)
{
    if (mac == NULL || mac->radio == NULL) {
        return HAL_ERROR;
    }

    /* Signed comparison handles HAL_GetTick() wrap-around safely. */
    if ((int32_t)(slot_start_ms - HAL_GetTick()) < 0) {
        return HAL_TIMEOUT;   /* slot has already passed */
    }
    while ((int32_t)(slot_start_ms - HAL_GetTick()) > 0) {
        HAL_Delay(1);
    }

    return csma_mac_send(mac, header, payload, payload_length);
}

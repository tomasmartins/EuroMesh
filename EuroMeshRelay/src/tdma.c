#include "tdma.h"

uint32_t tdma_next_frame_start_ms(uint32_t now_ms)
{
    uint32_t frames_elapsed = now_ms / TDMA_SUPERFRAME_PERIOD_MS;
    return (frames_elapsed + 1U) * TDMA_SUPERFRAME_PERIOD_MS;
}

uint32_t tdma_slot_start_ms(uint32_t frame_start_ms, uint8_t slot_index)
{
    return frame_start_ms + (uint32_t)slot_index * TDMA_SLOT_MS;
}

uint32_t tdma_next_slot_ms(uint32_t now_ms, uint8_t slot_index)
{
    uint32_t frame_start = tdma_next_frame_start_ms(now_ms);
    return tdma_slot_start_ms(frame_start, slot_index);
}

uint32_t tdma_reg_window_start_ms(uint32_t frame_start_ms)
{
    return tdma_slot_start_ms(frame_start_ms, TDMA_REG_SLOT_INDEX);
}

uint32_t tdma_reg_window_end_ms(uint32_t frame_start_ms)
{
    return tdma_reg_window_start_ms(frame_start_ms) + TDMA_REG_SLOT_MS;
}

bool tdma_is_in_reg_window(uint32_t frame_start_ms, uint32_t now_ms)
{
    uint32_t start = tdma_reg_window_start_ms(frame_start_ms);
    /* Unsigned subtraction correctly handles HAL_GetTick() wrap-around. */
    return (now_ms - start) < TDMA_REG_SLOT_MS;
}

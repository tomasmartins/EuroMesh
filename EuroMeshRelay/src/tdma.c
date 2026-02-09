#include "tdma.h"

uint32_t tdma_frame_start_ms(uint32_t now_ms)
{
    uint32_t frames_elapsed = now_ms / TDMA_FRAME_LENGTH_MS;

    return (frames_elapsed + 1U) * TDMA_FRAME_LENGTH_MS;
}

#ifndef TDMA_H
#define TDMA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TDMA_SLOT_LENGTH_MS          200U
#define TDMA_BEACON_SLOT_INDEX       0U
#define TDMA_SUBSCRIPTION_SLOT_INDEX 1U
#define TDMA_FRAME_SLOT_COUNT        2U
#define TDMA_FRAME_LENGTH_MS (TDMA_SLOT_LENGTH_MS * TDMA_FRAME_SLOT_COUNT)

uint32_t tdma_frame_start_ms(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif

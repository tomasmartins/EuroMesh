/*
 * emesh_telemetry.h — Node sensor telemetry payload format.
 *
 * Wire layout (5 bytes, all multi-byte fields little-endian):
 *
 *  Byte   Field      Size  Description
 *  ────   ────────   ────  ────────────────────────────────────────────────
 *   0     flags        1   EMESH_TELEM_FLAG_* bitmask
 *   1–2   batt_mv      2   Battery voltage in millivolts (uint16, LE)
 *   3–4   temp_cdeg    2   Temperature in centidegrees °C (int16, LE)
 *  ────────────────────────────────────────────────────────────────────────
 *  Total: 5 bytes.
 *
 * Op code: EMESH_OP_TELEMETRY_NODE = EMESH_OP(EMESH_OP_CLASS_TELEMETRY, 0x01)
 *          Gateway detects: (hdr.op >> 8) == EMESH_OP_CLASS_TELEMETRY
 */

#ifndef EMESH_TELEMETRY_H
#define EMESH_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#include "emesh_packet_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EMESH_TELEM_PAYLOAD_SIZE  5U

#define EMESH_TELEM_FLAG_BATT     0x01U  /* batt_mv field is valid       */
#define EMESH_TELEM_FLAG_CHARGING 0x02U  /* node is currently charging   */
#define EMESH_TELEM_FLAG_TEMP     0x04U  /* temp_cdeg field is valid     */

#define EMESH_OP_TELEMETRY_NODE   EMESH_OP(EMESH_OP_CLASS_TELEMETRY, 0x01U)

typedef struct {
    uint8_t  flags;
    uint16_t batt_mv;    /* millivolts, 0 if EMESH_TELEM_FLAG_BATT not set  */
    int16_t  temp_cdeg;  /* centidegrees °C, 0 if EMESH_TELEM_FLAG_TEMP not set */
} emesh_telemetry_t;

static inline void emesh_telemetry_encode(const emesh_telemetry_t *t,
                                           uint8_t *buf)
{
    buf[0] = t->flags;
    buf[1] = (uint8_t)(t->batt_mv);
    buf[2] = (uint8_t)(t->batt_mv >> 8);
    buf[3] = (uint8_t)((uint16_t)t->temp_cdeg);
    buf[4] = (uint8_t)((uint16_t)t->temp_cdeg >> 8);
}

static inline bool emesh_telemetry_decode(const uint8_t *buf, uint8_t len,
                                           emesh_telemetry_t *t)
{
    if (len < EMESH_TELEM_PAYLOAD_SIZE) return false;
    t->flags    = buf[0];
    t->batt_mv  = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    t->temp_cdeg = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* EMESH_TELEMETRY_H */

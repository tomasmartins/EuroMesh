/*
 * time_twoway.h — Two-way (NTP-style) time synchronisation codec.
 *
 * Allows end nodes to measure their clock offset and round-trip delay
 * against a GPS-disciplined relay or gateway.
 *
 *   Node                          Relay / Gateway
 *   ────                          ───────────────
 *   T1 = node's local UTC
 *   TX TIME_REQ ─────────────────────────────► T2 = relay GPS UTC at RX
 *                                              T3 = relay GPS UTC at TX
 *   RX TIME_RESP ◄──────────────────────────
 *   T4 = node's local UTC at RX
 *
 *   Offset = ((T2 − T1) + (T3 − T4)) / 2
 *   Delay  = ((T4 − T1) − (T3 − T2)) / 2
 *
 * TIME_REQ payload (3 bytes):
 *   [type:1][req_seq:2LE]
 *
 * TIME_RESP payload (27 bytes):
 *   [type:1][req_seq:2LE][T1_ms:8LE][T2_ms:8LE][T3_ms:8LE]
 */

#ifndef TIME_TWOWAY_H
#define TIME_TWOWAY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TIME_REQ_PAYLOAD_SIZE   3U
#define TIME_RESP_PAYLOAD_SIZE  27U

typedef struct {
    uint16_t req_seq;
} time_req_t;

typedef struct {
    uint16_t req_seq;
    uint64_t t1_ms;  /* node's UTC when REQ was sent (claimed) */
    uint64_t t2_ms;  /* relay's GPS UTC when REQ was received  */
    uint64_t t3_ms;  /* relay's GPS UTC when RESP was sent     */
} time_resp_t;

/* Node captures T4 locally on RESP reception. */
typedef struct {
    int64_t  offset_ms;   /* clock offset to apply: node_utc += offset_ms */
    int32_t  rtt_ms;      /* round-trip time in ms                         */
} time_twoway_result_t;

/* ── Codec ───────────────────────────────────────────────────────────────── */

uint8_t time_req_encode(const time_req_t *req,  uint8_t *buf, uint8_t capacity);
bool    time_req_decode(const uint8_t *buf, uint8_t length, time_req_t *out);

uint8_t time_resp_encode(const time_resp_t *resp, uint8_t *buf, uint8_t capacity);
bool    time_resp_decode(const uint8_t *buf, uint8_t length, time_resp_t *out);

/*
 * Compute offset and RTT from a completed exchange.
 * t4_ms is the node's local UTC at the moment TIME_RESP was received.
 */
void time_twoway_compute(const time_resp_t *resp, uint64_t t4_ms,
                         time_twoway_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* TIME_TWOWAY_H */

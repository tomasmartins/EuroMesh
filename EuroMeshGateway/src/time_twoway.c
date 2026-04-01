#include <stddef.h>
#include "time_twoway.h"
#include "emesh_packet_types.h"

/* ── TIME_REQ ────────────────────────────────────────────────────────────── */

uint8_t time_req_encode(const time_req_t *req, uint8_t *buf, uint8_t capacity)
{
    if (req == NULL || buf == NULL || capacity < TIME_REQ_PAYLOAD_SIZE) {
        return 0;
    }
    buf[0] = EMESH_PACKET_TYPE_TIME_REQ;
    buf[1] = (uint8_t)(req->req_seq);
    buf[2] = (uint8_t)(req->req_seq >> 8);
    return TIME_REQ_PAYLOAD_SIZE;
}

bool time_req_decode(const uint8_t *buf, uint8_t length, time_req_t *out)
{
    if (buf == NULL || out == NULL || length < TIME_REQ_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_TIME_REQ) {
        return false;
    }
    out->req_seq = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    return true;
}

/* ── TIME_RESP ───────────────────────────────────────────────────────────── */

static uint8_t encode_u64_le(uint64_t v, uint8_t *dst)
{
    dst[0] = (uint8_t)(v);
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v >> 16);
    dst[3] = (uint8_t)(v >> 24);
    dst[4] = (uint8_t)(v >> 32);
    dst[5] = (uint8_t)(v >> 40);
    dst[6] = (uint8_t)(v >> 48);
    dst[7] = (uint8_t)(v >> 56);
    return 8U;
}

static uint64_t decode_u64_le(const uint8_t *src)
{
    return (uint64_t) src[0]
         | ((uint64_t)src[1] << 8)
         | ((uint64_t)src[2] << 16)
         | ((uint64_t)src[3] << 24)
         | ((uint64_t)src[4] << 32)
         | ((uint64_t)src[5] << 40)
         | ((uint64_t)src[6] << 48)
         | ((uint64_t)src[7] << 56);
}

uint8_t time_resp_encode(const time_resp_t *resp, uint8_t *buf, uint8_t capacity)
{
    if (resp == NULL || buf == NULL || capacity < TIME_RESP_PAYLOAD_SIZE) {
        return 0;
    }
    buf[0] = EMESH_PACKET_TYPE_TIME_RESP;
    buf[1] = (uint8_t)(resp->req_seq);
    buf[2] = (uint8_t)(resp->req_seq >> 8);
    encode_u64_le(resp->t1_ms, &buf[3]);
    encode_u64_le(resp->t2_ms, &buf[11]);
    encode_u64_le(resp->t3_ms, &buf[19]);
    return TIME_RESP_PAYLOAD_SIZE;
}

bool time_resp_decode(const uint8_t *buf, uint8_t length, time_resp_t *out)
{
    if (buf == NULL || out == NULL || length < TIME_RESP_PAYLOAD_SIZE) {
        return false;
    }
    if (buf[0] != EMESH_PACKET_TYPE_TIME_RESP) {
        return false;
    }
    out->req_seq = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
    out->t1_ms   = decode_u64_le(&buf[3]);
    out->t2_ms   = decode_u64_le(&buf[11]);
    out->t3_ms   = decode_u64_le(&buf[19]);
    return true;
}

/* ── Two-way offset / RTT calculation ───────────────────────────────────── */

void time_twoway_compute(const time_resp_t *resp, uint64_t t4_ms,
                         time_twoway_result_t *result)
{
    int64_t t1;
    int64_t t2;
    int64_t t3;
    int64_t t4;

    if (resp == NULL || result == NULL) {
        return;
    }

    t1 = (int64_t)resp->t1_ms;
    t2 = (int64_t)resp->t2_ms;
    t3 = (int64_t)resp->t3_ms;
    t4 = (int64_t)t4_ms;

    /*
     * NTP formulas:
     *   offset = ((T2 − T1) + (T3 − T4)) / 2
     *   delay  = ((T4 − T1) − (T3 − T2)) / 2
     */
    result->offset_ms = ((t2 - t1) + (t3 - t4)) / 2;
    result->rtt_ms    = (int32_t)(((t4 - t1) - (t3 - t2)) / 2);
}

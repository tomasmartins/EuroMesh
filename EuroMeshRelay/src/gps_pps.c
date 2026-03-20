/*
 * gps_pps.c — GPS NMEA + PPS time source driver.
 *
 * NMEA sentence handled: $GPRMC (Recommended Minimum Specific GPS Data).
 * Wire format (comma-separated fields):
 *   $GPRMC,HHMMSS.ss,A,LLLL.LL,a,YYYYY.YY,a,x.x,x.x,DDMMYY,x.x,a*hh<CR><LF>
 *   Field 1: UTC time   — HHMMSS[.ss]
 *   Field 2: Status     — A = active, V = void
 *   Field 9: UTC date   — DDMMYY
 *
 * Timing model:
 *   The $GPRMC sentence carries the UTC second for the PPS edge that either
 *   preceded or will follow the sentence (module-dependent, typically the
 *   sentence describes the PREVIOUS second).  We capture the UTC second from
 *   $GPRMC and anchor it to the most recently recorded PPS tick so that the
 *   sub-second accuracy comes from the hardware PPS edge, not the serial link.
 *
 * UTC epoch calculation:
 *   Converts DDMMYY + HHMMSS to a Unix epoch (seconds since 1970-01-01 UTC)
 *   using a compact integer loop.  Two-digit years 00–69 → 2000–2069,
 *   70–99 → 1970–1999 (standard NMEA convention).
 */

#include "gps_pps.h"
#include "emesh_node_caps.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdlib.h>

/* ── Internal: NMEA utilities ────────────────────────────────────────────── */

/*
 * Verify the NMEA XOR checksum.
 * Sentence must start with '$' and contain '*' followed by two hex digits.
 * Returns true if valid.
 */
static bool nmea_checksum_ok(const char *sentence, uint8_t len)
{
    uint8_t calc = 0U;
    uint8_t i;

    if (len < 4U || sentence[0] != '$') {
        return false;
    }

    /* XOR everything between '$' and '*' (exclusive). */
    for (i = 1U; i < len; i++) {
        if (sentence[i] == '*') {
            /* Two hex chars must follow the '*'. */
            if ((uint8_t)(i + 2U) >= len) {
                return false;
            }
            char hi = sentence[i + 1U];
            char lo = sentence[i + 2U];
            uint8_t expected = (uint8_t)(
                (hi >= 'A' ? (hi - 'A' + 10) : (hi - '0')) << 4 |
                (lo >= 'A' ? (lo - 'A' + 10) : (lo - '0')));
            return calc == expected;
        }
        calc ^= (uint8_t)sentence[i];
    }
    return false; /* no '*' found */
}

/*
 * Parse an unsigned integer of exactly `digits` decimal digits from `s`.
 * Returns the value; does not advance the pointer.
 */
static uint32_t parse_fixed_uint(const char *s, uint8_t digits)
{
    uint32_t val = 0U;
    uint8_t  i;
    for (i = 0U; i < digits; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return 0U;
        }
        val = val * 10U + (uint32_t)(s[i] - '0');
    }
    return val;
}

/* ── Internal: date-to-epoch conversion ──────────────────────────────────── */

static bool is_leap_year(uint32_t y)
{
    return (y % 4U == 0U) && ((y % 100U != 0U) || (y % 400U == 0U));
}

static uint32_t days_in_month(uint32_t m, uint32_t y)
{
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    uint32_t d = mdays[m - 1U];
    if (m == 2U && is_leap_year(y)) {
        d = 29U;
    }
    return d;
}

/*
 * Convert calendar date/time to Unix epoch seconds.
 * year_2digit: 0–99 (NMEA convention: 00–69 → 2000–2069, 70–99 → 1970–1999).
 */
static uint64_t date_to_epoch_s(uint32_t day, uint32_t month,
                                uint32_t year_2digit,
                                uint32_t hour, uint32_t min, uint32_t sec)
{
    uint32_t year = (year_2digit >= 70U) ? (1900U + year_2digit)
                                         : (2000U + year_2digit);
    uint64_t days = 0U;
    uint32_t y, m;

    /* Days from 1970 up to (but not including) the target year. */
    for (y = 1970U; y < year; y++) {
        days += is_leap_year(y) ? 366U : 365U;
    }
    /* Days from Jan 1 up to (but not including) the target month. */
    for (m = 1U; m < month; m++) {
        days += days_in_month(m, year);
    }
    /* Days within the target month (1-indexed). */
    days += (uint64_t)(day - 1U);

    return days * 86400ULL
         + (uint64_t)hour * 3600ULL
         + (uint64_t)min  * 60ULL
         + (uint64_t)sec;
}

/* ── Internal: $GPRMC parser ─────────────────────────────────────────────── */

/*
 * Parse one complete $GPRMC sentence and update the fix state.
 * sentence[] is NUL-terminated, length bytes long (including NUL).
 */
static void parse_gprmc(gps_pps_t *gps, const char *sentence, uint8_t length)
{
    /* Verify sentence identity and checksum. */
    if (length < 6U || strncmp(sentence, "$GPRMC", 6) != 0) {
        return;
    }
    if (!nmea_checksum_ok(sentence, length)) {
        return;
    }

    /*
     * Walk comma-separated fields.
     * We need fields 1 (time), 2 (status), and 9 (date).
     */
    const char *p   = sentence;
    uint8_t     fld = 0U;
    const char *time_field   = NULL;
    const char *status_field = NULL;
    const char *date_field   = NULL;

    while (*p != '\0' && *p != '*') {
        if (*p == ',') {
            fld++;
            p++;
            switch (fld) {
            case 1U: time_field   = p; break;
            case 2U: status_field = p; break;
            case 9U: date_field   = p; break;
            default: break;
            }
        } else {
            p++;
        }
    }

    if (time_field == NULL || status_field == NULL || date_field == NULL) {
        return;
    }

    /* Status must be 'A' (active / fix valid). */
    if (status_field[0] != 'A') {
        gps->fix_valid = false;
        return;
    }

    /* Parse time: HHMMSS (at least 6 chars). */
    uint32_t hour = parse_fixed_uint(time_field,     2U);
    uint32_t min  = parse_fixed_uint(time_field + 2U, 2U);
    uint32_t sec  = parse_fixed_uint(time_field + 4U, 2U);

    /* Parse date: DDMMYY. */
    uint32_t day       = parse_fixed_uint(date_field,     2U);
    uint32_t month     = parse_fixed_uint(date_field + 2U, 2U);
    uint32_t year_2dig = parse_fixed_uint(date_field + 4U, 2U);

    /* Sanity check ranges. */
    if (hour > 23U || min > 59U || sec > 60U /* allow leap second */
        || day < 1U || day > 31U
        || month < 1U || month > 12U) {
        return;
    }

    gps->fix_valid      = true;
    gps->fix_utc_epoch_s = date_to_epoch_s(day, month, year_2dig,
                                            hour, min, sec);
}

/* ── Internal: sentence accumulator ─────────────────────────────────────── */

static void accumulate_byte(gps_pps_t *gps, uint8_t byte)
{
    char c = (char)byte;

    if (c == '$') {
        /* Start of a new sentence — reset buffer. */
        gps->nmea_buf[0] = '$';
        gps->nmea_len    = 1U;
        gps->nmea_overflow = false;
        return;
    }

    if (gps->nmea_overflow) {
        return;
    }

    if (c == '\n') {
        /* End of sentence: NUL-terminate and attempt to parse. */
        if (gps->nmea_len > 0U) {
            /* Strip trailing CR if present. */
            if (gps->nmea_buf[gps->nmea_len - 1U] == '\r') {
                gps->nmea_len--;
            }
            gps->nmea_buf[gps->nmea_len] = '\0';
            parse_gprmc(gps, gps->nmea_buf, gps->nmea_len);
        }
        gps->nmea_len = 0U;
        return;
    }

    if (gps->nmea_len >= (GPS_NMEA_BUF_SIZE - 1U)) {
        gps->nmea_overflow = true;
        return;
    }

    gps->nmea_buf[gps->nmea_len++] = c;
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void gps_pps_init(gps_pps_t *gps)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (gps == NULL) {
        return;
    }

    memset(gps, 0, sizeof(*gps));

    /* ── USART3 RX pin — PC11, AF7 ─────────────────────────────────────── */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio_init.Pin       = GPS_UART_RX_PIN;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = GPS_UART_RX_AF;
    HAL_GPIO_Init(GPS_UART_RX_PORT, &gpio_init);

    /* ── USART3 peripheral ──────────────────────────────────────────────── */
    __HAL_RCC_USART3_CLK_ENABLE();

    gps->huart.Instance          = GPS_UART_INSTANCE;
    gps->huart.Init.BaudRate     = GPS_UART_BAUD;
    gps->huart.Init.WordLength   = UART_WORDLENGTH_8B;
    gps->huart.Init.StopBits     = UART_STOPBITS_1;
    gps->huart.Init.Parity       = UART_PARITY_NONE;
    gps->huart.Init.Mode         = UART_MODE_RX;
    gps->huart.Init.HwFlowCtl   = UART_HWCONTROL_NONE;
    gps->huart.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&gps->huart);

    /* Arm the first single-byte interrupt receive. */
    (void)HAL_UART_Receive_IT(&gps->huart, &gps->rx_byte, 1U);

    HAL_NVIC_SetPriority(GPS_UART_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(GPS_UART_IRQn);

    /* ── PPS input — PA0, EXTI0, rising edge ────────────────────────────── */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init.Pin  = GPS_PPS_PIN;
    gpio_init.Mode = GPIO_MODE_IT_RISING;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Alternate = 0U;
    HAL_GPIO_Init(GPS_PPS_PORT, &gpio_init);

    HAL_NVIC_SetPriority(GPS_PPS_EXTI_IRQn, 4U, 0U); /* higher than UART */
    HAL_NVIC_EnableIRQ(GPS_PPS_EXTI_IRQn);
}

/* ── ISR entry points ────────────────────────────────────────────────────── */

void gps_pps_feed_byte(gps_pps_t *gps, uint8_t byte)
{
    if (gps == NULL) {
        return;
    }
    accumulate_byte(gps, byte);
    /* Re-arm for the next byte. */
    (void)HAL_UART_Receive_IT(&gps->huart, &gps->rx_byte, 1U);
}

void gps_pps_on_pps_edge(gps_pps_t *gps, uint32_t tick_ms)
{
    if (gps == NULL) {
        return;
    }
    gps->pps_seen    = true;
    gps->pps_tick_ms = tick_ms;
}

/* ── Query ───────────────────────────────────────────────────────────────── */

bool gps_pps_is_locked(const gps_pps_t *gps, uint32_t now_ms)
{
    if (gps == NULL) {
        return false;
    }
    if (!gps->fix_valid || !gps->pps_seen) {
        return false;
    }
    return (now_ms - gps->pps_tick_ms) < GPS_PPS_MAX_AGE_MS;
}

void gps_pps_apply_to_sync(gps_pps_t *gps,
                           time_sync_t *sync,
                           uint32_t now_ms)
{
    uint64_t utc_epoch_ms;

    if (gps == NULL || sync == NULL) {
        return;
    }
    if (!gps_pps_is_locked(gps, now_ms)) {
        return;
    }

    /*
     * Reconstruct UTC in milliseconds.
     *
     * fix_utc_epoch_s is the UTC second from the most recent $GPRMC sentence.
     * pps_tick_ms is the HAL_GetTick() value when the PPS edge fired.
     *
     * The PPS edge marks the start of the second described by the NMEA
     * sentence (or the next second, depending on the GPS module).  For a
     * standard module where the sentence describes the PRECEDING second, the
     * epoch for the PPS edge is fix_utc_epoch_s + 1.  We use +1 conservatively;
     * modules that send the sentence for the CURRENT second should configure
     * this offset to 0 via a compile-time flag.
     *
     * utc_epoch_ms = (fix_utc_epoch_s + 1) × 1000
     *
     * This anchors the epoch to the PPS pulse, giving sub-millisecond accuracy
     * (bounded by HAL_GetTick() resolution of 1 ms).
     */
#ifndef GPS_PPS_SENTENCE_OFFSET_S
#define GPS_PPS_SENTENCE_OFFSET_S  1U
#endif
    utc_epoch_ms = ((uint64_t)gps->fix_utc_epoch_s + GPS_PPS_SENTENCE_OFFSET_S)
                   * 1000ULL;

    /*
     * Apply to time_sync with src_stratum = EMESH_STRATUM_GPS (0).
     * time_sync_apply_sample will set sync->stratum = 1, correctly marking
     * this relay as a GPS-disciplined stratum-1 time server.
     */
    time_sync_apply_sample(sync,
                           utc_epoch_ms,
                           gps->pps_tick_ms,
                           EMESH_STRATUM_GPS,
                           gps->pps_tick_ms); /* update_tick anchored to PPS */
}

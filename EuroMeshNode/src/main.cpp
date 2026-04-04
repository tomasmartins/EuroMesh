/*
 * EuroMeshNode — Beacon RX + test DATA TX
 * Hardware : LILYGO T-Watch-S3  (ESP32-S3 + SX1262 LoRa + ST7789 LCD + AXP2101)
 *
 * What it does
 * ────────────
 * • Listens continuously for EuroMesh beacon frames from any gateway.
 * • Every TX_INTERVAL_S seconds sends a short DATA frame (broadcast).
 *   The gateway prints the payload when it receives it.
 *
 * Radio parameters — must match EuroMeshGateway exactly:
 *   869.525 MHz · SF7 · BW 125 kHz · CR 4/5 · 8-symbol preamble
 *   Non-inverted IQ · CRC on · Sync word 0x12 (private LoRa network)
 *
 * SPI buses (separate hardware peripherals, no conflict):
 *   Display  →  SPI   (FSPI / SPI2 hw)   SCK=18 MOSI=13 CS=12
 *   LoRa     →  loraSPI (HSPI / SPI3 hw) SCK=3  MISO=4  MOSI=1  CS=5
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <XPowersLib.h>

// ── Display pins ──────────────────────────────────────────────────────────
#define TFT_SCLK   18
#define TFT_MOSI   13
#define TFT_CS     12
#define TFT_DC     38
#define TFT_BL     45

// ── PMU (AXP2101) I2C ─────────────────────────────────────────────────────
#define PMU_SDA    10
#define PMU_SCL    11

// ── SX1262 LoRa pins ──────────────────────────────────────────────────────
static constexpr int LORA_SCK  = 3;
static constexpr int LORA_MISO = 4;
static constexpr int LORA_MOSI = 1;
static constexpr int LORA_NSS  = 5;
static constexpr int LORA_RST  = 8;
static constexpr int LORA_DIO1 = 9;
static constexpr int LORA_BUSY = 7;

// ── Radio parameters ──────────────────────────────────────────────────────
static constexpr float    FREQ_MHZ      = 869.525f;
static constexpr float    BW_KHZ        = 125.0f;
static constexpr uint8_t  SF            = 7;
static constexpr uint8_t  CR            = 5;       // 4/5 → RadioLib value 5
static constexpr uint8_t  SYNC_WORD     = 0x12;    // private LoRa (non-LoRaWAN)
static constexpr uint16_t PREAMBLE_SYMS = 8;
static constexpr float    TCXO_V        = 1.8f;    // TCXO supply on DIO3
static constexpr int8_t   TX_POWER_DBM  = 10;

// ── TX behaviour ──────────────────────────────────────────────────────────
// Send a test DATA packet this often (seconds after last TX / startup).
static constexpr uint32_t TX_INTERVAL_S = 30U;

// ── EuroMesh wire constants ───────────────────────────────────────────────
#define EMESH_TYPE_BEACON    0x01U
#define EMESH_TYPE_DATA      0x04U
#define EMESH_HEADER_SIZE    16U
#define BEACON_PAYLOAD_MIN   16U
#define BEACON_PAYLOAD_FULL  24U
#define EMESH_DEST_BROADCAST 0xFFFFFFFFUL
#define EMESH_FLAG_BROADCAST 0x20U

// ── RGB565 palette ────────────────────────────────────────────────────────
#define CLR_BG      0x0010U   // very dark blue
#define CLR_HEADER  0x0338U   // dark teal
#define CLR_TX_HDR  0x3800U   // dark amber (TX bar)
#define CLR_TEXT    0xFFFFU
#define CLR_DIM     0x8410U
#define CLR_GREEN   0x07E0U
#define CLR_ORANGE  0xFD20U
#define CLR_RED     0xF800U
#define CLR_CYAN    0x07FFU
#define CLR_YELLOW  0xFFE0U

// ── Hardware objects ──────────────────────────────────────────────────────
SPIClass         loraSPI(HSPI);
SX1262           radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSPI);
Adafruit_ST7789  tft(&SPI, TFT_CS, TFT_DC, /*rst=*/-1);
XPowersAXP2101   pmu;
bool             pmuOk = false;

// ── Node identity (derived from ESP32-S3 eFuse MAC) ───────────────────────
static uint32_t NODE_ID = 0;

// ── Radio state ───────────────────────────────────────────────────────────
enum class RadioMode { RX, TX, SLEEP };
static RadioMode radioMode = RadioMode::RX;

// ── State ─────────────────────────────────────────────────────────────────
static uint32_t lastBeaconMs  = 0;
static uint32_t lastTxMs      = 0;
static bool     beaconSeen    = false;
static uint64_t utcBaseMs     = 0;   // UTC epoch ms at last beacon sync
static uint32_t milliBase     = 0;   // millis() at last beacon sync
static uint16_t txSeq         = 0;
static uint32_t txCount       = 0;
static uint16_t rxSeq         = 0;  // seq of last received beacon

struct BeaconInfo {
    uint32_t src_id;
    float    rssi, snr;
    uint8_t  stratum;
    uint64_t utc_ms;
    bool     reg_open, has_telem;
    int8_t   tx_power;
    uint8_t  node_count;
    uint32_t uptime_s;
};
static BeaconInfo last = {};

// ── Little-endian read helpers ────────────────────────────────────────────
static inline uint32_t u32le(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1]<<8)
         | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24);
}
static inline uint64_t u64le(const uint8_t *b) {
    return (uint64_t)b[0]        | ((uint64_t)b[1]<<8)
         | ((uint64_t)b[2]<<16)  | ((uint64_t)b[3]<<24)
         | ((uint64_t)b[4]<<32)  | ((uint64_t)b[5]<<40)
         | ((uint64_t)b[6]<<48)  | ((uint64_t)b[7]<<56);
}
static inline void put_u16le(uint8_t *b, uint16_t v) {
    b[0] = (uint8_t)(v);  b[1] = (uint8_t)(v>>8);
}
static inline void put_u32le(uint8_t *b, uint32_t v) {
    b[0]=(uint8_t)v; b[1]=(uint8_t)(v>>8);
    b[2]=(uint8_t)(v>>16); b[3]=(uint8_t)(v>>24);
}

// ── Display ───────────────────────────────────────────────────────────────
static void drawHeader() {
    tft.fillRect(0, 0, 240, 30, CLR_HEADER);
    tft.setTextSize(2); tft.setTextColor(CLR_TEXT);
    if (utcBaseMs > 0) {
        uint64_t nowUtcMs = utcBaseMs + (uint64_t)(millis() - milliBase);
        time_t   t        = (time_t)(nowUtcMs / 1000ULL);
        struct tm *tm_utc = gmtime(&t);
        char timeBuf[9];
        snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d",
                 tm_utc->tm_hour, tm_utc->tm_min, tm_utc->tm_sec);
        tft.setCursor(6, 7); tft.print(timeBuf);
        // Node ID (last 4 hex digits), centred in the gap between time and battery
        char idBuf[7];
        snprintf(idBuf, sizeof(idBuf), "#%04X", (unsigned)(NODE_ID & 0xFFFFu));
        tft.setCursor(109, 7); tft.print(idBuf);
    } else {
        tft.setCursor(6, 7); tft.print("EuroMesh Node");
    }

    if (!pmuOk) return;
    bool    chg  = pmu.isCharging();
    float   batV = pmu.getBattVoltage() / 1000.0f;
    bool criticalLow = batV < 3.0f;
    uint16_t col = (chg && !criticalLow) ? CLR_CYAN
                 : (batV > 3.9f ? CLR_GREEN
                 : (batV > 3.5f ? CLR_ORANGE : CLR_RED));
    char buf[10];
    snprintf(buf, sizeof(buf), (chg && !criticalLow) ? "CHG" : "%.2fV", batV);
    tft.setTextColor(col);
    tft.setCursor(240 - (int16_t)(strlen(buf)*12) - 4, 7);
    tft.print(buf);
}

static void drawStatus() {
    tft.fillRect(0, 31, 240, 180, CLR_BG);

    if (!beaconSeen) {
        tft.setTextSize(2); tft.setTextColor(CLR_DIM);
        tft.setCursor(16, 85); tft.print("Listening...");
        tft.setTextSize(1); tft.setTextColor(CLR_DIM);
        tft.setCursor(20, 113); tft.print("869.525 MHz SF7 BW125");
        return;
    }

    const BeaconInfo &b = last;
    char buf[32];
    int16_t y = 38;

    // Source GW
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    tft.setCursor(4, y); tft.print("GW");
    tft.setTextSize(2); tft.setTextColor(CLR_TEXT, CLR_BG);
    snprintf(buf, sizeof(buf), "0x%08X", (unsigned)b.src_id);
    tft.setCursor(24, y); tft.print(buf);
    y += 22;

    // RSSI / SNR
    uint16_t rssiCol = b.rssi > -90.0f ? CLR_GREEN
                     : b.rssi > -110.0f ? CLR_ORANGE : CLR_RED;
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    tft.setCursor(4, y); tft.print("RSSI");
    tft.setTextSize(2); tft.setTextColor(rssiCol, CLR_BG);
    snprintf(buf, sizeof(buf), "%+.0fdBm", b.rssi);
    tft.setCursor(40, y); tft.print(buf);
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    tft.setCursor(152, y); tft.print("SNR");
    tft.setTextSize(2); tft.setTextColor(CLR_TEXT, CLR_BG);
    snprintf(buf, sizeof(buf), "%.1f", b.snr);
    tft.setCursor(177, y); tft.print(buf);
    y += 22;

    // Stratum
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    tft.setCursor(4, y); tft.print("Stratum");
    tft.setTextSize(2);
    tft.setTextColor(b.stratum == 0 ? CLR_GREEN : CLR_TEXT, CLR_BG);
    if (b.stratum == 255) snprintf(buf, sizeof(buf), "?");
    else snprintf(buf, sizeof(buf), "%u%s", b.stratum, b.stratum==0?" GPS":"");
    tft.setCursor(70, y); tft.print(buf);
    y += 22;

    // UTC — shown in milliseconds
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    tft.setCursor(4, y); tft.print("UTC ms");
    tft.setTextSize(2);
    if (b.utc_ms > 0) {
        // Print as 13-digit ms epoch, split across two lines if needed
        snprintf(buf, sizeof(buf), "%llu", (unsigned long long)b.utc_ms);
        tft.setTextColor(CLR_TEXT, CLR_BG);
    } else {
        snprintf(buf, sizeof(buf), "no sync");
        tft.setTextColor(CLR_DIM, CLR_BG);
    }
    tft.setCursor(62, y); tft.print(buf);
    y += 22;

    // Reg window + telem
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    tft.setCursor(4, y); tft.print("Reg");
    tft.setTextSize(2);
    tft.setTextColor(b.reg_open ? CLR_GREEN : CLR_DIM, CLR_BG);
    tft.setCursor(34, y); tft.print(b.reg_open ? "OPEN" : "closed");
    if (b.has_telem) {
        tft.setTextSize(2); tft.setTextColor(CLR_TEXT, CLR_BG);
        snprintf(buf, sizeof(buf), "  Up %uh%02um", b.uptime_s/3600,
                 (b.uptime_s%3600)/60);
        tft.setCursor(34 + (b.reg_open ? 48 : 72), y); tft.print(buf);
    }
    y += 22;

    // RX seq + age footer
    tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
    uint32_t age = (millis() - lastBeaconMs) / 1000u;
    snprintf(buf, sizeof(buf), "RX seq#%u  %us ago", rxSeq, age);
    tft.setCursor(4, y); tft.print(buf);
}

static void drawTxBar() {
    // Bottom 30 px: TX status bar
    tft.fillRect(0, 210, 240, 30, CLR_TX_HDR);
    tft.setTextSize(1); tft.setTextColor(CLR_YELLOW, CLR_TX_HDR);
    char buf[40];
    uint32_t age = txCount > 0 ? (millis() - lastTxMs)/1000u : 0;
    if (txCount == 0) {
        snprintf(buf, sizeof(buf), "TX: waiting (in %us)",
                 TX_INTERVAL_S - (millis()/1000u % TX_INTERVAL_S));
    } else {
        snprintf(buf, sizeof(buf), "TX seq#%u  %us ago  (total %u)",
                 txSeq, age, (unsigned)txCount);
    }
    tft.setCursor(4, 218); tft.print(buf);
}

// ── Build and transmit a test DATA frame ──────────────────────────────────
static void sendTestPacket() {
    // ── Build the 16-byte EuroMesh header ──────────────────────────────
    char     text[32];
    snprintf(text, sizeof(text), "EMNODE:%08X:SEQ:%u",
             (unsigned)NODE_ID, (unsigned)(txSeq + 1));
    uint8_t  pld_len = (uint8_t)strlen(text) + 1;  // include NUL

    uint8_t  frame[EMESH_HEADER_SIZE + 32];
    memset(frame, 0, sizeof(frame));

    frame[0]  = EMESH_TYPE_DATA;          // type
    frame[1]  = EMESH_FLAG_BROADCAST;     // flags — no ACK expected
    frame[2]  = 7;                        // TTL
    put_u32le(frame + 3,  NODE_ID);       // src_id
    put_u32le(frame + 7,  EMESH_DEST_BROADCAST); // dest_id = broadcast
    put_u16le(frame + 11, ++txSeq);       // seq
    // op = 0xFF01 (custom class, command 0x01)
    frame[13] = 0x01; frame[14] = 0xFF;   // op little-endian
    frame[15] = pld_len;                  // payload length
    memcpy(frame + EMESH_HEADER_SIZE, text, pld_len);

    uint8_t total = EMESH_HEADER_SIZE + pld_len;

    // ── Stop RX, transmit, restart RX ─────────────────────────────────
    radioMode = RadioMode::TX;
    radio.standby();

    int16_t err = radio.transmit(frame, total);
    lastTxMs = millis();
    txCount++;

    radio.startReceive();
    radioMode = RadioMode::RX;

    if (err == RADIOLIB_ERR_NONE) {
        Serial.printf("[TX] DATA  src=0x%08X  seq=%u  payload=\"%s\"\n",
                      (unsigned)NODE_ID, txSeq, text);
    } else {
        Serial.printf("[TX] ERROR %d\n", err);
    }

    drawTxBar();
}

// ── Process a received packet ─────────────────────────────────────────────
static void handlePacket() {
    size_t  len = radio.getPacketLength();
    if (len == 0 || len > 255) return;

    uint8_t buf[255];
    int16_t err  = radio.readData(buf, len);
    float   rssi = radio.getRSSI();
    float   snr  = radio.getSNR();

    if (err == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.printf("[RX] CRC error RSSI=%.1f\n", rssi);
        return;
    }
    if (err != RADIOLIB_ERR_NONE) {
        Serial.printf("[RX] readData err=%d\n", err);
        return;
    }
    if (len < EMESH_HEADER_SIZE + BEACON_PAYLOAD_MIN) return;
    if (buf[0] != EMESH_TYPE_BEACON)                  return;

    const uint8_t *p = buf + EMESH_HEADER_SIZE;
    if (p[0] != EMESH_TYPE_BEACON) return;

    BeaconInfo b = {};
    b.src_id   = u32le(buf + 3);
    b.rssi     = rssi;
    b.snr      = snr;
    b.utc_ms   = u64le(p + 2);
    b.stratum  = p[14];
    b.reg_open = (p[15] & 0x01u) != 0u;
    rxSeq      = (uint16_t)buf[11] | ((uint16_t)buf[12]<<8);

    if (len >= EMESH_HEADER_SIZE + BEACON_PAYLOAD_FULL) {
        b.has_telem  = true;
        b.tx_power   = (int8_t)p[17];
        b.node_count = p[19];
        b.uptime_s   = u32le(p + 20);
    }

    last         = b;
    lastBeaconMs = millis();
    beaconSeen   = true;
    if (b.utc_ms > 0) {
        utcBaseMs = b.utc_ms;
        milliBase = lastBeaconMs;
    }

    Serial.printf("[BEACON] src=0x%08X  rssi=%.1fdBm  snr=%.1fdB"
                  "  stratum=%u  reg=%s\n",
                  (unsigned)b.src_id, rssi, snr, b.stratum,
                  b.reg_open ? "open" : "closed");
    if (b.has_telem)
        Serial.printf("         txPwr=%ddBm  nodes=%u  uptime=%us\n",
                      (int)b.tx_power, b.node_count, b.uptime_s);

    drawHeader();
    drawStatus();
    drawTxBar();
}

// ─────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1500);

    // Node ID from ESP32-S3 eFuse MAC
    {
        uint64_t mac = ESP.getEfuseMac();
        NODE_ID = (uint32_t)(mac ^ (mac >> 32));
    }

    // Backlight
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Display (FSPI / SPI2)
    SPI.begin(TFT_SCLK, /*miso=*/-1, TFT_MOSI, TFT_CS);
    tft.init(240, 240);
    tft.setRotation(2);
    tft.setSPISpeed(40000000);
    tft.fillScreen(CLR_BG);
    tft.setTextSize(2); tft.setTextColor(CLR_TEXT);
    tft.setCursor(20, 90);  tft.print("EuroMesh Node");
    tft.setTextSize(1); tft.setTextColor(CLR_DIM);
    tft.setCursor(48, 118); tft.print("Initialising...");
    tft.setTextSize(1); tft.setTextColor(CLR_CYAN);
    char idbuf[20]; snprintf(idbuf, sizeof(idbuf), "ID 0x%08X", (unsigned)NODE_ID);
    tft.setCursor(56, 134); tft.print(idbuf);

    // PMU
    Wire.begin(PMU_SDA, PMU_SCL);
    pmuOk = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, PMU_SDA, PMU_SCL);
    Serial.printf("[PMU] AXP2101 %s\n", pmuOk ? "OK" : "not found");

    // LoRa (HSPI / SPI3)
    loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    Serial.print("[Radio] Init SX1262... ");
    int16_t err = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNC_WORD,
                              TX_POWER_DBM, PREAMBLE_SYMS, TCXO_V);
    if (err != RADIOLIB_ERR_NONE) {
        Serial.printf("FAILED (code %d)\n", err);
        tft.fillScreen(CLR_BG);
        tft.setTextSize(2); tft.setTextColor(CLR_RED);
        tft.setCursor(30, 90);  tft.print("LoRa INIT");
        tft.setCursor(50, 118); tft.print("FAILED");
        tft.setTextSize(1); tft.setTextColor(CLR_DIM);
        char b[20]; snprintf(b, sizeof(b), "code %d", err);
        tft.setCursor(72, 150); tft.print(b);
        tft.setCursor(10, 168); tft.print("Try TCXO_V = 1.6 or 0.0");
        while (true) delay(1000);
    }
    Serial.println("OK");
    Serial.printf("[Node] ID=0x%08X\n", (unsigned)NODE_ID);
    Serial.printf("[Radio] %.3f MHz  SF%d  BW%.0f kHz  CR4/%d  SW=0x%02X\n",
                  FREQ_MHZ, SF, BW_KHZ, CR, SYNC_WORD);
    Serial.printf("[TX] Will send DATA every %u s\n", TX_INTERVAL_S);

    radio.invertIQ(false);  // non-inverted — matches gateway
    radio.startReceive();
    radioMode = RadioMode::RX;

    // Full initial display
    tft.fillScreen(CLR_BG);
    drawHeader();
    drawStatus();
    drawTxBar();
    Serial.println("[EuroMeshNode] Running.");
}

// ── Periodic refresh timers ───────────────────────────────────────────────
static uint32_t lastPmuMs = 0;
static uint32_t lastAgeMs = 0;

void loop() {
    uint32_t now = millis();

    // ── PMU header refresh every 5 s ─────────────────────────────────────
    if (pmuOk && (now - lastPmuMs > 5000u)) {
        lastPmuMs = now;
        drawHeader();

        // ── Low-battery radio power save ─────────────────────────────────
        float batV = pmu.getBattVoltage() / 1000.0f;
        if (batV < 3.0f && radioMode != RadioMode::SLEEP) {
            radio.sleep();
            radioMode = RadioMode::SLEEP;
            Serial.println("[Radio] Sleeping — battery critical (<3.0V)");
        } else if (batV >= 3.1f && radioMode == RadioMode::SLEEP) {
            radio.startReceive();
            radioMode = RadioMode::RX;
            Serial.println("[Radio] Woke — battery recovered (>=3.1V)");
        }
    }

    // ── Age fields refresh every 1 s ─────────────────────────────────────
    if (now - lastAgeMs > 1000u) {
        lastAgeMs = now;
        if (beaconSeen) {
            // Redraw just the age line in the status area
            tft.fillRect(0, 190, 240, 18, CLR_BG);
            tft.setTextSize(1); tft.setTextColor(CLR_DIM, CLR_BG);
            char buf[40];
            uint32_t age = (now - lastBeaconMs) / 1000u;
            uint16_t seq = rxSeq;
            snprintf(buf, sizeof(buf), "RX seq#%u  %us ago", seq, age);
            tft.setCursor(4, 190); tft.print(buf);
        }
        drawTxBar();
    }

    // ── Periodic TX ──────────────────────────────────────────────────────
    if (radioMode != RadioMode::SLEEP && (now - lastTxMs) >= TX_INTERVAL_S * 1000u) {
        sendTestPacket();
    }

    // ── RX poll (no interrupt needed — polled via SPI) ────────────────────
    if (radioMode == RadioMode::RX && radio.available()) {
        handlePacket();
        // Always restart receive after reading (in case it didn't auto-restart)
        radio.startReceive();
    }
}

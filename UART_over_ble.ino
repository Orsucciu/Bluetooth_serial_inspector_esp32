#include <Arduino.h>
#include <NimBLEDevice.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// ---------- Debug toggle ----------
#define DEBUG 1
#if DEBUG
  static portMUX_TYPE serialMux = portMUX_INITIALIZER_UNLOCKED;
  #define DBG(...)   do { taskENTER_CRITICAL(&serialMux); Serial.print(__VA_ARGS__);   taskEXIT_CRITICAL(&serialMux); } while(0)
  #define DBGLN(...) do { taskENTER_CRITICAL(&serialMux); Serial.println(__VA_ARGS__); taskEXIT_CRITICAL(&serialMux); } while(0)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

// ---------- Decentlab UART ----------
#define DECENTLAB_RX        16
#define DECENTLAB_TX        17
constexpr unsigned long DECENTLAB_BAUD = 115200;
constexpr int  UART_TIMEOUT_MS         = 10;   // short timeout – don't block the loop
HardwareSerial DecentlabSerial(2);

// ---------- RGB LED (WS2812 on GPIO48) ----------
#define LED_PIN   48
#define NUM_LEDS  1
Adafruit_NeoPixel rgbLed(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------- BLE UART Service UUIDs (Nordic UART) ----------
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ---------- BLE MTU / chunk size ----------
// Default BLE MTU = 23 bytes → 20 bytes usable payload.
// NimBLE can negotiate higher MTUs (up to 517 bytes); we request 247,
// which gives a usable payload of 244 bytes (MTU - 3 for ATT overhead).
// We use 20 as a safe floor and query the negotiated value at runtime.
constexpr size_t BLE_MIN_PAYLOAD    = 20;
constexpr int    BLE_CHUNK_DELAY_MS = 10; // breathing room between chunks

// ---------- ASCII-strip output buffer ----------
//
// The PR36CTD debug UART carries two data streams with NO separator between them:
//
//   [binary RN2483 radio bytes][0000375998] keller: pressure = 10 / 8192 bar\r\n
//
// The binary is internal LoRaWAN radio (RN2483) traffic; the ASCII is the debug log.
// A line-based filter drops everything because both arrive before the \n.
//
// Fix: strip byte-by-byte. Forward only printable ASCII + \r\n\t; silently drop
// everything else. The useful [timestamp] lines come through intact; binary bytes
// are discarded in place without affecting the surrounding text.
//
constexpr size_t OUT_BUF_SIZE = 1024;
static uint8_t  outBuf[OUT_BUF_SIZE];
static size_t   outBufHead  = 0;
static size_t   outBufTail  = 0;
static size_t   outBufCount = 0;

#if DEBUG
static uint32_t outBufDropped = 0; // bytes silently overwritten when buffer is full
#endif

inline bool outBufEmpty() { return outBufCount == 0; }

void outBufPush(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (outBufCount == OUT_BUF_SIZE) {
            // Buffer full: drop the oldest byte to make room.
            // This is intentional (newest data wins), but we track drops in
            // debug mode so overflows don't pass silently during development.
#if DEBUG
            outBufDropped++;
            if (outBufDropped % 64 == 1) { // log on first drop, then every 64th
                DBG("WARNING: outBuf overflow, total dropped bytes: ");
                DBGLN(outBufDropped);
            }
#endif
            outBufTail = (outBufTail + 1) % OUT_BUF_SIZE;
            outBufCount--;
        }
        outBuf[outBufHead] = data[i];
        outBufHead = (outBufHead + 1) % OUT_BUF_SIZE;
        outBufCount++;
    }
}

size_t outBufPop(uint8_t* out, size_t maxLen) {
    size_t n = min(maxLen, outBufCount);
    for (size_t i = 0; i < n; i++) {
        out[i] = outBuf[outBufTail];
        outBufTail = (outBufTail + 1) % OUT_BUF_SIZE;
    }
    outBufCount -= n;
    return n;
}

// ---------- Activity timestamps ----------
volatile unsigned long lastBleActivity  = 0; // written on BLE task, read on main task
         unsigned long lastUartActivity = 0;
constexpr unsigned long ACTIVITY_LED_MS = 200;

// Pass a byte through the filter. Printable ASCII and \r \n \t are forwarded;
// everything else (null bytes, control chars, binary frame bytes) is dropped.
// lastUartActivity is only updated on \n so the LED reflects meaningful lines,
// not the constant background binary radio traffic.
inline void asciiStripFeed(uint8_t b) {
    if ((b >= 0x20 && b <= 0x7E) || b == '\r' || b == '\n' || b == '\t') {
        outBufPush(&b, 1);
        if (b == '\n') lastUartActivity = millis();
    }
    // Binary bytes silently discarded
}

// ---------- UART drain helper ----------
// Extracted into its own function so it can be called both from the main loop
// and from inside bleNotifyChunked, preventing the 256-byte hardware RX buffer
// from overflowing during inter-chunk BLE delays.
void drainUart() {
    uint8_t tmp[64];
    int available = DecentlabSerial.available();
    while (available > 0) {
        int toRead    = min(available, (int)sizeof(tmp));
        int bytesRead = DecentlabSerial.readBytes(tmp, toRead);
        if (bytesRead > 0) {
            for (int i = 0; i < bytesRead; i++) asciiStripFeed(tmp[i]);
        }
        available = DecentlabSerial.available();
    }
}

// ---------- BLE state ----------
NimBLEServer*         pServer           = nullptr;
NimBLECharacteristic* pTxCharacteristic = nullptr;
static bool           deviceConnected   = false;

// ---------- Server Callbacks ----------
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
        DBGLN("Client connected");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        DBGLN("Client disconnected – restarting advertising");
        NimBLEDevice::startAdvertising();
    }
};

// ---------- Write Callback (BLE → UART) ----------
class WriteCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        std::string rxData = pCharacteristic->getValue();
        if (rxData.empty()) return;

        DecentlabSerial.write((const uint8_t*)rxData.data(), rxData.length());

        // PR36CTD expects \r\n line endings; inject if the sender omitted them.
        char last = rxData.back();
        if (last != '\r' && last != '\n') {
            DecentlabSerial.write("\r\n", 2);
        }

        DBG("→ Decentlab: ");
        DBGLN(rxData.c_str());
        lastBleActivity = millis();
    }
};

// ---------- Helpers ----------

// Send `len` bytes via BLE notifications, chunked to the negotiated MTU.
// UART is drained during inter-chunk delays to prevent the hardware RX buffer
// from overflowing while the BLE stack catches up.
void bleNotifyChunked(const uint8_t* data, size_t len) {
    if (!deviceConnected || pServer->getConnectedCount() == 0) return;

    // Query the peer's negotiated MTU (usable payload = MTU - 3 for ATT overhead).
    // This bridge is designed for a single simultaneous client; getPeerInfo(0) is
    // safe here because we guard on getConnectedCount() > 0 above.
    // If multi-client support is ever added, iterate over all connected handles instead.
    uint16_t mtu = pServer->getPeerMTU(pServer->getPeerInfo(0).getConnHandle());
    size_t chunkSize = (mtu > 3) ? (size_t)(mtu - 3) : BLE_MIN_PAYLOAD;

    for (size_t offset = 0; offset < len; offset += chunkSize) {
        if (!deviceConnected) break; // connection dropped mid-send
        size_t n = min(chunkSize, len - offset);
        pTxCharacteristic->setValue(data + offset, n);
        pTxCharacteristic->notify();
        if (offset + chunkSize < len) {
            // Keep draining UART during the inter-chunk pause so the hardware
            // RX buffer doesn't overflow while we wait for the BLE stack.
            unsigned long start = millis();
            while (millis() - start < (unsigned long)BLE_CHUNK_DELAY_MS) {
                drainUart();
            }
        }
    }
}

// ---------- LED helper ----------
void updateLed() {
    unsigned long now = millis();
    bool bleActive  = (now - lastBleActivity  < ACTIVITY_LED_MS);
    bool uartActive = (now - lastUartActivity < ACTIVITY_LED_MS);

    uint32_t color;
    if      (bleActive && uartActive) color = rgbLed.Color(128, 0, 128); // purple  – both active
    else if (bleActive)               color = rgbLed.Color(0,   0, 255); // blue    – BLE only
    else if (uartActive)              color = rgbLed.Color(0, 255,   0); // green   – UART only
    else if (deviceConnected)         color = rgbLed.Color(0,  50,   0); // dim green – idle/connected
    else                              color = rgbLed.Color(0,   0,   0); // off – not connected
    rgbLed.setPixelColor(0, color);
    rgbLed.show();
}

// ---------- Static callback instances (no heap allocation) ----------
static ServerCallbacks serverCbs;
static WriteCallbacks  writeCbs;

// ================================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    // LED
    rgbLed.begin();
    rgbLed.setBrightness(50);
    rgbLed.clear();
    rgbLed.show();

    // UART to PR36CTD
    DecentlabSerial.begin(DECENTLAB_BAUD, SERIAL_8N1, DECENTLAB_RX, DECENTLAB_TX);
    DecentlabSerial.setTimeout(UART_TIMEOUT_MS);

    // BLE
    NimBLEDevice::init("Decentlab-Bridge");
    NimBLEDevice::setMTU(247); // negotiate extended MTU; usable payload = MTU - 3 = 244 bytes

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCbs);

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            NIMBLE_PROPERTY::NOTIFY);

    NimBLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_RX,
                            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pRxCharacteristic->setCallbacks(&writeCbs);

    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("Decentlab-Bridge");
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();

    DBGLN("NimBLE UART Bridge ready.");
}

// ================================================================
void loop() {
    // --- 1. Always drain the UART RX through the ASCII filter ---
    // Runs regardless of BLE connection state to prevent hardware buffer overflow.
    // Binary bytes are silently dropped; clean ASCII lines enter the outbound queue.
    drainUart();

    // --- 2. Flush clean outbound queue → BLE when connected ---
    if (deviceConnected && pServer->getConnectedCount() > 0 && !outBufEmpty()) {
        uint8_t chunk[64];
        size_t n = outBufPop(chunk, sizeof(chunk));
        if (n > 0) {
            bleNotifyChunked(chunk, n);
        }
    }

    updateLed();
}

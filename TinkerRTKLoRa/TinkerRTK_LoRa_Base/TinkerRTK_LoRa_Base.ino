// Copyright (c) 2025 Tinkerbug Robotics
//
// MIT License
//

#include <Arduino.h>

// Local configuration
#include "config.h"

// 3rd party Libraries
#include <Adafruit_NeoPixel.h>
#include <RadioLib.h>
#include <CRC.h>
#include <WiFi.h>
#include <WebServer.h>

// Tinkerbug library for SkyTraq configuration
#include <TR_SkyTraqNMEA.h>
TR_SkyTraqNMEA sky(Serial1);

// ---------- Web Server ----------
WebServer server(80);

// ---------- LoRa radio ----------
SX1262 radio = SX1262(new Module(config::L_CS, 
                                 config::L_DIO1,
                                 config::L_RST,
                                 config::L_BUSY,
                                 SPI));

// ---------- LoRa TX Packet ----------
struct TxPacket 
{
    uint8_t len;
    uint8_t data[config::MAX_PACKET_LENGTH];
};

DRAM_ATTR TxPacket txQueue[config::TX_QUEUE_SIZE];
volatile int txHead = 0;   // Next write
volatile int txTail = 0;   // Next read

enum RadioOp : uint8_t { OP_IDLE, OP_TX };
volatile RadioOp lastOp = OP_IDLE;

volatile bool txBusy      = false;   // Currently transmitting
volatile bool opDone      = false;   // DIO1 ISR flag

// ---------- Debug / Print Variables ----------
// Status/debug
volatile size_t        last_tx_bytes      = 0;    // Size of last TX packet
volatile unsigned long last_tx_end_ms     = 0;    // When last TX completed
volatile size_t        queued_bytes       = 0;    // Bytes waiting in queue
volatile size_t        max_queued_bytes   = 0;    // High-water mark

// ---------- TX Statistics (for prints + web UI) ----------
static uint32_t tx_active_start_ms       = 0;   // when current TX started
static uint32_t tx_time_accum_ms         = 0;   // ms spent transmitting in current window
static uint32_t tx_bytes_accum           = 0;   // bytes transmitted in current window
static uint32_t tx_stats_window_start_ms = 0;   // window start (ms)

volatile uint32_t tx_bytes_last_window   = 0;   // bytes in last completed window
volatile float    tx_duty_last_window    = 0.0f; // % TX time in last window

volatile uint32_t dbg_burst_count      = 0;
volatile uint32_t dbg_msg_total        = 0;
volatile uint32_t dbg_bad_crc24_count  = 0;

// ---------- Raw RTCM Burst Buffer ----------
DRAM_ATTR uint8_t rtcm_data[config::MAX_RTCM_LENGTH];

// ---------- RTCM byte FIFO + Per-message Length Queue ----------
DRAM_ATTR uint8_t rtcmByteQueue[config::RTCM_BYTE_QUEUE_SIZE];

size_t rtcmByteHead  = 0;  // next write index
size_t rtcmByteTail  = 0;  // next read index
size_t rtcmByteCount = 0;  // bytes currently in queue

// ---------- Max number of messages in the rtcmByteQueue ----------
DRAM_ATTR uint16_t rtcmMsgLenQueue[config::RTCM_MSG_QUEUE_SIZE];
size_t rtcmMsgHead   = 0;  // next write index
size_t rtcmMsgTail   = 0;  // next read index
size_t rtcmMsgCount  = 0;  // number of queued messages

// ---------- Number of Available Messages to Poll ----------
volatile size_t rtcmAvailableMsgs = 0;

// ---------- Mutex Protecting rtcmByteQueue + rtcmMsgLenQueue + counters ----------
SemaphoreHandle_t rtcmMutex = nullptr;

// ---------- Tasks ----------
TaskHandle_t rtcmParseTaskHandle = nullptr;
TaskHandle_t loraTxTaskHandle    = nullptr;


// ---------- Forward Declarations ----------
void startWeb();
void handleRoot();
void handleStatus();
uint16_t readSerialBuffer();
void decomposeAndQueueRtcm(uint16_t len);
void handleLoRaTx();
bool enqueuePacket(const uint8_t* payload, int lenWithoutCrc);
float readBatteryVoltage();
float estimateSocFromPerCellVoltage(float v_cell);
uint32_t colorFromSoc(float soc);
void showBatteryOnNeopixel(float soc);
void rtcmParseTask(void* arg);
void loraTxTask(void* arg);
uint32_t rtcmCrc24(const uint8_t* buf, size_t len);
uint16_t getRtcmMsgNumber(const uint8_t* msg, size_t len);
void debugInspectPacket(const uint8_t* payload, int lenWithoutCrc);
size_t rtcmPopMessagesToBufferLocked(uint8_t* outBuf, size_t maxLen);
void rtcmFlushLocked();
bool rtcmEnqueueMessageLocked(const uint8_t* msg, uint16_t len);

// ---------- LoRa ISR ----------
void IRAM_ATTR dio1ISR()
{
    opDone = true;
}

float voltage = 0.0;


// ---------- LED Indicator ----------
Adafruit_NeoPixel pixels(1, config::NEO_PIN, NEO_GRB + NEO_KHZ800);

// ======================= SETUP =================================

void setup()
{
    Serial.begin(115200);
    // Include only if debugging as it stops for a connected USB serial
    //while (!Serial){}; 
    delay(1000);

    // If the ESP32 resets print the reason
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.print("Reset reason: ");
    Serial.println((int)reason);

    Serial.println("\n=== RTK Base Station ===");

    // Initialize variables
    tx_stats_window_start_ms = millis();
    tx_time_accum_ms         = 0;
    tx_bytes_accum           = 0;
    tx_bytes_last_window     = 0;
    tx_duty_last_window      = 0.0f;

    // Start SPI for LoRa radio
    SPI.begin(config::L_SCK, config::L_MISO, config::L_MOSI, config::L_CS);

    // Initialize GPIO pins for the radio
    pinMode(config::L_BUSY, INPUT);
    pinMode(config::L_DIO1, INPUT);
    pinMode(config::L_RST, OUTPUT);

    // Create mutex
    rtcmMutex = xSemaphoreCreateMutex();
    if (!rtcmMutex)
    {
        Serial.println("FATAL: rtcmMutex creation failed");
        while (1) { delay(100); }
    }

    // Configure ADC for reading voltage
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(config::VOLTAGE_PIN, INPUT);
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.show();

    // Read initial voltage and update NeoPixel
    float v_batt = readBatteryVoltage();
    float v_cell = v_batt / config::NUM_BATTERY_CELLS;
    float soc    = estimateSocFromPerCellVoltage(v_cell);
    showBatteryOnNeopixel(soc);

    // Start GNSS NMEA from SkyTraq
    Serial1.begin(115200, SERIAL_8N1, config::GNSS_RX, config::GNSS_TX);

    // LoRa reset sequence
    digitalWrite(config::L_RST, LOW);
    delay(100);
    digitalWrite(config::L_RST, HIGH);
    delay(100);

    // RF switch via DIO2 (switch is internal to radio module)
    radio.setDio2AsRfSwitch(true);

    Serial.print("SX1262 initializing ... ");

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Radio init failed, code: "));
        Serial.println(state);
        while(1) delay(500);
    }

    if ((state = radio.setFrequency(config::FREQUENCY)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setFrequency failed");
        while(1) delay(100);
    }
    if ((state = radio.setBandwidth(config::BANDWIDTH)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setBandwidth failed");
        while(1) delay(100);
    }
    if ((state = radio.setSpreadingFactor(config::SPREADING_FACTOR)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setSpreadingFactor failed");
        while(1) delay(100);
    }
    if ((state = radio.setCodingRate(config::CODING_RATE)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setCodingRate failed");
        while(1) delay(100);
    }
    if ((state = radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setSyncWord failed");
        while(1) delay(100);
    }
    if ((state = radio.setOutputPower(config::OUTPUT_POWER)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setOutputPower failed");
        while(1) delay(100);
    }
    if ((state = radio.setPreambleLength(config::PREAMBLE_LENGTH)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setPreambleLength failed");
        while(1) delay(100);
    }
    Serial.println(" radio initialized successfully");

    // Non-blocking TX done callback
    radio.setDio1Action(dio1ISR);

    // Configure SkyTraq
    Serial.println("[GNSS] Configuring Skytraq to RTK base mode ...");
    sky.resetToDefaultsAndCheck();
    delay(2500);

    // Put receiver into RTK base using the configurable length and accuracy
    Serial.print("Configuring RTK base mode with survey in ...");
    while(sky.configureRtkBaseSurvey(config::SURVEY_IN_SEC,
                                     config::SURVEY_IN_M,
                                     false) != 1)
    {
        Serial.println("failed, retrying");
        delay(500);
    }
    Serial.println(" success");

    delay(1500);

    Serial.print("Configuring RTCM output ...");

    while (sky.configureRtcmOutput(true,
                                    0x00,  // 1 Hz
                                    true,  // 1005
                                    true,  // GPS MSM
                                    true,  // GLO
                                    true,  // GAL
                                    true,  // BDS
                                    0x1E,0x00,0x1E,0x1E,
                                    0x01,
                                    0x02,
                                    false
                                ) != 1)
    {
        Serial.println(" failed, retrying");
        delay(500);
    }
    Serial.println(" success");

    // Want to see the RTCM messages and not NMEA, so disable NMEA
    // Expect some warnings for NMEA types not already enabled
    if (!sky.disableStandardNmea())
    {
        Serial.println("Warning: some NMEA disable commands NACKed or timed out.");
    }

    // Web UI
    startWeb();

    // Create RTOS tasks for parsing RTCM data and transmitting via LoRa
    // This allows other processing to proceed when these task block

    // Designate the core for the tasks to run on (0 is the WiFi/Website core)
    BaseType_t workerCore = 1;

    if (xTaskCreatePinnedToCore(
        rtcmParseTask,
        "RTCM_PARSE_TASK",
        8192,
        nullptr,
        2,
        &rtcmParseTaskHandle,
        workerCore) != pdPASS)
    {
        Serial.println("FATAL: failed to create rtcmParseTask, stopping");
        while (1) { delay(100); }
    }

    if(xTaskCreatePinnedToCore(
        loraTxTask,
        "LORA_TX_TASK",
        8192,
        nullptr,
        2,
        &loraTxTaskHandle,
        workerCore) != pdPASS)
    {
        Serial.println("FATAL: failed to create xTaskCreatePinnedToCore, stopping");
        while (1) { delay(100); }
    }

    Serial.print("RTCM_PARSE_TASK on core ");
    Serial.println(workerCore);
    Serial.print("LORA_TX_TASK on core ");
    Serial.println(workerCore);

    Serial.println("Setup complete");
}

// ======================= LOOP ==================================

void loop()
{
    // Read battery periodically
    static uint32_t last_update = 0;
    if (millis() - last_update >= config::PWR_UPDATE_PERIOD_MS)
    {
        last_update = millis();

        voltage = readBatteryVoltage();
        float v_cell = voltage / config::NUM_BATTERY_CELLS;
        float soc    = estimateSocFromPerCellVoltage(v_cell);
        showBatteryOnNeopixel(soc);
    }

    // Update web server
    server.handleClient();
    delay(5);
}

// ======================= TASK 1: Parse RTCM =====================

void rtcmParseTask(void* arg)
{
    (void)arg;

    uint32_t lastAcceptMs = 0;

    for (;;)
    {
        // Always drain UART so the GNSS doesn't back up
        uint16_t len = readSerialBuffer();

        if (len > 0)
        {
            uint32_t now = millis();

            if (now - lastAcceptMs >= config::RTCM_ACCEPT_INTERVAL_MS)
            {
                // Only every 2s do we actually parse & enqueue
                decomposeAndQueueRtcm(len);
                lastAcceptMs = now;
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

// ======================= TASK 2: LoRa TX ========================

void loraTxTask(void* arg)
{
    (void)arg;

    uint8_t  payload[config::MAX_PACKET_LENGTH];
    uint32_t lastStatusMs = millis();

    for (;;)
    {
        // Run LoRa TX state machine
        handleLoRaTx();

        size_t bytesToSend = 0;

        // Grab the bytes to send
        if (rtcmAvailableMsgs > 0)
        {
            if (xSemaphoreTake(rtcmMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                bytesToSend = rtcmPopMessagesToBufferLocked(payload, config::MAX_PACKET_LENGTH);
                xSemaphoreGive(rtcmMutex);
            }
        }

        // Send available bytes
        if (bytesToSend > 0)
        {
            (void)enqueuePacket(payload, (int)bytesToSend);
        }

        // Periodic status print (and window stats)
        uint32_t now = millis();
        if (now - lastStatusMs >= config::STATUS_PERIOD_MS)
        {
            uint32_t windowMs = now - tx_stats_window_start_ms;
            float duty = 0.0f;
            if (windowMs > 0)
            {
                duty = (100.0f * (float)tx_time_accum_ms) / (float)windowMs;
            }

            // Snapshot for web UI
            tx_bytes_last_window = tx_bytes_accum;
            tx_duty_last_window  = duty;

            // Simple compact line
            Serial.printf(
                "TX window: %u bytes, duty: %.1f%%, queue: %u bytes\n",
                (unsigned)tx_bytes_last_window,
                (double)tx_duty_last_window,
                (unsigned)queued_bytes
            );

            // Reset window accumulators
            tx_time_accum_ms         = 0;
            tx_bytes_accum           = 0;
            tx_stats_window_start_ms = now;
            lastStatusMs             = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ======================= RTCM Read ==============================

// Read one burst of RTCM from GNSS into rtcm_data[]
uint16_t readSerialBuffer()
{
    if (!Serial1.available())
    {
        return 0;
    }

    int  input_pos = 0;
    bool gotData   = false;
    unsigned long last_read_time = millis();

    // Read until no data for ~50 ms
    while (Serial1.available() || (millis() - last_read_time) < 50)
    {
        if (Serial1.available())
        {
            int in_byte = Serial1.read();
            if (in_byte >= 0)
            {
                if (input_pos < (int)config::MAX_RTCM_LENGTH)
                {
                    rtcm_data[input_pos++] = (uint8_t)in_byte;
                }
                last_read_time = millis();
                gotData        = true;
            }
        }
        // Yield to other tasks
        taskYIELD();
    }

    if (gotData && input_pos > 0)
    {
        dbg_burst_count++;
        return (uint16_t)input_pos;
    }

    return 0;
}

// ========== Parse & enqueue RTCM messages into shared FIFO =========

void decomposeAndQueueRtcm(uint16_t read_data_length)
{
    if (read_data_length == 0) return;

    size_t i = 0;

    while (i + 6 <= read_data_length)
    {
        if (rtcm_data[i] != 0xD3)
        {
            i++;
            continue;
        }

        // Incomplete header
        if (i + 3 > read_data_length)
        {
            break;
        }

        uint8_t b1 = rtcm_data[i + 1];
        uint8_t b2 = rtcm_data[i + 2];

        uint16_t msgLen = ((b1 & 0x03) << 8) | b2;
        size_t   fullLen = 3 + msgLen + 3;  // D3 + 2 len bits + payload + CRC24

        if (msgLen > 1023 || i + fullLen > read_data_length)
        {
            // Incomplete/oversize message in this burst; bail and let next burst finish
            break;
        }

        uint32_t calc24 = rtcmCrc24(&rtcm_data[i], 3 + msgLen);
        uint32_t rx24 =
            (uint32_t(rtcm_data[i + 3 + msgLen])     << 16) |
            (uint32_t(rtcm_data[i + 3 + msgLen + 1]) << 8)  |
            (uint32_t(rtcm_data[i + 3 + msgLen + 2]));

        if (calc24 != rx24)
        {
            dbg_bad_crc24_count++;
            i++; // skip this byte and resync
            continue;
        }

        // Valid RTCM message
        dbg_msg_total++;

        // Enqueue into shared byte FIFO / length queue
        if (xSemaphoreTake(rtcmMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            rtcmEnqueueMessageLocked(&rtcm_data[i], (uint16_t)fullLen);
            xSemaphoreGive(rtcmMutex);
        }

        i += fullLen;
    }
}

// ============ RTCM queue helpers (must be called with rtcmMutex held) ==========

// Completely flush both byte FIFO and message queue
void rtcmFlushLocked()
{
    rtcmByteHead  = 0;
    rtcmByteTail  = 0;
    rtcmByteCount = 0;

    rtcmMsgHead   = 0;
    rtcmMsgTail   = 0;
    rtcmMsgCount  = 0;

    rtcmAvailableMsgs = 0;
}

// Enqueue one full RTCM message (D3...CRC24) into the byte FIFO & length queue.
// If not enough room, flush everything and start fresh.
bool rtcmEnqueueMessageLocked(const uint8_t* msg, uint16_t len)
{
    if (!msg || len == 0 || len > config::RTCM_BYTE_QUEUE_SIZE)
    {
        return false;
    }

    // If this message wouldn't fit OR there's no room for another length entry,
    // wipe everything and start over so we don't get too far behind.
    if ((rtcmByteCount + len > config::RTCM_BYTE_QUEUE_SIZE) ||
        (rtcmMsgCount >= config::RTCM_MSG_QUEUE_SIZE))
    {
        rtcmFlushLocked();
    }

    // Write message bytes into circular byte queue
    size_t idx = rtcmByteHead;
    for (uint16_t i = 0; i < len; i++)
    {
        rtcmByteQueue[idx] = msg[i];
        idx++;
        if (idx >= config::RTCM_BYTE_QUEUE_SIZE)
        {
            idx = 0;
        }
    }
    rtcmByteHead  = idx;
    rtcmByteCount += len;

    // Record message length
    rtcmMsgLenQueue[rtcmMsgHead] = len;
    rtcmMsgHead = (rtcmMsgHead + 1) % config::RTCM_MSG_QUEUE_SIZE;
    rtcmMsgCount++;

    rtcmAvailableMsgs = rtcmMsgCount;
    return true;
}

// Copy out up to maxLen bytes of *whole messages* into outBuf.
// Removes those bytes and messages from the queues.
// Returns number of bytes copied.
size_t rtcmPopMessagesToBufferLocked(uint8_t* outBuf, size_t maxLen)
{
    if (!outBuf || maxLen == 0 || rtcmMsgCount == 0)
    {
        return 0;
    }

    size_t bytesCopied = 0;

    while (rtcmMsgCount > 0)
    {
        uint16_t mlen = rtcmMsgLenQueue[rtcmMsgTail];

        // If this message doesn't fit in remaining space, stop
        if (bytesCopied + mlen > maxLen)
        {
            break;
        }

        // Copy mlen bytes from byte queue into outBuf
        for (uint16_t i = 0; i < mlen; i++)
        {
            outBuf[bytesCopied++] = rtcmByteQueue[rtcmByteTail];
            rtcmByteTail++;
            if (rtcmByteTail >= config::RTCM_BYTE_QUEUE_SIZE)
            {
                rtcmByteTail = 0;
            }
        }
        rtcmByteCount -= mlen;

        // Pop one length entry
        rtcmMsgTail = (rtcmMsgTail + 1) % config::RTCM_MSG_QUEUE_SIZE;
        rtcmMsgCount--;
    }

    rtcmAvailableMsgs = rtcmMsgCount;
    return bytesCopied;
}

// ======================= LoRa TX state machine ==================

bool txQueueEmpty()
{
    return txHead == txTail;
}

TxPacket* frontPacket()
{
    if (txQueueEmpty()) return nullptr;
    return &txQueue[txTail];
}

void popPacket()
{
    if (!txQueueEmpty())
    {
        queued_bytes -= txQueue[txTail].len;
        txTail = (txTail + 1) % config::TX_QUEUE_SIZE;
    }
}

bool enqueuePacket(const uint8_t* payload, int lenWithoutCrc)
{
    if (lenWithoutCrc <= 0) return false;
    if (lenWithoutCrc + 2 > config::MAX_PACKET_LENGTH)
    {
        // Payload too large for a single LoRa packet
        return false;
    }

    int curHead  = txHead;
    int curTail  = txTail;
    int nextHead = (curHead + 1) % config::TX_QUEUE_SIZE;

    if (nextHead == curTail)
    {
        // TX queue full, drop silently (or keep a very short error if you want)
        // Serial.println("[LORA_Q] TX queue full, dropping packet");
        return false;
    }

    TxPacket& p = txQueue[curHead];
    memcpy(p.data, payload, lenWithoutCrc);

    // Outer CRC16 over payload
    uint16_t crc = calcCRC16(p.data, lenWithoutCrc);
    p.data[lenWithoutCrc]     = uint8_t(crc >> 8);
    p.data[lenWithoutCrc + 1] = uint8_t(crc & 0xFF);
    p.len = lenWithoutCrc + 2;

    txHead = nextHead;

    queued_bytes += p.len;
    if (queued_bytes > max_queued_bytes)
    {
        max_queued_bytes = queued_bytes;
    }

    return true;
}

void handleLoRaTx()
{
    uint32_t now = millis();

    // If ISR fired, finish TX
    if (opDone)
    {
        opDone = false;

        if (lastOp == OP_TX && txBusy)
        {
            int      st   = radio.finishTransmit();
            uint32_t now2 = millis();

            if (st == RADIOLIB_ERR_NONE)
            {
                last_tx_end_ms = now2;

                // Accumulate TX time and bytes for the current window
                if (tx_active_start_ms != 0 && now2 >= tx_active_start_ms)
                {
                    tx_time_accum_ms += (now2 - tx_active_start_ms);
                }
                tx_bytes_accum += last_tx_bytes;
            }
            else
            {
                Serial.printf("[LORA_TXERR] finish st=%d\n", st);
            }

            txBusy = false;
            lastOp = OP_IDLE;
            tx_active_start_ms = 0;
        }
    }

    // Start next TX if idle and we have queued packets
    if (!txBusy && !txQueueEmpty())
    {
        TxPacket* p = frontPacket();
        if (p)
        {
            last_tx_bytes = p->len;

            int st = radio.startTransmit(p->data, p->len);
            if (st == RADIOLIB_ERR_NONE)
            {
                txBusy = true;
                lastOp = OP_TX;
                tx_active_start_ms = now;   // mark TX start for timing

                popPacket();
            }
            else
            {
                // Failed to start, drop this packet and move on
                popPacket();
            }
        }
    }
}

// ======================= Battery Helpers ========================

// Read battery voltage at pack terminals (approx)
float readBatteryVoltage()
{
    // Read raw ADC value
    int raw = analogRead(config::VOLTAGE_PIN);

    // Convert to voltage at ADC pin
    float v_adc = (raw * config::VREF) / config::ADC_MAX;

    // Compute actual input voltage (before divider)
    float voltage = v_adc * ((config::R1 + config::R2) / config::R2);

    return voltage;

}

// Estimate state-of-charge (0..100%) from per-cell voltage
// Very rough LiPo mapping:
//   4.20 V ≈ 100%
//   3.70 V ≈ 40-50%
//   3.30 V ≈ 0%
// Clamp and do a simple linear approximation between 3.3 and 4.2.
float estimateSocFromPerCellVoltage(float v_cell)
{
    const float V_MIN = 3.30f;  // "empty" (under-load)
    const float V_MAX = 4.20f;  // full

    if (v_cell <= V_MIN) return 0.0f;
    if (v_cell >= V_MAX) return 100.0f;

    float soc = (v_cell - V_MIN) * 100.0f / (V_MAX - V_MIN);
    return soc;
}

// Map SOC to a color:
//   0%   -> Red
//   50%  -> Yellow
//   100% -> Green
uint32_t colorFromSoc(float soc)
{
    soc = constrain(soc, 0.0f, 100.0f);

    uint8_t r, g, b;
    b = 0;

    if (soc < 50.0f)
    {
        // 0..50%: Red -> Yellow  (R=255, G:0->255)
        r = 255;
        g = (uint8_t)map((long)(soc * 10), 0, 500, 0, 255);
    } else {
        // 50..100%: Yellow -> Green (R:255->0, G=255)
        g = 255;
        r = (uint8_t)map((long)((soc - 50.0f) * 10), 0, 500, 255, 0);
    }

    return pixels.Color(r, g, b);
}

void showBatteryOnNeopixel(float soc)
{
    uint32_t c = colorFromSoc(soc);
    pixels.setPixelColor(0, c);
    pixels.show();
}



// ======================= Web UI ===============================

const char* INDEX_HTML = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>RTK Base Station (RTCM TX)</title>
<meta name="viewport" content="width=device-width, initial-scale=1" />
<style>
  body{font-family:system-ui,Arial,sans-serif;margin:16px;background:#0b0f14;color:#e7eef5}
  .card{max-width:720px;padding:16px;border-radius:14px;background:#151d26;box-shadow:0 8px 24px rgba(0,0,0,.25)}
  h1{margin:0 0 12px 0;font-size:20px}
  .row{display:flex;gap:18px;flex-wrap:wrap}
  .tile{flex:1 1 160px;background:#0f1620;padding:14px 16px;border-radius:10px}
  .big{font-size:24px;font-weight:700;margin:6px 0}
  .unit{opacity:.8}
  .meta{opacity:.7;font-size:12px;margin-top:10px}
</style>
</head>
<body>
<div class="card">
  <h1>RTK Base Station (RTCM TX only)</h1>
  <div class="row">
    <div class="tile">
      <div>Battery Voltage</div>
      <div class="big"><span id="vin">--</span> <span class="unit">V</span></div>
    </div>
    <div class="tile">
      <div>Last LoRa TX</div>
      <div class="big"><span id="tx">--</span> <span class="unit">bytes</span></div>
    </div>
    <div class="tile">
      <div>TX Queue</div>
      <div class="big"><span id="qbytes">--</span> <span class="unit">bytes</span></div>
    </div>
    <div class="tile">
      <div>Max Queue</div>
      <div class="big"><span id="qmax">--</span> <span class="unit">bytes</span></div>
    </div>
    <div class="tile">
      <div>Time Since Last TX</div>
      <div class="big"><span id="txage">--</span> <span class="unit">s</span></div>
    </div>
        <div class="tile">
      <div>Bytes (last 2 s)</div>
      <div class="big"><span id="txbyteswin">--</span></div>
    </div>
    <div class="tile">
      <div>% TX Busy (last 5 s)</div>
      <div class="big"><span id="txduty">--</span> <span class="unit">%</span></div>
    </div>
  </div>
  <div class="meta">Last Updated: <span id="ts">--</span></div>
</div>
<script>
async function update(){
  try{
    const r = await fetch('/status');
    const j = await r.json();

    if (typeof j.vin === "number")
        document.getElementById('vin').textContent    = j.vin.toFixed(3);

    document.getElementById('tx').textContent     = j.last_tx_bytes;
    document.getElementById('qbytes').textContent = j.queued_bytes;
    document.getElementById('qmax').textContent   = j.max_queued_bytes;

    if (j.last_tx_age_ms >= 0) {
      const secs = j.last_tx_age_ms / 1000.0;
      document.getElementById('txage').textContent = secs.toFixed(1);
    } else {
      document.getElementById('txage').textContent = "—";
    }

    document.getElementById('txbyteswin').textContent = j.tx_bytes_window ?? "--";

    if (typeof j.tx_duty_pct === "number") {
      document.getElementById('txduty').textContent = j.tx_duty_pct.toFixed(1);
    } else {
      document.getElementById('txduty').textContent = "--";
    }

    const d = new Date(j.millis);
    document.getElementById('ts').textContent  = d.toLocaleTimeString();
  }catch(e){
    console.log(e);
  }
}
update();
setInterval(update, 1000);
</script>
</body>
</html>
)HTML";

void handleRoot() 
{
    server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() 
{
    float          v     = voltage;
    size_t         tx    = last_tx_bytes;
    unsigned long  nowms = millis();
    long           age   = (last_tx_end_ms == 0) ? -1L : long(nowms - last_tx_end_ms);

    String json = "{";
    json += "\"vin\":" + String(v, 6) + ",";
    json += "\"last_tx_bytes\":" + String(tx) + ",";
    json += "\"millis\":" + String(nowms) + ",";
    json += "\"last_tx_time_ms\":" + String(last_tx_end_ms) + ",";
    json += "\"last_tx_age_ms\":" + String(age) + ",";
    json += "\"queued_bytes\":" + String(queued_bytes) + ",";
    json += "\"max_queued_bytes\":" + String(max_queued_bytes) + ",";
    json += "\"tx_bytes_window\":" + String(tx_bytes_last_window) + ",";
    json += "\"tx_duty_pct\":" + String(tx_duty_last_window, 2);
    json += "}";

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

void startWeb() 
{
    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    WiFi.softAP(config::AP_SSID, config::AP_PASS);
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP up: "); Serial.println(ip);

    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.begin();
    Serial.println("HTTP server started on / and /status");
}

// ======================= RTCM helpers ===========================

// CRC-24Q used by RTCM3
uint32_t rtcmCrc24(const uint8_t* buf, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= ((uint32_t)buf[i]) << 16;
        for (int b = 0; b < 8; b++)
        {
            crc <<= 1;
            if (crc & 0x1000000)
            {
            crc ^= 0x1864CFB;
            }
        }
    }
    return crc & 0xFFFFFF;
}

// Extract RTCM3 message number from a complete, CRC-checked message.
uint16_t getRtcmMsgNumber(const uint8_t* msg, size_t len) 
{
    if (!msg || len < 6 || msg[0] != 0xD3) 
    {
        return 0;
    }
    uint16_t type = ((uint16_t(msg[3]) << 4) | (uint16_t(msg[4]) >> 4)) & 0x0FFF;
    return type;
}

void debugInspectPacket(const uint8_t* payload, int lenWithoutCrc)
{
    if (lenWithoutCrc < 6) 
    {
        return;
    }
}

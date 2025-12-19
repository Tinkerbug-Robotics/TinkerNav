// Copyright (c) 2025 Tinkerbug Robotics
//
// MIT License
//

#include <Arduino.h>

// Local configuration
#include "config.h"

// 3rd party Libraries
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <WebServer.h>
#include <RadioLib.h>
#include <CRC.h>

// Tinkerbug library for the rover web UI
#include <TR_RTKRoverWebUI.h>

// Tinkerbug library for SkyTraq configuration
#include <TR_SkyTraqNMEA.h>
TR_SkyTraqNMEA sky(Serial1);

// Separate UART for RTCM corrections -> RXD2 pin on receiver
HardwareSerial SerialRTCM(2);

// ======================= LoRa Radio =============================

SX1262 radio = SX1262(new Module(config::L_CS,
                                 config::L_DIO1,
                                 config::L_RST,
                                 config::L_BUSY,
                                 SPI));

// ISR flag for LoRa DIO1
volatile bool loraIrqFlag = false;

void IRAM_ATTR dio1ISR()
{
    loraIrqFlag = true;
}

// ======================= RX Stats / State =======================

static uint8_t loraRxBuf[config::MAX_PACKET_LENGTH];

volatile uint32_t last_rx_ms        = 0;   // millis() of last good RX
volatile uint32_t last_rx_len       = 0;   // LoRa payload length (incl. CRC16)
volatile uint32_t last_rtcm_len     = 0;   // forwarded RTCM bytes (excl. CRC16)
volatile int16_t  last_rssi         = 0;
volatile float    last_snr          = 0.0f;

volatile uint32_t total_rtcm_bytes  = 0;
volatile uint32_t total_packets_ok  = 0;
volatile uint32_t total_crc_errors  = 0;
volatile uint32_t total_radio_errors = 0;

// GNSS data for UI
TR_SkyTraqNMEA::GnssNmeaData g_latest;
bool g_has_gnss = false;
unsigned long g_lastRtcmMs         = 0;
uint16_t      g_lastRtcmBlockBytes = 0;

// HTTP server for status web page
WebServer web(80);

// Web UI wrapper
TR_RTKRoverWebUI roverUI(web,
                         g_latest,
                         g_has_gnss,
                         g_lastRtcmMs,
                         g_lastRtcmBlockBytes);

// ======================= Battery / Neopixel =====================

float voltage = 0.0f;
Adafruit_NeoPixel pixels(1, config::NEO_PIN, NEO_GRB + NEO_KHZ800);

// ======================= Forward Declarations =====================
float readBatteryVoltage();
float estimateSocFromPerCellVoltage(float v_cell);
uint32_t colorFromGnssQuality(uint8_t q);
void showGnssQualityOnNeopixel(uint8_t gnss_quality);
void handleLoRaRx();

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== RTK Rover (RTCM via RXD2 + Web UI) ===");

    // Init stats
    last_rx_ms        = 0;
    last_rx_len       = 0;
    last_rtcm_len     = 0;
    total_rtcm_bytes  = 0;
    total_packets_ok  = 0;
    total_crc_errors  = 0;
    total_radio_errors = 0;

    // SPI for LoRa
    SPI.begin(config::L_SCK, config::L_MISO, config::L_MOSI, config::L_CS);

    // Radio GPIO
    pinMode(config::L_BUSY, INPUT);
    pinMode(config::L_DIO1, INPUT);
    pinMode(config::L_RST,  OUTPUT);

    // ADC for voltage
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(config::VOLTAGE_PIN, INPUT);

    // NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.show();

    // Initial battery read
    float v_batt = readBatteryVoltage();
    float v_cell = v_batt / config::NUM_BATTERY_CELLS;
    float soc    = estimateSocFromPerCellVoltage(v_cell);
    voltage = v_batt;

    // GNSS UART (SkyTraq)
    Serial1.begin(115200, SERIAL_8N1, config::GNSS_RX, config::GNSS_TX);
    SerialRTCM.begin(115200, SERIAL_8N1, -1, config::GNSS_RXD2);

    sky.resetToDefaultsAndCheck();
    delay(2000);

    // LoRa reset sequence
    digitalWrite(config::L_RST, LOW);
    delay(100);
    digitalWrite(config::L_RST, HIGH);
    delay(100);

    // Optional RF switch via DIO2 if used on your board
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

    // Hook DIO1 to ISR and start continuous receive
    radio.setDio1Action(dio1ISR);
    state = radio.startReceive();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(config::AP_SSID, config::AP_PASS);
    Serial.print("Rover AP up, IP: ");
    Serial.println(WiFi.softAPIP());

    // Start web UI
    roverUI.begin();
    Serial.println("HTTP server started on port 80");

}

void loop()
{
    // Periodic battery update + NeoPixel
    static uint32_t lastBatteryMs = 0;
    uint32_t now = millis();

    if (now - lastBatteryMs >= config::PWR_UPDATE_PERIOD_MS)
    {
        lastBatteryMs = now;
        voltage = readBatteryVoltage();

        float v_cell = voltage / config::NUM_BATTERY_CELLS;
        float soc    = estimateSocFromPerCellVoltage(v_cell);

        // Update NeoPixel based on GNSS quality
        // Use the battery periodic to keep the update reasonable
        // Could use base station methods to update NeoPixel based
        // on battery charge if preferred
        showGnssQualityOnNeopixel(g_latest.gnss_quality_indicator);
    }

    // Handle LoRa RX events
    handleLoRaRx();

    // ---- Read GNSS UART and feed NMEA parser (primary port) ----
    while (Serial1.available())
    {
        char c = (char)Serial1.read();
        sky.feedChar(c);
        yield();
    }

    // Grab latest parsed data if there is a new coherent update
    TR_SkyTraqNMEA::GnssNmeaData temp;
    if (sky.getData(temp))
    {
        g_latest   = temp;
        g_has_gnss = true;
    }

    // Web UI
    roverUI.loop();

    yield();

}

// ======================= LoRa RX Handler ========================

void handleLoRaRx()
{
    if (!loraIrqFlag) 
    {
        return;
    }
    loraIrqFlag = false;

    // Read the received packet into loraRxBuf
    int16_t state = radio.readData(loraRxBuf, sizeof(loraRxBuf));
    if (state != RADIOLIB_ERR_NONE)
    {
        if (state == RADIOLIB_ERR_CRC_MISMATCH)
        {
            total_crc_errors++;
            Serial.println("[LoRa] CRC error at radio layer");
        }
        else
        {
            total_radio_errors++;
            Serial.print("[LoRa] readData error: ");
            Serial.println(state);
        }
        // Re-arm receiver
        radio.startReceive();
        return;
    }

    // Actual length of received frame
    size_t rxLen = radio.getPacketLength();
    if (rxLen < 3 || rxLen > sizeof(loraRxBuf))
    {
        total_radio_errors++;
        Serial.printf("[LoRa] Invalid length: %u\n", (unsigned)rxLen);
        radio.startReceive();
        return;
    }

    // Outer CRC16 check (last 2 bytes of the frame)
    if (rxLen < 3)
    {
      radio.startReceive();
      return;
    }

    size_t payloadLen   = rxLen - 2;  // bytes that should be RTCM
    uint16_t rxCrc      = (uint16_t(loraRxBuf[payloadLen]) << 8) |
                          uint16_t(loraRxBuf[payloadLen + 1]);
    uint16_t calc       = calcCRC16(loraRxBuf, payloadLen);

    if (rxCrc != calc)
    {
        total_crc_errors++;
        Serial.println("[LoRa] Outer CRC16 mismatch, dropping packet");
        radio.startReceive();
        return;
    }
  
    // Forward RTCM bytes to SkyTraq UART
    SerialRTCM.write(loraRxBuf, payloadLen);

    // Re-arm receiver
    radio.startReceive();

    // Good packet; forward RTCM payload to SkyTraq
    last_rx_ms    = millis();
    last_rx_len   = rxLen;
    last_rtcm_len = payloadLen;

    // Stats: accumulate RTCM bytes
    total_rtcm_bytes += payloadLen;
    total_packets_ok++;

    // Record link quality
    last_rssi = radio.getRSSI();
    last_snr  = radio.getSNR();

    Serial.printf("[LoRa] OK: rxLen=%u, rtcm=%u, RSSI=%d dBm, SNR=%.1f dB\n",
                (unsigned)rxLen,
                (unsigned)payloadLen,
                (int)last_rssi,
                (double)last_snr);

    // Update UI-facing RTK stats
    g_lastRtcmMs         = last_rx_ms;
    g_lastRtcmBlockBytes = payloadLen;
}

// ======================= Battery Helpers ========================

float readBatteryVoltage()
{
    int raw = analogRead(config::VOLTAGE_PIN);
    float v_adc = (raw * config::VREF) / config::ADC_MAX;
    float voltage = v_adc * ((config::R1 + config::R2) / config::R2);
    return voltage;
}

float estimateSocFromPerCellVoltage(float v_cell)
{
    const float V_MIN = 3.30f;
    const float V_MAX = 4.20f;

    if (v_cell <= V_MIN) return 0.0f;
    if (v_cell >= V_MAX) return 100.0f;

    float soc = (v_cell - V_MIN) * 100.0f / (V_MAX - V_MIN);
    return soc;
}

// Map GNSS quality indicator to NeoPixel color
uint32_t colorFromGnssQuality(uint8_t q)
{
    uint8_t r = 0, g = 0, b = 0;

    switch (q)
    {
        case 0: // No Fix
            r = 255; g = 0;   b = 0;
            break;

        case 1: // GPS
        case 2: // DGPS
        case 3: // PPS
            r = 255; g = 255; b = 0;   // Yellow
            break;

        case 4: // RTK Fixed
            r = 0;   g = 0;   b = 255; // Blue
            break;

        case 5: // RTK Float
            r = 0;   g = 255; b = 0;   // Green
            break;

        case 6: // Dead Reckoning
        default: // Unknown
            r = 255; g = 0;   b = 0;   // Red
            break;
    }

    return pixels.Color(r, g, b);
}

// Drive the NeoPixel based on GNSS quality
void showGnssQualityOnNeopixel(uint8_t gnss_quality)
{
    uint32_t c = colorFromGnssQuality(gnss_quality);
    pixels.setPixelColor(0, c);
    pixels.show();
}




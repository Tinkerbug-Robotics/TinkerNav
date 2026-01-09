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
#include <Preferences.h>

// Tinkerbug library for the rover web UI
#include <TR_RTKRoverWebUI.h>

// Tinkerbug library for SkyTraq configuration
#include <TR_SkyTraqNMEA.h>
TR_SkyTraqNMEA sky(Serial1);

// Separate UART for RTCM corrections -> RXD2 pin on receiver
HardwareSerial SerialRTCM(2);

// WiFi server for incoming RTCM stream from base
WiFiServer rtcmServer(config::RTCM_PORT);
WiFiClient rtcmClient;

// HTTP server for status web page
WebServer web(80);

// GNSS data for UI
TR_SkyTraqNMEA::GnssNmeaData g_latest;
bool g_has_gnss = false;
unsigned long g_lastRtcmMs         = 0;
uint16_t      g_lastRtcmBlockBytes = 0;

// Store settings in permanent memory, fall back to defaults only
// if no value exists in preferences memory
Preferences prefs;
uint8_t g_num_battery_cells = config::NUM_BATTERY_CELLS;

// Web UI wrapper
TR_RTKRoverWebUI roverUI(web,
                         g_latest,
                         g_has_gnss,
                         g_lastRtcmMs,
                         g_lastRtcmBlockBytes,
                         g_num_battery_cells);

// ======================= Battery / Neopixel =====================
float voltage = 0.0f;
Adafruit_NeoPixel pixels(1, config::NEO_PIN, NEO_GRB + NEO_KHZ800);

// ======================= Forward Declarations =====================
float readBatteryVoltage();
float estimateSocFromPerCellVoltage(float v_cell);
uint32_t colorFromGnssQuality(uint8_t q);
void showGnssQualityOnNeopixel(uint8_t gnss_quality);

// ======================= SETUP =================================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== RTK Rover (RTCM via RXD2 + Web UI) ===");

    // ADC for voltage
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(config::VOLTAGE_PIN, INPUT);

    // Load saved battery cell count (NVS). If not present, use config.h default.
    prefs.begin("rover", false);  // namespace "rover", RW
    uint8_t saved = prefs.getUChar("cells", 0xFF);
    if (saved >= 1 && saved <= 6)
    {
        g_num_battery_cells = saved;
        Serial.printf("Loaded NUM_BATTERY_CELLS from NVS: %u\n", g_num_battery_cells);
    }
    else
    {
        g_num_battery_cells = config::NUM_BATTERY_CELLS;
        Serial.printf("No saved NUM_BATTERY_CELLS; using config.h default: %u\n", g_num_battery_cells);
    }
    prefs.end();

    // Initial battery read
    float v_batt = readBatteryVoltage();
    float v_cell = v_batt / g_num_battery_cells;
    float soc    = estimateSocFromPerCellVoltage(v_cell);
    voltage = v_batt;
    Serial.print("Battery voltage: ");Serial.println(v_batt);
    Serial.print("State of charge: ");Serial.println(soc);

    // NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.show();

    // GNSS UART (SkyTraq)
    Serial1.begin(115200, SERIAL_8N1, config::GNSS_RX, config::GNSS_TX);
    SerialRTCM.begin(115200, SERIAL_8N1, -1, config::GNSS_RXD2);

    // Reset SkyTraq to defaults
    sky.resetToDefaultsAndCheck();
    delay(2500);

    // Set to rover mode
    sky.configureRtkRoverMode(false);

    // Setup WiFi access point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(config::AP_SSID, config::AP_PASS);
    Serial.print("Rover AP up, IP: ");
    Serial.println(WiFi.softAPIP());

    // RTCM server on WiFi access point
    rtcmServer.begin();
    rtcmServer.setNoDelay(true);
    Serial.print("RTCM server listening on port ");
    Serial.println(config::RTCM_PORT);

    // Start web UI
    roverUI.begin();
    Serial.println("HTTP server started on port 80");
}

// ======================= LOOP ==================================

void loop()
{
    // ---- Handle RTCM TCP server ----
    if (!rtcmClient || !rtcmClient.connected())
    {
        if (rtcmClient)
        {
            Serial.println("Previous RTCM client disconnected.");
            rtcmClient.stop();
        }

        WiFiClient newClient = rtcmServer.available();
        if (newClient)
        {
            Serial.println("RTCM client connected from base:");
            Serial.println(newClient.remoteIP());
            rtcmClient = newClient;
            rtcmClient.setNoDelay(true);
            g_lastRtcmBlockBytes = 0;
            g_lastRtcmMs         = 0;
        }
    }

    // If we have a connected client, forward RTCM bytes to GNSS RXD2 UART
    if (rtcmClient && rtcmClient.connected())
    {
        uint16_t blockCount = 0;

        while (rtcmClient.available())
        {
            int b = rtcmClient.read();
            if (b < 0) break;

            SerialRTCM.write((uint8_t)b);
            g_lastRtcmMs = millis();
            blockCount++;
        }

        if (blockCount > 0) 
        {
            g_lastRtcmBlockBytes = blockCount;
        }
    }

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

        // Periodic battery update + NeoPixel
    static uint32_t lastBatteryMs = 0;
    uint32_t now = millis();

    // Periodic battery update + NeoPixel
    if (now - lastBatteryMs >= config::PWR_UPDATE_PERIOD_MS)
    {
        lastBatteryMs = now;
        voltage = readBatteryVoltage();

        float v_cell = voltage / g_num_battery_cells;
        float soc    = estimateSocFromPerCellVoltage(v_cell);

        // Update NeoPixel based on GNSS quality
        // Use the battery periodic to keep the update reasonable
        // Could use base station methods to update NeoPixel based
        // on battery charge if preferred
        showGnssQualityOnNeopixel(g_latest.gnss_quality_indicator);
    }

    yield();  // keep loop cooperative
}

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

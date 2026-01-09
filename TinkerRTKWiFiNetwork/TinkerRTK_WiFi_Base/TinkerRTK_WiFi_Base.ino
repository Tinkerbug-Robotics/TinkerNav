#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"

// GNSS
#include <TR_SkyTraqNMEA.h>
TR_SkyTraqNMEA sky(Serial1);

// RTCM data
char rtcm_data[config::MAX_RTCM_LENGTH];

// Connect to the WiFi access point on the rover to send RTCM data
WiFiClient rtcmClient;

// Rover AP IP (192.168.4.1 for softAP)
IPAddress roverIP(192,168,4,1);

// LED indicator
Adafruit_NeoPixel pixels(1, config::NEO_PIN, NEO_GRB + NEO_KHZ800);

// ======================= SETUP =================================

void setup()
{
    Serial.begin(115200);
    //while (!Serial){};

    delay(1000);
    Serial.println("\n=== RTK Base Station ===");

    // Configure ADC for reading voltage
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(config::VOLTAGE_PIN, INPUT);
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.show();

    // Read initial voltage and display
    float v_batt = readBatteryVoltage();
    float v_cell = v_batt / config::NUM_BATTERY_CELLS;
    float soc    = estimateSocFromPerCellVoltage(v_cell);
    Serial.print("Vbat = ");
    Serial.print(v_batt, 3);
    Serial.print(" V  (");
    Serial.print(config::NUM_BATTERY_CELLS);
    Serial.print("S, per cell = ");
    Serial.print(v_cell, 3);
    Serial.print(" V)  -> SoC ≈ ");
    Serial.print(soc, 1);
    Serial.println(" %");
    showBatteryOnNeopixel(soc);

    // GNSS NMEA from SkyTraq
    Serial1.begin(115200, SERIAL_8N1, config::GNSS_RX, config::GNSS_TX);

    // Configure SkyTraq

    // Reset to defaults
    Serial.println("Configuring SkyTraq Receiver");
    sky.resetToDefaultsAndCheck();
    delay(2000);
    
    // Put receiver into RTK base + survey mode with defaults
    // 5 min or 2 meter survey in
    Serial.println("    Configuring RTK base mode for survey in ...");
    int command_result = sky.configureRtkBaseSurvey(300,10,false);
    if (command_result != 1)
    {
        Serial.println("    Warning: RTK base survey-in command not accepted");
        Serial.println("    If other commands succesfull, likley because receiver is already in base mode");
    }
    delay(1500);

    Serial.println("    Configuring output ...");
    if (sky.configureRtcmOutput(true,
                                0x00,   // 1 Hz
                                true,   // 1005
                                true,   // GPS MSM
                                true,   // GLO
                                true,   // GAL
                                true,   // BDS
                                0x1E,0x00,0x1E,0x1E,
                                0x01,
                                0x02,
                                false)!= 1)
    {
        Serial.println("    RTCM output configuration failed, stopping");
        while (1) { delay(100); }
    }

    if (!sky.disableStandardNmea())
    {
        Serial.println("    Warning: some NMEA disable commands NACKed or timed out.");
    }

    // Connect to rover's WiFi access point and then RTCM server
    connectWiFi();
    connectRtcmServer();

    Serial.println("Setup complete");
}

// ======================= LOOP ==================================

void loop() 
{

    // Ensure Wi-Fi is still up
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi lost, reconnecting...");
        rtcmClient.stop();
        connectWiFi();
        connectRtcmServer();
    }

    // Ensure TCP is still connected
    if (!rtcmClient.connected())
    {
        Serial.println("RTCM server connection lost, reconnecting...");
        rtcmClient.stop();
        connectRtcmServer();
    }

    processGnssInput();

    static uint32_t last_update = 0;
    if (millis() - last_update >= config::PWR_UPDATE_PERIOD_MS)
    {
        last_update = millis();

        float v_batt = readBatteryVoltage();
        float v_cell = v_batt / config::NUM_BATTERY_CELLS;
        float soc    = estimateSocFromPerCellVoltage(v_cell);

        // Serial.print("Vbat = ");
        // Serial.print(v_batt, 3);
        // Serial.print(" V  (");
        // Serial.print(config::NUM_BATTERY_CELLS);
        // Serial.print("S, per cell = ");
        // Serial.print(v_cell, 3);
        // Serial.print(" V)  -> SoC ≈ ");
        // Serial.print(soc, 1);
        // Serial.println(" %");

        //showBatteryOnNeopixel(soc);
    }

    // Yield to other processes
    yield();
}

void processGnssInput()
{
    // If there's nothing waiting at all, don't block
    if (!Serial1.available())
    {
        return;
    }

    uint32_t lastRx = millis();
    bool gotAny = false;
    uint16_t byte_count = 0;

    while (true)
    {
        while (Serial1.available())
        {
            uint8_t b = Serial1.read();
            gotAny = true;
            lastRx = millis();

            // We no longer care about NMEA here; receiver has NMEA disabled.
            // Just treat everything coming out as RTCM / binary.
            if (byte_count < config::MAX_RTCM_LENGTH)
            {
                rtcm_data[byte_count++] = b;
            }
        }

        // If we never got any bytes, just return quickly
        if (!gotAny)
        {
            return;
        }

        // Check for quiet period
        if (millis() - lastRx >= 50)
        {
            // No new bytes for 50 ms → consider this burst done and send data
            if (byte_count > 0 && rtcmClient.connected())
            {
                rtcmClient.write((const uint8_t*)rtcm_data, byte_count);
                Serial.print("Sending burst of ");
                Serial.print(byte_count);
                Serial.println(" bytes");
            }
            break;
        }

        // Yield to WiFi/other tasks
        yield();
    }
}

// Connect to rover WiFi access point
void connectWiFi() 
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(config::AP_SSID, config::AP_PASS);
    Serial.print("Connecting to rover access point");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
    Serial.print("Connected, IP: "); Serial.println(WiFi.localIP());
}

bool connectRtcmServer() 
{
    Serial.print("Connecting to RTCM server on rover ...");
    if (!rtcmClient.connect(roverIP, config::RTCM_PORT)) 
    {
        Serial.println(" failed");
        return false;
    }
    rtcmClient.setNoDelay(true);
    Serial.println(" OK");
    return true;
}

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
// We clamp and do a simple linear approximation between 3.3 and 4.2.
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

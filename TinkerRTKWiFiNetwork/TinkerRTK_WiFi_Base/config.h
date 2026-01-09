#ifndef CONFIG_H
#define CONFIG_H

struct config 
{

    // NMEA from SkyTraq
    static constexpr uint8_t GNSS_TX = 47;
    static constexpr uint8_t GNSS_RX = 48;

    // Maximum length of a set of RTCM data
    static constexpr uint16_t MAX_RTCM_LENGTH = 2500;

    static constexpr char* AP_SSID = "tinkerbug_rover";
    static constexpr char* AP_PASS = "Tinkerbug";

    static constexpr uint16_t RTCM_PORT = 2101;

    // Voltage reading
    static constexpr int VOLTAGE_PIN = 8;     // ADC input
    static constexpr float R1 = 220000.0;     // 220kΩ
    static constexpr float R2 = 100000.0;     // 100kΩ
    static constexpr float VREF = 3.3;        // ADC reference voltage
    static constexpr int ADC_MAX = 4095;      // 12-bit ADC on ESP32-S3

    // Neopix LED driver pin
    static constexpr int NEO_PIN = 26;

    // Battery type 
    static constexpr int NUM_BATTERY_CELLS = 2;

    // Power update period (ms)
    static constexpr int PWR_UPDATE_PERIOD_MS = 2000;


};

#endif // CONFIG_H

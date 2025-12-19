#ifndef CONFIG_H
#define CONFIG_H

struct config 
{
    // Access point name and password
    inline static constexpr const char* AP_SSID = "tinkerbug_rover";
    inline static constexpr const char* AP_PASS = "Tinkerbug";

    // NMEA from SkyTraq
    static constexpr uint8_t GNSS_TX = 35;
    static constexpr uint8_t GNSS_RX = 36;

    //  LoRa Radio Pins (SX1262)
    static constexpr uint8_t  L_DIO1 = 44;
    static constexpr uint8_t  L_CS   = 7;
    static constexpr uint8_t  L_BUSY = 43;
    static constexpr uint8_t  L_RST  = 6;
    static constexpr uint8_t  L_MISO = 2;
    static constexpr uint8_t  L_MOSI = 4;
    static constexpr uint8_t  L_SCK  = 5;

    // Neopix LED Driver Pin
    static constexpr int NEO_PIN = 26;

    // LoRa Radio Configuration
    static constexpr float   FREQUENCY        = 905.0;
    static constexpr float   BANDWIDTH        = 250.0;
    static constexpr uint8_t SPREADING_FACTOR = 8;
    static constexpr uint8_t CODING_RATE      = 5;
    static constexpr uint8_t OUTPUT_POWER     = 22;
    static constexpr uint8_t PREAMBLE_LENGTH  = 12;

    // LoRa Data
    static constexpr int TX_QUEUE_SIZE            = 8;
    static constexpr uint16_t MAX_RTCM_LENGTH     = 2000;
    static constexpr int MAX_PACKET_LENGTH        = 250;
    static constexpr size_t RTCM_BYTE_QUEUE_SIZE  = 5000;
    static constexpr size_t RTCM_MSG_QUEUE_SIZE   = 100;
    static const uint32_t STATUS_PERIOD_MS        = 2000;
    static const uint32_t RTCM_ACCEPT_INTERVAL_MS = 1000;

    // Skytraq Survey-in Configuration (ints)
    static constexpr uint32_t SURVEY_IN_SEC = 120;
    static constexpr uint32_t SURVEY_IN_M = 10;

    // Voltage Reading
    static constexpr int NUM_BATTERY_CELLS = 2; // Number of cells in battery
    static constexpr int VOLTAGE_PIN = 8;       // ADC input pin
    static constexpr float R1 = 220000.0;       // 220kΩ
    static constexpr float R2 = 100000.0;       // 100kΩ
    static constexpr float VREF = 3.3;          // ADC reference voltage
    static constexpr int ADC_MAX = 4095;        // 12-bit ADC on ESP32-S3

    // Power Update Period (ms)
    static constexpr int PWR_UPDATE_PERIOD_MS = 2000;

};

#endif // CONFIG_H

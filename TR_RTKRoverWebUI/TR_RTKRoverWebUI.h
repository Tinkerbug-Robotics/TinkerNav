// Copyright (c) 2026 Tinkerbug Robotics
//
// MIT License
//

#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TR_SkyTraqNMEA.h>

class TR_RTKRoverWebUI
{
public:

    // Constructor
    TR_RTKRoverWebUI(WebServer& server,
                     TR_SkyTraqNMEA::GnssNmeaData& latest,
                     bool& hasGnss,
                     unsigned long& lastRtcmMs,
                     uint16_t& lastRtcmBlockBytes,
                     uint8_t& numBatteryCellsRuntime);

    // Register routes and start the HTTP server
    void begin();

    // Call this from loop()
    void loop();

private:

    WebServer& web_;
    TR_SkyTraqNMEA::GnssNmeaData& g_latest_;
    bool& g_has_gnss_;
    unsigned long& g_lastRtcmMs_;
    uint16_t& g_lastRtcmBlockBytes_;
    uint8_t& num_battery_cells_;

    // Internal helpers
    const char* rtkModeFromQuality(uint8_t q);

    void handleRoot();
    void handleGnssJson();
    void handleSetCells();
    void handleNotFound();
    
};

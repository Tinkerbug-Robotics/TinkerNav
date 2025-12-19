// Copyright (c) 2025 Tinkerbug Robotics
//
// MIT License
//
// Library to interact with SkyTraq receiver for programming and
// reading


#pragma once
#include <Arduino.h>


class TR_SkyTraqNMEA 
{

public:

    // Constructor
    explicit TR_SkyTraqNMEA(HardwareSerial& s);

    // --- Skytraq Configuration Helpers ---

    // Configure RTK mode to BASE + SURVEY operational function.
    // Used for the base station in a base station/rove pair.
    // surveyLen_s: survey length in seconds 
    // stdDev_m:    standard deviation threshold in meters 
    // saveToFlash: false = SRAM only, true = SRAM + Flash
    // Returns true if ACKed, false on NACK/timeout.
    bool configureRtkBaseSurvey(uint32_t surveyLen_s,
                                uint32_t stdDev_m,
                                bool saveToFlash);

    // Restore defaults + Cold restart
    bool resetToDefaultsAndCheck();
    
    // Set update rate (valid range is receiver dependent)
    bool setUpdateRateHz(uint8_t hz);
    
    // Enable/disable a specific sentence type GGA/GSA/RMC/etc.
    bool setNmeaSentence(const char three[4], bool on);
  
    // Disable a group of common NMEA types
    bool disableStandardNmea();
  
    // Configure RTCM message output 
    bool configureRtcmOutput(bool     enableRtcm,
                             uint8_t  msmRateCode,
                             bool     enable1005,
                             bool     enableGpsMsm,
                             bool     enableGloMsm,
                             bool     enableGalMsm,
                             bool     enableBdsMsm,
                             uint8_t  gpsEphRate,
                             uint8_t  gloEphRate,
                             uint8_t  bdsEphRate,
                             uint8_t  galEphRate,
                             uint8_t  msmType,
                             uint8_t  version,
                             bool     saveToFlash);
      
    // Configure receiver as RTK rover
    // saveToFlash = true to store in flash, false = SRAM only
    bool configureRtkRoverMode(bool saveToFlash);
  
    // Feed characters to parser 
    void feedChar(char c);
        
    // Structure of data to extract from NMEA reads
    struct GnssNmeaData 
    {
        // Maximum number of satellites considered
        static const int MAX_SATS = 64;

        // GNSS data
        uint32_t time_stamp;
        float latitude;
        float longitude;
        float altitude;
        uint8_t gnss_quality_indicator;
        float   hdop;
        
        // Velocity (from RMC / PSTI,030)
        double vel_e = NAN;
        double vel_n = NAN;
        double vel_u = NAN;
        
        // RTK data
        float   rtk_age;
        float   rtk_ratio;

        // Date/time
        int      year        = 0;
        uint8_t  month       = 0;
        uint8_t  day         = 0;
        uint8_t  hour        = 0;
        uint8_t  minute      = 0;
        uint8_t  second      = 0;
        uint16_t millisecond = 0;

        // Constellation enum (used by JSON builder)
        enum Constellation : uint8_t
        {
            CONST_UNKNOWN = 0,
            CONST_GPS     = 1,
            CONST_GAL     = 2,
            CONST_BDS     = 3,
            CONST_GLO     = 4
        };

        // Number of satellites used in solution (from GGA)
        uint8_t num_sats = 0;
        
        // Number of satellites in view
        uint8_t sats_in_view = 0;

        // --- By Satellite Data ---
        // PRN / SVID
        uint8_t sat_prn[MAX_SATS] = {0};
        
        // SNR in dB
        float sat_snr[MAX_SATS] = {0}; 
        
        // True if PRN appears in GSA
        bool sat_used[MAX_SATS] = {false};
        
        // Satellite constellation: 
        // 0=unknown,1=GPS,2=GAL,3=BDS,4=GLO
        uint8_t sat_constellation[MAX_SATS] = {CONST_UNKNOWN};
        
        // milli seconds when satellite was last seen in GSV
        uint32_t sat_last_ms[MAX_SATS] = {0};
        
        // Elevation (deg, 0..90) from GSV
        float sat_elev_deg[MAX_SATS] = {0};

        // Azimuth (deg, 0..359) from GSV
        float sat_az_deg[MAX_SATS]   = {0};
    };

    // Skytraq return enum
    enum CmdResult : int {CR_TIMEOUT = -1, CR_NACK = 0, CR_ACK = 1};

    // Try to fetch a copy of the latest coherent data; 
    // returns true if updated since last call
    bool getData(GnssNmeaData &out) 
    {
        bool changed = false;
        noInterrupts();
        if (have_new_) 
        {
            out = data_;
            have_new_ = false;
            changed = true;
        }
        interrupts();
        return changed;
    }
  
private:

    // Parsing constants and data
    static constexpr size_t kLINE_BUF_MAX = 192;
    char   line_[kLINE_BUF_MAX];
    size_t li_ = 0;

    // Current data and flags
    volatile bool have_new_ = false;
    GnssNmeaData  data_{};

    // Serial port used
    HardwareSerial& port_;

    // --- Binary helpers ---
    void drain(uint32_t ms);
    void sendFrame(const uint8_t* payload, 
                      uint16_t len);
                      
    int readSkyFrame(uint8_t* out, 
                     uint16_t& out_len,
                     uint32_t timeout_ms);
                     
    // Wait for response and return result
    CmdResult waitAckOrNack(uint8_t expect_id, 
                            int expect_subid,
                            uint32_t timeout_ms);

    // --- NMEA helpers ---
    static bool verifyChecksum(const char* s, 
                               size_t n);
    void handleLine(char* s, 
                    size_t n);
    static int splitCSV(char* s, 
                        const char* fields[],
                        int max_fields);
    static double nmeaToDecimal(const char* ddmm_mmmm, 
                                char hemi);

    // Specific sentence parsers
    void parseGGA(const char* const* f, int nf);
    void parseRMC(const char* const* f, int nf);
    void parseGSA(const char* const* f, int nf);
    void parseGSV(const char* const* f, int nf);
    void parsePSTI030(const char* const* f, int nf);
    void parsePSTI032(const char* const* f, int nf);
  
    // Helper for saving satellite data
    int findOrAllocSatSlot(uint8_t prn);
    
};
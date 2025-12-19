// Copyright (c) 2025 Tinkerbug Robotics
//
// MIT License
//

#include "TR_SkyTraqNMEA.h"
#include <ctype.h>
#include <math.h>
#include <string.h>

TR_SkyTraqNMEA::TR_SkyTraqNMEA(HardwareSerial& serial)
                : port_(serial)
{
    memset(&data_, 0, sizeof(data_));
}

void TR_SkyTraqNMEA::drain(uint32_t ms)
{
    uint32_t t0 = millis();
    while (millis() - t0 < ms)
    {
        while (port_.available()) (void)port_.read();
        delay(1);
    }
}

void TR_SkyTraqNMEA::sendFrame(const uint8_t* payload, 
                               uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; ++i)
    {
        cs ^= payload[i];
    }

    uint8_t hdr[4] = { 0xA0, 0xA1, (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
    port_.write(hdr, 4);
    port_.write(payload, len);
    port_.write(cs);
    port_.write((uint8_t)0x0D);
    port_.write((uint8_t)0x0A);
}

int TR_SkyTraqNMEA::readSkyFrame(uint8_t* out, 
                                 uint16_t& out_len, 
                                 uint32_t timeout_ms)
{
    enum { WA, WB, LH, LL, PAY, CS, CR, LF } st = WA;
    uint16_t need = 0, idx = 0;
    uint8_t cs = 0;
    uint32_t t0 = millis();

    for (;;)
    {
        if (millis() - t0 > timeout_ms)
        {
            return -1;
        }
        if (!port_.available())
        {
            delay(1);
            continue;
        }

        uint8_t b = (uint8_t)port_.read();

        // Skip NMEA text while hunting for binary frames
        if (b == '$')
        {
            // eat until newline
            while (millis() - t0 <= timeout_ms)
            {
                if (!port_.available())
                {
                    delay(1);
                    continue;
                }
                if ((uint8_t)port_.read() == '\n')
                {
                    break;
                }
            }
            continue;
        }

        switch (st) 
        {
            case WA:
                st = (b == 0xA0) ? WB : WA;
                break;
            case WB:
                st = (b == 0xA1) ? LH : WA;
                break;
            case LH:
                need = ((uint16_t)b) << 8;
                st = LL;
                break;
            case LL:
                need |= b;
                idx = 0;
                cs = 0;
                if (need > 512) return -2;
                st = PAY;
                break;
            case PAY:
                out[idx++] = b;
                cs ^= b;
                if (idx >= need)
                {
                    st = CS;
                }
                break;
            case CS:
                st = CR;
                break;
            case CR:
                st = (b == 0x0D) ? LF : WA;
                break;
            case LF:
            if (b == 0x0A)
            {
                out_len = need;
                
                // Message ID
                return (int)out[0];
            }
            st = WA;
            break;
        }
    }
}

TR_SkyTraqNMEA::CmdResult
TR_SkyTraqNMEA::waitAckOrNack(uint8_t expect_id, 
                              int expect_subid, uint32_t timeout_ms)
{
    uint8_t frame[512];
    uint16_t flen = 0;
    int mid = 0;
    uint32_t t_end = millis() + timeout_ms;

    for (;;) 
    {
        if ((int32_t)(t_end - millis()) <= 0)
        {
            return CR_TIMEOUT;
        }

        mid = readSkyFrame(frame, flen, timeout_ms);
        if (mid < 0)
        {
            return CR_TIMEOUT;
        }

        if (mid == 0x83 || mid == 0x84)
        {
            bool id_ok  = (flen >= 2 && frame[1] == expect_id);
            bool sub_ok = (expect_subid < 0) || (flen >= 3 && frame[2] == (uint8_t)expect_subid);
            if (!id_ok || !sub_ok)
            {
                continue;
            }
            return (mid == 0x83) ? CR_ACK : CR_NACK;
        }
    }
}

// Reset receiver to default settings and check response
bool TR_SkyTraqNMEA::resetToDefaultsAndCheck()
{
    drain(30);

    const uint8_t restore[] = { 0x64, 0x1E };
    Serial.println(F("Sending Restore Defaults..."));
    sendFrame(restore, sizeof(restore));
    CmdResult r1 = waitAckOrNack(0x64, 0x1E, 1200);
    Serial.printf("Restore Defaults result: %s\n",
                  (r1 == CR_ACK ? "ACK" : (r1 == CR_NACK ? "NACK" : "TIMEOUT")));

    const uint8_t restartCold[] = { 0x04, 0x02 };
    Serial.println(F("Sending Cold Restart..."));
    sendFrame(restartCold, sizeof(restartCold));
    CmdResult r2 = waitAckOrNack(0x04, -1, 1500);
    Serial.printf("Cold Restart result: %s\n",
                  (r2 == CR_ACK ? "ACK" : (r2 == CR_NACK ? "NACK" : "TIMEOUT")));

    return (r1 == CR_ACK) || (r2 == CR_ACK);
}

bool TR_SkyTraqNMEA::setUpdateRateHz(uint8_t hz)
{
    uint8_t p[] = { 0x0E, hz, 0x01 };
    Serial.printf("Set UpdateRate=%u Hz...\n", hz);
    sendFrame(p, sizeof(p));
    CmdResult r = waitAckOrNack(0x0E, -1, 1000);
    Serial.printf("UpdateRate result: %s\n",
                  (r == CR_ACK ? "ACK" : (r == CR_NACK ? "NACK" : "TIMEOUT")));
    return (r == CR_ACK) || (r == CR_NACK);
}

bool TR_SkyTraqNMEA::setNmeaSentence(const char three[4], bool on)
{
    uint8_t p[] =
    {
        0x64, 0x3B,
        (uint8_t)three[0],
        (uint8_t)three[1],
        (uint8_t)three[2],
        (uint8_t)(on ? 0x01 : 0x00),
        0x01
    };

    Serial.printf("NMEA %c%c%c %s...\n",
                  three[0], three[1], three[2],
                  on ? "on" : "off");
    sendFrame(p, sizeof(p));
    CmdResult r = waitAckOrNack(0x64, 0x3B, 1000);
    Serial.printf("NMEA %c%c%c result: %s\n",
                  three[0], three[1], three[2],
                  (r == CR_ACK ? "ACK" : (r == CR_NACK ? "NACK" : "TIMEOUT")));
    return (r == CR_ACK) || (r == CR_NACK);
}

bool TR_SkyTraqNMEA::disableStandardNmea()
{
    bool ok = true;

    ok &= setNmeaSentence("GGA", false);
    ok &= setNmeaSentence("GSA", false);
    ok &= setNmeaSentence("GSV", false);
    ok &= setNmeaSentence("RMC", false);
    ok &= setNmeaSentence("VTG", false);
    ok &= setNmeaSentence("GLL", false);

    Serial.printf("disableStandardNmea overall: %s\n",
                  ok ? "OK" : "SOME NACK/TIMEOUT");
    return ok;
}

//  NMEA parsing 

void TR_SkyTraqNMEA::feedChar(char c)
{
    if (c == '\r')
    {
        return;
    }

    if (c == '\n')
    {
        if (li_ > 0)
        {
            if (li_ >= kLINE_BUF_MAX)
            {
                li_ = kLINE_BUF_MAX - 1;
            }
            line_[li_] = '\0';
            handleLine(line_, li_);
            li_ = 0;
        }
        return;
    }

    if (li_ < kLINE_BUF_MAX - 1)
    {
        line_[li_++] = c;
    }
    else
    {
        // Overflow: reset softly
        li_ = 0;
    }
}

bool TR_SkyTraqNMEA::verifyChecksum(const char* s, size_t n)
{
    if (n < 4 || !s || s[0] != '$')
    {
        return false;
    }

    const char* star = strrchr(s, '*');
    if (!star || (star - s) < 3 || (size_t)(star - s) >= n - 2)
    {
        return false;
    }

    uint8_t cs = 0;
    for (const char* p = s + 1; p < star; ++p)
    {
        cs ^= (uint8_t)(*p);
    }

    auto hx = [](char h) -> int
    {
        if (h >= '0' && h <= '9') return h - '0';
        if (h >= 'A' && h <= 'F') return h - 'A' + 10;
        if (h >= 'a' && h <= 'f') return h - 'a' + 10;
        return -1;
    };

    int hi = hx(star[1]);
    int lo = hx(star[2]);
    if (hi < 0 || lo < 0)
    {
        return false;
    }

    int want = (hi << 4) | lo;
    return want == (int)cs;
}

int TR_SkyTraqNMEA::splitCSV(char* s,
                            const char* fields[],
                            int max_fields)
{
    int n = 0;
    if (!s || !*s || !fields || max_fields <= 0)
    {
        return 0;
    }

    fields[n++] = s;

    for (char* p = s; *p; ++p)
    {
        if (*p == ',' || *p == '*')
        {
            *p = '\0';
            if (n < max_fields)
            {
                fields[n++] = p + 1;
            }

            if (*(p + 1) == 0 || *(p + 1) == '\r' || *(p + 1) == '\n')
            {
                break;
            }
        }
    }

    return n;
}

double TR_SkyTraqNMEA::nmeaToDecimal(const char* ddmm_mmmm, char hemi)
{
    if (!ddmm_mmmm || !*ddmm_mmmm)
    {
        return NAN;
    }

    const char* dot = strchr(ddmm_mmmm, '.');
    const char* end = ddmm_mmmm + strlen(ddmm_mmmm);
    if (!dot)
    {
        dot = end;
    }

    if (dot - ddmm_mmmm < 3)
    {
        return NAN;
    }

    const char* min_start = dot - 2;

    char degbuf[8];
    size_t deglen = (size_t)(min_start - ddmm_mmmm);
    if (deglen >= sizeof(degbuf))
    {
        return NAN;
    }

    memcpy(degbuf, ddmm_mmmm, deglen);
    degbuf[deglen] = '\0';

    double deg  = strtod(degbuf, nullptr);
    double mins = strtod(min_start, nullptr);
    double dec  = deg + mins / 60.0;

    if (hemi == 'S' || hemi == 'W')
    {
        dec = -dec;
    }

    return dec;
}

void TR_SkyTraqNMEA::handleLine(char* s, size_t n)
{
    if (!s || n < 6 || s[0] != '$')
    {
        return;
    }

    if (!verifyChecksum(s, n))
    {
        return;
    }

    const char* f[32];
    int nf = splitCSV(s, f, 32);
    if (nf <= 0)
    {
        return;
    }

    const char* head = f[0];
    size_t headLen = strlen(head);

    auto endsWith = [&](const char* suf) -> bool
    {
        size_t ls = strlen(suf);
        return (headLen >= ls) && (strcmp(head + (headLen - ls), suf) == 0);
    };

    if (endsWith("GGA"))
    {
        parseGGA(f, nf);
    }
    else if (endsWith("RMC"))
    {
        parseRMC(f, nf);
    }
    else if (endsWith("GSV"))
    {
        parseGSV(f, nf);
    }
    else if (endsWith("GSA"))
    {
        parseGSA(f, nf);
    }
    else if (strncmp(head, "$PSTI", 5) == 0 && nf >= 2)
    {
        if (strcmp(f[1], "032") == 0)
        {
            parsePSTI032(f, nf);
        }
        else if (strcmp(f[1], "030") == 0)
        {
            parsePSTI030(f, nf);
        }
    }
}

// ===================== Specific sentence parsers =====================

// GGA: position, fix quality, altitude — authoritative for lat/lon/alt
void TR_SkyTraqNMEA::parseGGA(const char* const* f, int nf)
{
    // $--GGA,UTC,lat,N/S,lon,E/W,fix,numsats,hdop,alt,M,geoid,M,age,ref
    if (nf < 11)
    {
        return;
    }

    if (f[2][0] && f[3][0] && f[4][0] && f[5][0])
    {
        double lat = nmeaToDecimal(f[2], f[3][0]);
        double lon = nmeaToDecimal(f[4], f[5][0]);
        if (isfinite(lat) && isfinite(lon))
        {
            data_.latitude  = (float)lat;
            data_.longitude = (float)lon;
        }
    }

    if (f[6][0])
    {
        int q = atoi(f[6]);
        if (q < 0) q = 0;
        if (q > 8) q = 8;
        data_.gnss_quality_indicator = (uint8_t)q;
    }

    if (nf > 7 && f[7][0])
    {
        int ns = atoi(f[7]);
        if (ns < 0)  ns = 0;
        if (ns > 99) ns = 99;
        data_.num_sats = (uint8_t)ns;
    }

    if (nf > 8 && f[8][0])
    {
        double h = strtod(f[8], nullptr);
        if (isfinite(h))
        {
            data_.hdop = (float)h;
        }
    }

    if (f[9][0])
    {
        double alt = strtod(f[9], nullptr);
        if (isfinite(alt))
        {
            data_.altitude = (float)alt;
        }
    }

    data_.time_stamp = micros();
    have_new_ = true;
}

// RMC: time, date, status, speed, course.
// NOTE: DO NOT override lat/lon/alt here; GGA is the authority.
void TR_SkyTraqNMEA::parseRMC(const char* const* f, int nf)
{
    if (nf < 10)
    {
        return;
    }

    const char* utc  = f[1];
    const char* stat = f[2];
    const char* date = f[9];

    bool valid = (stat && stat[0] == 'A');

    if (utc && utc[0] && date && date[0] && valid)
    {
        int hh = 0, mm = 0, ss = 0, ms = 0;
        if (strlen(utc) >= 6)
        {
            hh = (utc[0] - '0') * 10 + (utc[1] - '0');
            mm = (utc[2] - '0') * 10 + (utc[3] - '0');
            ss = (utc[4] - '0') * 10 + (utc[5] - '0');

            const char* dot = strchr(utc, '.');
            if (dot && isdigit((unsigned char)dot[1]))
            {
                int ms_val = 0;
                int mul = 100;
                for (const char* p = dot + 1; *p && mul > 0 && isdigit((unsigned char)*p); ++p)
                {
                    ms_val += (*p - '0') * mul;
                    mul /= 10;
                }
                ms = ms_val;
            }
        }

        int dd = 0, mon = 0, yy = 0;
        if (strlen(date) == 6 &&
            isdigit((unsigned char)date[0]) &&
            isdigit((unsigned char)date[1]) &&
            isdigit((unsigned char)date[2]) &&
            isdigit((unsigned char)date[3]) &&
            isdigit((unsigned char)date[4]) &&
            isdigit((unsigned char)date[5]))
        {
            dd  = (date[0] - '0') * 10 + (date[1] - '0');
            mon = (date[2] - '0') * 10 + (date[3] - '0');
            yy  = (date[4] - '0') * 10 + (date[5] - '0');

            int fullYear = (yy >= 80) ? (1900 + yy) : (2000 + yy);

            data_.year        = fullYear;
            data_.month       = mon;
            data_.day         = dd;
            data_.hour        = hh;
            data_.minute      = mm;
            data_.second      = ss;
            data_.millisecond = ms;
        }
    }

    data_.time_stamp = micros();
    have_new_ = true;
}

void TR_SkyTraqNMEA::parsePSTI032(const char* const* f, int nf)
{
    if (nf > 14)
    {
        if (f[13][0])
        {
            double age = strtod(f[13], nullptr);
            if (isfinite(age)) data_.rtk_age = (float)age;
        }
        if (f[14][0])
        {
            double ratio = strtod(f[14], nullptr);
            if (isfinite(ratio)) data_.rtk_ratio = (float)ratio;
        }

        data_.time_stamp = micros();
        have_new_ = true;
    }
}

// PSTI,030: Recommended Minimum 3D GNSS Data for SkyTraq.
//   0: "$PSTI"
//   1: "030"
//   2: UTC time
//   3: status / fix type
//   4: latitude
//   5: N/S
//   6: longitude
//   7: E/W
//   8: altitude (m)
//   9: vel East (m/s)
//  10: vel North (m/s)
//  11: vel Up (m/s)
//  12: date
//  13: quality / mode
//  14: RTK age (s)
//  15: RTK ratio
//
void TR_SkyTraqNMEA::parsePSTI030(const char* const* f, int nf)
{
    if (nf < 16)
    {
        return;
    }

    if (f[9][0])
    {
        double ve = strtod(f[9], nullptr);
        if (isfinite(ve)) data_.vel_e = (float)ve;
    }
    if (f[10][0])
    {
        double vn = strtod(f[10], nullptr);
        if (isfinite(vn)) data_.vel_n = (float)vn;
    }
    if (f[11][0])
    {
        double vu = strtod(f[11], nullptr);
        if (isfinite(vu)) data_.vel_u = (float)vu;
    }

    if (f[14][0])
    {
        double age = strtod(f[14], nullptr);
        if (isfinite(age)) data_.rtk_age = (float)age;
    }
    if (f[15][0])
    {
        double ratio = strtod(f[15], nullptr);
        if (isfinite(ratio)) data_.rtk_ratio = (float)ratio;
    }

    data_.time_stamp = micros();
    have_new_ = true;
}

// Parse GSV (satellites in view) from $G?GSV sentences.
// head is the 5-char NMEA header, e.g. "$GPGSV", "$GBGSV",
// "$GAGSV", "$GLGSV", "$GNGSV".
void TR_SkyTraqNMEA::parseGSV(const char* const* f, int nf)
{
    // f[0] = "$GPGSV", "$GBGSV", "$GAGSV", etc.
    if (nf < 4)
    {
        return;
    }

    const char* head = f[0];

    // Determine constellation from talker ID (2 chars after '$G')
    uint8_t thisConst = GnssNmeaData::CONST_UNKNOWN;
    if (head && head[0] == '$')
    {
        char t1 = head[1];
        char t2 = head[2];

        // Common patterns:
        //  GP = GPS
        //  GA = Galileo
        //  GB / BD = BeiDou (SkyTraq can use GB or BD)
        //  GL = GLONASS
        //  GN = mixed GNSS (we leave as UNKNOWN here)
        if (t1 == 'G' && t2 == 'P') 
        {
            thisConst = GnssNmeaData::CONST_GPS;
        } 
        else if (t1 == 'G' && t2 == 'A') 
        {
            thisConst = GnssNmeaData::CONST_GAL;
        } 
        else if ((t1 == 'G' && t2 == 'B') ||
                 (t1 == 'B' && t2 == 'D')) 
        {
            thisConst = GnssNmeaData::CONST_BDS;
        } 
        else if (t1 == 'G' && t2 == 'L') 
        {
            thisConst = GnssNmeaData::CONST_GLO;
        } 
        else 
        {
            // GN or other: leave CONST_UNKNOWN for now
            thisConst = GnssNmeaData::CONST_UNKNOWN;
        }
    }

    uint32_t now_ms = millis();

    // We do NOT trust f[3] directly for "sats_in_view" because 
    // it is per constellation. Instead, we compute sats_in_view 
    // below as the count of distinct PRNs with a recent GSV update.

    // Process each group of 4 fields
    for (int base = 4; base + 3 < nf; base += 4)
    {
        const char* prnStr = f[base + 0];
        const char* elStr  = f[base + 1];
        const char* azStr  = f[base + 2];
        const char* snrStr = f[base + 3];

        if (!prnStr || !prnStr[0])
        {
            continue;
        }

        int prnVal = atoi(prnStr);
        if (prnVal <= 0 || prnVal > 255)
        {
          continue;
        }

        float elVal = 0.0f;
        float azVal = 0.0f;
        float snrVal = 0.0f;

        if (elStr && elStr[0]) elVal = (float)atof(elStr);
        if (azStr && azStr[0]) azVal = (float)atof(azStr);
        // Clamp elevtion and azimuth to reasonable values
        elVal = std::max(0.0f, std::min(90.0f, elVal));
        azVal = fmodf(azVal, 360.0f);
        if (azVal < 0.0f) azVal += 360.0f;
        
        if (snrStr && snrStr[0]) snrVal = (float)atof(snrStr);

        int slot = findOrAllocSatSlot((uint8_t)prnVal);
        if (slot < 0)
        {
            // no room left
            continue;
        }

        data_.sat_snr[slot]           = snrVal;
        data_.sat_constellation[slot] = thisConst;
        data_.sat_elev_deg[slot]      = elVal;
        data_.sat_az_deg[slot]        = azVal;
        data_.sat_last_ms[slot]       = now_ms;
        // sat_used[] is handled by GSA; leave as-is here
    }

    // Recompute sats_in_view as "satellites seen in last 
    // few seconds" so we don't count long-gone PRNs.
    const uint32_t MAX_AGE_MS = 5000; // 5 seconds
    uint8_t count = 0;
    for (int i = 0; i < GnssNmeaData::MAX_SATS; ++i) 
    {
        if (data_.sat_prn[i] == 0) 
        {
            continue;
        }
        if ((now_ms - data_.sat_last_ms[i]) <= MAX_AGE_MS)
        {
            ++count;
        }
    }
    data_.sats_in_view = count;
}

void TR_SkyTraqNMEA::parseGSA(const char* const* f, int nf)
{
  // GSA layout:
  //  f[0] = "$GPGSA" / "$GNGSA" etc.
  //  f[1] = mode (M/A)
  //  f[2] = fix type
  //  f[3..14] = up to 12 PRNs used in solution
  //  f[15] = PDOP, f[16] = HDOP, f[17] = VDOP

  if (nf < 4) {
    return;
  }

  // First, clear "used" flags; they will be re-set below
  for (int i = 0; i < GnssNmeaData::MAX_SATS; ++i) {
    data_.sat_used[i] = false;
  }

  for (int i = 3; i <= 14 && i < nf; ++i) {
    const char* prnStr = f[i];
    if (!prnStr || !prnStr[0]) {
      continue;
    }

    int prnVal = atoi(prnStr);
    if (prnVal <= 0 || prnVal > 255) {
      continue;
    }

    // Find existing slot only; GSA shouldn't create new satellites
    for (int j = 0; j < GnssNmeaData::MAX_SATS; ++j) {
      if (data_.sat_prn[j] == prnVal) {
        data_.sat_used[j] = true;
        break;
      }
    }
  }
}

// ---------------------------------------------------------
// Configure RTK base mode with Survey operational function
// (ID: 0x6A, SID: 0x06)
// ---------------------------------------------------------
//
// SkyTraq AN0037 "Configure RTK Mode and Operational Function" notes:
//  - Message body length: 37 bytes
//  - Field 1: RTK Mode
//        0 = RTK rover mode
//        1 = RTK base mode
//        2 = RTK precisely kinematic base mode
//  - Field 2: RTK Operational Function (when RTK Mode = base)
//        0 = Kinematic
//        1 = Survey
//        2 = Static
//  - Fields 3–6 : Saved Survey Length (UINT32, seconds, big endian)
//  - Fields 7–10: Standard Deviation (UINT32, meters, big endian)
//  - Fields 11–36: latitude/longitude/alt etc. for static mode (we leave 0)
//  - Field 37: Attributes
//        0 = update SRAM only
//        1 = update SRAM + Flash
//
// We send payload = [0x6A, 0x06, body(37 bytes)] to sendFrame(),
// and then wait for ACK/NACK on 0x6A / 0x06.
//
bool TR_SkyTraqNMEA::configureRtkBaseSurvey(uint32_t surveyLen_s,
                                            uint32_t stdDev_m,
                                            bool     saveToFlash)
{
  // Clamp per SkyTraq doc (defensive)
  if (surveyLen_s < 60)        surveyLen_s = 60;       // at least 60 s
  if (surveyLen_s > 1209600)   surveyLen_s = 1209600;  // 14 days

  if (stdDev_m < 3)            stdDev_m = 3;           // 3 m
  if (stdDev_m > 100)          stdDev_m = 100;         // 100 m

  // Total payload length PL must be 37:
  //  2 bytes: ID (0x6A), SubID (0x06)
  // 35 bytes: body
  uint8_t payload[37];
  memset(payload, 0, sizeof(payload));

  // Message ID / Sub-ID
  payload[0] = 0x6A;   // ID
  payload[1] = 0x06;   // Sub-ID: Configure RTK Mode & Operational Function

  // ---- Body starts at payload[2] ----
  uint8_t* body = &payload[2];

  // Field 1: RTK Mode = 1 (RTK base mode)
  body[0] = 0x01;

  // Field 2: RTK Operational Function (base mode)
  // 1 = Survey
  body[1] = 0x01;

  // Field 3–6: Survey Length (UINT32, seconds, big-endian)
  body[2] = (surveyLen_s >> 24) & 0xFF;
  body[3] = (surveyLen_s >> 16) & 0xFF;
  body[4] = (surveyLen_s >>  8) & 0xFF;
  body[5] = (surveyLen_s      ) & 0xFF;

  // Field 7–10: Standard Deviation (UINT32, meters, big-endian)
  uint32_t stdDev_u32 = stdDev_m;
  body[6]  = (stdDev_u32 >> 24) & 0xFF;
  body[7]  = (stdDev_u32 >> 16) & 0xFF;
  body[8]  = (stdDev_u32 >>  8) & 0xFF;
  body[9]  = (stdDev_u32      ) & 0xFF;

  // Fields 11–36:
  //   Latitude (8 bytes), Longitude (8 bytes), Altitude (4), Baseline constraint (4)
  // For Survey base mode, these can remain 0 and will be filled by the receiver
  // during/after survey. We already zeroed the whole payload with memset(),
  // so body[10]..body[33] are all 0.

  // Field 37: Attributes (byte index 34 in body)
  //   0 = update SRAM only
  //   1 = update SRAM + Flash
  body[34] = saveToFlash ? 0x01 : 0x00;

  // Send the frame
  sendFrame(payload, sizeof(payload));

  // Wait for ACK or NACK for (ID 0x6A, Sub-ID 0x06)
  CmdResult r = waitAckOrNack(0x6A, 0x06, 2000);
  Serial.printf("configureRtkBaseSurvey: result=%d\n", (int)r);

  return (r == CR_ACK);
}


bool TR_SkyTraqNMEA::configureRtcmOutput(
    bool     enableRtcm,
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
    bool     saveToFlash)
{
  // Body layout (16 bytes), matching your known-good example:
  //
  //  [0]  RTCM output enable
  //  [1]  MSM message rate code (0x00 -> 1 Hz in your example)
  //  [2]  Enable 1005 ARP
  //  [3]  Enable GPS MSM   (1074)
  //  [4]  Enable GLO MSM   (1084)
  //  [5]  Enable GAL MSM   (1094)
  //  [6]  Enable SBAS MSM  (we'll leave 0 = disable)
  //  [7]  Enable QZSS MSM  (0 = disable)
  //  [8]  Enable BDS MSM   (1124)
  //  [9]  GPS eph rate     (1019)
  //  [10] GLO eph rate     (1020)
  //  [11] BDS eph rate     (1042)
  //  [12] GAL eph rate     (1046)
  //  [13] MSM type         (1 -> MSM4)
  //  [14] Version          (0x02 in your example)
  //  [15] Save to flash    (0=SRAM only, 1=SRAM+Flash)

  uint8_t body[16];

  body[0]  = enableRtcm   ? 0x01 : 0x00;
  body[1]  = msmRateCode;
  body[2]  = enable1005   ? 0x01 : 0x00;
  body[3]  = enableGpsMsm ? 0x01 : 0x00;
  body[4]  = enableGloMsm ? 0x01 : 0x00;
  body[5]  = enableGalMsm ? 0x01 : 0x00;
  body[6]  = 0x00;                    // SBAS disabled
  body[7]  = 0x00;                    // QZSS disabled
  body[8]  = enableBdsMsm ? 0x01 : 0x00;
  body[9]  = gpsEphRate;
  body[10] = gloEphRate;
  body[11] = bdsEphRate;
  body[12] = galEphRate;
  body[13] = msmType;                 // 0x01 -> MSM4
  body[14] = version;                 // 0x02
  body[15] = saveToFlash ? 0x01 : 0x00;

  // Build SkyTraq payload: [ID][body...]
  // ID = 0x20, body = 16 bytes → total len = 17 (0x0011), which sendFrame() will encode.
  uint8_t payload[1 + sizeof(body)];
  payload[0] = 0x20;                  // Message ID
  memcpy(&payload[1], body, sizeof(body));

  Serial.println(F("Sending RTCM output configuration (0x20)..."));

  sendFrame(payload, sizeof(payload));

  CmdResult r = waitAckOrNack(0x20, -1, 1000);
  Serial.printf("configureRtcmOutput: result=%d (%s)\n",
                (int)r,
                (r == CR_ACK ? "ACK" : (r == CR_NACK ? "NACK" : "TIMEOUT")));

  return (r == CR_ACK);
}

// ---------------------------------------------------------
// Configure RTK Mode (simple) – rover/base selector
// (ID: 0x6A, SID: 0x01)
// ---------------------------------------------------------
//
// From AN0028/AN0037 "Configure RTK Mode":
//   Message body length: 2 bytes
//   Field 1: RTK Mode
//        0 = RTK rover mode
//        1 = RTK base mode
//   Field 2: Attributes
//        0 = update SRAM only
//        1 = update SRAM + Flash
//
// Our payload to sendFrame() is:
//   [0x6A, 0x01, RTK_Mode, Attributes]
//
bool TR_SkyTraqNMEA::configureRtkRoverMode(bool saveToFlash)
{
  // Optional: flush any leftover bytes
  drain(30);

  // Payload layout for sendFrame():
  //   [0] = Message ID   = 0x6A
  //   [1] = Sub-ID       = 0x01 (Configure RTK Mode)
  //   [2] = RTK Mode     = 0x00 (Rover)
  //   [3] = Attributes   = 0x00 (SRAM only) or 0x01 (SRAM + Flash)
  uint8_t payload[4];
  payload[0] = 0x6A;                 // ID
  payload[1] = 0x01;                 // Sub-ID: Configure RTK Mode
  payload[2] = 0x00;                 // Mode: 0 = rover
  payload[3] = saveToFlash ? 0x01    // Attributes: save or not
                           : 0x00;

  Serial.println(F("Sending Configure RTK Mode (rover, 0x6A/0x01)..."));
  sendFrame(payload, sizeof(payload));

  // ACK/NACK for (ID=0x6A, SubID=0x01)
  CmdResult r = waitAckOrNack(0x6A, 0x01, 1000);
  Serial.printf("configureRtkRoverMode: result=%d (%s)\n",
                (int)r,
                (r == CR_ACK ? "ACK"
                 : (r == CR_NACK ? "NACK" : "TIMEOUT")));

  // Be strict here so you know it really took:
  return (r == CR_ACK);
}

int TR_SkyTraqNMEA::findOrAllocSatSlot(uint8_t prn)
{
  // 1) Look for an existing slot with this PRN
  for (int i = 0; i < GnssNmeaData::MAX_SATS; ++i) {
    if (data_.sat_prn[i] == prn) {
      return i;
    }
  }

  // 2) Otherwise, look for a free slot (PRN 0)
  for (int i = 0; i < GnssNmeaData::MAX_SATS; ++i) {
    if (data_.sat_prn[i] == 0) {
      data_.sat_prn[i] = prn;
      return i;
    }
  }

  // 3) Table full -> drop this satellite
  return -1;
}

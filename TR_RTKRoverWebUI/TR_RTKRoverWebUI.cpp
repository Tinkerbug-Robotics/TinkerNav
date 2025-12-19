// Copyright (c) 2025 Tinkerbug Robotics
//
// MIT License
//

#include <TR_RTKRoverWebUI.h>

// -------- Constructor --------
TR_RTKRoverWebUI::TR_RTKRoverWebUI(WebServer& server,
                                   TR_SkyTraqNMEA::GnssNmeaData& latest,
                                   bool& hasGnss,
                                   unsigned long& lastRtcmMs,
                                   uint16_t& lastRtcmBlockBytes)
  : web_(server),
    g_latest_(latest),
    g_has_gnss_(hasGnss),
    g_lastRtcmMs_(lastRtcmMs),
    g_lastRtcmBlockBytes_(lastRtcmBlockBytes)
{
}

// -------- Public API --------

void TR_RTKRoverWebUI::begin()
{
    // Register routes using capturing lambdas that forward to member functions
    web_.on("/", HTTP_GET, [this]() { this->handleRoot(); });
    web_.on("/gnss.json", HTTP_GET, [this]() { this->handleGnssJson(); });
    web_.onNotFound([this]() { this->handleNotFound(); });

    web_.begin();
}

void TR_RTKRoverWebUI::loop()
{
    web_.handleClient();
}

// -------- Internal helpers --------

const char* TR_RTKRoverWebUI::rtkModeFromQuality(uint8_t q)
{
    // SkyTraq / NMEA GGA quality:
    // 0 = invalid
    // 1 = GPS fix
    // 2 = DGPS
    // 4 = RTK fixed
    // 5 = RTK float
    switch (q)
    {
        case 0:  return "No Fix";
        case 1:  return "GPS";
        case 2:  return "DGPS";
        case 3:  return "PPS";
        case 4:  return "RTK Fixed";   // 4 = Fixed
        case 5:  return "RTK Float";   // 5 = Float
        case 6:  return "Dead Reckoning";
        default: return "Unknown";
    }
}

// ======================= HTTP IMPLEMENTATION ====================

void TR_RTKRoverWebUI::handleRoot()
{
  // Exact same HTML/CSS/JS as your last working version,
  // with ALL SATELLITES (no fading, single group).
  static const char page[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>RTK Rover Status</title>
  <meta name="viewport" content="width=device-width,initial-scale=1">

  <style>
    body {
      font-family: Arial, Helvetica, sans-serif;
      margin: 0;
      padding: 1rem;
      background: #111;
      color: #eee;
    }

    h1 {
      font-size: 1.4rem;
      margin-bottom: 0.5rem;
    }

    .top-row {
      display: flex;
      flex-wrap: wrap;
      gap: 1rem;
    }

    .card {
      background: #1e1e1e;
      border-radius: 8px;
      padding: 1rem;
      margin-bottom: 1rem;
      box-shadow: 0 2px 4px rgba(0,0,0,0.4);
    }

    .card-grow {
      flex: 1 1 0;
      min-width: 260px;
    }

    .card-fixed {
      flex: 0 0 260px;
    }

    .label {
      color: #aaa;
      font-size: 0.8rem;
      text-transform: uppercase;
    }

    .value {
      font-size: 1.1rem;
      margin-bottom: 0.5rem;
    }

    .mode-fixed { color: #4caf50; font-weight: bold; }
    .mode-float { color: #ffeb3b; font-weight: bold; }
    .mode-nofix { color: #f44336; font-weight: bold; }

    .age-good   { color: #4caf50; }
    .age-ok     { color: #ffeb3b; }
    .age-bad    { color: #f44336; }

    .ratio-high { color: #4caf50; }
    .ratio-med  { color: #ffeb3b; }
    .ratio-low  { color: #f44336; }

    .rtcm-fresh { color: #4caf50; }
    .rtcm-ok    { color: #ffeb3b; }
    .rtcm-stale { color: #f44336; }

    /* Satellite SNR UI */
    .sat-summary {
      font-size: 0.9rem;
      margin-bottom: 0.5rem;
      color: #ccc;
    }

    .sat-section {
      margin-top: 0.5rem;
    }

    .sat-section-title {
      font-size: 0.85rem;
      color: #aaa;
      text-transform: uppercase;
      margin: 0.25rem 0 0.25rem 0;
    }

    .sat-row {
      display: flex;
      align-items: center;
      margin-bottom: 4px;
      font-size: 0.85rem;
    }

    .sat-prn {
      width: 4.5rem;
      color: #ccc;
    }

    .sat-bar-wrap {
      position: relative;
      flex: 1;
      background: #333;
      border-radius: 4px;
      overflow: hidden;
      margin-right: 0.5rem;
      height: 10px;
    }

    .sat-bar {
      position: absolute;
      top: 0;
      bottom: 0;
      left: 0;
      width: 0;
      background: #4caf50; /* default, overridden by SNR class */
      transition: width 0.2s ease;
    }

    .sat-snr {
      width: 3.5rem;
      text-align: right;
      color: #ddd;
    }

    /* SNR colors */
    .snr-low  { background: #f44336; } /* red   */
    .snr-med  { background: #ffeb3b; } /* yellow*/
    .snr-high { background: #4caf50; } /* green */

    /* Vertical SNR tick marks at 20/30/40/50 dB on 0..60 scale */
    .snr-tick {
      position: absolute;
      top: 0;
      bottom: 0;
      width: 1px;
      background: #555;
      opacity: 0.7;
      pointer-events: none;
    }

    .snr-tick-20 { left: 33.333%; }  /* 20 / 60 */
    .snr-tick-30 { left: 50.000%; }  /* 30 / 60 */
    .snr-tick-40 { left: 66.667%; }  /* 40 / 60 */
    .snr-tick-50 { left: 83.333%; }  /* 50 / 60 */
    
        /* Layout: place tiles side-by-side in rows when wide enough */
    .card-row {
      display: flex;
      flex-wrap: wrap;
      gap: 1rem;
    }
    .card-row .card {
      flex: 1 1 0;
      min-width: 280px;
    }

    /* Sky plot */
    .skyplot-container {
      position: relative;
      width: 100%;
      padding-top: 100%; /* square */
    }

    #skyPlotSvg {
      position: absolute;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
    }

    .skyplot-bg-circle {
      fill: none;
      stroke: #333;
      stroke-width: 0.003;
    }

    .skyplot-axis {
      stroke: #333;
      stroke-width: 0.003;
    }

    .sky-sat-label {
      fill: #000;
      font-size: 0.08px;
      text-anchor: middle;
      dominant-baseline: central;
      pointer-events: none;
    }

    .sky-snr-low  { fill: #f44336; } /* red   */
    .sky-snr-med  { fill: #ffeb3b; } /* yellow*/
    .sky-snr-high { fill: #4caf50; } /* green */
    
  </style>
</head>

<body>
  <h1>RTK Rover Status</h1>

  <div class="card">
    <div class="label">GNSS Fix</div>
    <div id="fixMode" class="value mode-nofix">--</div>

    <div class="label">Lat / Lon / Alt (m)</div>
    <div id="position" class="value">--</div>
  </div>

  <div class="top-row">
    <div class="card card-grow">
      <div class="label">RTK Age (s)</div>
      <div id="rtkAge" class="value age-bad">--</div>

      <div class="label">RTK Ratio</div>
      <div id="rtkRatio" class="value ratio-low">--</div>
    </div>

    <div class="card card-fixed">
      <div class="label">RTCM Link Age (s)</div>
      <div id="rtcmAge" class="value rtcm-stale">--</div>

      <div class="label">Last RTCM Block Size (bytes)</div>
      <div id="rtcmBytes" class="value">--</div>
    </div>
  </div>

  <div class="card-row">
    <!-- Left: sky plot -->
    <div class="card">
      <div class="label">Satellite Sky Plot</div>
      <div class="skyplot-container">
        <svg id="skyPlotSvg" viewBox="-1 -1 2 2" preserveAspectRatio="xMidYMid meet"></svg>
      </div>
    </div>
    
    <!-- Right: bar SNRs (existing behavior) -->
    <div class="card">
      <div class="label">Satellites (Used / In View)</div>
      <div id="satSummary" class="value sat-summary">--</div>

      <div class="sat-section">
        <div class="sat-section-title">GPS</div>
        <div id="satBarsGPS"></div>
      </div>

      <div class="sat-section">
        <div class="sat-section-title">Galileo</div>
        <div id="satBarsGAL"></div>
      </div>

      <div class="sat-section">
        <div class="sat-section-title">BeiDou</div>
        <div id="satBarsBDS"></div>
      </div>

      <div class="sat-section">
        <div class="sat-section-title">GLONASS</div>
        <div id="satBarsGLO"></div>
      </div>

      <div class="sat-section">
        <div class="sat-section-title">Other / Mixed</div>
        <div id="satBarsOTHER"></div>
      </div>
    </div>
  </div>

  <script>
    function buildSatRows(list) {
      let html = '';

      list.forEach(s => {
        let snrVal = (typeof s.snr === 'number') ? s.snr : 0;
        let snrClamped = Math.max(0, Math.min(60, snrVal));
        let widthPct   = (snrClamped / 60.0) * 100.0;

        let snrClass = 'snr-low';
        if (snrVal >= 35)      snrClass = 'snr-high';
        else if (snrVal >= 25) snrClass = 'snr-med';

        html +=
          '<div class="sat-row">' +
            '<div class="sat-prn">PRN ' + s.prn + '</div>' +
            '<div class="sat-bar-wrap">' +
              '<div class="snr-tick snr-tick-20"></div>' +
              '<div class="snr-tick snr-tick-30"></div>' +
              '<div class="snr-tick snr-tick-40"></div>' +
              '<div class="snr-tick snr-tick-50"></div>' +
              '<div class="sat-bar ' + snrClass +
                '" style="width:' + widthPct.toFixed(1) + '%;"></div>' +
            '</div>' +
            '<div class="sat-snr">' + snrVal.toFixed(0) + ' dB</div>' +
          '</div>';
      });

      return html;
    }
    function updateSkyPlot(sats) {
      const svg = document.getElementById('skyPlotSvg');
      if (!svg) return;

      // Clear old content
      while (svg.firstChild) {
        svg.removeChild(svg.firstChild);
      }

      // Background: range circles (30°, 60°, 90°) and cross axes
      const rings = [1.0, 2.0/3.0, 1.0/3.0];  // outer = 0°, inner = 60°, center ~90°
      rings.forEach(r => {
        const c = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        c.setAttribute('cx', '0');
        c.setAttribute('cy', '0');
        c.setAttribute('r', r.toString());
        c.setAttribute('class', 'skyplot-bg-circle');
        svg.appendChild(c);
      });

      const axes = [
        {x1: -1, y1: 0,  x2: 1,  y2: 0},
        {x1: 0,  y1: -1, x2: 0,  y2: 1}
      ];
      axes.forEach(a => {
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        line.setAttribute('x1', a.x1);
        line.setAttribute('y1', a.y1);
        line.setAttribute('x2', a.x2);
        line.setAttribute('y2', a.y2);
        line.setAttribute('class', 'skyplot-axis');
        svg.appendChild(line);
      });

      // Helper: SNR → color class
      function snrClassFor(snrVal) {
        if (snrVal >= 35) return 'sky-snr-high';
        if (snrVal >= 25) return 'sky-snr-med';
        return 'sky-snr-low';
      }

      // Draw each satellite
      sats.forEach(s => {
        if (typeof s.el !== 'number' || typeof s.az !== 'number') return;

        const el = Math.max(0, Math.min(90, s.el));
        const az = s.az; // deg, 0 = North

        // radius: 0 at zenith (90°), 1 at horizon (0°)
        const r = (90 - el) / 90.0;

        const azRad = az * Math.PI / 180.0;
        // 0° at top, clockwise: x = r * sin(az), y = -r * cos(az)
        const x = r * Math.sin(azRad);
        const y = -r * Math.cos(azRad);

        const snrVal = (typeof s.snr === 'number') ? s.snr : 0;
        const cls    = snrClassFor(snrVal);

        const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');

        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circle.setAttribute('cx', x.toString());
        circle.setAttribute('cy', y.toString());
        circle.setAttribute('r', '0.07');
        circle.setAttribute('class', cls);
        g.appendChild(circle);

        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.setAttribute('x', x.toString());
        label.setAttribute('y', y.toString());
        label.setAttribute('class', 'sky-sat-label');
        label.textContent = String(s.prn);
        g.appendChild(label);

        svg.appendChild(g);
      });
    }
    
    function updateStatus() {
      fetch('/gnss.json')
        .then(r => r.json())
        .then(d => {
          let fixElem    = document.getElementById('fixMode');
          let posElem    = document.getElementById('position');
          let ageElem    = document.getElementById('rtkAge');
          let ratioElem  = document.getElementById('rtkRatio');
          let rtcmElem   = document.getElementById('rtcmAge');
          let rtcmBytes  = document.getElementById('rtcmBytes');
          let satSummary  = document.getElementById('satSummary');
          let satBarsGPS  = document.getElementById('satBarsGPS');
          let satBarsGAL  = document.getElementById('satBarsGAL');
          let satBarsBDS  = document.getElementById('satBarsBDS');
          let satBarsGLO  = document.getElementById('satBarsGLO');
          let satBarsOTHER = document.getElementById('satBarsOTHER');

          // GNSS fix
          if (!d.hasFix) {
            fixElem.textContent = 'No Fix';
            fixElem.className   = 'value mode-nofix';
            posElem.textContent = '--';
          } else {
            fixElem.textContent = d.rtkMode + ' (Q=' + d.quality + ')';

            if (d.rtkMode.includes('Fixed'))
              fixElem.className = 'value mode-fixed';
            else if (d.rtkMode.includes('Float'))
              fixElem.className = 'value mode-float';
            else
              fixElem.className = 'value';

            if (d.lat === null || d.lon === null) {
              posElem.textContent = '--';
            } else {
              let alt = (d.alt === null) ? '--' : d.alt.toFixed(2);
              posElem.textContent =
                d.lat.toFixed(7) + ' , ' +
                d.lon.toFixed(7) + ' , ' + alt;
            }
          }

          // RTK age
          if (d.rtkAge === null) {
            ageElem.textContent = '--';
            ageElem.className   = 'value age-bad';
          } else {
            ageElem.textContent = d.rtkAge.toFixed(2);
            ageElem.className =
              'value ' + (d.rtkAge < 1 ? 'age-good' :
                          d.rtkAge < 5 ? 'age-ok'   : 'age-bad');
          }

          // RTK ratio
          if (d.rtkRatio === null) {
            ratioElem.textContent = '--';
            ratioElem.className   = 'value ratio-low';
          } else {
            ratioElem.textContent = d.rtkRatio.toFixed(2);
            ratioElem.className =
              'value ' + (d.rtkRatio > 3   ? 'ratio-high' :
                          d.rtkRatio > 1.5 ? 'ratio-med'  : 'ratio-low');
          }

          // RTCM age
          if (d.rtcmAge === null) {
            rtcmElem.textContent = '--';
            rtcmElem.className   = 'value rtcm-stale';
          } else {
            rtcmElem.textContent = d.rtcmAge.toFixed(1);
            rtcmElem.className =
              'value ' + (d.rtcmAge < 1 ? 'rtcm-fresh' :
                          d.rtcmAge < 5 ? 'rtcm-ok'    : 'rtcm-stale');
          }

          // Last RTCM block size
          if (typeof d.lastRtcmBlockBytes === 'number') {
            rtcmBytes.textContent = d.lastRtcmBlockBytes.toString();
          } else {
            rtcmBytes.textContent = '--';
          }

          // Satellites
          if (!d.sats || !Array.isArray(d.sats) || d.sats.length === 0) {
            satSummary.textContent = '--';
            satBarsGPS.innerHTML   = '';
            satBarsGAL.innerHTML   = '';
            satBarsBDS.innerHTML   = '';
            satBarsGLO.innerHTML   = '';
            satBarsOTHER.innerHTML = '';
            updateSkyPlot([]);  // clears plot
          } else {
            const inView = (typeof d.satsInView === 'number')
                           ? d.satsInView
                           : d.sats.length;

            let usedCount = 0;
            d.sats.forEach(s => { if (s.used) usedCount++; });

            let reportedUsed = (typeof d.numSats === 'number') ? d.numSats : usedCount;
            let usedClamped  = Math.min(Math.max(reportedUsed, usedCount), inView);

            satSummary.textContent = usedClamped + ' / ' + inView;

            // Group satellites by constellation string from JSON ("GPS","GAL","BDS","GLO","OTHER")
            const gps   = [];
            const gal   = [];
            const bds   = [];
            const glo   = [];
            const other = [];

            d.sats.forEach(s => {
              switch (s.const) {
                case 'GPS':  gps.push(s);   break;
                case 'GAL':  gal.push(s);   break;
                case 'BDS':  bds.push(s);   break;
                case 'GLO':  glo.push(s);   break;
                default:     other.push(s); break;
              }
            });

            satBarsGPS.innerHTML   = buildSatRows(gps);
            satBarsGAL.innerHTML   = buildSatRows(gal);
            satBarsBDS.innerHTML   = buildSatRows(bds);
            satBarsGLO.innerHTML   = buildSatRows(glo);
            satBarsOTHER.innerHTML = buildSatRows(other);

            // Keep the sky plot using ALL satellites
            updateSkyPlot(d.sats);
          }
        })
        .catch(e => {
          console.log(e);
        });
    }

    updateStatus();
    setInterval(updateStatus, 1000);
  </script>
</body>
</html>
)rawliteral";

  web_.send_P(200, "text/html", page);
}

void TR_RTKRoverWebUI::handleGnssJson()
{
  String json = "{";

  json += "\"hasFix\":";
  json += (g_has_gnss_ ? "true" : "false");

  if (g_has_gnss_) 
  {
    // Basic fix info
    json += ",\"quality\":";   json += String(g_latest_.gnss_quality_indicator);

    if (isfinite(g_latest_.latitude)) 
    {
      json += ",\"lat\":"; json += String(g_latest_.latitude, 7);
    } else {
      json += ",\"lat\":null";
    }

    if (isfinite(g_latest_.longitude)) 
    {
      json += ",\"lon\":"; json += String(g_latest_.longitude, 7);
    } else {
      json += ",\"lon\":null";
    }

    if (isfinite(g_latest_.altitude)) 
    {
      json += ",\"alt\":"; json += String(g_latest_.altitude, 2);
    } else {
      json += ",\"alt\":null";
    }

    if (isfinite(g_latest_.rtk_age)) 
    {
      json += ",\"rtkAge\":"; json += String(g_latest_.rtk_age, 2);
    } else {
      json += ",\"rtkAge\":null";
    }

    if (isfinite(g_latest_.rtk_ratio)) 
    {
      json += ",\"rtkRatio\":"; json += String(g_latest_.rtk_ratio, 2);
    } else {
      json += ",\"rtkRatio\":null";
    }

    json += ",\"rtkMode\":\"";
    json += rtkModeFromQuality(g_latest_.gnss_quality_indicator);
    json += "\"";

    // Satellite counts
    json += ",\"numSats\":";    json += String(g_latest_.num_sats);
    json += ",\"satsInView\":"; json += String(g_latest_.sats_in_view);

    // Satellite list
    json += ",\"sats\":[";
    bool first = true;
    uint32_t now_ms = millis();

    for (int i = 0; i < TR_SkyTraqNMEA::GnssNmeaData::MAX_SATS; ++i) 
    {
      uint8_t prn = g_latest_.sat_prn[i];
      if (prn == 0) continue;   // unused slot

      if (!first) {
        json += ",";
      }
      first = false;

      json += "{";
      json += "\"prn\":";      json += String(prn);
      json += ",\"snr\":";     json += String(g_latest_.sat_snr[i]);
      json += ",\"used\":";    json += (g_latest_.sat_used[i] ? "true" : "false");

      // Constellation string from enum
      uint8_t c = g_latest_.sat_constellation[i];
      const char* cstr = "OTHER";
      if (c == TR_SkyTraqNMEA::GnssNmeaData::CONST_GPS) {
        cstr = "GPS";
      } else if (c == TR_SkyTraqNMEA::GnssNmeaData::CONST_GAL) {
        cstr = "GAL";
      } else if (c == TR_SkyTraqNMEA::GnssNmeaData::CONST_BDS) {
        cstr = "BDS";
      } else if (c == TR_SkyTraqNMEA::GnssNmeaData::CONST_GLO) {
        cstr = "GLO";
      }
      json += ",\"const\":\""; json += cstr; json += "\"";

      // Elevation / azimuth in degrees from GSV, if available
      json += ",\"el\":"; json += String(g_latest_.sat_elev_deg[i], 1);
      json += ",\"az\":"; json += String(g_latest_.sat_az_deg[i], 1);

      if (g_latest_.sat_last_ms[i] != 0) {
        float age_s = (now_ms - g_latest_.sat_last_ms[i]) / 1000.0f;
        if (age_s < 3600.0f && age_s >= 0.0f) {
          json += ",\"age\":"; json += String(age_s, 1);
        } else {
          json += ",\"age\":null";
        }
      } else {
        json += ",\"age\":null";
      }

      json += "}";
    }
    json += "]";
  } else {
    json += ",\"quality\":0";
    json += ",\"lat\":null,\"lon\":null,\"alt\":null";
    json += ",\"rtkAge\":null,\"rtkRatio\":null";
    json += ",\"rtkMode\":\"No Fix\"";
    json += ",\"numSats\":0,\"satsInView\":0,\"sats\":[ ]";
  }

  // RTCM link age (seconds) based on millis()
  if (g_lastRtcmMs_ == 0) {
    json += ",\"rtcmAge\":null";
  } else {
    float age_s = (millis() - g_lastRtcmMs_) / 1000.0f;
    json += ",\"rtcmAge\":"; json += String(age_s, 1);
  }

  // Last RTCM block size (bytes)
  json += ",\"lastRtcmBlockBytes\":";
  json += String(g_lastRtcmBlockBytes_);

  json += "}";

  web_.send(200, "application/json", json);
}

void TR_RTKRoverWebUI::handleNotFound()
{
    web_.send(404, "text/plain", "Not found");
}
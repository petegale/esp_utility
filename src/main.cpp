
#define ESP32_CAN_TX_PIN GPIO_NUM_22
#define ESP32_CAN_RX_PIN GPIO_NUM_19

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include "M5Atom.h"
#include "InterpolationLib.h"

// NMEA
#include <Preferences.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

// ─── WiFi / Network ───────────────────────────────────────────────────────────
const char *ssid     = "sensor";
const char *password = "12345678";

// ─── UDP (existing sensor input) ──────────────────────────────────────────────
WiFiUDP udp;
const int udpPort = 10110;
char incomingPacket[255];

// ─── Web Server + WebSocket ───────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─── Tank calibration ────────────────────────────────────────────────────────
// FIX #7: fuelValues corrected to 3 to match the 3-point calibration array
const int fuelValues  = 3;
double fuelInput[3]   = {0.0, 41.0, 78.0};
double fuelOutput[3]  = {0.0, 52.0, 100.0};
int    fuelCapacity   = 200;  // litres
int    fuelLevel      = 0;

const int waterValues = 3;
double waterInput[3]  = {10.0, 50.0, 75.0};
double waterOutput[3] = {25.0, 60.0, 80.0};
int    waterCapacity  = 200;  // litres
int    waterLevel     = 0;

// ─── Autopilot State ──────────────────────────────────────────────────────────
String apMode        = "STBY";
double lockedHeading = -1;    // -1 = no data
double vesselHeading = -1;    // -1 = no data

// ─── Instrument data ──────────────────────────────────────────────────────────
double depthM        = -1;    // metres, -1 = no data
double sogKn         = -1;    // knots,  -1 = no data
double awsKn         = -1;    // apparent wind speed knots, -1 = no data
double awaDeg        = -1;    // apparent wind angle degrees, -1 = no data
double cogDeg        = -1;    // course over ground degrees, -1 = no data

// ─── AIS target data ──────────────────────────────────────────────────────────
// Store up to 20 AIS targets. Older targets expire after AIS_TARGET_TIMEOUT_MS.
struct AisTarget {
  uint32_t      mmsi;
  double        lat;
  double        lon;
  double        cogDeg;
  double        sogKn;
  char          name[21];     // 20 chars + null
  unsigned long lastSeen;
  bool          active;
};
const int MAX_AIS_TARGETS = 20;
AisTarget aisTargets[MAX_AIS_TARGETS];
const unsigned long AIS_TARGET_TIMEOUT_MS = 300000; // 5 minutes

// Own vessel position for AIS radar centre
double ownLat = -999;
double ownLon = -999;

// ─── NFU / Direct steering state ─────────────────────────────────────────────
volatile int  nfuRate    = 0;   // volatile: written in WS callback, read in loop
unsigned long nfuLastMsg = 0;
unsigned long nfuLastSend = 0;
const unsigned long NFU_TIMEOUT_MS      = 500;  // stop if no message for 500ms
const unsigned long NFU_SEND_INTERVAL_MS = 100; // send NFU commands at 10Hz

// ─── Broadcast rate limiter ───────────────────────────────────────────────────
unsigned long lastBroadcast = 0;
const unsigned long BROADCAST_INTERVAL_MS = 200; // max 5Hz to WebSocket clients

// ─── NMEA2000 ─────────────────────────────────────────────────────────────────
// FIX #13: TransmitMessages trimmed to only what we actually send
const unsigned long TransmitMessages[] PROGMEM = {
  127505L,  // Fluid Level
  130311L,  // Temperature
  0
};

// ─── Debug mode ───────────────────────────────────────────────────────────────
// To enable bus sniffing, uncomment the three lines in setup() marked DEBUG.
// Prints every PGN in human-readable text to Serial. Use this to capture
// what your AP48 actually sends when you press each mode button, then compare
// against the byte sequences in SendModeCommand() below.
// Re-comment before normal deployment.

// ─── HTML UI (PROGMEM) ────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Nav</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: -apple-system, sans-serif;
      background: #0a1628;
      color: #fff;
      display: flex;
      flex-direction: column;
      /* 100dvh (dynamic viewport height) accounts for Android browser chrome.
         Falls back to 100vh on older browsers that don't support dvh. */
      height: 100vh;
      height: 100dvh;
      overflow: hidden;
      user-select: none;
      -webkit-user-select: none;
    }

    /* ── Screen container ── */
    #screens {
      flex: 1;
      overflow: hidden;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 12px 12px 0 12px;
    }

    .screen { display: none; width: 100%; max-width: 360px; flex-direction: column; align-items: center; }
    .screen.active { display: flex; }

    /* ── Tab bar ── */
    #tabbar {
      display: flex;
      background: #0d1e2e;
      border-top: 1px solid #1a2e40;
      padding: 6px 0 10px 0;
      flex-shrink: 0;
    }
    .tab {
      flex: 1;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 3px;
      padding: 6px 0;
      cursor: pointer;
      -webkit-tap-highlight-color: transparent;
      color: #445566;
      font-size: 0.65em;
      letter-spacing: 0.5px;
      transition: color 0.15s;
    }
    .tab svg { width: 22px; height: 22px; }
    .tab.active { color: #44aaff; }

    /* ═══════════════════════════════════════════
       AUTOPILOT SCREEN
    ═══════════════════════════════════════════ */
    #ap-status {
      background: #1a2a3a;
      border-radius: 12px;
      padding: 10px 24px;
      text-align: center;
      margin-bottom: 12px;
      width: 100%;
    }
    #mode-display {
      font-size: 2em;
      font-weight: bold;
      letter-spacing: 4px;
    }
    #mode-display.stby    { color: #ff6644; }
    #mode-display.auto    { color: #00ff88; }
    #mode-display.wind    { color: #44aaff; }
    #mode-display.nav     { color: #ffcc00; }
    #mode-display.nodrift { color: #aa88ff; }
    #mode-display.nfu     { color: #ff8800; }
    #heading-display { font-size: 1em; color: #aabbcc; margin-top: 2px; }

    #hdg-controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      width: 100%;
      margin-bottom: 8px;
    }

    #nfu-controls {
      display: none;
      flex-direction: column;
      align-items: center;
      width: 100%;
      margin-bottom: 8px;
    }
    #nfu-controls.active { display: flex; }

    #wheel-label { font-size: 0.8em; color: #8899aa; margin-bottom: 6px; letter-spacing: 1px; }

    #wheel-container {
      width: 200px; height: 200px;
      margin-bottom: 8px;
      touch-action: none;
    }
    #wheel-svg { width: 100%; height: 100%; cursor: grab; }

    #rudder-bar-wrap {
      width: 100%;
      background: #1a2a3a;
      border-radius: 8px;
      height: 24px;
      position: relative;
      overflow: hidden;
      margin-bottom: 3px;
    }
    #rudder-bar-port { position:absolute; right:50%; top:0; bottom:0; background:#44aaff; width:0; transition:width 0.1s; }
    #rudder-bar-stbd { position:absolute; left:50%;  top:0; bottom:0; background:#ff6644; width:0; transition:width 0.1s; }
    #rudder-centre   { position:absolute; left:50%;  top:0; bottom:0; width:2px; background:#556677; transform:translateX(-50%); }
    #rudder-label { display:flex; justify-content:space-between; width:100%; font-size:0.72em; color:#556677; margin-bottom:8px; }

    #mode-controls {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 7px;
      width: 100%;
      margin-bottom: 8px;
    }

    button {
      background: #0d1e2e;
      color: #445566;
      border: 2px solid #1a2e40;
      border-radius: 10px;
      padding: 16px 4px;
      font-size: 0.95em;
      font-weight: bold;
      cursor: pointer;
      transition: all 0.15s ease;
      -webkit-tap-highlight-color: transparent;
      letter-spacing: 1px;
    }

    .btn-stby    { border-color:#5a2a1a; color:#7a4433; }
    .btn-auto    { border-color:#1a4a2a; color:#336644; }
    .btn-wind    { border-color:#1a3a5a; color:#336688; }
    .btn-nav     { border-color:#4a4010; color:#776622; }
    .btn-nodrift { border-color:#3a2a5a; color:#664488; }
    .btn-nfu     { border-color:#4a2a10; color:#774422; }
    .btn-hdg     { padding:18px 4px; font-size:1.1em; background:#1a3050; color:#aabbcc; border-color:#2a4060; }
    .btn-hdg:active { background:#2a5080; }

    .btn-stby.active    { background:#5a1a0a; border-color:#ff6644; color:#ff6644; box-shadow:0 0 12px #ff664466; }
    .btn-auto.active    { background:#0a3a1a; border-color:#00ff88; color:#00ff88; box-shadow:0 0 12px #00ff8866; }
    .btn-wind.active    { background:#0a1a3a; border-color:#44aaff; color:#44aaff; box-shadow:0 0 12px #44aaff66; }
    .btn-nav.active     { background:#3a3000; border-color:#ffcc00; color:#ffcc00; box-shadow:0 0 12px #ffcc0066; }
    .btn-nodrift.active { background:#2a0a3a; border-color:#aa88ff; color:#aa88ff; box-shadow:0 0 12px #aa88ff66; }
    .btn-nfu.active     { background:#3a1a00; border-color:#ff8800; color:#ff8800; box-shadow:0 0 12px #ff880066; }

    #conn { font-size: 0.7em; color: #445566; margin-top: 4px; }

    /* ═══════════════════════════════════════════
       INSTRUMENTS SCREEN
    ═══════════════════════════════════════════ */
    .inst-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      width: 100%;
      margin-top: 4px;
    }
    .inst-box {
      background: #1a2a3a;
      border-radius: 12px;
      padding: 16px 10px 12px 10px;
      text-align: center;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 4px;
    }
    .inst-label {
      font-size: 0.65em;
      color: #8899aa;
      letter-spacing: 2px;
      text-transform: uppercase;
    }
    .inst-value {
      font-size: 2.4em;
      font-weight: bold;
      color: #fff;
      line-height: 1;
    }
    .inst-unit {
      font-size: 0.7em;
      color: #556677;
      letter-spacing: 1px;
    }
    .inst-value.depth { color: #44aaff; }
    .inst-value.sog   { color: #00ff88; }
    .inst-value.aws   { color: #ffcc00; }
    .inst-value.awa   { color: #ff8800; }

    /* Wind direction arrow */
    #awa-arrow-wrap {
      width: 100%;
      display: flex;
      justify-content: center;
      margin-top: 4px;
    }
    #awa-svg { width: 44px; height: 44px; }

    /* ═══════════════════════════════════════════
       AIS RADAR SCREEN
    ═══════════════════════════════════════════ */
    #ais-header {
      width: 100%;
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
      font-size: 0.7em;
      color: #8899aa;
    }
    #ais-range-label { color: #44aaff; font-weight: bold; }
    .ais-range-btn {
      background: #1a2a3a;
      border: 1px solid #2a4060;
      color: #aabbcc;
      border-radius: 6px;
      padding: 4px 10px;
      font-size: 1em;
      cursor: pointer;
    }

    #radar-wrap {
      width: 100%;
      aspect-ratio: 1;
      max-width: 340px;
      position: relative;
    }
    #radar-svg { width: 100%; height: 100%; }

    #ais-list {
      width: 100%;
      margin-top: 8px;
      font-size: 0.68em;
      color: #8899aa;
      max-height: 100px;
      overflow-y: auto;
    }
    .ais-row {
      display: flex;
      justify-content: space-between;
      padding: 3px 0;
      border-bottom: 1px solid #1a2a3a;
      color: #aabbcc;
    }
    .ais-row .mmsi { color: #556677; font-size: 0.9em; }
    #ais-no-data { color: #445566; text-align: center; padding: 16px; font-size: 0.9em; }

    /* ── AIS Target popup ── */
    #ais-popup {
      display: none;
      position: fixed;
      top: 0; left: 0; right: 0; bottom: 0;
      background: rgba(0,0,0,0.7);
      z-index: 100;
      align-items: center;
      justify-content: center;
    }
    #ais-popup.visible { display: flex; }
    #ais-popup-card {
      background: #1a2a3a;
      border: 1px solid #2a4a6a;
      border-radius: 14px;
      padding: 20px;
      width: 290px;
      position: relative;
    }
    #ais-popup-name {
      font-size: 1.3em; font-weight: bold;
      color: #ff6644; margin-bottom: 14px;
      letter-spacing: 1px;
    }
    .popup-row {
      display: flex; justify-content: space-between;
      padding: 5px 0; border-bottom: 1px solid #1a3050;
      font-size: 0.85em;
    }
    .popup-row:last-child { border-bottom: none; }
    .popup-key   { color: #8899aa; }
    .popup-value { color: #fff; font-weight: bold; }
    #ais-popup-close {
      margin-top: 16px; width: 100%;
      background: #2a3a4a; border: 1px solid #3a5a7a;
      color: #aabbcc; border-radius: 8px;
      padding: 10px; font-size: 0.9em; cursor: pointer;
    }
    #ais-popup-close:active { background: #3a5a7a; }
  </style>
</head>
<body>

  <!-- ═══ SCREEN CONTAINER ═══ -->
  <div id="screens">

    <!-- ── Autopilot screen ── -->
    <div id="screen-ap" class="screen active">
      <div id="ap-status">
        <div id="mode-display" class="stby">STBY</div>
        <div id="heading-display">HDG: ---&deg;</div>
      </div>

      <div id="hdg-controls">
        <button class="btn-hdg" onclick="send('PORT10')">&lt;&lt; 10&deg;</button>
        <button class="btn-hdg" onclick="send('STBD10')">10&deg; &gt;&gt;</button>
        <button class="btn-hdg" onclick="send('PORT1')">&lt; 1&deg;</button>
        <button class="btn-hdg" onclick="send('STBD1')">1&deg; &gt;</button>
      </div>

      <div id="nfu-controls">
        <div id="wheel-label">DRAG TO STEER &mdash; RETURN TO CENTRE TO STOP</div>
        <div id="wheel-container">
          <svg id="wheel-svg" viewBox="0 0 220 220" xmlns="http://www.w3.org/2000/svg">
            <circle cx="110" cy="110" r="100" fill="#1a2a3a" stroke="#2a4a6a" stroke-width="3"/>
            <g id="grip-marks" stroke="#3a6a9a" stroke-width="3" stroke-linecap="round">
              <line x1="110" y1="18" x2="110" y2="34"/>
              <line x1="110" y1="186" x2="110" y2="202"/>
              <line x1="18" y1="110" x2="34" y2="110"/>
              <line x1="186" y1="110" x2="202" y2="110"/>
              <line x1="39.4" y1="39.4" x2="50.8" y2="50.8"/>
              <line x1="169.2" y1="169.2" x2="180.6" y2="180.6"/>
              <line x1="180.6" y1="39.4" x2="169.2" y2="50.8"/>
              <line x1="50.8" y1="169.2" x2="39.4" y2="180.6"/>
            </g>
            <g id="spokes" stroke="#2a5080" stroke-width="6" stroke-linecap="round">
              <line x1="110" y1="30" x2="110" y2="80"/>
              <line x1="110" y1="140" x2="110" y2="190"/>
              <line x1="30" y1="110" x2="80" y2="110"/>
              <line x1="140" y1="110" x2="190" y2="110"/>
            </g>
            <circle cx="110" cy="110" r="22" fill="#1a3050" stroke="#3a6a9a" stroke-width="3"/>
            <circle id="indicator" cx="110" cy="35" r="7" fill="#ff8800"/>
          </svg>
        </div>
        <div id="rudder-bar-wrap">
          <div id="rudder-bar-port"></div>
          <div id="rudder-bar-stbd"></div>
          <div id="rudder-centre"></div>
        </div>
        <div id="rudder-label"><span>PORT</span><span>STBD</span></div>
      </div>

      <div id="mode-controls">
        <button id="btn-stby"    class="btn-stby    active" onclick="send('MODE:STBY')">STBY</button>
        <button id="btn-auto"    class="btn-auto"           onclick="send('MODE:AUTO')">AUTO</button>
        <button id="btn-wind"    class="btn-wind"           onclick="send('MODE:WIND')">WIND</button>
        <button id="btn-nav"     class="btn-nav"            onclick="send('MODE:NAV')">NAV</button>
        <button id="btn-nodrift" class="btn-nodrift"        onclick="send('MODE:NODRIFT')">NO DRIFT</button>
        <button id="btn-nfu"     class="btn-nfu"            onclick="send('MODE:NFU')">DIRECT</button>
      </div>

      <div id="conn">Connecting...</div>
    </div>

    <!-- ── Instruments screen ── -->
    <div id="screen-inst" class="screen">
      <div class="inst-grid">
        <div class="inst-box">
          <div class="inst-label">Depth</div>
          <div class="inst-value depth" id="inst-depth">---</div>
          <div class="inst-unit">metres</div>
        </div>
        <div class="inst-box">
          <div class="inst-label">Speed</div>
          <div class="inst-value sog" id="inst-sog">---</div>
          <div class="inst-unit">knots SOG</div>
        </div>
        <div class="inst-box">
          <div class="inst-label">Wind Speed</div>
          <div class="inst-value aws" id="inst-aws">---</div>
          <div class="inst-unit">knots AWS</div>
        </div>
        <div class="inst-box">
          <div class="inst-label">Wind Angle</div>
          <div class="inst-value awa" id="inst-awa">---</div>
          <div class="inst-unit">deg apparent</div>
          <div id="awa-arrow-wrap">
            <svg id="awa-svg" viewBox="0 0 44 44" xmlns="http://www.w3.org/2000/svg">
              <circle cx="22" cy="22" r="20" fill="#1a3050" stroke="#2a4060" stroke-width="1.5"/>
              <!-- Arrow group, rotated by JS -->
              <g id="awa-arrow" transform="rotate(0,22,22)">
                <polygon points="22,4 26,28 22,24 18,28" fill="#ff8800"/>
              </g>
              <!-- N marker -->
              <text x="22" y="40" text-anchor="middle" font-size="7" fill="#556677">S</text>
              <text x="22" y="9"  text-anchor="middle" font-size="7" fill="#aabbcc">F</text>
              <text x="6"  y="25" text-anchor="middle" font-size="7" fill="#556677">P</text>
              <text x="38" y="25" text-anchor="middle" font-size="7" fill="#556677">S</text>
            </svg>
          </div>
        </div>
      </div>
    </div>

    <!-- ── AIS Radar screen ── -->
    <div id="screen-ais" class="screen">
      <div id="ais-header">
        <button class="ais-range-btn" onclick="changeRange(-1)">&#8722;</button>
        <span>Range: <span id="ais-range-label">5 nm</span></span>
        <button class="ais-range-btn" onclick="changeRange(1)">&#43;</button>
      </div>

      <div id="radar-wrap">
        <svg id="radar-svg" viewBox="0 0 340 340" xmlns="http://www.w3.org/2000/svg">
          <!-- Range rings -->
          <circle cx="170" cy="170" r="170" fill="#080f1a" stroke="#1a2a3a" stroke-width="1"/>
          <circle cx="170" cy="170" r="127" fill="none" stroke="#1a2a3a" stroke-width="1"/>
          <circle cx="170" cy="170" r="85"  fill="none" stroke="#1a2a3a" stroke-width="1"/>
          <circle cx="170" cy="170" r="42"  fill="none" stroke="#1a2a3a" stroke-width="1"/>
          <!-- Cross hairs -->
          <line x1="170" y1="0"   x2="170" y2="340" stroke="#1a2a3a" stroke-width="1"/>
          <line x1="0"   y1="170" x2="340" y2="170" stroke="#1a2a3a" stroke-width="1"/>
          <!-- Ring labels -->
          <text x="173" y="48"  font-size="9" fill="#2a4060" id="ring-label-4"></text>
          <text x="173" y="90"  font-size="9" fill="#2a4060" id="ring-label-3"></text>
          <text x="173" y="132" font-size="9" fill="#2a4060" id="ring-label-2"></text>
          <text x="173" y="174" font-size="9" fill="#2a4060" id="ring-label-1"></text>
          <!-- N indicator -->
          <text x="170" y="14" text-anchor="middle" font-size="11" fill="#44aaff" font-weight="bold">N</text>
          <!-- Own vessel -->
          <polygon points="170,158 175,178 170,174 165,178" fill="#00ff88"/>
          <!-- AIS targets rendered here by JS -->
          <g id="ais-targets-group"></g>
        </svg>
      </div>

      <div id="ais-list">
        <div id="ais-no-data">No AIS targets</div>
      </div>
    </div>

  </div><!-- /screens -->

  <!-- ═══ AIS TARGET POPUP ═══ -->
  <div id="ais-popup" onclick="closePopup(event)">
    <div id="ais-popup-card">
      <div id="ais-popup-name">---</div>
      <div id="ais-popup-rows"></div>
      <button id="ais-popup-close" onclick="closePopup()">Close</button>
    </div>
  </div>

  <!-- ═══ TAB BAR ═══ -->
  <div id="tabbar">
    <div class="tab active" onclick="showScreen('ap')" id="tab-ap">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round">
        <circle cx="12" cy="12" r="10"/>
        <line x1="12" y1="2" x2="12" y2="6"/>
        <line x1="12" y1="18" x2="12" y2="22"/>
        <line x1="4.22" y1="4.22" x2="7.05" y2="7.05"/>
        <line x1="16.95" y1="16.95" x2="19.78" y2="19.78"/>
        <circle cx="12" cy="12" r="2" fill="currentColor" stroke="none"/>
        <line x1="12" y1="12" x2="16" y2="8" stroke-width="2.2"/>
      </svg>
      PILOT
    </div>
    <div class="tab" onclick="showScreen('inst')" id="tab-inst">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round">
        <rect x="3" y="3" width="18" height="18" rx="3"/>
        <line x1="8" y1="17" x2="8" y2="11"/>
        <line x1="12" y1="17" x2="12" y2="7"/>
        <line x1="16" y1="17" x2="16" y2="13"/>
      </svg>
      INSTR
    </div>
    <div class="tab" onclick="showScreen('ais')" id="tab-ais">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round">
        <circle cx="12" cy="12" r="10"/>
        <circle cx="12" cy="12" r="4"/>
        <line x1="12" y1="2" x2="12" y2="8"/>
        <line x1="12" y1="16" x2="12" y2="22"/>
        <line x1="2" y1="12" x2="8" y2="12"/>
        <line x1="16" y1="12" x2="22" y2="12"/>
      </svg>
      AIS
    </div>
  </div>

<script>
  // ══════════════════════════════════════════════
  // STATE
  // ══════════════════════════════════════════════
  const modeButtonMap = {
    'STBY':'btn-stby','AUTO':'btn-auto','WIND':'btn-wind',
    'NAV':'btn-nav','NODRIFT':'btn-nodrift','NFU':'btn-nfu'
  };

  let state = {
    mode: 'STBY', heading: -1, vessel: -1,
    depth: -1, sog: -1, cog: -1,
    aws: -1, awa: -1,
    ownLat: null, ownLon: null,
    targets: []
  };

  let nfuInterval = null;
  let wheelAngle  = 0;

  // AIS radar range
  const rangeOptions = [0.5, 1, 2, 5, 10, 20];
  let rangeIdx = 3; // default 5nm
  const RADAR_R = 170; // SVG radius in px

  // ══════════════════════════════════════════════
  // WEBSOCKET
  // ══════════════════════════════════════════════
  let ws;
  function connect() {
    ws = new WebSocket('ws://' + location.hostname + '/ws');
    ws.onopen  = () => { document.getElementById('conn').textContent = 'Connected'; };
    ws.onclose = () => {
      document.getElementById('conn').textContent = 'Disconnected \u2014 retrying...';
      stopNfu();
      setTimeout(connect, 2000);
    };
    ws.onmessage = (e) => {
      try {
        const d = JSON.parse(e.data);
        Object.assign(state, d);
        updateAllScreens();
      } catch(err) {}
    };
  }

  function send(cmd) {
    if (ws && ws.readyState === 1) ws.send(cmd);
  }

  // ══════════════════════════════════════════════
  // SCREEN SWITCHING
  // ══════════════════════════════════════════════
  function showScreen(name) {
    document.querySelectorAll('.screen').forEach(s => s.classList.remove('active'));
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.getElementById('screen-' + name).classList.add('active');
    document.getElementById('tab-' + name).classList.add('active');
    if (name !== 'ap') stopNfu();
  }

  // ══════════════════════════════════════════════
  // UPDATE ALL SCREENS FROM STATE
  // ══════════════════════════════════════════════
  function updateAllScreens() {
    updateAutopilot();
    updateInstruments();
    updateAis();
  }

  // ── Autopilot ─────────────────────────────────
  function updateAutopilot() {
    const mode = state.mode || 'STBY';
    const el = document.getElementById('mode-display');
    el.textContent = mode;
    el.className   = mode.toLowerCase().replace(' ','');

    Object.values(modeButtonMap).forEach(id =>
      document.getElementById(id).classList.remove('active'));
    const aid = modeButtonMap[mode];
    if (aid) document.getElementById(aid).classList.add('active');

    const isNfu = mode === 'NFU';
    document.getElementById('hdg-controls').style.display = isNfu ? 'none' : 'grid';
    document.getElementById('nfu-controls').className     = isNfu ? 'active' : '';
    if (!isNfu) stopNfu();

    // FIX #2: heading display — use >= 0 not > 0 so heading 0 (due north) shows correctly
    const h = parseFloat(state.heading);
    document.getElementById('heading-display').textContent =
      'HDG: ' + (h >= 0 ? h.toFixed(1) : '---') + '\u00B0';
  }

  // ── Instruments ───────────────────────────────
  function fmt(val, dp) {
    const v = parseFloat(val);
    return (v >= 0) ? v.toFixed(dp) : '---';
  }

  function updateInstruments() {
    document.getElementById('inst-depth').textContent = fmt(state.depth, 1);
    document.getElementById('inst-sog').textContent   = fmt(state.sog,   1);
    document.getElementById('inst-aws').textContent   = fmt(state.aws,   1);

    const awa = parseFloat(state.awa);
    if (awa >= 0) {
      // Normalise to -180..+180 and display with P/S suffix
      let display = awa > 180 ? awa - 360 : awa;
      const side = display >= 0 ? 'S' : 'P';
      document.getElementById('inst-awa').textContent = Math.abs(display).toFixed(0) + '\u00B0 ' + side;
      document.getElementById('awa-arrow').setAttribute('transform', 'rotate(' + awa + ',22,22)');
    } else {
      document.getElementById('inst-awa').textContent = '---';
      document.getElementById('awa-arrow').setAttribute('transform', 'rotate(0,22,22)');
    }
  }

  // ── AIS Radar ─────────────────────────────────
  function changeRange(dir) {
    rangeIdx = Math.max(0, Math.min(rangeOptions.length - 1, rangeIdx + dir));
    document.getElementById('ais-range-label').textContent = rangeOptions[rangeIdx] + ' nm';
    updateRingLabels();
    updateAis();
  }

  function updateRingLabels() {
    const r = rangeOptions[rangeIdx];
    // 4 rings at 25%, 50%, 75%, 100% of range
    ['1','2','3','4'].forEach((i, idx) => {
      const nm = (r * (idx + 1) / 4).toFixed(r < 2 ? 2 : 1);
      document.getElementById('ring-label-' + i).textContent = nm + 'nm';
    });
  }

  function latLonToRadar(lat, lon, ownLat, ownLon, rangeNm) {
    // Convert lat/lon offset to radar SVG coordinates
    const dLat = (lat - ownLat) * 60;  // nautical miles
    const dLon = (lon - ownLon) * 60 * Math.cos(ownLat * Math.PI / 180);
    const scale = RADAR_R / rangeNm;
    // North up: x = east, y = north (inverted for SVG)
    return {
      x: 170 + dLon * scale,
      y: 170 - dLat * scale
    };
  }

  function updateAis() {
    updateRingLabels();
    const grp  = document.getElementById('ais-targets-group');
    const list = document.getElementById('ais-list');
    const targets = state.targets || [];
    const rangeNm = rangeOptions[rangeIdx];

    grp.innerHTML  = '';
    list.innerHTML = '';

    if (!targets.length || !state.ownLat) {
      list.innerHTML = '<div id="ais-no-data">No AIS targets</div>';
      return;
    }

    let visibleCount = 0;
    targets.forEach(t => {
      const pos = latLonToRadar(t.lat, t.lon, state.ownLat, state.ownLon, rangeNm);
      const inRange = pos.x >= 0 && pos.x <= 340 && pos.y >= 0 && pos.y <= 340;

      // Draw COG vector and target triangle on radar
      if (inRange) {
        visibleCount++;
        // COG vector line (scaled to ~10min travel)
        if (t.sog > 0.5) {
          const vecLen = (t.sog / 60 * 10) * (RADAR_R / rangeNm); // 10min vector
          const cogRad = t.cog * Math.PI / 180;
          const vx = pos.x + Math.sin(cogRad) * vecLen;
          const vy = pos.y - Math.cos(cogRad) * vecLen;
          const line = document.createElementNS('http://www.w3.org/2000/svg','line');
          line.setAttribute('x1', pos.x); line.setAttribute('y1', pos.y);
          line.setAttribute('x2', vx);    line.setAttribute('y2', vy);
          line.setAttribute('stroke','#ff880066'); line.setAttribute('stroke-width','1.5');
          grp.appendChild(line);
        }

        // Compute bearing/distance once for both radar and list popup
        const bd = state.ownLat ? bearingDistance(state.ownLat, state.ownLon, t.lat, t.lon) : null;

        // Target triangle
        const tri = document.createElementNS('http://www.w3.org/2000/svg','polygon');
        const cogRad = (t.cog || 0) * Math.PI / 180;
        const s = 7;
        const p1x = pos.x + Math.sin(cogRad) * s;
        const p1y = pos.y - Math.cos(cogRad) * s;
        const p2x = pos.x + Math.sin(cogRad + 2.4) * (s * 0.7);
        const p2y = pos.y - Math.cos(cogRad + 2.4) * (s * 0.7);
        const p3x = pos.x + Math.sin(cogRad - 2.4) * (s * 0.7);
        const p3y = pos.y - Math.cos(cogRad - 2.4) * (s * 0.7);
        tri.setAttribute('points', p1x+','+p1y+' '+p2x+','+p2y+' '+p3x+','+p3y);
        tri.setAttribute('fill','#ff6644');
        tri.setAttribute('stroke','#ff4422');
        tri.setAttribute('stroke-width','1');
        tri.style.cursor = 'pointer';
        tri.addEventListener('click', () => openPopup(t, bd));
        grp.appendChild(tri);

        // Transparent hit area -- larger tap target than the triangle alone
        const hit = document.createElementNS('http://www.w3.org/2000/svg','circle');
        hit.setAttribute('cx', pos.x); hit.setAttribute('cy', pos.y);
        hit.setAttribute('r','14');
        hit.setAttribute('fill','transparent');
        hit.style.cursor = 'pointer';
        hit.addEventListener('click', () => openPopup(t, bd));
        grp.appendChild(hit);

        // Name label
        if (t.name) {
          const txt = document.createElementNS('http://www.w3.org/2000/svg','text');
          txt.setAttribute('x', pos.x + 9); txt.setAttribute('y', pos.y + 4);
          txt.setAttribute('font-size','9'); txt.setAttribute('fill','#aabbcc');
          txt.style.cursor = 'pointer';
          txt.textContent = t.name.trim();
          txt.addEventListener('click', () => openPopup(t, bd));
          grp.appendChild(txt);
        }
      }

      // List entry -- always shown, tappable for popup
      const row = document.createElement('div');
      row.className = 'ais-row';
      row.style.cursor = 'pointer';
      const name = (t.name && t.name.trim()) ? t.name.trim() : '---';
      const rowBd = state.ownLat ? bearingDistance(state.ownLat, state.ownLon, t.lat, t.lon) : null;
      row.innerHTML =
        '<span>' + name + '</span>' +
        '<span>' + (rowBd ? rowBd.dist.toFixed(1) + 'nm ' + rowBd.brg.toFixed(0) + '\u00B0' : '') + '</span>' +
        '<span class="mmsi">' + t.mmsi + '</span>';
      row.addEventListener('click', () => openPopup(t, rowBd));
      list.appendChild(row);
    });

    if (!visibleCount && targets.length) {
      const d = document.createElement('div');
      d.id = 'ais-no-data';
      d.textContent = targets.length + ' target(s) outside range';
      list.prepend(d);
    }
  }

  function bearingDistance(lat1, lon1, lat2, lon2) {
    const R = 3440.065; // nm
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat/2)**2 +
              Math.cos(lat1*Math.PI/180) * Math.cos(lat2*Math.PI/180) * Math.sin(dLon/2)**2;
    const dist = R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    const y = Math.sin(dLon) * Math.cos(lat2*Math.PI/180);
    const x = Math.cos(lat1*Math.PI/180)*Math.sin(lat2*Math.PI/180) -
              Math.sin(lat1*Math.PI/180)*Math.cos(lat2*Math.PI/180)*Math.cos(dLon);
    const brg = (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
    return { dist, brg };
  }

  // ══════════════════════════════════════════════
  // AIS POPUP
  // ══════════════════════════════════════════════
  function openPopup(t, bd) {
    const name = (t.name && t.name.trim()) ? t.name.trim() : 'Unknown';
    document.getElementById('ais-popup-name').textContent = name;
    const rows = [
      ['MMSI',     t.mmsi],
      ['SOG',      (t.sog != null ? t.sog.toFixed(1) + ' kn' : '---')],
      ['COG',      (t.cog != null ? t.cog.toFixed(0) + '\u00B0' : '---')],
      ['Bearing',  (bd ? bd.brg.toFixed(1) + '\u00B0' : '---')],
      ['Distance', (bd ? bd.dist.toFixed(2) + ' nm' : '---')],
      ['Position', t.lat.toFixed(4) + '\u00B0N  ' + t.lon.toFixed(4) + '\u00B0E'],
    ];
    const rowsEl = document.getElementById('ais-popup-rows');
    rowsEl.innerHTML = rows.map(([k,v]) =>
      '<div class="popup-row"><span class="popup-key">' + k +
      '</span><span class="popup-value">' + v + '</span></div>'
    ).join('');
    document.getElementById('ais-popup').classList.add('visible');
  }

  function closePopup(e) {
    if (!e || e.target === document.getElementById('ais-popup') ||
        e.target === document.getElementById('ais-popup-close')) {
      document.getElementById('ais-popup').classList.remove('visible');
    }
  }

  // ══════════════════════════════════════════════
  // WHEEL / NFU
  // ══════════════════════════════════════════════
  const wheelSvg  = document.getElementById('wheel-svg');
  const gripMarks = document.getElementById('grip-marks');
  const spokes    = document.getElementById('spokes');
  const indicator = document.getElementById('indicator');
  const portBar   = document.getElementById('rudder-bar-port');
  const stbdBar   = document.getElementById('rudder-bar-stbd');

  // MAX_ANGLE = 270: three-quarter turn gives full rudder rate.
  // Increase to make wheel less sensitive.
  const CX = 110, CY = 110, MAX_ANGLE = 270;
  let isDragging = false, lastAngle = 0;

  function getAngleFromEvent(e) {
    const rect  = wheelSvg.getBoundingClientRect();
    const touch = e.touches ? e.touches[0] : e;
    const x = touch.clientX - rect.left  - (rect.width  / 2);
    const y = touch.clientY - rect.top   - (rect.height / 2);
    return Math.atan2(x, -y) * (180 / Math.PI);
  }

  function onDragStart(e) { e.preventDefault(); isDragging = true; lastAngle = getAngleFromEvent(e); }

  function onDragMove(e) {
    if (!isDragging) return;
    e.preventDefault();
    const angle = getAngleFromEvent(e);
    let delta = angle - lastAngle;
    if (delta >  180) delta -= 360;
    if (delta < -180) delta += 360;
    lastAngle  = angle;
    wheelAngle = Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, wheelAngle + delta));
    applyWheelVisual(wheelAngle);
    applyNfuRate(wheelAngle);
  }

  function onDragEnd(e) { isDragging = false; }

  function applyWheelVisual(angle) {
    const t = 'rotate(' + angle + ',' + CX + ',' + CY + ')';
    gripMarks.setAttribute('transform', t);
    spokes.setAttribute('transform', t);
    indicator.setAttribute('transform', t);
  }

  function applyNfuRate(angle) {
    const rate = Math.round((angle / MAX_ANGLE) * 100);
    portBar.style.width = rate < 0 ? Math.abs(rate / 2) + '%' : '0';
    stbdBar.style.width = rate > 0 ? (rate / 2) + '%' : '0';
    send('NFU:' + rate);
    if (rate !== 0 && !nfuInterval) {
      nfuInterval = setInterval(() => send('NFU:' + Math.round((wheelAngle / MAX_ANGLE) * 100)), 200);
    } else if (rate === 0 && nfuInterval) {
      clearInterval(nfuInterval);
      nfuInterval = null;
    }
  }

  function stopNfu() {
    wheelAngle = 0;
    applyWheelVisual(0);
    portBar.style.width = '0';
    stbdBar.style.width = '0';
    if (nfuInterval) { clearInterval(nfuInterval); nfuInterval = null; }
    send('NFU:0');
  }

  wheelSvg.addEventListener('touchstart', onDragStart, { passive: false });
  wheelSvg.addEventListener('touchmove',  onDragMove,  { passive: false });
  wheelSvg.addEventListener('touchend',   onDragEnd);
  wheelSvg.addEventListener('mousedown',  onDragStart);
  window.addEventListener('mousemove',    onDragMove);
  window.addEventListener('mouseup',      onDragEnd);

  // Init
  updateRingLabels();
  connect();
</script>
</body>
</html>
)rawliteral";

// ─── Forward declarations ─────────────────────────────────────────────────────
void SendN2kTankLevel(double, double, tN2kFluidType);
void SendN2kTemperature(int, tN2kTempSource, double);
void HandleN2kMessage(const tN2kMsg &);
void SendModeCommand(const char*);
void SendNfuCommand(int rate);
void SendHeadingAdjust(int degrees);
void BroadcastState();
void handleWsMessage(const String&);
int  bounds(int);
int  findOrAllocAisSlot(uint32_t mmsi);
void expireAisTargets();
String buildStateJson();

// ─── NMEA2000 message handler ─────────────────────────────────────────────────
void HandleN2kMessage(const tN2kMsg &N2kMsg) {
  bool stateChanged = false;

  switch (N2kMsg.PGN) {

    case 65288: {
      // Autopilot mode — Navico/B&G
      // Byte[1]: 0=STBY,1=AUTO,2=NFU,3=NODRIFT,4=WIND,5=NAV
      uint8_t mode = N2kMsg.Data[1];
      const char* modes[] = {"STBY","AUTO","NFU","NODRIFT","WIND","NAV"};
      apMode = (mode < 6) ? modes[mode] : "----";
      stateChanged = true;
      break;
    }

    case 65359: {
      // Autopilot locked heading — Navico/B&G
      uint16_t raw = (uint16_t)N2kMsg.Data[2] | ((uint16_t)N2kMsg.Data[3] << 8);
      lockedHeading = RadToDeg(raw * 0.0001);
      stateChanged = true;
      break;
    }

    case 127250: {
      // Vessel heading
      double heading, deviation, variation;
      uint8_t ref;
      if (ParseN2kHeading(N2kMsg, ref, heading, deviation, variation)) {
        if (!N2kIsNA(heading)) { vesselHeading = RadToDeg(heading); stateChanged = true; }
      }
      break;
    }

    case 128267: {
      // Water depth
      uint8_t sid;
      double depth, offset, range;
      if (ParseN2kWaterDepth(N2kMsg, sid, depth, offset, range)) {
        if (!N2kIsNA(depth)) { depthM = depth; stateChanged = true; }
      }
      break;
    }

    case 129026: {
      // COG & SOG rapid update
      uint8_t sid, ref;
      double cog, sog;
      if (ParseN2kCOGSOGRapid(N2kMsg, sid, ref, cog, sog)) {
        if (!N2kIsNA(sog)) { sogKn  = msToKnots(sog); stateChanged = true; }
        if (!N2kIsNA(cog)) { cogDeg = RadToDeg(cog);  stateChanged = true; }
      }
      break;
    }

    case 130306: {
      // Wind speed and angle
      uint8_t sid;
      double aws, awa;
      tN2kWindReference ref;
      if (ParseN2kWindSpeed(N2kMsg, sid, aws, awa, ref)) {
        // Only use apparent wind (ref 0 = true, 1 = magnetic, 2 = apparent)
        if (ref == N2kWind_Apparent) {
          if (!N2kIsNA(aws)) { awsKn  = msToKnots(aws); stateChanged = true; }
          if (!N2kIsNA(awa)) { awaDeg = RadToDeg(awa);  stateChanged = true; }
        }
      }
      break;
    }

    case 129025: {
      // Position rapid update (own vessel)
      double lat, lon;
      if (ParseN2kPositionRapid(N2kMsg, lat, lon)) {
        if (!N2kIsNA(lat) && !N2kIsNA(lon)) {
          ownLat = lat; ownLon = lon;
          stateChanged = true;
        }
      }
      break;
    }

    case 129038: {
      // AIS Class A position report
      tN2kAISRepeat    repeat;
      uint32_t         mmsi;
      tN2kAISNavStatus navStatus;
      double           rot, sog, cog, hdg, lat, lon;
      uint8_t          raim, sid;
      if (ParseN2kAISClassAReport(N2kMsg, repeat, mmsi, lat, lon, cog, sog, hdg, rot, navStatus, raim, sid)) {
        int idx = findOrAllocAisSlot(mmsi);
        if (idx >= 0) {
          aisTargets[idx].mmsi    = mmsi;
          aisTargets[idx].lat     = lat;
          aisTargets[idx].lon     = lon;
          aisTargets[idx].cogDeg  = N2kIsNA(cog) ? 0 : RadToDeg(cog);
          aisTargets[idx].sogKn   = N2kIsNA(sog) ? 0 : msToKnots(sog);
          aisTargets[idx].lastSeen = millis();
          aisTargets[idx].active  = true;
          stateChanged = true;
        }
      }
      break;
    }

    case 129041: {
      // AIS Class B position report
      tN2kAISRepeat repeat;
      uint32_t      mmsi;
      double        sog, cog, lat, lon, hdg;
      bool          raim, assigned, dsc;
      uint8_t       sid;
      tN2kAISUnit   unit;
      tN2kAISMode   mode;
      if (ParseN2kAISClassBReport(N2kMsg, repeat, mmsi, lat, lon, cog, sog, hdg, raim, assigned, dsc, unit, mode, sid)) {
        int idx = findOrAllocAisSlot(mmsi);
        if (idx >= 0) {
          aisTargets[idx].mmsi     = mmsi;
          aisTargets[idx].lat      = lat;
          aisTargets[idx].lon      = lon;
          aisTargets[idx].cogDeg   = N2kIsNA(cog) ? 0 : RadToDeg(cog);
          aisTargets[idx].sogKn    = N2kIsNA(sog) ? 0 : msToKnots(sog);
          aisTargets[idx].lastSeen = millis();
          aisTargets[idx].active   = true;
          stateChanged = true;
        }
      }
      break;
    }

    case 129809: {
      // AIS Class B static data (name)
      tN2kAISRepeat repeat;
      uint32_t      mmsi;
      char          name[21];
      if (ParseN2kAISClassBStaticDataPartA(N2kMsg, repeat, mmsi, name)) {
        int idx = findOrAllocAisSlot(mmsi);
        if (idx >= 0) {
          strncpy(aisTargets[idx].name, name, 20);
          aisTargets[idx].name[20] = 0;
          stateChanged = true;
        }
      }
      break;
    }

    case 129794: {
      // AIS Class A static and voyage data (name)
      tN2kAISRepeat repeat;
      uint32_t      mmsi;
      uint8_t       aisVersion, imoNumber, typeOfShip;
      char          callSign[8], name[21], destination[21];
      double        length, beam, posRefStbd, posRefBow, draught;
      uint16_t      ETAdate; uint32_t ETAtime;
      tN2kGNSStype  gnssType;
      uint8_t       dte;
      tN2kAISTranceiverInfo info;
      if (ParseN2kAISClassAStaticAndVoyageRelatedData(N2kMsg, repeat, mmsi,
            imoNumber, callSign, name, typeOfShip, length, beam,
            posRefStbd, posRefBow, ETAdate, ETAtime, draught,
            destination, aisVersion, gnssType, dte, info)) {
        int idx = findOrAllocAisSlot(mmsi);
        if (idx >= 0) {
          strncpy(aisTargets[idx].name, name, 20);
          aisTargets[idx].name[20] = 0;
          stateChanged = true;
        }
      }
      break;
    }
  }

  // FIX #6: rate-limited broadcast — max 5Hz regardless of message rate
  if (stateChanged) BroadcastState();
}

// ─── AIS slot management ──────────────────────────────────────────────────────
int findOrAllocAisSlot(uint32_t mmsi) {
  // First look for existing slot with this MMSI
  for (int i = 0; i < MAX_AIS_TARGETS; i++) {
    if (aisTargets[i].active && aisTargets[i].mmsi == mmsi) return i;
  }
  // Then find a free or expired slot
  for (int i = 0; i < MAX_AIS_TARGETS; i++) {
    if (!aisTargets[i].active) {
      memset(&aisTargets[i], 0, sizeof(AisTarget));
      return i;
    }
  }
  // All slots full — evict the oldest
  int oldest = 0;
  for (int i = 1; i < MAX_AIS_TARGETS; i++) {
    if (aisTargets[i].lastSeen < aisTargets[oldest].lastSeen) oldest = i;
  }
  memset(&aisTargets[oldest], 0, sizeof(AisTarget));
  return oldest;
}

void expireAisTargets() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_AIS_TARGETS; i++) {
    if (aisTargets[i].active && (now - aisTargets[i].lastSeen) > AIS_TARGET_TIMEOUT_MS) {
      aisTargets[i].active = false;
    }
  }
}

// ─── Build state JSON for WebSocket broadcast ─────────────────────────────────
String buildStateJson() {
  // FIX #2: heading uses -1 as sentinel so client can distinguish "no data" from heading 0
  String j = "{";
  j += "\"mode\":\""   + apMode + "\",";
  j += "\"heading\":"  + String(lockedHeading, 1) + ",";
  j += "\"vessel\":"   + String(vesselHeading,  1) + ",";
  j += "\"depth\":"    + String(depthM,  1) + ",";
  j += "\"sog\":"      + String(sogKn,   1) + ",";
  j += "\"cog\":"      + String(cogDeg,  1) + ",";
  j += "\"aws\":"      + String(awsKn,   1) + ",";
  j += "\"awa\":"      + String(awaDeg,  1) + ",";
  j += "\"ownLat\":"   + String(ownLat,  6) + ",";
  j += "\"ownLon\":"   + String(ownLon,  6) + ",";
  j += "\"targets\":[";
  bool first = true;
  for (int i = 0; i < MAX_AIS_TARGETS; i++) {
    if (!aisTargets[i].active) continue;
    if (!first) j += ",";
    first = false;
    j += "{";
    j += "\"mmsi\":"  + String(aisTargets[i].mmsi)         + ",";
    j += "\"lat\":"   + String(aisTargets[i].lat,   6)     + ",";
    j += "\"lon\":"   + String(aisTargets[i].lon,   6)     + ",";
    j += "\"cog\":"   + String(aisTargets[i].cogDeg, 1)    + ",";
    j += "\"sog\":"   + String(aisTargets[i].sogKn,  1)    + ",";
    j += "\"name\":\"" + String(aisTargets[i].name) + "\"";
    j += "}";
  }
  j += "]}";
  return j;
}

// ─── Broadcast state to all WebSocket clients ─────────────────────────────────
// FIX #6: rate-limited to BROADCAST_INTERVAL_MS
void BroadcastState() {
  unsigned long now = millis();
  if (now - lastBroadcast < BROADCAST_INTERVAL_MS) return;
  lastBroadcast = now;
  ws.textAll(buildStateJson());
}

// ─── Mode command — PGN 65341 ─────────────────────────────────────────────────
// Byte[1] mode values for AP48 + Zeus:
// 0x00=STBY,0x01=AUTO,0x02=NFU,0x03=NODRIFT,0x04=WIND,0x05=NAV
// *** Verify with debug logging against your physical AP48 before use ***
void SendModeCommand(const char* mode) {
  uint8_t modeByte = 0xFF;
  if      (strcmp(mode,"STBY")    == 0) modeByte = 0x00;
  else if (strcmp(mode,"AUTO")    == 0) modeByte = 0x01;
  else if (strcmp(mode,"NFU")     == 0) modeByte = 0x02;
  else if (strcmp(mode,"NODRIFT") == 0) modeByte = 0x03;
  else if (strcmp(mode,"WIND")    == 0) modeByte = 0x04;
  else if (strcmp(mode,"NAV")     == 0) modeByte = 0x05;
  if (modeByte == 0xFF) return;

  tN2kMsg N2kMsg;
  N2kMsg.Init(6, 65341, 204, 255);
  N2kMsg.AddByte(0x01); N2kMsg.AddByte(modeByte); N2kMsg.AddByte(0x00);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  NMEA2000.SendMsg(N2kMsg);
}

// ─── NFU / Direct steering — PGN 65345 ───────────────────────────────────────
void SendNfuCommand(int rate) {
  if (rate == 0) return;
  uint8_t  dir    = (rate > 0) ? 1 : 0;
  uint16_t amount = (uint16_t)(abs(rate) * 0.1745);
  tN2kMsg N2kMsg;
  N2kMsg.Init(6, 65345, 204, 255);
  N2kMsg.AddByte(0x01); N2kMsg.AddByte(dir);
  N2kMsg.AddByte(amount & 0xFF); N2kMsg.AddByte((amount >> 8) & 0xFF);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  NMEA2000.SendMsg(N2kMsg);
}

// ─── Heading adjust — PGN 65345 ──────────────────────────────────────────────
void SendHeadingAdjust(int degrees) {
  uint16_t amount = (uint16_t)(abs(degrees) * 10000 * M_PI / 180.0);
  uint8_t  dir    = (degrees > 0) ? 1 : 0;
  tN2kMsg N2kMsg;
  N2kMsg.Init(6, 65345, 204, 255);
  N2kMsg.AddByte(0x01); N2kMsg.AddByte(dir);
  N2kMsg.AddByte(amount & 0xFF); N2kMsg.AddByte((amount >> 8) & 0xFF);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  NMEA2000.SendMsg(N2kMsg);
}

// ─── WebSocket message parser ─────────────────────────────────────────────────
void handleWsMessage(const String& msg) {
  if (msg.startsWith("MODE:")) {
    SendModeCommand(msg.substring(5).c_str());
    M5.dis.drawpix(0, 0xff8800);

  } else if (msg.startsWith("NFU:")) {
    int rate = constrain(msg.substring(4).toInt(), -100, 100);
    nfuRate    = rate;
    nfuLastMsg = millis();
    M5.dis.drawpix(0, 0xff4400);

  } else if (msg == "PORT1")  { SendHeadingAdjust(-1);  M5.dis.drawpix(0, 0x0000ff); }
  else if   (msg == "STBD1")  { SendHeadingAdjust(1);   M5.dis.drawpix(0, 0x0000ff); }
  else if   (msg == "PORT10") { SendHeadingAdjust(-10); M5.dis.drawpix(0, 0x0000ff); }
  else if   (msg == "STBD10") { SendHeadingAdjust(10);  M5.dis.drawpix(0, 0x0000ff); }
}

// ─── WebSocket event handler ──────────────────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      // FIX #1: copy into local buffer before null-terminating
      char buf[64];
      size_t copyLen = min(len, sizeof(buf) - 1);
      memcpy(buf, data, copyLen);
      buf[copyLen] = 0;
      String msg = String(buf);
      msg.trim();
      handleWsMessage(msg);
    }
    // Note: multi-packet frames silently dropped — acceptable for short commands
  }
  // FIX: kill NFU immediately on any client disconnect
  if (type == WS_EVT_DISCONNECT) {
    nfuRate = 0;
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  // FIX #4: NodeAddress and preferences removed — not used
  // FIX #3: ReceiveMessages removed — not needed, SetMsgHandler receives all

  M5.begin(true, false, true);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nSensor hub starting...");
  M5.dis.drawpix(0, 0xff0000);

  // Initialise AIS target array
  memset(aisTargets, 0, sizeof(aisTargets));

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(udpPort);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server up at http://192.168.4.1");

  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetProductInformation("00000001", 100, "Sensor Network",
                                 "1.0007 Apr 2026", "1.0007 Apr 2026");
  NMEA2000.SetDeviceInformation(1, 150, 75, 2046);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  NMEA2000.SetMsgHandler(HandleN2kMessage);

  // ── DEBUG: uncomment to sniff all NMEA2000 bus traffic ───────────────────
  // NMEA2000.SetForwardStream(&Serial);
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  // NMEA2000.SetForwardOwnMessages(false);
  // ── END DEBUG ─────────────────────────────────────────────────────────────

  NMEA2000.Open();
  Serial.println("NMEA2000 up");
  M5.dis.drawpix(0, 0x00ff00);
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
// FIX #9: delay(50) removed — loop runs free, all timing via millis()
void loop() {
  NMEA2000.ParseMessages();
  ws.cleanupClients();

  // FIX #5: NFU send rate decoupled from loop speed — sends at NFU_SEND_INTERVAL_MS
  unsigned long now = millis();
  if (nfuRate != 0) {
    if (now - nfuLastMsg > NFU_TIMEOUT_MS) {
      nfuRate = 0;
      Serial.println("NFU timeout — stopping");
    } else if (now - nfuLastSend >= NFU_SEND_INTERVAL_MS) {
      SendNfuCommand(nfuRate);
      nfuLastSend = now;
    }
  }

  // Expire stale AIS targets every 30 seconds
  static unsigned long lastExpiry = 0;
  if (now - lastExpiry > 30000) {
    expireAisTargets();
    lastExpiry = now;
  }

  // UDP sensor handling (unchanged)
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      M5.dis.drawpix(0, 0x0000ff);
      incomingPacket[len] = 0;
      String sentence = String(incomingPacket);
      Serial.print("UDP: ");
      Serial.print(sentence);

      if (sentence.startsWith("$XDR")) {
        int start = sentence.indexOf(",F,");
        int end   = sentence.indexOf(",L,", start);
        if (start > 0 && end > start) {
          String valueStr = sentence.substring(start + 3, end);
          if (sentence.indexOf("FUEL") > 0) {
            fuelLevel = bounds((int)Interpolation::Linear(fuelInput, fuelOutput, fuelValues, valueStr.toFloat(), true));
            SendN2kTankLevel(fuelLevel, fuelCapacity, N2kft_Fuel);
            Serial.print(" / fuel: "); Serial.println(fuelLevel);
          } else if (sentence.indexOf("WATER") > 0) {
            waterLevel = bounds((int)Interpolation::Linear(waterInput, waterOutput, waterValues, valueStr.toFloat(), true));
            SendN2kTankLevel(waterLevel, waterCapacity, N2kft_Water);
            Serial.print(" / water: "); Serial.println(waterLevel);
          } else if (sentence.indexOf("temp0") > 0) {
            SendN2kTemperature(1, N2kts_ExhaustGasTemperature, CToKelvin(valueStr.toDouble()));
            Serial.print(" / exhaust: "); Serial.println(valueStr);
          } else if (sentence.indexOf("temp1") > 0) {
            SendN2kTemperature(2, N2kts_EngineRoomTemperature, CToKelvin(valueStr.toDouble()));
            Serial.print(" / engine room: "); Serial.println(valueStr);
          }
        }
      }
      M5.dis.drawpix(0, 0x00ff00);
    }
  }
}

// ─── N2K send functions ───────────────────────────────────────────────────────
void SendN2kTankLevel(double Level, double Capacity, tN2kFluidType FluidType) {
  tN2kMsg N2kMsg;
  SetN2kPGN127505(N2kMsg, 1, FluidType, Level, Capacity);
  NMEA2000.SendMsg(N2kMsg);
}

void SendN2kTemperature(int i, tN2kTempSource tempSource, double val) {
  tN2kMsg N2kMsg;
  SetN2kTemperature(N2kMsg, 1, i, tempSource, val, CToKelvin(21.6));
  NMEA2000.SendMsg(N2kMsg);
}

int bounds(int val) {
  if (val < 0)   return 0;
  if (val > 100) return 100;
  return val;
}

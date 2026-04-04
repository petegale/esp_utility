
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
void buildStateJson(char* buf, size_t bufSize);
void updateAisTarget(int idx, uint32_t mmsi, double lat, double lon, double cog, double sog);
void loadSettings();
void saveSettings();
void handleSettingsMessage(const String& msg);

// ─── Persistent settings ─────────────────────────────────────────────────────
Preferences prefs;

// WiFi mode: "AP" = create hotspot, "STA" = join existing network
char wifiMode[4]     = "AP";
char apSsid[33]      = "sensor";
char apPassword[65]  = "12345678";
char staSsid[33]     = "";
char staPassword[65] = "";

// Autopilot type: "BG" (B&G/Navico), "GARMIN", "RAYMARINE", "SIMRAD", "GENERIC"
char apType[12]      = "BG";

// ─── Autopilot command configuration per type ────────────────────────────────
// PGN and byte mappings for different autopilot brands
struct ApConfig {
  unsigned long modePgn;
  unsigned long steerPgn;
  unsigned long statusPgn;
  uint8_t       srcAddr;
  uint8_t       modeStby;
  uint8_t       modeAuto;
  uint8_t       modeNfu;
  uint8_t       modeNoDrift;
  uint8_t       modeWind;
  uint8_t       modeNav;
};

ApConfig apCfg;

void applyApConfig() {
  if (strcmp(apType, "BG") == 0 || strcmp(apType, "SIMRAD") == 0) {
    // B&G / Simrad / Navico
    apCfg = {65341, 65345, 65288, 204, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
  } else if (strcmp(apType, "GARMIN") == 0) {
    // Garmin — uses same proprietary PGNs but different source conventions
    apCfg = {65341, 65345, 65288, 204, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
  } else if (strcmp(apType, "RAYMARINE") == 0) {
    // Raymarine — placeholder, same structure pending field verification
    apCfg = {65341, 65345, 65288, 204, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
  } else {
    // GENERIC fallback
    apCfg = {65341, 65345, 65288, 204, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
  }
}

// ─── UDP (existing sensor input) ──────────────────────────────────────────────
WiFiUDP udp;
const int udpPort = 10110;
char incomingPacket[255];

// ─── Web Server + WebSocket ───────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─── Tank calibration ────────────────────────────────────────────────────────
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
double ownLat = NAN;  // FIX #6: use NAN instead of -999 sentinel
double ownLon = NAN;

// ─── NFU / Direct steering state ─────────────────────────────────────────────
volatile int  nfuRate    = 0;
unsigned long nfuLastMsg = 0;
unsigned long nfuLastSend = 0;
const unsigned long NFU_TIMEOUT_MS      = 500;
const unsigned long NFU_SEND_INTERVAL_MS = 100;

// ─── Broadcast rate limiter ───────────────────────────────────────────────────
unsigned long lastBroadcast = 0;
const unsigned long BROADCAST_INTERVAL_MS = 200; // max 5Hz

// ─── Client cleanup rate limiter ─────────────────────────────────────────────
// FIX #11: Don't call cleanupClients every loop iteration
unsigned long lastCleanup = 0;
const unsigned long CLEANUP_INTERVAL_MS = 1000;

// ─── NMEA2000 ─────────────────────────────────────────────────────────────────
const unsigned long TransmitMessages[] PROGMEM = {
  127505L,  // Fluid Level
  130311L,  // Temperature
  0
};

// ─── JSON buffer ──────────────────────────────────────────────────────────────
// FIX #10: Pre-allocated buffer instead of String concatenation
// Worst case: ~200 bytes header + 20 targets * ~120 bytes each = ~2600 bytes
static char jsonBuf[3072];

// ─── HTML UI (PROGMEM) ────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Nav</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; user-select: none; -webkit-user-select: none; }

    body {
      font-family: -apple-system, sans-serif;
      background: #111;
      color: #fff;
      display: flex;
      flex-direction: column;
      height: 100vh;
      height: 100dvh;
      overflow: hidden;
    }

    /* ── Screens ── */
    #screens {
      flex: 1;
      overflow: hidden;
      display: flex;
      flex-direction: column;
    }

    .screen { display: none; flex-direction: column; flex: 1; overflow: hidden; padding: 16px 16px 0; }
    .screen.active { display: flex; }

    /* ── Tab bar ── */
    #tabbar {
      display: flex;
      background: #1a1a1a;
      border-top: 1px solid #2a2a2a;
      padding: 6px 0 10px;
      flex-shrink: 0;
    }
    .tab {
      flex: 1; display: flex; flex-direction: column;
      align-items: center; gap: 3px; padding: 6px 0;
      cursor: pointer; color: #555;
      font-size: 11px; font-weight: 600; letter-spacing: 1px;
      transition: color 0.15s;
    }
    .tab svg { width: 22px; height: 22px; }
    .tab.active { color: #fff; }

    /* ════════════════════════════════
       AUTOPILOT SCREEN
    ════════════════════════════════ */
    #screen-ap { padding-bottom: 0; }

    .ap-topbar {
      display: flex; justify-content: space-between;
      align-items: center; margin-bottom: 6px;
    }
    .ap-label { font-size: 13px; font-weight: 600; color: #666; letter-spacing: 3px; }

    #ap-status {
      font-size: 56px; font-weight: 800;
      letter-spacing: 2px; line-height: 1;
      margin-bottom: 16px; transition: color 0.2s;
      color: #cc2200;
    }

    .heading-box {
      border: 1.5px solid #333; border-radius: 4px;
      padding: 14px 18px 12px; margin-bottom: 14px;
    }
    .heading-box-label {
      font-size: 11px; color: #666; letter-spacing: 3px;
      text-align: center; margin-bottom: 4px;
    }
    #ap-hdg-value {
      font-size: 72px; font-weight: 800; color: #fff;
      text-align: center; line-height: 1; letter-spacing: -2px;
      margin-bottom: 10px;
    }
    .heading-divider { border: none; border-top: 1px solid #2a2a2a; margin-bottom: 8px; }
    .heading-secondary { display: flex; justify-content: space-between; padding: 0 4px; }
    .heading-secondary span { font-size: 13px; color: #666; letter-spacing: 1px; }
    .heading-secondary b { color: #ccc; font-weight: 700; }

    /* Adjust buttons */
    #adj-grid {
      display: grid; grid-template-columns: 1fr 1fr 1fr 1fr;
      gap: 10px; margin-bottom: 14px; transition: opacity 0.2s;
    }
    #adj-grid.disabled { opacity: 0.25; pointer-events: none; }
    #adj-grid.hidden   { display: none; }

    .adj-btn {
      border: 2px solid #aaa; border-radius: 4px; aspect-ratio: 1;
      display: flex; flex-direction: column;
      align-items: center; justify-content: center;
      cursor: pointer; gap: 5px;
    }
    .adj-btn:active { background: #222; }
    .adj-btn svg    { width: 26px; height: 26px; }
    .adj-btn span   { font-size: 11px; color: #aaa; font-weight: 600; letter-spacing: 1px; }

    /* Wheel */
    #wheel-area {
      display: none; flex-direction: column;
      align-items: center; margin-bottom: 14px; gap: 8px;
    }
    #wheel-area.visible { display: flex; }
    .wheel-hint { font-size: 10px; color: #555; letter-spacing: 2px; text-align: center; }
    #wheel-svg  { touch-action: none; cursor: grab; }

    .rudder-bar-wrap {
      width: 100%; height: 14px; background: #1a1a1a;
      border-radius: 4px; position: relative; overflow: hidden;
    }
    #rudder-bar-port { position: absolute; right: 50%; top: 0; bottom: 0; background: #2266aa; width: 0; }
    #rudder-bar-stbd { position: absolute; left:  50%; top: 0; bottom: 0; background: #772299; width: 0; }
    .rudder-centre   { position: absolute; left:  50%; top: 0; bottom: 0; width: 1px; background: #333; transform: translateX(-50%); }
    .rudder-labels   { display: flex; justify-content: space-between; font-size: 10px; color: #444; font-weight: 700; letter-spacing: 1px; width: 100%; }

    /* Bottom controls */
    .ap-bottom { margin-top: auto; padding-bottom: 14px; }

    .mode-row {
      display: flex; align-items: center;
      border: 1.5px solid #444; border-radius: 4px;
      padding: 14px 16px; margin-bottom: 10px; cursor: pointer;
    }
    .mode-row:active { background: #1a1a1a; }
    .mode-prefix { font-size: 14px; color: #888; font-weight: 600; letter-spacing: 1px; margin-right: 10px; flex-shrink: 0; }
    #mode-value  { font-size: 18px; font-weight: 800; letter-spacing: 1px; flex: 1; color: #00cc55; }

    .engage-btn {
      border: 2px solid #006622; border-radius: 4px;
      padding: 17px; display: flex; align-items: center;
      justify-content: center; gap: 14px; cursor: pointer;
      transition: background 0.15s, border-color 0.15s;
    }
    .engage-btn:active { opacity: 0.85; }
    #engage-label { font-size: 22px; font-weight: 800; color: #fff; letter-spacing: 2px; }

    #conn { font-size: 11px; color: #444; text-align: center; margin-top: 8px; }

    /* Mode popup */
    .popup-overlay {
      display: none; position: fixed; inset: 0;
      background: rgba(0,0,0,0.85); z-index: 100;
      align-items: flex-end; justify-content: center;
    }
    .popup-overlay.open { display: flex; }
    .popup-sheet {
      background: #1a1a1a; border-radius: 12px 12px 0 0;
      width: 100%; max-width: 480px; padding: 16px 20px 32px;
    }
    .popup-handle { width: 36px; height: 4px; background: #333; border-radius: 2px; margin: 0 auto 18px; }
    .popup-title  { font-size: 10px; color: #555; letter-spacing: 3px; margin-bottom: 12px; }
    .mode-popup-row {
      display: flex; align-items: center; justify-content: space-between;
      padding: 13px 14px; border-radius: 6px; margin-bottom: 7px;
      cursor: pointer; border: 1.5px solid #222;
    }
    .mode-popup-row:active { opacity: 0.8; }
    .mode-popup-name { font-size: 16px; font-weight: 800; letter-spacing: 1px; }
    .mode-popup-desc { font-size: 10px; color: #555; margin-top: 2px; }
    .mode-check {
      width: 20px; height: 20px; border-radius: 50%;
      border: 1.5px solid #333; display: flex;
      align-items: center; justify-content: center;
      flex-shrink: 0; font-size: 11px; font-weight: 800; color: #000;
    }

    /* ════════════════════════════════
       INSTRUMENTS SCREEN
    ════════════════════════════════ */
    .inst-screen-title {
      font-size: 11px; color: #555; letter-spacing: 3px;
      margin-bottom: 14px; font-weight: 600;
    }

    .inst-hero {
      border: 1.5px solid #333; border-radius: 4px;
      padding: 14px 18px; margin-bottom: 10px;
      display: flex; align-items: flex-end; justify-content: space-between;
    }
    .inst-hero-left .inst-lbl  { font-size: 11px; color: #555; letter-spacing: 2px; margin-bottom: 4px; }
    .inst-hero-left .inst-big  { font-size: 64px; font-weight: 800; color: #00cc55; line-height: 1; letter-spacing: -2px; }
    .inst-hero-left .inst-unit { font-size: 13px; color: #555; letter-spacing: 1px; margin-top: 2px; }
    .inst-hero-right { text-align: right; padding-bottom: 6px; }
    .inst-hero-right .inst-lbl  { font-size: 11px; color: #555; letter-spacing: 2px; margin-bottom: 4px; }
    .inst-hero-right .inst-sub  { font-size: 28px; font-weight: 800; color: #aaa; line-height: 1; }

    .inst-grid {
      display: grid; grid-template-columns: 1fr 1fr;
      gap: 10px; margin-bottom: 10px;
    }
    .inst-box {
      border: 1.5px solid #333; border-radius: 4px;
      padding: 14px 12px;
    }
    .inst-lbl  { font-size: 11px; color: #555; letter-spacing: 2px; margin-bottom: 4px; }
    .inst-big  { font-size: 36px; font-weight: 800; line-height: 1; letter-spacing: -1px; }
    .inst-unit { font-size: 11px; color: #555; letter-spacing: 1px; margin-top: 3px; }

    .inst-row {
      border: 1.5px solid #333; border-radius: 4px;
      padding: 14px 18px; display: flex;
      align-items: flex-end; justify-content: space-between;
    }

    .awa-arrow-wrap { display: flex; justify-content: flex-end; padding-bottom: 2px; }
    #awa-svg { width: 40px; height: 40px; }

    /* ════════════════════════════════
       AIS SCREEN
    ════════════════════════════════ */
    .ais-screen-title {
      font-size: 11px; color: #555; letter-spacing: 3px;
      margin-bottom: 10px; font-weight: 600;
    }
    #ais-range-row {
      display: flex; align-items: center; gap: 6px;
      margin-bottom: 10px; overflow-x: auto; padding-bottom: 2px;
    }
    .range-label { font-size: 10px; color: #555; letter-spacing: 1px; flex-shrink: 0; }
    .range-pills { display: flex; gap: 5px; flex-shrink: 0; }
    .range-pill {
      background: #1a1a1a; border: 1.5px solid #333;
      color: #555; border-radius: 20px; padding: 4px 12px;
      font-size: 12px; font-weight: 700; cursor: pointer;
      letter-spacing: 0.5px;
    }
    .range-pill.active { border-color: #fff; color: #fff; }

    #radar-wrap { width: 100%; aspect-ratio: 1; max-width: 100%; position: relative; }
    #radar-svg  { width: 100%; height: 100%; }

    #ais-list {
      width: 100%; margin-top: 8px;
      overflow-y: auto; flex: 1;
    }
    .ais-row {
      display: flex; justify-content: space-between; align-items: center;
      padding: 8px 0; border-bottom: 1px solid #222; cursor: pointer;
    }
    .ais-row:active { background: #1a1a1a; }
    .ais-name { font-size: 13px; font-weight: 700; color: #ff6644; }
    .ais-sub  { font-size: 10px; color: #555; margin-top: 1px; }
    .ais-dist { text-align: right; font-size: 13px; font-weight: 700; color: #ccc; }
    .ais-brg  { font-size: 10px; color: #555; text-align: right; margin-top: 1px; }
    #ais-no-data { color: #444; text-align: center; padding: 20px; font-size: 13px; }

    #ais-popup {
      display: none; position: fixed; inset: 0;
      background: rgba(0,0,0,0.85); z-index: 200;
      align-items: flex-end; justify-content: center;
    }
    #ais-popup.visible { display: flex; }
    #ais-popup-card {
      background: #1a1a1a; border-radius: 12px 12px 0 0;
      width: 100%; max-width: 480px; padding: 16px 20px 32px;
    }
    #ais-popup-name {
      font-size: 20px; font-weight: 800; color: #ff6644;
      margin-bottom: 14px; letter-spacing: 1px;
    }
    .popup-row {
      display: flex; justify-content: space-between;
      padding: 8px 0; border-bottom: 1px solid #222; font-size: 13px;
    }
    .popup-row:last-child { border-bottom: none; }
    .popup-key   { color: #555; }
    .popup-value { color: #fff; font-weight: 700; }
    #ais-popup-close {
      margin-top: 16px; width: 100%; background: #222;
      border: 1.5px solid #333; color: #ccc;
      border-radius: 4px; padding: 12px;
      font-size: 14px; font-weight: 700; cursor: pointer; letter-spacing: 1px;
    }
    #ais-popup-close:active { background: #2a2a2a; }

    /* ════════════════════════════════
       SETTINGS SCREEN
    ════════════════════════════════ */
    #screen-settings {
      overflow-y: auto; padding-bottom: 32px;
    }

    /* Back button */
    .settings-back {
      display: flex; align-items: center; gap: 8px;
      margin-bottom: 20px; cursor: pointer;
      color: #888; font-size: 13px; font-weight: 600;
      letter-spacing: 1px;
    }
    .settings-back:active { color: #fff; }
    .settings-back svg { width: 18px; height: 18px; }

    /* Section group */
    .s-group { margin-bottom: 28px; }
    .s-group-label {
      font-size: 11px; color: #555; letter-spacing: 2px;
      font-weight: 700; margin-bottom: 8px; padding: 0 4px;
    }
    .s-card {
      background: #1a1a1a; border-radius: 8px; overflow: hidden;
    }

    /* List-style fields inside card */
    .s-row {
      display: flex; align-items: center;
      padding: 14px 16px; border-bottom: 1px solid #222;
      position: relative;
    }
    .s-row:last-child { border-bottom: none; }
    .s-row-icon {
      width: 32px; height: 32px; border-radius: 8px;
      display: flex; align-items: center; justify-content: center;
      flex-shrink: 0; margin-right: 12px;
    }
    .s-row-icon svg { width: 18px; height: 18px; }
    .s-row-body { flex: 1; min-width: 0; }
    .s-row-label { font-size: 14px; color: #fff; font-weight: 500; }
    .s-row-sub   { font-size: 11px; color: #555; margin-top: 2px; }
    .s-row-value { font-size: 13px; color: #888; flex-shrink: 0; margin-left: 8px; }

    /* Status dot */
    .s-dot {
      width: 8px; height: 8px; border-radius: 50%;
      background: #444; flex-shrink: 0; margin-left: 8px;
    }
    .s-dot.on { background: #00cc55; }

    /* Inline input (full-width, floated label style) */
    .s-field-card {
      background: #1a1a1a; border-radius: 8px;
      padding: 12px 16px; margin-bottom: 2px;
    }
    .s-field-card + .s-field-card { margin-top: 2px; }
    .s-field-label {
      font-size: 11px; color: #555; letter-spacing: 1px;
      margin-bottom: 5px; font-weight: 600;
    }
    .s-input {
      width: 100%; background: transparent; border: none;
      border-bottom: 1.5px solid #333; padding: 4px 0 6px;
      color: #fff; font-size: 16px;
      font-family: -apple-system, sans-serif;
      outline: none; user-select: text; -webkit-user-select: text;
    }
    .s-input:focus { border-bottom-color: #00cc55; }
    .s-input::placeholder { color: #444; font-size: 14px; }

    /* Segmented control for radio groups */
    .s-segment {
      display: flex; background: #111;
      border-radius: 8px; padding: 3px; gap: 2px;
      margin-top: 4px;
    }
    .s-seg-btn {
      flex: 1; text-align: center; padding: 9px 4px;
      border-radius: 6px; font-size: 12px; font-weight: 700;
      color: #555; cursor: pointer; transition: all 0.15s;
      letter-spacing: 0.5px;
    }
    .s-seg-btn.active {
      background: #2a2a2a; color: #fff;
    }
    .s-seg-btn:active { opacity: 0.8; }

    /* AP type chips */
    .s-chips { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 8px; }
    .s-chip {
      background: #111; border: 1.5px solid #2a2a2a;
      border-radius: 20px; padding: 7px 14px;
      font-size: 12px; font-weight: 700; color: #555;
      cursor: pointer; transition: all 0.15s;
    }
    .s-chip.active { border-color: #00cc55; color: #00cc55; background: #051a0a; }
    .s-chip:active { opacity: 0.8; }

    /* Action buttons */
    .s-action-btn {
      display: flex; align-items: center; justify-content: center;
      gap: 10px; width: 100%; padding: 15px;
      border-radius: 8px; border: none; margin-top: 8px;
      font-size: 15px; font-weight: 700; cursor: pointer;
      letter-spacing: 0.5px; transition: opacity 0.15s;
    }
    .s-action-btn:active { opacity: 0.75; }
    .s-action-btn.primary {
      background: #00cc55; color: #000;
    }
    .s-action-btn.danger {
      background: #1a1a1a; color: #cc4400;
      border: 1.5px solid #2a2a2a;
    }
    .s-action-btn svg { width: 18px; height: 18px; }

    .s-status {
      font-size: 12px; text-align: center;
      margin-top: 10px; min-height: 18px;
    }
    .s-status.ok  { color: #00cc55; }
    .s-status.err { color: #cc4400; }

    .s-note {
      font-size: 11px; color: #444; margin-top: 8px;
      line-height: 1.5; padding: 0 4px;
    }
  </style>
</head>
<body>

<div id="screens">

  <!-- ════ AUTOPILOT ════ -->
  <div id="screen-ap" class="screen active">

    <div class="ap-topbar">
      <div class="ap-label">AUTOPILOT</div>
      <svg onclick="showScreen('settings')" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#555" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" style="cursor:pointer;">
        <circle cx="12" cy="12" r="3"/>
        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/>
      </svg>
    </div>

    <div id="ap-status">STANDBY</div>

    <div class="heading-box">
      <div class="heading-box-label">HEADING</div>
      <div id="ap-hdg-value">---&deg;</div>
      <hr class="heading-divider">
      <div class="heading-secondary">
        <span>ACTUAL: <b id="actual-hdg">---&deg;</b></span>
        <span>RUDDER: <b id="rudder-val">---</b></span>
      </div>
    </div>

    <div id="adj-grid" class="disabled">
      <div class="adj-btn" onclick="apAdjust(-10)">
        <svg viewBox="0 0 28 24" fill="none" stroke="#fff" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="16 18 9 12 16 6"/><polyline points="22 18 15 12 22 6"/>
        </svg>
        <span>10&deg;</span>
      </div>
      <div class="adj-btn" onclick="apAdjust(-1)">
        <svg viewBox="0 0 24 24" fill="none" stroke="#fff" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="15 18 9 12 15 6"/>
        </svg>
        <span>1&deg;</span>
      </div>
      <div class="adj-btn" onclick="apAdjust(1)">
        <svg viewBox="0 0 24 24" fill="none" stroke="#fff" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="9 18 15 12 9 6"/>
        </svg>
        <span>1&deg;</span>
      </div>
      <div class="adj-btn" onclick="apAdjust(10)">
        <svg viewBox="0 0 28 24" fill="none" stroke="#fff" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="6 18 13 12 6 6"/><polyline points="12 18 19 12 12 6"/>
        </svg>
        <span>10&deg;</span>
      </div>
    </div>

    <div id="wheel-area">
      <div class="wheel-hint">DRAG TO STEER &middot; RETURN TO CENTRE TO STOP</div>
      <svg id="wheel-svg" width="200" height="200" viewBox="0 0 220 220">
        <circle cx="110" cy="110" r="100" fill="#1a1a1a" stroke="#333" stroke-width="2"/>
        <g id="wg" stroke="#444" stroke-width="3" stroke-linecap="round">
          <line x1="110" y1="18" x2="110" y2="34"/><line x1="110" y1="186" x2="110" y2="202"/>
          <line x1="18" y1="110" x2="34" y2="110"/><line x1="186" y1="110" x2="202" y2="110"/>
          <line x1="39.4" y1="39.4" x2="50.8" y2="50.8"/><line x1="169.2" y1="169.2" x2="180.6" y2="180.6"/>
          <line x1="180.6" y1="39.4" x2="169.2" y2="50.8"/><line x1="50.8" y1="169.2" x2="39.4" y2="180.6"/>
        </g>
        <g id="ws" stroke="#333" stroke-width="8" stroke-linecap="round">
          <line x1="110" y1="30" x2="110" y2="80"/><line x1="110" y1="140" x2="110" y2="190"/>
          <line x1="30" y1="110" x2="80" y2="110"/><line x1="140" y1="110" x2="190" y2="110"/>
        </g>
        <circle cx="110" cy="110" r="24" fill="#111" stroke="#444" stroke-width="2"/>
        <circle id="wind-ind" cx="110" cy="35" r="8" fill="#ff8800"/>
      </svg>
      <div class="rudder-bar-wrap">
        <div id="rudder-bar-port"></div>
        <div id="rudder-bar-stbd"></div>
        <div class="rudder-centre"></div>
      </div>
      <div class="rudder-labels"><span>&laquo; PORT</span><span>STBD &raquo;</span></div>
    </div>

    <div class="ap-bottom">
      <div class="mode-row" onclick="openModePopup()">
        <span class="mode-prefix">MODE:</span>
        <span id="mode-value">HEADING</span>
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#555" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="6 9 12 15 18 9"/>
        </svg>
      </div>
      <div class="engage-btn" id="engage-btn" onclick="toggleEngage()" style="background:#111;">
        <svg id="engage-icon" width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="#fff" stroke-width="2.5" stroke-linecap="round">
          <path d="M18.36 6.64a9 9 0 1 1-12.73 0"/><line x1="12" y1="2" x2="12" y2="12"/>
        </svg>
        <span id="engage-label">ENGAGE</span>
      </div>
      <div id="conn">Connecting...</div>
    </div>
  </div>

  <!-- ════ INSTRUMENTS ════ -->
  <div id="screen-inst" class="screen">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:14px;">
      <div class="inst-screen-title" style="margin-bottom:0;">INSTRUMENTS</div>
      <svg onclick="showScreen('settings')" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#555" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" style="cursor:pointer;">
        <circle cx="12" cy="12" r="3"/>
        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/>
      </svg>
    </div>

    <div class="inst-hero">
      <div class="inst-hero-left">
        <div class="inst-lbl">SPEED OVER GROUND</div>
        <div class="inst-big" id="inst-sog" style="color:#00cc55;">---</div>
        <div class="inst-unit">knots</div>
      </div>
      <div class="inst-hero-right">
        <div class="inst-lbl">COG</div>
        <div class="inst-sub" id="inst-cog">---&deg;</div>
      </div>
    </div>

    <div class="inst-grid">
      <div class="inst-box">
        <div class="inst-lbl">WIND SPEED</div>
        <div class="inst-big" id="inst-aws" style="color:#ffcc00;">---</div>
        <div class="inst-unit">knots AWS</div>
      </div>
      <div class="inst-box">
        <div class="inst-lbl">WIND ANGLE</div>
        <div class="inst-big" id="inst-awa" style="color:#ff8800;">---</div>
        <div class="inst-unit">apparent</div>
        <div class="awa-arrow-wrap">
          <svg id="awa-svg" viewBox="0 0 44 44" xmlns="http://www.w3.org/2000/svg">
            <circle cx="22" cy="22" r="20" fill="#1a1a1a" stroke="#333" stroke-width="1.5"/>
            <g id="awa-arrow" transform="rotate(0,22,22)">
              <polygon points="22,4 26,28 22,24 18,28" fill="#ff8800"/>
            </g>
            <text x="22" y="40" text-anchor="middle" font-size="7" fill="#444">S</text>
            <text x="22" y="9"  text-anchor="middle" font-size="7" fill="#888">F</text>
            <text x="6"  y="25" text-anchor="middle" font-size="7" fill="#444">P</text>
            <text x="38" y="25" text-anchor="middle" font-size="7" fill="#444">S</text>
          </svg>
        </div>
      </div>
    </div>

    <div class="inst-row">
      <div>
        <div class="inst-lbl">DEPTH</div>
        <div class="inst-big" id="inst-depth" style="color:#44aaff;">---</div>
        <div class="inst-unit">metres</div>
      </div>
    </div>
  </div>

  <!-- ════ AIS ════ -->
  <div id="screen-ais" class="screen">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;">
      <div class="ais-screen-title" style="margin-bottom:0;">AIS RADAR</div>
      <svg onclick="showScreen('settings')" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="#555" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" style="cursor:pointer;">
        <circle cx="12" cy="12" r="3"/>
        <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1-2.83 2.83l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-4 0v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83-2.83l.06-.06A1.65 1.65 0 0 0 4.68 15a1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1 0-4h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 2.83-2.83l.06.06A1.65 1.65 0 0 0 9 4.68a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 4 0v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 2.83l-.06.06A1.65 1.65 0 0 0 19.4 9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 0 4h-.09a1.65 1.65 0 0 0-1.51 1z"/>
      </svg>
    </div>

    <div id="ais-range-row">
      <span class="range-label">RANGE</span>
      <div class="range-pills">
        <div class="range-pill" onclick="setRange(0)">0.5</div>
        <div class="range-pill" onclick="setRange(1)">1</div>
        <div class="range-pill" onclick="setRange(2)">2</div>
        <div class="range-pill active" onclick="setRange(3)">5</div>
        <div class="range-pill" onclick="setRange(4)">10</div>
        <div class="range-pill" onclick="setRange(5)">20</div>
      </div>
    </div>

    <div id="radar-wrap">
      <svg id="radar-svg" viewBox="0 0 340 340" xmlns="http://www.w3.org/2000/svg">
        <circle cx="170" cy="170" r="170" fill="#0d0d0d" stroke="#222" stroke-width="1"/>
        <circle cx="170" cy="170" r="127" fill="none" stroke="#222" stroke-width="1"/>
        <circle cx="170" cy="170" r="85"  fill="none" stroke="#222" stroke-width="1"/>
        <circle cx="170" cy="170" r="42"  fill="none" stroke="#222" stroke-width="1"/>
        <line x1="170" y1="0"   x2="170" y2="340" stroke="#222" stroke-width="1"/>
        <line x1="0"   y1="170" x2="340" y2="170" stroke="#222" stroke-width="1"/>
        <text x="173" y="48"  font-size="9" fill="#333" id="ring-label-4"></text>
        <text x="173" y="90"  font-size="9" fill="#333" id="ring-label-3"></text>
        <text x="173" y="132" font-size="9" fill="#333" id="ring-label-2"></text>
        <text x="173" y="174" font-size="9" fill="#333" id="ring-label-1"></text>
        <text id="hdg-up-label" x="170" y="13" text-anchor="middle" font-size="11" fill="#888" font-weight="bold">---&#xb0;</text>
        <line id="north-tick" x1="170" y1="0" x2="170" y2="10" stroke="#444" stroke-width="2"/>
        <polygon points="170,158 175,178 170,174 165,178" fill="#00cc55"/>
        <g id="ais-targets-group"></g>
      </svg>
    </div>

    <div id="ais-list">
      <div id="ais-no-data">No AIS targets</div>
    </div>
  </div>

  <!-- ════ SETTINGS ════ -->
  <div id="screen-settings" class="screen">

    <!-- Back -->
    <div class="settings-back" onclick="showScreen('ap')">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round">
        <polyline points="15 18 9 12 15 6"/>
      </svg>
      BACK
    </div>

    <!-- Network status -->
    <div class="s-group">
      <div class="s-group-label">NETWORK</div>
      <div class="s-card">
        <div class="s-row">
          <div class="s-row-icon" style="background:#051a0a;">
            <svg viewBox="0 0 24 24" fill="none" stroke="#00cc55" stroke-width="1.8" stroke-linecap="round">
              <path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/>
              <path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><line x1="12" y1="20" x2="12.01" y2="20" stroke-width="3"/>
            </svg>
          </div>
          <div class="s-row-body">
            <div class="s-row-label" id="s-wifi-info">---</div>
            <div class="s-row-sub"  id="s-wifi-ip"></div>
          </div>
          <div class="s-dot" id="s-wifi-dot"></div>
        </div>
      </div>
    </div>

    <!-- WiFi mode -->
    <div class="s-group">
      <div class="s-group-label">WIFI MODE</div>
      <div class="s-card" style="padding:14px 16px;">
        <div class="s-segment" id="wifi-mode-group">
          <div class="s-seg-btn active" data-val="AP"  onclick="setWifiMode('AP')">Access Point</div>
          <div class="s-seg-btn"        data-val="STA" onclick="setWifiMode('STA')">Join Network</div>
        </div>
      </div>
    </div>

    <!-- AP credentials -->
    <div class="s-group" id="ap-settings">
      <div class="s-group-label">ACCESS POINT</div>
      <div class="s-field-card">
        <div class="s-field-label">NETWORK NAME</div>
        <input class="s-input" id="s-ap-ssid" type="text" placeholder="sensor" maxlength="32" autocomplete="off" autocapitalize="none">
      </div>
      <div class="s-field-card">
        <div class="s-field-label">PASSWORD</div>
        <input class="s-input" id="s-ap-pass" type="password" placeholder="min 8 characters" maxlength="63" autocomplete="off">
      </div>
      <div class="s-note">Other devices connect directly to this hotspot. Leave password blank for an open network.</div>
    </div>

    <!-- STA credentials -->
    <div class="s-group" id="sta-settings" style="display:none;">
      <div class="s-group-label">NETWORK TO JOIN</div>
      <div class="s-field-card">
        <div class="s-field-label">NETWORK NAME (SSID)</div>
        <input class="s-input" id="s-sta-ssid" type="text" placeholder="your-wifi-name" maxlength="32" autocomplete="off" autocapitalize="none">
      </div>
      <div class="s-field-card">
        <div class="s-field-label">PASSWORD</div>
        <input class="s-input" id="s-sta-pass" type="password" placeholder="wifi password" maxlength="63" autocomplete="off">
      </div>
      <div class="s-note">If the device cannot connect it will fall back to Access Point mode automatically.</div>
    </div>

    <!-- Autopilot type -->
    <div class="s-group">
      <div class="s-group-label">AUTOPILOT</div>
      <div class="s-card" style="padding:14px 16px;">
        <div class="s-chips" id="ap-type-group">
          <div class="s-chip active" data-val="BG"        onclick="setApType('BG')">B&amp;G / Navico</div>
          <div class="s-chip"        data-val="SIMRAD"    onclick="setApType('SIMRAD')">Simrad</div>
          <div class="s-chip"        data-val="GARMIN"    onclick="setApType('GARMIN')">Garmin</div>
          <div class="s-chip"        data-val="RAYMARINE" onclick="setApType('RAYMARINE')">Raymarine</div>
          <div class="s-chip"        data-val="GENERIC"   onclick="setApType('GENERIC')">Generic</div>
        </div>
      </div>
      <div class="s-note">Configures NMEA2000 proprietary PGNs for mode commands. Verify with debug logging before use at sea.</div>
    </div>

    <!-- Actions -->
    <button class="s-action-btn primary" onclick="saveSettings()">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round">
        <path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/>
        <polyline points="17 21 17 13 7 13 7 21"/>
        <polyline points="7 3 7 8 15 8"/>
      </svg>
      Save &amp; Reboot
    </button>
    <button class="s-action-btn danger" onclick="factoryReset()">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round">
        <polyline points="1 4 1 10 7 10"/>
        <path d="M3.51 15a9 9 0 1 0 .49-4"/>
      </svg>
      Factory Reset
    </button>
    <div class="s-status" id="s-status"></div>

  </div>



</div><!-- /screens -->

<!-- Tab bar -->
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

<!-- Mode popup -->
<div class="popup-overlay" id="mode-popup" onclick="if(event.target===this)closeModePopup()">
  <div class="popup-sheet">
    <div class="popup-handle"></div>
    <div class="popup-title">SELECT MODE</div>
    <div id="mode-popup-list"></div>
  </div>
</div>

<!-- AIS target popup -->
<div id="ais-popup" onclick="closeAisPopup(event)">
  <div id="ais-popup-card">
    <div id="ais-popup-name">---</div>
    <div id="ais-popup-rows"></div>
    <button id="ais-popup-close" onclick="closeAisPopup()">CLOSE</button>
  </div>
</div>

<script>
// ════════════════════════════════
// WEBSOCKET
// ════════════════════════════════
let ws;
function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen  = () => { document.getElementById('conn').textContent = 'Connected'; };
  ws.onclose = () => {
    document.getElementById('conn').textContent = 'Disconnected \u2014 retrying...';
    apEngaged = false; updateEngageState();
    setTimeout(connect, 2000);
  };
  ws.onmessage = (e) => {
    try {
      const d = JSON.parse(e.data);
      if (d.settings !== undefined) {
        applySettingsFromServer(d.settings);
      } else {
        updateFromServer(d);
      }
    } catch(err) {}
  };
}

function wsSend(cmd) { if (ws && ws.readyState === 1) ws.send(cmd); }

// FIX #4: Send NFU:0 on page unload to stop steering immediately
window.addEventListener('beforeunload', function() {
  if (nfuRate !== 0) { wsSend('NFU:0'); }
});

function updateFromServer(d) {
  // Autopilot
  if (d.mode !== undefined) {
    const modeMap = { STBY:'STANDBY', AUTO:'HEADING', NFU:'DIRECT', NODRIFT:'NO DRIFT', WIND:'WIND', NAV:'NAV' };
    const display = modeMap[d.mode] || d.mode;
    if (d.mode === 'STBY') {
      apEngaged = false; updateEngageState();
    } else {
      apEngaged = true;
      currentMode = display;
      setModeDisplay(display);
      updateEngageState();
    }
  }
  if (d.vessel !== undefined && d.vessel >= 0) {
    vesselHeading = d.vessel;
  }
  // FIX #3: Use server-provided heading, not hardcoded value
  if (d.heading >= 0) {
    document.getElementById('ap-hdg-value').textContent  = d.heading.toFixed(1) + '\u00B0';
    document.getElementById('actual-hdg').textContent    = d.vessel >= 0 ? d.vessel.toFixed(1) + '\u00B0' : '---\u00B0';
  }
  // Instruments
  if (d.sog   !== undefined && d.sog   >= 0) document.getElementById('inst-sog').textContent   = d.sog.toFixed(1);
  if (d.cog   !== undefined && d.cog   >= 0) document.getElementById('inst-cog').textContent   = d.cog.toFixed(0) + '\u00B0';
  if (d.aws   !== undefined && d.aws   >= 0) document.getElementById('inst-aws').textContent   = d.aws.toFixed(1);
  if (d.depth !== undefined && d.depth >= 0) document.getElementById('inst-depth').textContent = d.depth.toFixed(1);
  if (d.awa   !== undefined && d.awa   >= 0) {
    let awa = d.awa; let side = awa > 180 ? 'P' : 'S';
    let disp = awa > 180 ? 360 - awa : awa;
    document.getElementById('inst-awa').textContent = disp.toFixed(0) + '\u00B0 ' + side;
    document.getElementById('awa-arrow').setAttribute('transform', 'rotate(' + awa + ',22,22)');
  }
  // FIX #6: Check for NaN/null own position before setting
  if (d.ownLat !== undefined && d.ownLat !== null && isFinite(d.ownLat)) { ownLat = d.ownLat; ownLon = d.ownLon; }
  if (d.targets !== undefined) { aisTargets = d.targets; updateAisDisplay(); }
}

// ════════════════════════════════
// SCREEN SWITCHING
// ════════════════════════════════
function showScreen(name) {
  document.querySelectorAll('.screen').forEach(s => s.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
  document.getElementById('screen-' + name).classList.add('active');
  const tabEl = document.getElementById('tab-' + name);
  if (tabEl) tabEl.classList.add('active');
  if (name !== 'ap') stopNfu();
  if (name === 'settings') requestSettings();
}

// ════════════════════════════════
// AUTOPILOT
// ════════════════════════════════
const AP_MODES = [
  { key:'HEADING',  wsKey:'AUTO',    color:'#00cc55', bg:'#051a0a', bd:'#006622', desc:'Hold compass heading' },
  { key:'WIND',     wsKey:'WIND',    color:'#44aaff', bg:'#021833', bd:'#1a5a99', desc:'Hold apparent wind angle' },
  { key:'NO DRIFT', wsKey:'NODRIFT', color:'#aa88ff', bg:'#150a2a', bd:'#5533aa', desc:'Hold GPS track over ground' },
  { key:'NAV',      wsKey:'NAV',     color:'#ffcc00', bg:'#1a1500', bd:'#887700', desc:'Follow active GPS route' },
  { key:'DIRECT',   wsKey:'NFU',     color:'#ff8800', bg:'#1a0d00', bd:'#884400', desc:'Steering wheel control' },
];

let apEngaged    = false;
let currentMode  = 'HEADING';
let nfuRate      = 0;
let nfuInterval  = null;
let wheelAngle   = 0;
let wDragging    = false;
let wLastAng     = 0;
const MAX_ANG    = 270;
const WCX = 110, WCY = 110;

function getModeObj(key) { return AP_MODES.find(m => m.key === key); }

function setModeDisplay(key) {
  const m = getModeObj(key);
  if (!m) return;
  document.getElementById('mode-value').textContent  = key;
  document.getElementById('mode-value').style.color  = m.color;
}

function updateControls() {
  const adj   = document.getElementById('adj-grid');
  const wheel = document.getElementById('wheel-area');
  const isDirect = currentMode === 'DIRECT';
  if (isDirect) {
    adj.classList.add('hidden');
    wheel.classList.add('visible');
  } else {
    wheel.classList.remove('visible');
    adj.classList.remove('hidden');
    adj.classList.toggle('disabled', !apEngaged);
  }
}

function updateEngageState() {
  const btn    = document.getElementById('engage-btn');
  const lbl    = document.getElementById('engage-label');
  const icon   = document.getElementById('engage-icon');
  const status = document.getElementById('ap-status');
  const m      = getModeObj(currentMode);
  if (apEngaged) {
    btn.style.borderColor = '#880000';
    btn.style.background  = '#1a0000';
    lbl.textContent = 'STANDBY';
    icon.setAttribute('stroke', '#ff4422');
    status.textContent = currentMode;
    status.style.color = m ? m.color : '#fff';
  } else {
    btn.style.borderColor = '#006622';
    btn.style.background  = '#111';
    lbl.textContent = 'ENGAGE';
    icon.setAttribute('stroke', '#fff');
    status.textContent = 'STANDBY';
    status.style.color = '#cc2200';
    document.getElementById('ap-hdg-value').textContent  = '---\u00B0';
    document.getElementById('actual-hdg').textContent    = '---\u00B0';
    document.getElementById('rudder-val').textContent    = '---';
  }
  updateControls();
}

function toggleEngage() {
  apEngaged = !apEngaged;
  if (apEngaged) {
    const m = getModeObj(currentMode);
    wsSend('MODE:' + (m ? m.wsKey : 'AUTO'));
    // FIX #3: Don't hardcode heading — let server state update the display
  } else {
    stopNfu();
    wsSend('MODE:STBY');
  }
  updateEngageState();
}

function apAdjust(deg) {
  if (!apEngaged || currentMode === 'DIRECT') return;
  if (deg === -1)  wsSend('PORT1');
  if (deg === 1)   wsSend('STBD1');
  if (deg === -10) wsSend('PORT10');
  if (deg === 10)  wsSend('STBD10');
}

// ── Mode popup ──────────────────────────────────
function buildModePopup() {
  const list = document.getElementById('mode-popup-list');
  list.innerHTML = '';
  AP_MODES.forEach(m => {
    const active = m.key === currentMode;
    const row = document.createElement('div');
    row.className = 'mode-popup-row';
    row.style.background  = active ? m.bg  : 'transparent';
    row.style.borderColor = active ? m.bd  : '#222';
    row.innerHTML =
      '<div><div class="mode-popup-name" style="color:' + m.color + '">' + m.key + '</div>' +
      '<div class="mode-popup-desc">' + m.desc + '</div></div>' +
      '<div class="mode-check" style="' + (active ? 'background:'+m.color+';border:none;' : '') + '">' +
      (active ? '\u2713' : '') + '</div>';
    row.onclick = () => selectMode(m.key);
    list.appendChild(row);
  });
}

function openModePopup()  { buildModePopup(); document.getElementById('mode-popup').classList.add('open'); }
function closeModePopup() { document.getElementById('mode-popup').classList.remove('open'); }

function selectMode(key) {
  currentMode = key;
  setModeDisplay(key);
  const m = getModeObj(key);
  if (apEngaged && m) {
    wsSend('MODE:' + m.wsKey);
    document.getElementById('ap-status').textContent = key;
    document.getElementById('ap-status').style.color = m.color;
  }
  updateControls();
  setTimeout(closeModePopup, 200);
}

// ── NFU wheel ───────────────────────────────────
function stopNfu() {
  nfuRate = 0; wheelAngle = 0;
  rotateWheel(0); updateBars(0);
  if (nfuInterval) { clearInterval(nfuInterval); nfuInterval = null; }
  wsSend('NFU:0');
}

function rotateWheel(a) {
  const t = 'rotate(' + a + ',' + WCX + ',' + WCY + ')';
  ['wg','ws','wind-ind'].forEach(id => { const el = document.getElementById(id); if (el) el.setAttribute('transform', t); });
}

function updateBars(a) {
  const rate = Math.round((a / MAX_ANG) * 100);
  document.getElementById('rudder-bar-port').style.width = rate < 0 ? Math.abs(rate / 2) + '%' : '0';
  document.getElementById('rudder-bar-stbd').style.width = rate > 0 ? (rate / 2) + '%' : '0';
}

function getWheelAng(e) {
  const wsvg = document.getElementById('wheel-svg');
  const r = wsvg.getBoundingClientRect();
  const t = e.touches ? e.touches[0] : e;
  return Math.atan2(t.clientX - r.left - r.width / 2, -(t.clientY - r.top - r.height / 2)) * 180 / Math.PI;
}

function onWheelStart(e) { e.preventDefault(); wDragging = true; wLastAng = getWheelAng(e); }
function onWheelMove(e) {
  if (!wDragging) return; e.preventDefault();
  let d = getWheelAng(e) - wLastAng; wLastAng = getWheelAng(e);
  if (d > 180) d -= 360; if (d < -180) d += 360;
  wheelAngle = Math.max(-MAX_ANG, Math.min(MAX_ANG, wheelAngle + d));
  rotateWheel(wheelAngle); updateBars(wheelAngle);
  const rate = Math.round((wheelAngle / MAX_ANG) * 100);
  nfuRate = rate;
  wsSend('NFU:' + rate);
  if (rate !== 0 && !nfuInterval) {
    nfuInterval = setInterval(() => wsSend('NFU:' + Math.round((wheelAngle / MAX_ANG) * 100)), 200);
  } else if (rate === 0 && nfuInterval) {
    clearInterval(nfuInterval); nfuInterval = null;
  }
}
function onWheelEnd() { wDragging = false; }

const wsvg = document.getElementById('wheel-svg');
wsvg.addEventListener('mousedown',  onWheelStart);
wsvg.addEventListener('touchstart', onWheelStart, { passive: false });
window.addEventListener('mousemove', onWheelMove);
wsvg.addEventListener('touchmove',  onWheelMove, { passive: false });
window.addEventListener('mouseup',  onWheelEnd);
wsvg.addEventListener('touchend',   onWheelEnd);

// ════════════════════════════════
// AIS
// ════════════════════════════════
const RANGE_OPTIONS = [0.5, 1, 2, 5, 10, 20];
let rangeIdx      = 3;
let ownLat        = null, ownLon = null;
let aisTargets    = [];
let vesselHeading = null;

function setRange(idx) {
  rangeIdx = idx;
  document.querySelectorAll('.range-pill').forEach((p, i) => p.classList.toggle('active', i === idx));
  updateRingLabels();
  updateAisDisplay();
}

function updateRingLabels() {
  const r = RANGE_OPTIONS[rangeIdx];
  [1,2,3,4].forEach((n, i) => {
    const nm = (r * (i + 1) / 4).toFixed(r < 2 ? 2 : 1);
    document.getElementById('ring-label-' + n).textContent = nm + 'nm';
  });
}

function latLonToRadar(lat, lon) {
  if (!ownLat || !isFinite(ownLat)) return null; // FIX #6
  const dLat = (lat - ownLat) * 60;
  const dLon = (lon - ownLon) * 60 * Math.cos(ownLat * Math.PI / 180);
  const scale = 170 / RANGE_OPTIONS[rangeIdx];
  const nx = dLon * scale;
  const ny = -dLat * scale;
  const hdgRad = (vesselHeading || 0) * Math.PI / 180;
  const rx =  nx * Math.cos(-hdgRad) - ny * Math.sin(-hdgRad);
  const ry =  nx * Math.sin(-hdgRad) + ny * Math.cos(-hdgRad);
  return { x: 170 + rx, y: 170 + ry };
}

function bearingDistance(lat1, lon1, lat2, lon2) {
  const R = 3440.065;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(dLat/2)**2 + Math.cos(lat1*Math.PI/180)*Math.cos(lat2*Math.PI/180)*Math.sin(dLon/2)**2;
  const dist = R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  const y = Math.sin(dLon)*Math.cos(lat2*Math.PI/180);
  const x = Math.cos(lat1*Math.PI/180)*Math.sin(lat2*Math.PI/180) - Math.sin(lat1*Math.PI/180)*Math.cos(lat2*Math.PI/180)*Math.cos(dLon);
  const brg = (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
  return { dist, brg };
}

function updateAisDisplay() {
  updateRingLabels();
  const hdgLbl = document.getElementById('hdg-up-label');
  if (hdgLbl) hdgLbl.textContent = vesselHeading !== null ? Math.round(vesselHeading) + '\u00B0' : '---\u00B0';
  const northTick = document.getElementById('north-tick');
  if (northTick) {
    const ntAngle = -(vesselHeading || 0);
    const cx = 170, cy = 170, r = 170;
    const rad = (ntAngle - 90) * Math.PI / 180;
    const x1 = cx + r * Math.cos(rad);
    const y1 = cy + r * Math.sin(rad);
    const x2 = cx + (r - 10) * Math.cos(rad);
    const y2 = cy + (r - 10) * Math.sin(rad);
    northTick.setAttribute('x1', x1.toFixed(1)); northTick.setAttribute('y1', y1.toFixed(1));
    northTick.setAttribute('x2', x2.toFixed(1)); northTick.setAttribute('y2', y2.toFixed(1));
  }
  const grp  = document.getElementById('ais-targets-group');
  const list = document.getElementById('ais-list');
  grp.innerHTML = ''; list.innerHTML = '';

  if (!aisTargets.length) {
    list.innerHTML = '<div id="ais-no-data">No AIS targets</div>'; return;
  }

  aisTargets.forEach(t => {
    const pos = latLonToRadar(t.lat, t.lon);
    const bd  = (ownLat && isFinite(ownLat)) ? bearingDistance(ownLat, ownLon, t.lat, t.lon) : null;
    const inRange = pos && pos.x >= 0 && pos.x <= 340 && pos.y >= 0 && pos.y <= 340;

    if (inRange && pos) {
      const cogRad = ((t.cog || 0) - (vesselHeading || 0)) * Math.PI / 180;
      const s = 8;
      if ((t.sog || 0) > 0.5) {
        const vl = ((t.sog || 0) / 60 * 10) * (170 / RANGE_OPTIONS[rangeIdx]);
        const line = document.createElementNS('http://www.w3.org/2000/svg','line');
        line.setAttribute('x1', pos.x); line.setAttribute('y1', pos.y);
        line.setAttribute('x2', pos.x + Math.sin(cogRad)*vl);
        line.setAttribute('y2', pos.y - Math.cos(cogRad)*vl);
        line.setAttribute('stroke','#ff660044'); line.setAttribute('stroke-width','1.5');
        grp.appendChild(line);
      }
      const p1x = pos.x + Math.sin(cogRad)*s, p1y = pos.y - Math.cos(cogRad)*s;
      const p2x = pos.x + Math.sin(cogRad+2.4)*(s*.7), p2y = pos.y - Math.cos(cogRad+2.4)*(s*.7);
      const p3x = pos.x + Math.sin(cogRad-2.4)*(s*.7), p3y = pos.y - Math.cos(cogRad-2.4)*(s*.7);
      const tri = document.createElementNS('http://www.w3.org/2000/svg','polygon');
      tri.setAttribute('points', p1x+','+p1y+' '+p2x+','+p2y+' '+p3x+','+p3y);
      tri.setAttribute('fill','#ff6644'); tri.setAttribute('stroke','#ff4422'); tri.setAttribute('stroke-width','1');
      tri.style.cursor = 'pointer';
      tri.addEventListener('click', () => openAisPopup(t, bd));
      grp.appendChild(tri);
      const hit = document.createElementNS('http://www.w3.org/2000/svg','circle');
      hit.setAttribute('cx', pos.x); hit.setAttribute('cy', pos.y); hit.setAttribute('r','14');
      hit.setAttribute('fill','transparent'); hit.style.cursor = 'pointer';
      hit.addEventListener('click', () => openAisPopup(t, bd));
      grp.appendChild(hit);
      if (t.name && t.name.trim()) {
        const txt = document.createElementNS('http://www.w3.org/2000/svg','text');
        txt.setAttribute('x', pos.x + 10); txt.setAttribute('y', pos.y + 4);
        txt.setAttribute('font-size','9'); txt.setAttribute('fill','#888');
        txt.style.cursor = 'pointer';
        txt.textContent = t.name.trim();
        txt.addEventListener('click', () => openAisPopup(t, bd));
        grp.appendChild(txt);
      }
    }

    const row = document.createElement('div');
    row.className = 'ais-row';
    const name = (t.name && t.name.trim()) ? t.name.trim() : '---';
    const sog  = (t.sog || 0).toFixed(1);
    const cog  = (t.cog || 0).toFixed(0);
    row.innerHTML =
      '<div><div class="ais-name">' + name + '</div><div class="ais-sub">' + sog + ' kn &middot; ' + cog + '\u00B0</div></div>' +
      '<div><div class="ais-dist">' + (bd ? bd.dist.toFixed(1) + ' nm' : '---') + '</div>' +
      '<div class="ais-brg">' + (bd ? bd.brg.toFixed(0) + '\u00B0' : '') + '</div></div>';
    row.addEventListener('click', () => openAisPopup(t, bd));
    list.appendChild(row);
  });
}

function openAisPopup(t, bd) {
  const name = (t.name && t.name.trim()) ? t.name.trim() : 'Unknown';
  document.getElementById('ais-popup-name').textContent = name;
  const rows = [
    ['MMSI',     t.mmsi || '---'],
    ['SOG',      ((t.sog || 0).toFixed(1)) + ' kn'],
    ['COG',      ((t.cog || 0).toFixed(0)) + '\u00B0'],
    ['Bearing',  bd ? bd.brg.toFixed(1) + '\u00B0' : '---'],
    ['Distance', bd ? bd.dist.toFixed(2) + ' nm' : '---'],
    ['Position', (t.lat||0).toFixed(4) + '\u00B0N \u00A0 ' + (t.lon||0).toFixed(4) + '\u00B0E'],
  ];
  document.getElementById('ais-popup-rows').innerHTML = rows.map(([k,v]) =>
    '<div class="popup-row"><span class="popup-key">' + k + '</span><span class="popup-value">' + v + '</span></div>'
  ).join('');
  document.getElementById('ais-popup').classList.add('visible');
}

function closeAisPopup(e) {
  if (!e || e.target === document.getElementById('ais-popup') || e.target === document.getElementById('ais-popup-close')) {
    document.getElementById('ais-popup').classList.remove('visible');
  }
}

// ════════════════════════════════
// SETTINGS
// ════════════════════════════════
let settingsWifiMode = 'AP';
let settingsApType   = 'BG';

function requestSettings() {
  wsSend('SETTINGS:GET');
}

function applySettingsFromServer(s) {
  if (s.wifiMode) {
    settingsWifiMode = s.wifiMode;
    setWifiMode(s.wifiMode, true);
  }
  if (s.apSsid)     document.getElementById('s-ap-ssid').value = s.apSsid;
  if (s.apPassword)  document.getElementById('s-ap-pass').value = s.apPassword;
  if (s.staSsid)     document.getElementById('s-sta-ssid').value = s.staSsid;
  if (s.staPassword) document.getElementById('s-sta-pass').value = s.staPassword;
  if (s.apType) {
    settingsApType = s.apType;
    setApType(s.apType, true);
  }
  // Update wifi status display
  if (s.wifiConnected !== undefined) {
    const dot = document.getElementById('s-wifi-dot');
    const info = document.getElementById('s-wifi-info');
    const ip = document.getElementById('s-wifi-ip');
    dot.classList.toggle('connected', s.wifiConnected);
    if (s.wifiMode === 'AP') {
      info.textContent = 'Access Point: ' + (s.apSsid || 'sensor');
      ip.textContent = s.ip || '192.168.4.1';
    } else {
      info.textContent = s.wifiConnected ? ('Connected to ' + (s.staSsid || '')) : 'Disconnected';
      ip.textContent = s.ip || '';
    }
  }
}

function setWifiMode(mode, noSend) {
  settingsWifiMode = mode;
  document.querySelectorAll('#wifi-mode-group .s-seg-btn').forEach(el => {
    el.classList.toggle('active', el.dataset.val === mode);
  });
  document.getElementById('ap-settings').style.display  = mode === 'AP'  ? 'block' : 'none';
  document.getElementById('sta-settings').style.display = mode === 'STA' ? 'block' : 'none';
}

function setApType(type, noSend) {
  settingsApType = type;
  document.querySelectorAll('#ap-type-group .s-chip').forEach(el => {
    el.classList.toggle('active', el.dataset.val === type);
  });
}

function saveSettings() {
  const payload = {
    wifiMode:    settingsWifiMode,
    apSsid:      document.getElementById('s-ap-ssid').value.trim(),
    apPassword:  document.getElementById('s-ap-pass').value,
    staSsid:     document.getElementById('s-sta-ssid').value.trim(),
    staPassword: document.getElementById('s-sta-pass').value,
    apType:      settingsApType
  };

  // Validate
  if (settingsWifiMode === 'AP') {
    if (!payload.apSsid) { showStatus('Network name required', true); return; }
    if (payload.apPassword.length > 0 && payload.apPassword.length < 8) {
      showStatus('Password must be at least 8 characters', true); return;
    }
  } else {
    if (!payload.staSsid) { showStatus('SSID required', true); return; }
  }

  wsSend('SETTINGS:' + JSON.stringify(payload));
  showStatus('Saving... device will reboot', false);
}

function factoryReset() {
  if (confirm('Reset all settings to defaults? The device will reboot.')) {
    wsSend('SETTINGS:RESET');
    showStatus('Resetting... device will reboot', false);
  }
}

function showStatus(msg, isError) {
  const el = document.getElementById('s-status');
  el.textContent = msg;
  el.className = 's-status ' + (isError ? 'err' : 'ok');
  if (!isError) setTimeout(() => { el.textContent = ''; el.className = 's-status'; }, 5000);
}

// ── Init ──────────────────────────────────────
updateRingLabels();
setModeDisplay('HEADING');
connect();
</script>
</body>
</html>

)rawliteral";

// ─── NMEA2000 message handler ─────────────────────────────────────────────────
void HandleN2kMessage(const tN2kMsg &N2kMsg) {
  bool stateChanged = false;

  switch (N2kMsg.PGN) {

    case 65288: {
      // Autopilot mode — Navico/B&G (also used as generic status PGN)
      uint8_t mode = N2kMsg.Data[1];
      const char* modes[] = {"STBY","AUTO","NFU","NODRIFT","WIND","NAV"};
      apMode = (mode < 6) ? modes[mode] : "----";
      stateChanged = true;
      break;
    }

    case 65359: {
      // Autopilot locked heading
      uint16_t raw = (uint16_t)N2kMsg.Data[2] | ((uint16_t)N2kMsg.Data[3] << 8);
      lockedHeading = RadToDeg(raw * 0.0001);
      stateChanged = true;
      break;
    }

    case 127250: {
      // Vessel heading
      uint8_t sid;
      double heading, deviation, variation;
      tN2kHeadingReference ref;
      if (ParseN2kHeading(N2kMsg, sid, heading, deviation, variation, ref)) {
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
      uint8_t sid;
      tN2kHeadingReference ref;
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
      uint8_t          msgId, seconds;
      tN2kAISRepeat    repeat;
      uint32_t         mmsi;
      double           lat, lon, cog, sog, hdg, rot;
      bool             accuracy, raim;
      tN2kAISNavStatus navStatus;
      if (ParseN2kAISClassAPosition(N2kMsg, msgId, repeat, mmsi,
            lat, lon, accuracy, raim, seconds,
            cog, sog, hdg, rot, navStatus)) {
        int idx = findOrAllocAisSlot(mmsi);
        if (idx >= 0) {
          // FIX #13: Deduplicated into helper
          updateAisTarget(idx, mmsi, lat, lon,
            N2kIsNA(cog) ? 0 : RadToDeg(cog),
            N2kIsNA(sog) ? 0 : msToKnots(sog));
          stateChanged = true;
        }
      }
      break;
    }

    case 129041: {
      // AIS Class B position report
      uint8_t     msgId, seconds;
      tN2kAISRepeat repeat;
      uint32_t    mmsi;
      double      lat, lon, cog, sog, hdg;
      bool        accuracy, raim, display, dsc, band, msg22, assigned;
      tN2kAISUnit unit;
      tN2kAISMode mode;
      if (ParseN2kAISClassBPosition(N2kMsg, msgId, repeat, mmsi,
            lat, lon, accuracy, raim, seconds,
            cog, sog, hdg, unit, display, dsc, band, msg22, mode, assigned)) {
        int idx = findOrAllocAisSlot(mmsi);
        if (idx >= 0) {
          // FIX #13: Deduplicated into helper
          updateAisTarget(idx, mmsi, lat, lon,
            N2kIsNA(cog) ? 0 : RadToDeg(cog),
            N2kIsNA(sog) ? 0 : msToKnots(sog));
          stateChanged = true;
        }
      }
      break;
    }

    case 129809: {
      // AIS Class B static data (name)
      uint8_t       msgId;
      tN2kAISRepeat repeat;
      uint32_t      mmsi;
      char          name[21];
      if (ParseN2kAISClassBStaticPartA(N2kMsg, msgId, repeat, mmsi, name, sizeof(name))) {
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
      uint8_t        msgId;
      tN2kAISRepeat  repeat;
      uint32_t       mmsi, imoNumber;
      char           callSign[8], name[21], destination[21];
      uint8_t        typeOfShip;
      double         length, beam, posRefStbd, posRefBow, draught, ETAtime;
      uint16_t       ETAdate;
      tN2kAISVersion aisVersion;
      tN2kGNSStype   gnssType;
      tN2kAISDTE     dte;
      if (ParseN2kAISClassAStatic(N2kMsg, msgId, repeat, mmsi,
            imoNumber, callSign, sizeof(callSign), name, sizeof(name),
            typeOfShip, length, beam, posRefStbd, posRefBow,
            ETAdate, ETAtime, draught,
            destination, sizeof(destination),
            aisVersion, gnssType, dte)) {
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

  if (stateChanged) BroadcastState();
}

// ─── AIS helper: deduplicated target update ──────────────────────────────────
// FIX #13: Single function for both Class A and Class B position updates
void updateAisTarget(int idx, uint32_t mmsi, double lat, double lon, double cog, double sog) {
  aisTargets[idx].mmsi     = mmsi;
  aisTargets[idx].lat      = lat;
  aisTargets[idx].lon      = lon;
  aisTargets[idx].cogDeg   = cog;
  aisTargets[idx].sogKn    = sog;
  aisTargets[idx].lastSeen = millis();
  aisTargets[idx].active   = true;
}

// ─── AIS slot management ──────────────────────────────────────────────────────
int findOrAllocAisSlot(uint32_t mmsi) {
  for (int i = 0; i < MAX_AIS_TARGETS; i++) {
    if (aisTargets[i].active && aisTargets[i].mmsi == mmsi) return i;
  }
  for (int i = 0; i < MAX_AIS_TARGETS; i++) {
    if (!aisTargets[i].active) {
      memset(&aisTargets[i], 0, sizeof(AisTarget));
      return i;
    }
  }
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

// ─── JSON escape helper ──────────────────────────────────────────────────────
// FIX #5: Escape AIS names that may contain quotes or backslashes
static void jsonEscapeStr(char* dest, size_t destSize, const char* src) {
  size_t di = 0;
  for (size_t si = 0; src[si] && di < destSize - 1; si++) {
    char c = src[si];
    if (c == '"' || c == '\\') {
      if (di + 2 >= destSize) break;
      dest[di++] = '\\';
      dest[di++] = c;
    } else if (c >= 0x20) {  // skip control characters
      dest[di++] = c;
    }
  }
  dest[di] = 0;
}

// ─── Build state JSON using pre-allocated buffer ─────────────────────────────
// FIX #10: snprintf into static buffer instead of String concatenation
void buildStateJson(char* buf, size_t bufSize) {
  int pos = snprintf(buf, bufSize,
    "{\"mode\":\"%s\",\"heading\":%.1f,\"vessel\":%.1f,"
    "\"depth\":%.1f,\"sog\":%.1f,\"cog\":%.1f,"
    "\"aws\":%.1f,\"awa\":%.1f,"
    "\"ownLat\":%s,\"ownLon\":%s,"
    "\"targets\":[",
    apMode.c_str(), lockedHeading, vesselHeading,
    depthM, sogKn, cogDeg, awsKn, awaDeg,
    isnan(ownLat) ? "null" : String(ownLat, 6).c_str(),
    isnan(ownLon) ? "null" : String(ownLon, 6).c_str()
  );

  bool first = true;
  char escapedName[44]; // 20 chars * 2 (worst case all escaped) + null
  for (int i = 0; i < MAX_AIS_TARGETS && pos < (int)bufSize - 200; i++) {
    if (!aisTargets[i].active) continue;
    jsonEscapeStr(escapedName, sizeof(escapedName), aisTargets[i].name);
    pos += snprintf(buf + pos, bufSize - pos,
      "%s{\"mmsi\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"cog\":%.1f,\"sog\":%.1f,\"name\":\"%s\"}",
      first ? "" : ",",
      (unsigned long)aisTargets[i].mmsi,
      aisTargets[i].lat, aisTargets[i].lon,
      aisTargets[i].cogDeg, aisTargets[i].sogKn,
      escapedName
    );
    first = false;
  }
  snprintf(buf + pos, bufSize - pos, "]}");
}

// ─── Broadcast state to all WebSocket clients ────────────────────────────────
void BroadcastState() {
  unsigned long now = millis();
  if (now - lastBroadcast < BROADCAST_INTERVAL_MS) return;
  lastBroadcast = now;
  buildStateJson(jsonBuf, sizeof(jsonBuf));
  ws.textAll(jsonBuf);
}

// ─── Mode command ────────────────────────────────────────────────────────────
void SendModeCommand(const char* mode) {
  uint8_t modeByte = 0xFF;
  if      (strcmp(mode,"STBY")    == 0) modeByte = apCfg.modeStby;
  else if (strcmp(mode,"AUTO")    == 0) modeByte = apCfg.modeAuto;
  else if (strcmp(mode,"NFU")     == 0) modeByte = apCfg.modeNfu;
  else if (strcmp(mode,"NODRIFT") == 0) modeByte = apCfg.modeNoDrift;
  else if (strcmp(mode,"WIND")    == 0) modeByte = apCfg.modeWind;
  else if (strcmp(mode,"NAV")     == 0) modeByte = apCfg.modeNav;
  if (modeByte == 0xFF) return;

  tN2kMsg N2kMsg;
  N2kMsg.Init(6, apCfg.modePgn, apCfg.srcAddr, 255);
  N2kMsg.AddByte(0x01); N2kMsg.AddByte(modeByte); N2kMsg.AddByte(0x00);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  NMEA2000.SendMsg(N2kMsg);
}

// ─── NFU / Direct steering ───────────────────────────────────────────────────
void SendNfuCommand(int rate) {
  if (rate == 0) return;
  uint8_t  dir    = (rate > 0) ? 1 : 0;
  uint16_t amount = (uint16_t)(abs(rate) * 0.1745);
  tN2kMsg N2kMsg;
  N2kMsg.Init(6, apCfg.steerPgn, apCfg.srcAddr, 255);
  N2kMsg.AddByte(0x01); N2kMsg.AddByte(dir);
  N2kMsg.AddByte(amount & 0xFF); N2kMsg.AddByte((amount >> 8) & 0xFF);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  NMEA2000.SendMsg(N2kMsg);
}

// ─── Heading adjust ──────────────────────────────────────────────────────────
// FIX #2: Guard against uint16_t overflow for large angles
void SendHeadingAdjust(int degrees) {
  int clamped = constrain(abs(degrees), 0, 30); // safe range for uint16_t
  uint16_t amount = (uint16_t)(clamped * 10000.0 * M_PI / 180.0);
  uint8_t  dir    = (degrees > 0) ? 1 : 0;
  tN2kMsg N2kMsg;
  N2kMsg.Init(6, apCfg.steerPgn, apCfg.srcAddr, 255);
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
  else if   (msg.startsWith("SETTINGS:")) {
    handleSettingsMessage(msg.substring(9));
  }
}

// ─── Settings message handler ─────────────────────────────────────────────────
void handleSettingsMessage(const String& msg) {
  if (msg == "GET") {
    // Send current settings to client
    char settingsBuf[512];
    snprintf(settingsBuf, sizeof(settingsBuf),
      "{\"settings\":{"
      "\"wifiMode\":\"%s\","
      "\"apSsid\":\"%s\","
      "\"apPassword\":\"%s\","
      "\"staSsid\":\"%s\","
      "\"staPassword\":\"%s\","
      "\"apType\":\"%s\","
      "\"wifiConnected\":true,"
      "\"ip\":\"%s\""
      "}}",
      wifiMode, apSsid, apPassword, staSsid, staPassword, apType,
      (strcmp(wifiMode, "AP") == 0) ? WiFi.softAPIP().toString().c_str() : WiFi.localIP().toString().c_str()
    );
    ws.textAll(settingsBuf);

  } else if (msg == "RESET") {
    // Factory reset
    prefs.begin("sensor", false);
    prefs.clear();
    prefs.end();
    Serial.println("Factory reset — rebooting");
    delay(500);
    ESP.restart();

  } else {
    // Parse JSON settings payload
    // Simple manual JSON parsing (no ArduinoJson dependency)
    auto extractField = [](const String& json, const char* key, char* dest, size_t maxLen) {
      String search = String("\"") + key + "\":\"";
      int start = json.indexOf(search);
      if (start < 0) return;
      start += search.length();
      int end = json.indexOf("\"", start);
      if (end < 0 || end == start) return;
      String val = json.substring(start, end);
      strncpy(dest, val.c_str(), maxLen - 1);
      dest[maxLen - 1] = 0;
    };

    char newWifiMode[4] = "";
    char newApSsid[33] = "";
    char newApPass[65] = "";
    char newStaSsid[33] = "";
    char newStaPass[65] = "";
    char newApType[12] = "";

    extractField(msg, "wifiMode",    newWifiMode, sizeof(newWifiMode));
    extractField(msg, "apSsid",      newApSsid,   sizeof(newApSsid));
    extractField(msg, "apPassword",  newApPass,    sizeof(newApPass));
    extractField(msg, "staSsid",     newStaSsid,   sizeof(newStaSsid));
    extractField(msg, "staPassword", newStaPass,    sizeof(newStaPass));
    extractField(msg, "apType",      newApType,    sizeof(newApType));

    // Validate
    if (strlen(newWifiMode) == 0) return;

    // Save to Preferences
    prefs.begin("sensor", false);
    prefs.putString("wifiMode",   newWifiMode);
    prefs.putString("apSsid",     newApSsid);
    prefs.putString("apPassword", newApPass);
    prefs.putString("staSsid",    newStaSsid);
    prefs.putString("staPassword",newStaPass);
    prefs.putString("apType",     newApType);
    prefs.end();

    Serial.println("Settings saved — rebooting");
    delay(500);
    ESP.restart();
  }
}

// ─── Load settings from NVS ──────────────────────────────────────────────────
void loadSettings() {
  prefs.begin("sensor", true); // read-only

  String wm = prefs.getString("wifiMode", "AP");
  strncpy(wifiMode, wm.c_str(), sizeof(wifiMode) - 1);

  String as = prefs.getString("apSsid", "sensor");
  strncpy(apSsid, as.c_str(), sizeof(apSsid) - 1);

  String ap = prefs.getString("apPassword", "12345678");
  strncpy(apPassword, ap.c_str(), sizeof(apPassword) - 1);

  String ss = prefs.getString("staSsid", "");
  strncpy(staSsid, ss.c_str(), sizeof(staSsid) - 1);

  String sp = prefs.getString("staPassword", "");
  strncpy(staPassword, sp.c_str(), sizeof(staPassword) - 1);

  String at = prefs.getString("apType", "BG");
  strncpy(apType, at.c_str(), sizeof(apType) - 1);

  prefs.end();

  applyApConfig();
}

// ─── WebSocket event handler ──────────────────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      // Use larger buffer for settings JSON payloads
      char buf[384];
      size_t copyLen = min(len, sizeof(buf) - 1);
      memcpy(buf, data, copyLen);
      buf[copyLen] = 0;
      String msg = String(buf);
      msg.trim();
      handleWsMessage(msg);
    }
  }
  if (type == WS_EVT_DISCONNECT) {
    nfuRate = 0;
  }
}

// ─── WiFi setup ──────────────────────────────────────────────────────────────
void setupWifi() {
  if (strcmp(wifiMode, "STA") == 0 && strlen(staSsid) > 0) {
    // Try to join existing network
    Serial.printf("Connecting to WiFi: %s\n", staSsid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSsid, staPassword);

    // Wait up to 10 seconds
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
      return;
    }

    // Fallback to AP mode
    Serial.println("\nSTA connection failed — falling back to AP mode");
    WiFi.disconnect();
  }

  // AP mode (default or fallback)
  WiFi.mode(WIFI_AP);
  if (strlen(apPassword) >= 8) {
    WiFi.softAP(apSsid, apPassword);
  } else {
    WiFi.softAP(apSsid); // open network if password too short
  }
  Serial.printf("AP started: %s  IP: %s\n", apSsid, WiFi.softAPIP().toString().c_str());
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nSensor hub starting...");
  M5.dis.drawpix(0, 0xff0000);

  // Load persistent settings
  loadSettings();

  // Initialise AIS target array
  memset(aisTargets, 0, sizeof(aisTargets));

  // WiFi (AP or STA based on settings)
  setupWifi();

  udp.begin(udpPort);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", FPSTR(INDEX_HTML));
  });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server up");

  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250); // FIX #16: Set receive buffer too
  NMEA2000.SetProductInformation("00000001", 100, "Sensor Network",
                                 "1.0008 Apr 2026", "1.0008 Apr 2026");
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
void loop() {
  NMEA2000.ParseMessages();

  unsigned long now = millis();

  // FIX #11: Only clean up WebSocket clients once per second
  if (now - lastCleanup >= CLEANUP_INTERVAL_MS) {
    ws.cleanupClients();
    lastCleanup = now;
  }

  // NFU steering with timeout and rate-limited sends
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

  // UDP sensor handling
  // FIX #15: Use C string functions instead of Arduino String where possible
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      M5.dis.drawpix(0, 0x0000ff);
      incomingPacket[len] = 0;

      Serial.print("UDP: ");
      Serial.print(incomingPacket);

      if (strncmp(incomingPacket, "$XDR", 4) == 0) {
        // Find ",F," marker
        const char* fMarker = strstr(incomingPacket, ",F,");
        const char* lMarker = fMarker ? strstr(fMarker, ",L,") : NULL;
        if (fMarker && lMarker && lMarker > fMarker + 3) {
          // Extract value between ",F," and ",L,"
          char valBuf[16];
          int valLen = min((int)(lMarker - fMarker - 3), (int)sizeof(valBuf) - 1);
          memcpy(valBuf, fMarker + 3, valLen);
          valBuf[valLen] = 0;
          float value = atof(valBuf);

          if (strstr(incomingPacket, "FUEL")) {
            fuelLevel = bounds((int)Interpolation::Linear(fuelInput, fuelOutput, fuelValues, value, true));
            SendN2kTankLevel(fuelLevel, fuelCapacity, N2kft_Fuel);
            Serial.printf(" / fuel: %d\n", fuelLevel);
          } else if (strstr(incomingPacket, "WATER")) {
            waterLevel = bounds((int)Interpolation::Linear(waterInput, waterOutput, waterValues, value, true));
            SendN2kTankLevel(waterLevel, waterCapacity, N2kft_Water);
            Serial.printf(" / water: %d\n", waterLevel);
          } else if (strstr(incomingPacket, "temp0")) {
            SendN2kTemperature(1, N2kts_ExhaustGasTemperature, CToKelvin((double)value));
            Serial.printf(" / exhaust: %.1f\n", value);
          } else if (strstr(incomingPacket, "temp1")) {
            SendN2kTemperature(2, N2kts_EngineRoomTemperature, CToKelvin((double)value));
            Serial.printf(" / engine room: %.1f\n", value);
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

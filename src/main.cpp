
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
uint8_t computeDataAvail();
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
    // B&G / Simrad / Navico — confirmed from bus capture Apr 2026
    // 65341 = command PGN (send mode commands here)
    // 65305 = status PGN (read engagement state from here)
    // modeStby=0x02 used in 65341 command. Engaged status read from 65305 byte 4.
    apCfg = {65341, 65345, 65305, 204, 0x02, 0x00, 0x03, 0x04, 0x05, 0x06};
  } else if (strcmp(apType, "GARMIN") == 0) {
    apCfg = {65341, 65345, 65305, 204, 0x02, 0x00, 0x03, 0x04, 0x05, 0x06};
  } else if (strcmp(apType, "RAYMARINE") == 0) {
    apCfg = {65341, 65345, 65305, 204, 0x02, 0x00, 0x03, 0x04, 0x05, 0x06};
  } else {
    apCfg = {65341, 65345, 65305, 204, 0x02, 0x00, 0x03, 0x04, 0x05, 0x06};
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
double rudderDeg     = -999;  // degrees, -999 = no data (0 is valid)
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

// ─── Data availability tracking ──────────────────────────────────────────────
// Each flag records the last millis() a valid value was received from that source.
// 0 = never received. Cleared after DATA_STALE_MS of silence.
const unsigned long DATA_STALE_MS = 10000; // 10 seconds
unsigned long lastSeenHeading  = 0; // PGN 127250
unsigned long lastSeenAP       = 0; // PGN 65288
unsigned long lastSeenWind     = 0; // PGN 130306
unsigned long lastSeenPosition = 0; // PGN 129025
unsigned long lastSeenRudder   = 0; // PGN 127245
unsigned long lastSeenXTE      = 0; // PGN 129283 (active route / XTE)
unsigned long lastSeenCOGSOG   = 0; // PGN 129026

// Bitmask broadcast to UI — bits defined here, checked in JS
// bit 0 = heading, 1 = AP, 2 = wind, 3 = position, 4 = rudder, 5 = XTE/route, 6 = COG/SOG
#define AVAIL_HEADING  (1<<0)
#define AVAIL_AP       (1<<1)
#define AVAIL_WIND     (1<<2)
#define AVAIL_POSITION (1<<3)
#define AVAIL_RUDDER   (1<<4)
#define AVAIL_XTE      (1<<5)
#define AVAIL_COGSOG   (1<<6)

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
static char jsonBuf[3200]; // expanded for avail/rudder fields

// ─── HTML UI (PROGMEM, gzip-compressed) ──────────────────────────────────────
// Generated at build time by generate_ui.py from src/index.html.
// Provides: INDEX_HTML (uint8_t[]) and INDEX_HTML_LEN (uint32_t).
// Edit src/index.html — never edit src/ui.h directly.
#include "ui.h"

// ─── NMEA2000 message handler ─────────────────────────────────────────────────
void HandleN2kMessage(const tN2kMsg &N2kMsg) {
  bool stateChanged = false;

  switch (N2kMsg.PGN) {

    case 65305: {
      // AP48 actual status PGN — confirmed from bus capture comparison
      // Byte 0-1: 0x41,0x9F = Navico manufacturer code
      // Byte 2:   0x64 (always)
      // Byte 3:   sub-command / instance (varies: 0x02 or 0x0A)
      // Byte 4:   STATUS BYTE — 0x02/0x08 = STANDBY, 0x10 = ENGAGED
      // Bytes 5-7: 0x00,0x00,0x00
      if (N2kMsg.DataLen < 5) break;
      Serial.printf("[AP STATUS 65305] src=%d bytes:", N2kMsg.Source);
      for (int i = 0; i < N2kMsg.DataLen && i < 8; i++) Serial.printf(" %02X", N2kMsg.Data[i]);
      uint8_t statusByte = N2kMsg.Data[4];
      Serial.printf(" → status=0x%02X\n", statusByte);
      // 0x10 = engaged (AUTO heading hold confirmed from capture)
      // 0x02/0x08 = standby — other engaged modes TBC
      if (statusByte == 0x10) {
        apMode = "AUTO";
      } else if (statusByte == 0x02 || statusByte == 0x08) {
        apMode = "STBY";
        lockedHeading = -1; // clear locked heading so UI reverts to vessel heading
      }
      lastSeenAP = millis();
      stateChanged = true;
      break;
    }

    case 65341: {
      // AP48 keepalive/command PGN — used to send mode commands TO the AP
      // Also broadcast by AP48 as heartbeat (byte 4 = 0x02 always in our captures)
      // Log if status byte changes so we can learn the command encoding
      if (N2kMsg.DataLen < 5) break;
      uint8_t b4 = N2kMsg.Data[4];
      if (b4 != 0x02) { // only log if unexpected — 0x02 is normal heartbeat
        Serial.printf("[65341 UNEXPECTED] src=%d byte4=0x%02X bytes:", N2kMsg.Source, b4);
        for (int i = 0; i < N2kMsg.DataLen && i < 8; i++) Serial.printf(" %02X", N2kMsg.Data[i]);
        Serial.println();
      }
      break;
    }

    case 65359: {
      // Navico locked/commanded heading — only trust the NAC-3 (src=4)
      if (N2kMsg.Source != 4) break;
      if (N2kMsg.DataLen < 7) break;
      uint16_t raw = (uint16_t)N2kMsg.Data[5] | ((uint16_t)N2kMsg.Data[6] << 8);
      if (raw == 0xFFFF) break;
      double hdg = RadToDeg(raw * 0.0001);
      Serial.printf("[65359]  raw=%u bytes[5]=%02X bytes[6]=%02X decoded=%.4f° floor=%d\n",
                    raw, N2kMsg.Data[5], N2kMsg.Data[6], hdg, (int)hdg);
      break;
    }

    case 127250: {
      // Vessel heading
      uint8_t sid;
      double heading, deviation, variation;
      tN2kHeadingReference ref;
      if (ParseN2kHeading(N2kMsg, sid, heading, deviation, variation, ref)) {
        if (!N2kIsNA(heading)) { vesselHeading = RadToDeg(heading); lastSeenHeading = millis(); stateChanged = true; }
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
        if (!N2kIsNA(sog)) { sogKn  = msToKnots(sog); lastSeenCOGSOG = millis(); stateChanged = true; }
        if (!N2kIsNA(cog)) { cogDeg = RadToDeg(cog); stateChanged = true; }
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
          if (!N2kIsNA(aws)) { awsKn  = msToKnots(aws); lastSeenWind = millis(); stateChanged = true; }
          if (!N2kIsNA(awa)) { awaDeg = RadToDeg(awa); stateChanged = true; }
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
          lastSeenPosition = millis(); stateChanged = true;
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

    case 127245: {
      // Rudder angle — required for DIRECT/NFU mode
      uint8_t sid;
      double rudder, angleOrder;
      tN2kRudderDirectionOrder dir;
      if (ParseN2kRudder(N2kMsg, rudder, sid, dir, angleOrder)) {
        if (!N2kIsNA(rudder)) {
          rudderDeg = RadToDeg(rudder);
          lastSeenRudder = millis();
          stateChanged = true;
        }
      }
      break;
    }

    case 129283: {
      // Cross Track Error — only broadcast when a route is active
      uint8_t     sid;
      tN2kXTEMode xteMode;
      bool        navTerminated;
      double      xte;
      if (ParseN2kXTE(N2kMsg, sid, xteMode, navTerminated, xte)) {
        if (!navTerminated && !N2kIsNA(xte)) {
          lastSeenXTE = millis();
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

    case 130850: {
      // Navico proprietary command/status PGN — log ALL occurrences for analysis
      Serial.printf("[130850] src=%d len=%d:", N2kMsg.Source, N2kMsg.DataLen);
      for (int i = 0; i < N2kMsg.DataLen && i < 12; i++) Serial.printf(" %02X", N2kMsg.Data[i]);
      Serial.println();
      break;
    }

    case 130851: {
      // NAC-3 command echo PGN — log ALL occurrences
      Serial.printf("[130851] src=%d len=%d:", N2kMsg.Source, N2kMsg.DataLen);
      for (int i = 0; i < N2kMsg.DataLen && i < 12; i++) Serial.printf(" %02X", N2kMsg.Data[i]);
      Serial.println();
      break;
    }

    case 127237: {
      // Heading/Track Control (NAC-3, src=4)
      // Bytes [5-6]: Heading To Steer (0.0001 rad LE)
      // NAC-3 rounds up to next degree internally — 127237 is consistently 1°
      // higher than what chartplotters display. Subtract 1 to match.
      if (N2kMsg.Source == 4 && N2kMsg.DataLen >= 7) {
        uint16_t raw = (uint16_t)N2kMsg.Data[5] | ((uint16_t)N2kMsg.Data[6] << 8);
        if (raw != 0xFFFF) {
          double decoded = RadToDeg(raw * 0.0001) - 1.0;
          lockedHeading = fmod(decoded + 360.0, 360.0);
          stateChanged = true;
        }
      }
      break;
    }

    default: {
      // Raw logging disabled — was used to find heading adjust PGN (now found)
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

// ─── Data availability bitmask ───────────────────────────────────────────────
uint8_t computeDataAvail() {
  unsigned long now = millis();
  uint8_t avail = 0;
  if (now - lastSeenHeading  < DATA_STALE_MS) avail |= AVAIL_HEADING;
  if (now - lastSeenAP       < DATA_STALE_MS) avail |= AVAIL_AP;
  if (now - lastSeenWind     < DATA_STALE_MS) avail |= AVAIL_WIND;
  if (now - lastSeenPosition < DATA_STALE_MS) avail |= AVAIL_POSITION;
  if (now - lastSeenRudder   < DATA_STALE_MS) avail |= AVAIL_RUDDER;
  if (now - lastSeenXTE      < DATA_STALE_MS) avail |= AVAIL_XTE;
  if (now - lastSeenCOGSOG   < DATA_STALE_MS) avail |= AVAIL_COGSOG;
  return avail;
}

// ─── Build state JSON using pre-allocated buffer ─────────────────────────────
void buildStateJson(char* buf, size_t bufSize) {
  char latBuf[16], lonBuf[16];
  uint8_t avail = computeDataAvail();
  int pos = snprintf(buf, bufSize,
    "{\"mode\":\"%s\",\"heading\":%.1f,\"vessel\":%.1f,"
    "\"depth\":%.1f,\"sog\":%.1f,\"cog\":%.1f,"
    "\"aws\":%.1f,\"awa\":%.1f,"
    "\"rudder\":%.1f,\"avail\":%u,"
    "\"ownLat\":%s,\"ownLon\":%s,"
    "\"targets\":[",
    apMode.c_str(), lockedHeading, vesselHeading,
    depthM, sogKn, cogDeg, awsKn, awaDeg,
    rudderDeg, (unsigned)avail,
    isnan(ownLat) ? "null" : (snprintf(latBuf, sizeof(latBuf), "%.6f", ownLat), latBuf),
    isnan(ownLon) ? "null" : (snprintf(lonBuf, sizeof(lonBuf), "%.6f", ownLon), lonBuf)
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
  // Confirmed from bus captures Apr 2026:
  // NAC-3 commanded via PGN 130850 (Navico proprietary)
  // Byte 6: 0x09=engage heading hold, 0x06=disengage (standby)
  // The NAC-3 locks onto current vessel heading at moment of engage.
  bool engage = (strcmp(mode,"STBY") != 0);
  uint8_t subB = engage ? 0x09 : 0x06;

  tN2kMsg N2kMsg;
  N2kMsg.Init(6, 130850, apCfg.srcAddr, 255);
  N2kMsg.AddByte(0x41); N2kMsg.AddByte(0x9F);
  N2kMsg.AddByte(0x04);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0x0A);
  N2kMsg.AddByte(subB);
  N2kMsg.AddByte(0x00);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF);

  Serial.printf("[SEND MODE] mode=%s → PGN 130850 byte6=0x%02X\n", mode, subB);
  NMEA2000.SendMsg(N2kMsg);
}

// ─── NFU / Direct steering ───────────────────────────────────────────────────
void SendNfuCommand(int rate) {
  // NFU/DIRECT steering — format unverified for NAC-3, needs bus capture
  // Using 130850 with sub-byte approach as placeholder
  if (rate == 0) return;
  // TODO: capture Zeus sending heading adjust to determine correct format
  Serial.printf("[NFU] rate=%d — command format unverified for NAC-3\n", rate);
}

// ─── Heading adjust ──────────────────────────────────────────────────────────
// Confirmed from 130851 NAC-3 echo and on-water testing Apr 2026:
// PGN 130850, byte6=0x1A
// Byte 8:    0x01 = ADD to heading (STBD), 0x02 = SUBTRACT from heading (PORT)
// Bytes 9-10: adjustment amount in 0.0001 rad units (uint16 LE)
//             1° = 175 units, 10° = 1745 units
// NOTE: DO NOT use 0x00 or other values for byte 8 — causes NAC-3 to set
// absolute heading rather than relative adjust.
void SendHeadingAdjust(int degrees) {
  if (degrees == 0) return;

  bool add = (degrees > 0); // positive = STBD = add to heading
  int  deg = abs(degrees);

  // Convert degrees to 0.0001 rad units
  uint16_t amt = (uint16_t)(deg * (M_PI / 180.0) / 0.0001);

  tN2kMsg N2kMsg;
  N2kMsg.Init(6, 130850, apCfg.srcAddr, 255);
  N2kMsg.AddByte(0x41); N2kMsg.AddByte(0x9F); // Navico manufacturer code
  N2kMsg.AddByte(0x04);                        // command type
  N2kMsg.AddByte(0xFF); N2kMsg.AddByte(0xFF); // N/A
  N2kMsg.AddByte(0x0A);                        // sub-command A (constant)
  N2kMsg.AddByte(0x1A);                        // heading adjust sub-command
  N2kMsg.AddByte(0x00);                        // constant
  N2kMsg.AddByte(add ? 0x03 : 0x02);          // direction: TBC — 0x02=PORT(subtract), 0x03=STBD(add)?
  N2kMsg.AddByte(amt & 0xFF);                  // amount low byte
  N2kMsg.AddByte((amt >> 8) & 0xFF);           // amount high byte
  N2kMsg.AddByte(0xFF);                        // constant

  Serial.printf("[HDG ADJUST] %+d° → dir=0x%02X amt=%d\n",
                degrees, add ? 0x01 : 0x02, amt);
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
  if (type == WS_EVT_CONNECT) {
    // Send current state immediately to newly connected client
    buildStateJson(jsonBuf, sizeof(jsonBuf));
    client->text(jsonBuf);
  }
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
    // Serve gzip-compressed HTML — browser decompresses transparently
    AsyncWebServerResponse *response = request->beginResponse(
      200, "text/html", INDEX_HTML, INDEX_HTML_LEN);
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-cache");
    request->send(response);
  });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server up");

  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.SetProductInformation("00000001", 100, "Sensor Network",
                                 "1.0008 Apr 2026", "1.0008 Apr 2026");
  NMEA2000.SetDeviceInformation(1, 150, 75, 2046);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  NMEA2000.SetMsgHandler(HandleN2kMessage);

  // ── DEBUG: uncomment to log all bus traffic to Serial ───────────────────
  // NMEA2000.SetForwardStream(&Serial);
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  // NMEA2000.SetForwardOwnMessages(true);
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

  // Periodic state broadcast — ensures UI stays updated even if no NMEA events fire
  static unsigned long lastPeriodicBroadcast = 0;
  if (now - lastPeriodicBroadcast >= 500) {
    lastPeriodicBroadcast = now;
    buildStateJson(jsonBuf, sizeof(jsonBuf));
    ws.textAll(jsonBuf);
    Serial.printf("[STATE] vessel=%.1f locked=%.1f mode=%s\n",
                  vesselHeading, lockedHeading, apMode.c_str());
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

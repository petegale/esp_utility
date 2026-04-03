
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

// ─── Tank calibration (unchanged) ────────────────────────────────────────────
const int fuelValues = 2;
double fuelInput[3]   = {0.0, 41.0, 78.0};
double fuelOutput[3]  = {0.0, 52.0, 100.0};
int    fuelCapacity   = 200;
int    fuelLevel      = 0;

const int waterValues = 3;
double waterInput[3]  = {10.0, 50.0, 75.0};
double waterOutput[3] = {25.0, 60.0, 80.0};
int    waterCapacity  = 200;
int    waterLevel     = 0;

// ─── Autopilot State ──────────────────────────────────────────────────────────
String apMode        = "STBY";
double lockedHeading = 0;
double vesselHeading = 0;

// ─── NMEA2000 ─────────────────────────────────────────────────────────────────
int NodeAddress;
Preferences preferences;

const unsigned long TransmitMessages[] PROGMEM = {
  127505L,  // Fluid Level
  130311L,  // Temperature
  127488L,  // Engine Rapid / RPM
  127508L,  // Battery Status
  0
};

const unsigned long ReceiveMessages[] PROGMEM = {
  65288L,   // Autopilot mode
  65359L,   // Autopilot locked heading
  127250L,  // Vessel heading
  0
};

// ─── Debug mode ───────────────────────────────────────────────────────────────
// To enable bus sniffing, uncomment the three lines in setup() marked DEBUG
// This will print all NMEA2000 traffic to Serial in human-readable text,
// including raw PGN bytes from your B&G keypad — useful for verifying the
// autopilot command byte sequences before going live.
// Leave disabled in normal operation to reduce Serial noise.

// ─── HTML for phone UI ────────────────────────────────────────────────────────
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Autopilot</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, sans-serif;
      background: #0a1628;
      color: #fff;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 20px;
    }
    h1 { font-size: 1.1em; color: #8899aa; margin-bottom: 16px; letter-spacing: 2px; }
    #status {
      background: #1a2a3a;
      border-radius: 12px;
      padding: 16px 32px;
      text-align: center;
      margin-bottom: 24px;
      width: 100%;
      max-width: 320px;
    }
    #mode {
      font-size: 2em;
      font-weight: bold;
      color: #00ff88;
      letter-spacing: 4px;
    }
    #mode.standby { color: #ff6644; }
    #heading-display { font-size: 1.1em; color: #aabbcc; margin-top: 6px; }
    .btn-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      width: 100%;
      max-width: 320px;
      margin-bottom: 12px;
    }
    button {
      background: #1a3050;
      color: #fff;
      border: 2px solid #2a4060;
      border-radius: 12px;
      padding: 22px 10px;
      font-size: 1.3em;
      font-weight: bold;
      cursor: pointer;
      transition: background 0.1s;
      -webkit-tap-highlight-color: transparent;
    }
    button:active { background: #2a5080; }
    .btn-stby {
      background: #3a1a1a;
      border-color: #ff6644;
      color: #ff6644;
    }
    .btn-auto {
      background: #1a3a2a;
      border-color: #00ff88;
      color: #00ff88;
    }
    #conn {
      margin-top: 16px;
      font-size: 0.8em;
      color: #556677;
    }
  </style>
</head>
<body>
  <h1>AUTOPILOT</h1>
  <div id="status">
    <div id="mode" class="standby">STBY</div>
    <div id="heading-display">HDG: ---°</div>
  </div>
  <div class="btn-grid">
    <button ontouchstart="send('PORT10')"  onclick="send('PORT10')">◄◄ 10°</button>
    <button ontouchstart="send('STBD10')"  onclick="send('STBD10')">10° ►►</button>
    <button ontouchstart="send('PORT1')"   onclick="send('PORT1')"> ◄  1°</button>
    <button ontouchstart="send('STBD1')"   onclick="send('STBD1')">  1° ►</button>
  </div>
  <div class="btn-grid">
    <button class="btn-stby" ontouchstart="send('STBY')" onclick="send('STBY')">STANDBY</button>
    <button class="btn-auto" ontouchstart="send('AUTO')" onclick="send('AUTO')">AUTO</button>
  </div>
  <div id="conn">Connecting...</div>

  <script>
    let ws;
    function connect() {
      ws = new WebSocket('ws://' + location.hostname + '/ws');
      ws.onopen    = () => { document.getElementById('conn').textContent = 'Connected'; };
      ws.onclose   = () => { document.getElementById('conn').textContent = 'Disconnected — retrying...'; setTimeout(connect, 2000); };
      ws.onmessage = (e) => {
        const d = JSON.parse(e.data);
        const modeEl = document.getElementById('mode');
        modeEl.textContent = d.mode;
        modeEl.className   = d.mode === 'STBY' ? 'standby' : '';
        document.getElementById('heading-display').textContent =
          'HDG: ' + (d.heading > 0 ? d.heading.toFixed(1) : '---') + '°';
      };
    }
    function send(cmd) { if (ws && ws.readyState === 1) ws.send(cmd); }
    connect();
  </script>
</body>
</html>
)rawliteral";

// ─── Forward declarations ──────────────────────────────────────────────────────
void SendN2kTankLevel(double, double, tN2kFluidType);
void SendN2kTemperature(int, tN2kTempSource, double);
void HandleN2kMessage(const tN2kMsg &);
void SendAutopilotCommand(const char*);
void BroadcastAutopilotState();
int  bounds(int);

// ─── NMEA2000 message handler ─────────────────────────────────────────────────
void HandleN2kMessage(const tN2kMsg &N2kMsg) {
  switch (N2kMsg.PGN) {

    case 65288: { // Autopilot mode — Navico/B&G specific
      uint8_t mode = N2kMsg.Data[1];
      switch (mode) {
        case 0:  apMode = "STBY"; break;
        case 1:  apMode = "AUTO"; break;
        case 2:  apMode = "WIND"; break;
        case 3:  apMode = "NAV";  break;
        default: apMode = "----"; break;
      }
      BroadcastAutopilotState();
      break;
    }

    case 65359: { // Autopilot locked heading — Navico/B&G specific
      uint16_t raw = (uint16_t)N2kMsg.Data[2] | ((uint16_t)N2kMsg.Data[3] << 8);
      lockedHeading = RadToDeg(raw * 0.0001);
      BroadcastAutopilotState();
      break;
    }

    case 127250: { // Vessel heading (standard NMEA2000)
      double heading;
      uint8_t ref;
      double deviation, variation;
      if (ParseN2kHeading(N2kMsg, ref, heading, deviation, variation)) {
        if (!N2kIsNA(heading)) vesselHeading = RadToDeg(heading);
      }
      BroadcastAutopilotState();
      break;
    }
  }
}

// ─── Push state to all connected WebSocket clients ────────────────────────────
void BroadcastAutopilotState() {
  String json = "{\"mode\":\"" + apMode + "\","
                + "\"heading\":" + String(lockedHeading, 1) + ","
                + "\"vessel\":"  + String(vesselHeading, 1) + "}";
  ws.textAll(json);
}

// ─── Send B&G autopilot commands via NMEA2000 ─────────────────────────────────
void SendAutopilotCommand(const char* cmd) {
  tN2kMsg N2kMsg;

  if (strcmp(cmd, "STBY") == 0) {
    N2kMsg.Init(6, 65341, 204, 255);
    N2kMsg.AddByte(0x01);
    N2kMsg.AddByte(0x00); // Standby
    N2kMsg.AddByte(0x00);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    NMEA2000.SendMsg(N2kMsg);

  } else if (strcmp(cmd, "AUTO") == 0) {
    N2kMsg.Init(6, 65341, 204, 255);
    N2kMsg.AddByte(0x01);
    N2kMsg.AddByte(0x01); // Auto
    N2kMsg.AddByte(0x00);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    N2kMsg.AddByte(0xFF);
    NMEA2000.SendMsg(N2kMsg);

  } else {
    int8_t degrees = 0;
    if      (strcmp(cmd, "PORT1")  == 0) degrees = -1;
    else if (strcmp(cmd, "STBD1")  == 0) degrees =  1;
    else if (strcmp(cmd, "PORT10") == 0) degrees = -10;
    else if (strcmp(cmd, "STBD10") == 0) degrees =  10;

    if (degrees != 0) {
      uint16_t amount = (uint16_t)(abs(degrees) * 10000 * M_PI / 180.0);
      uint8_t  dir    = (degrees > 0) ? 1 : 0;

      N2kMsg.Init(6, 65345, 204, 255);
      N2kMsg.AddByte(0x01);
      N2kMsg.AddByte(dir);
      N2kMsg.AddByte(amount & 0xFF);
      N2kMsg.AddByte((amount >> 8) & 0xFF);
      N2kMsg.AddByte(0xFF);
      N2kMsg.AddByte(0xFF);
      N2kMsg.AddByte(0xFF);
      N2kMsg.AddByte(0xFF);
      NMEA2000.SendMsg(N2kMsg);
    }
  }
}

// ─── WebSocket event handler ──────────────────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      data[len] = 0;
      String cmd = String((char*)data);
      cmd.trim();
      SendAutopilotCommand(cmd.c_str());
      M5.dis.drawpix(0, 0x0000ff); // blue flash on command
    }
  }
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nSensor hub starting...");
  M5.dis.drawpix(0, 0xff0000); // Red

  // WiFi AP
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // UDP (existing sensors)
  udp.begin(udpPort);

  // Web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Web server up at http://192.168.4.1");

  // NMEA2000
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetProductInformation("00000001", 100, "Sensor Network",
                                 "1.0004 Apr 2026", "1.0004 Apr 2026");
  NMEA2000.SetDeviceInformation(1, 150, 75, 2046);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  NMEA2000.SetMsgHandler(HandleN2kMessage);

  // ── DEBUG: uncomment these three lines to sniff all NMEA2000 bus traffic ──
  // Prints every PGN in human-readable text to Serial. Use this to capture
  // what your B&G keypad actually sends when you press autopilot buttons,
  // then compare against the byte sequences in SendAutopilotCommand() above.
  // Remember to re-comment before normal deployment.
  //
  // NMEA2000.SetForwardStream(&Serial);
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  // NMEA2000.SetForwardOwnMessages(false);  // exclude our own outgoing msgs
  // ── END DEBUG ─────────────────────────────────────────────────────────────

  NMEA2000.Open();
  Serial.println("NMEA2000 up");

  M5.dis.drawpix(0, 0x00ff00); // Green
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
  NMEA2000.ParseMessages();
  ws.cleanupClients();

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      M5.dis.drawpix(0, 0x0000ff); // blue
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
    }
  }

  delay(50);
  M5.dis.drawpix(0, 0x00ff00); // green
}

// ─── N2K send functions (unchanged) ──────────────────────────────────────────
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

# ESP Utility: Sensor Hub & Autopilot Controller

> **Note:** This project is also an experiment in AI-assisted development.

**[Live UI demo](https://petegale.github.io/esp_utility/)** — interactive prototype with simulated data, works on mobile.

An M5Stack Atom-based ESP32 project that:
- Receives sensor data from wireless sensors via UDP and transmits it onto an **NMEA2000** network
- Acts as a **web-based autopilot controller** for B&G / Navico autopilot systems (tested with AP48 + Zeus)
- Displays **instrument data** (depth, SOG, apparent wind) read from the NMEA2000 bus
- Shows **AIS targets** on a heading-up radar display

Access is via a mobile-optimised web UI served directly from the ESP32 — no app required.

---

## Hardware

| Component | Notes |
|---|---|
| **M5Stack Atom** | ESP32-based, built-in RGB LED, USB-C |
| **SN65HVD230 CAN transceiver** | 3.3V native — works directly with ESP32. Isolated version recommended for marine use |

### Wiring

```
ESP32 GPIO22  →  CAN TX (SN65HVD230)
ESP32 GPIO19  →  CAN RX (SN65HVD230)
ESP32 3.3V    →  VCC
ESP32 GND     →  GND

SN65HVD230 CANH  →  NMEA2000 CAN High
SN65HVD230 CANL  →  NMEA2000 CAN Low
```

---

## Network Architecture

```
Wireless sensors (UDP) ──►┐
                           │
                    ESP32 (M5Stack Atom)
                           │
                    ┌──────┴──────┐
                    │             │
              NMEA2000 bus    WiFi AP (192.168.4.1)
                    │             │
              B&G Autopilot   Phone browser
              Instruments     (web UI)
              AIS receiver
```

The ESP32 creates its own WiFi access point (`sensor` / `12345678`). Connect your phone to this network and open `http://192.168.4.1` in a browser. The UI can be saved as a home screen shortcut for one-tap access.

---

## Web UI Screens

### Autopilot
- Full mode switching: **STBY**, **AUTO**, **WIND**, **NAV**, **NO DRIFT**, **DIRECT**
- ±1° and ±10° heading adjust buttons
- **DIRECT (NFU) mode**: drag wheel to steer, 270° rotation = full rudder rate
- Mode buttons reflect actual autopilot state read from the NMEA2000 bus
- Safety timeout: rudder stops if phone disconnects or WiFi drops

### Instruments
- Depth (metres)
- Speed Over Ground (knots)
- Apparent Wind Speed (knots)
- Apparent Wind Angle with rotating direction arrow

### AIS Radar
- Heading-up radar display centred on own vessel
- Adjustable range: 0.5 / 1 / 2 / 5 / 10 / 20 nm
- Targets shown as triangles pointing in COG direction
- 10-minute COG vector lines
- **Heading-up** orientation — own vessel bow always points to the top of the display; a north tick on the radar edge shows the current bearing to north
- The vessel heading is displayed at the top of the radar plot
- Tap any target for details (name, MMSI, SOG, COG, bearing, distance)
- Targets expire after 5 minutes of no updates
- Scrollable target list below radar

---

## NMEA2000 PGNs

### Transmitted
| PGN | Data |
|---|---|
| 127505 | Fluid Level (fuel, water) |
| 130311 | Temperature (exhaust, engine room) |

### Received
| PGN | Data |
|---|---|
| 65288 | Autopilot mode (Navico/B&G proprietary) |
| 65359 | Autopilot locked heading (Navico/B&G proprietary) |
| 127250 | Vessel heading |
| 128267 | Water depth |
| 129025 | Own vessel position |
| 129026 | COG & SOG |
| 130306 | Apparent wind speed & angle |
| 129038 | AIS Class A position |
| 129041 | AIS Class B position |
| 129794 | AIS Class A static data (vessel name) |
| 129809 | AIS Class B static data (vessel name) |

### Autopilot Command PGNs (Navico/B&G proprietary)
| PGN | Function |
|---|---|
| 65341 | Mode change command |
| 65345 | Heading adjust / NFU rudder command |

> ⚠️ **Important**: The autopilot command byte sequences are based on community reverse-engineering of the Navico/B&G protocol for the AP48 + Zeus combination. **Verify these against your physical hardware using debug logging before use at sea.** See [Debug Mode](#debug-mode) below.

---

## Autopilot Mode Bytes (PGN 65341, Byte[1])

| Mode | Byte | Precondition |
|---|---|---|
| Standby | `0x00` | None |
| Auto (heading hold) | `0x01` | Valid compass heading on bus |
| Direct (NFU) | `0x02` | None |
| No Drift | `0x03` | Valid GPS position & COG on bus |
| Wind | `0x04` | Valid apparent wind on bus |
| Nav | `0x05` | Active route set on Zeus chartplotter |

---

## Sensor Input (UDP)

Wireless sensors send NMEA `$XDR` sentences to UDP port `10110`. The ESP32 parses these and converts to NMEA2000:

| Sensor keyword | Data | NMEA2000 PGN |
|---|---|---|
| `FUEL` | Fuel level (raw sensor value) | 127505 |
| `WATER` | Water level (raw sensor value) | 127505 |
| `temp0` | Exhaust gas temperature (°C) | 130311 |
| `temp1` | Engine room temperature (°C) | 130311 |

Fuel and water readings are passed through a configurable interpolation table to correct for non-linear sender characteristics. Edit the calibration arrays in `main.cpp`:

```cpp
double fuelInput[3]  = {0.0, 41.0, 78.0};   // raw sensor values
double fuelOutput[3] = {0.0, 52.0, 100.0};  // calibrated % full
```

---

## Debug Mode

To capture what your physical AP48 actually sends on the NMEA2000 bus:

1. Uncomment the three lines in `setup()` marked `DEBUG`:
```cpp
NMEA2000.SetForwardStream(&Serial);
NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
NMEA2000.SetForwardOwnMessages(false);
```
2. Flash and open Serial Monitor at 115200 baud
3. Press each mode button on the physical AP48
4. Note the PGN 65341 byte sequences
5. Compare against `SendModeCommand()` in `main.cpp` and adjust if needed
6. Re-comment the debug lines before normal deployment

---

## Dependencies (PlatformIO)

```ini
lib_deps =
    ttlappalainen/NMEA2000-library@^4.22.1
    ttlappalainen/NMEA2000_esp32@^1.0.3
    m5stack/M5Atom@^0.1.3
    fastled/FastLED@^3.10.2
    mathieucarbou/ESPAsyncWebServer@^3.3.12
```

---

## UI Test File

A standalone HTML test file is maintained in `ui_test/autopilot_ui_test.html`. This simulates the full UI including instrument data and AIS targets without requiring the ESP32. Open it in any browser or send to a phone via AirDrop / file share.

---

## Safety Notes

- This system sends commands to a safety-critical device (autopilot). Always verify operation in safe conditions before relying on it at sea.
- The WiFi AP is password protected — keep the password secure to prevent unintended control by other devices.
- The DIRECT (NFU) mode has a 500ms safety timeout — if the phone disconnects, rudder movement stops automatically.
- Switching to STANDBY from the web UI disengages the autopilot. Ensure someone is at the helm before doing so.
- AIS data is for situational awareness only — always maintain a proper watch.

---

## Version History

| Version | Date | Notes |
|---|---|---|
| 1.0001 | Sep 2025 | Initial sensor hub |
| 1.0004 | Apr 2026 | Autopilot control added |
| 1.0005 | Apr 2026 | Full mode switching, NFU wheel |
| 1.0006 | Apr 2026 | Active button states, 270° wheel sensitivity |
| 1.0007 | Apr 2026 | Bug fixes, instruments screen, AIS radar |

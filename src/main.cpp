
#define ESP32_CAN_TX_PIN GPIO_NUM_22 // If you use ESP32 and do not have TX on default IO 16, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_19 // If you use ESP32 and do not have RX on default IO 4, uncomment this and and modify definition to match your CAN TX pin.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "M5Atom.h"
#include "InterpolationLib.h"

//NMEA STUFF
#include <Preferences.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>


//definitions
const char *ssid = "sensor";
const char *password = "12345678";

//calibration pairs
const int fuelValues = 2;
double   fuelInput[3]   = {0.0, 41.0, 78.0};
double fuelOutput[3] = {0.0, 52.0, 100.0}; //these values are % of tank full
int fuelCapacity = 200; //fuel capacity in Litres
int fuelLevel = 0; //current fuel level %

const int waterValues = 3;
double   waterInput[3]   = {10.0, 50.0, 75.0};
double waterOutput[3] = {25.0, 60.0, 80.0}; //these values are % of tank full
int waterCapacity = 200; //water capacity in Litres
int waterLevel = 0; //current water level %


WiFiUDP udp;
const int udpPort = 10110;
char incomingPacket[255];

int NodeAddress;  // To store last Node Address
Preferences preferences;             // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {127505L, // Fluid Level
                                                  130311L, // Temperature  (or alternatively 130312L or 130316L)
                                                  127488L, // Engine Rapid / RPM
                                                  127508L, // Battery Status
                                                  0
                                                 };


// Forward declarations for functions
void SendN2kTankLevel(double, double, tN2kFluidType);
void SendN2kTemperature(int, tN2kTempSource, double);
int bounds(int);




//SETUP

void setup() {

  int i = 0;
  M5.begin(true, false, true); 
  Serial.begin(115200);

  delay(500); // Wait for serial to initialize
  Serial.println("\n\nSensor hub starting...");
  M5.dis.drawpix(0, 0xff0000); //Red
  
  // Start Wi-Fi in AP mode
  WiFi.softAP(ssid, password);
  Serial.println("Access point up...");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  
  udp.begin(udpPort);



  //NMEA SETUP
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  // Set Product information
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Sensor Network",  // Manufacturer's Model ID
                                 "1.0003 20 Sept 2025",  // Manufacturer's Software version code
                                 "1.0003 20 Sept 2025" // Manufacturer's Model version
                                 );
    // Set device information
  NMEA2000.SetDeviceInformation(1, // Unique number. Use e.g. Serial number.
                                150, // Device function=Analog to NMEA 2000 Gateway. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Inter/Intranetwork Device. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );
  // Uncomment 3 rows below to see, what device will send to bus                           
   //NMEA2000.SetForwardStream(&Serial);  // PC output to default serial port
   //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
   //NMEA2000.SetForwardOwnMessages(false); // Do not print own messages.

  // We act as real node on bus. Some devices does not show messages, if they can not request information.
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  //NMEA2000.EnableForward(EnableForward); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.Open();

  Serial.println("NMEA up...");
  M5.dis.drawpix(0, 0x00ff00); //green

  //Interpolation test here...
  /*
    delay(4500);
  for (float i=0;i<=100;i+=0.5) {
    double v = Interpolation::Linear(fuelInput, fuelOutput, fuelValues, i, true);
    //static double Linear(double xValues[], double yValues[], int numValues, double pointX, bool trim = true);
    Serial.print("Input: ");
    Serial.print(i);
    Serial.print("Output: ");
    Serial.println(v);
  }
  Serial.println("Interpolation test done.");
    */
}



void loop() {
  // put your main code here, to run repeatedly:
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
       M5.dis.drawpix(0, 0x0000ff); //blue
      incomingPacket[len] = 0;
      String sentence = String(incomingPacket);

      Serial.print("Received: ");
      Serial.print(sentence);
      
      if (sentence.startsWith("$XDR")) {
        int start = sentence.indexOf(",F,");
        int end = sentence.indexOf(",L,", start);
        if (start > 0 && end > start) {
          String valueStr = sentence.substring(start + 3, end);
          if (sentence.indexOf("FUEL") > 0) {
            //interpolate the fuel level
            fuelLevel = bounds((int)Interpolation::Linear(fuelInput, fuelOutput, fuelValues, valueStr.toFloat(), true));
            SendN2kTankLevel(fuelLevel,fuelCapacity,N2kft_Fuel); 
            Serial.print(" / fuel Level: ");
            Serial.println(fuelLevel);
          } else if (sentence.indexOf("WATER") > 0) {
            //interpolate the water level
            waterLevel = bounds((int)Interpolation::Linear(waterInput, waterOutput, waterValues, valueStr.toFloat(), true));
            SendN2kTankLevel(waterLevel,waterCapacity,N2kft_Water); 
            Serial.print(" / water Level: ");
            Serial.println(waterLevel);
          } else if (sentence.indexOf("temp0") > 0) {
            SendN2kTemperature(1, N2kts_ExhaustGasTemperature,CToKelvin(valueStr.toDouble()));
            Serial.print(" / Exhaust temp: ");
            Serial.println(valueStr);
          } else if (sentence.indexOf("temp1") > 0) {
            SendN2kTemperature(2, N2kts_EngineRoomTemperature,CToKelvin(valueStr.toDouble()));
            Serial.print(" / Secondary temp: ");
            Serial.println(valueStr);
          }
        }
      }


    }
  }
  delay(50);
  M5.dis.drawpix(0, 0x00ff00); //green
}
//N2kft_Fuel //N2kft_Water //N2kft_GrayWater //N2kft_LiveWell //N2kft_Oil //N2kft_BlackWater //N2kft_FuelGasoline //N2kft_Error //N2kft_Unavailable
// put function definitions here:

void SendN2kTankLevel(double Level, double Capacity, tN2kFluidType FluidType) {
    tN2kMsg N2kMsg;
    SetN2kPGN127505(N2kMsg,1,FluidType,Level,Capacity);
    NMEA2000.SendMsg(N2kMsg);
    NMEA2000.ParseMessages();
}

void SendN2kTemperature(int i,tN2kTempSource tempSource, double val) {
  tN2kMsg N2kMsg;
  SetN2kTemperature(N2kMsg, 1, i, tempSource, val ,CToKelvin(21.6));
  NMEA2000.SendMsg(N2kMsg);
  NMEA2000.ParseMessages();
}
// N2kts_EngineRoomTemperature N2kts_ExhaustGasTemperature

int bounds(int val) {
  if (val < 0) return 0;
  if (val > 100) return 100;
  return val;
} 

  
  
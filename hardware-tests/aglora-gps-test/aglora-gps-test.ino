/*
GPS TEST for AGLoRa device
https://github.com/Udj13/AGLoRa/
*/
#include <SoftwareSerial.h>

// Please setup library "TinyGPSPlus by Mikal Hart" from Arduino IDE library manager
#include <TinyGPSPlus.h>

// ============ GPS PIN SETTINGS ============
static const int GPS_PIN_RX = 7, GPS_PIN_TX = 8;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(GPS_PIN_RX, GPS_PIN_TX);

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  Serial.println();
  Serial.println();
  Serial.println(F("AGLoRa fast GPS connection test"));
  Serial.println(F("TinyGPSPlus library"));
  Serial.println();
}

unsigned long lastGPSPrintTime;
static const unsigned int PERIODIC_GPS_INTERVAL = 2000;  // milliseconds


void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0) {
    char _nmea = gpsSerial.read();
    Serial.print(_nmea);
    if (gps.encode(_nmea)) {
      if (millis() > lastGPSPrintTime + PERIODIC_GPS_INTERVAL) {
        displayInfo();
        lastGPSPrintTime = millis();
      }
    }


    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS detected: check wiring."));
      while (true)
        ;
    }
  }
}

void displayInfo() {
  Serial.println();
  Serial.println();
  Serial.print(F("Tiny GPS Plus parser | "));
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

/*
Project AGLoRa (abbreviation of Arduino + GPS + LoRa)
Tiny and chip LoRa GPS tracker

Copyright 2021 Eugeny Shlyagin (shlyagin@gmail.com)

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Modules used:
- Arduino UNO/Nano (ATMEGA328P, not ATmega168)
- GPS NMEA Module (Generic)
- LoRa EBYTE E220-900T22D (868 MHz) or EBYTE E32-E433T30D (433 MHz)
- Bluetooth BLE AT-09 or HC-05
*/

// ========== GENERAL SETTINGS ========== 
#define NAME_LENGTH 6                   // the same value for all devices
char MY_NAME[NAME_LENGTH] = "Rick";  // name of current tracker, NAME_LENGTH characters
// ========== SERIAL DEBUGGING ========== 
#define DEBUG_MODE false


// ============ OTHER SETTINGS ========== 
// ============ GPS SETTINGS ============ 
#define GPS_PIN_RX 8
#define GPS_PIN_TX 7
#define GPS_PACKET_INTERVAL 10000 // milliseconds
// NOTE: GPS is valid, if LED_BUILTIN is HIGH
// =========== LORA SETTINGS =========== 
#define LORA_PIN_RX 2
#define LORA_PIN_TX 3
#define LORA_PIN_M0 4
#define LORA_PIN_M1 5
#define LORA_PIN_AX 6
int loraChannel = 5;  // default LORA channel
// ======= END OF SETTINGS =============


#include <SoftwareSerial.h>  // standart library
#include <TinyGPSPlus.h>         // install from Arduino IDE
#include "EBYTE.h"           // other-side library

// ========== GPS section ==========
TinyGPSPlus gps;
SoftwareSerial gps_ss(GPS_PIN_RX, GPS_PIN_TX);

// ========== LORA section ==========
struct DATA {
  float lat;
  float lon;
  unsigned char sat;

  unsigned short year;
  unsigned char month;
  unsigned char day;

  unsigned char hour;
  unsigned char minute;
  unsigned char second;

  char id[NAME_LENGTH];
};

DATA loraDataPacket;
SoftwareSerial lora_ss(LORA_PIN_RX, LORA_PIN_TX);
EBYTE loraTransceiver(&lora_ss, LORA_PIN_M0, LORA_PIN_M1, LORA_PIN_AX);

unsigned long lastLoraPacketTime;


// ========== SETUP ==========  
void setup()
{
  Serial.begin(9600);

  // BLE initialisation or debug via serial
  if (!DEBUG_MODE) setupBLE();

  // GPS initialisation
  gps_ss.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); // GPS valid indicator
  if (DEBUG_MODE) displayGPSInfo();

  // LORA initialisation
  lora_ss.begin(9600);
  loraTransceiver.init();
  loraTransceiver.SetAddressH(0xFF);
  loraTransceiver.SetAddressL(0xFF);
  loraTransceiver.SetChannel(loraChannel);
  loraTransceiver.SaveParameters(PERMANENT);  // save the parameters to the unit

  if (DEBUG_MODE) {
    Serial.println("AGLORA tracker started...");
    Serial.println("Current LORA parameters:");
    loraTransceiver.PrintParameters();          // print all parameters for debugging
  }
}

// ========== MAIN LOOP ==========  
void loop()
{
  ListenLORA();  // listening and send to serial
  sendGPStoLORA(); 
}


// =========================================== Listen LORA =====================================
void ListenLORA() {
  lora_ss.listen();     //switch to lora software serial
  for (unsigned long start = millis(); millis() - start < GPS_PACKET_INTERVAL;) {
    if (lora_ss.available()) {
      loraTransceiver.GetStruct(&loraDataPacket, sizeof(loraDataPacket));
      // if you got data, update the checker
      lastLoraPacketTime  = millis();

      // DEBUG_MODE
      if (DEBUG_MODE){      // dump out what was just received
        Serial.print("ID: "); Serial.print(loraDataPacket.id);
        Serial.print(" LAT: "); Serial.print(loraDataPacket.lat, 6);
        Serial.print(" LON: "); Serial.print(loraDataPacket.lon, 6);
        Serial.print(" SAT: "); Serial.print(loraDataPacket.sat);

        Serial.print(" Date: "); Serial.print(loraDataPacket.year);
        Serial.print("/"); Serial.print(loraDataPacket.month);
        Serial.print("/"); Serial.print(loraDataPacket.day);

        Serial.print(" Time: "); Serial.print(loraDataPacket.hour);
        Serial.print(":"); Serial.print(loraDataPacket.minute);
        Serial.print(":"); Serial.println(loraDataPacket.second);

        Serial.print("BLE data packet: |");
      }

      getNewData(loraDataPacket);
      writeStorageToBLE();

      if (DEBUG_MODE) Serial.println(" | end of BLE data.");
    }
    else {
      // if the time checker is over some prescribed amount
      // let the user know there is no incoming data
      if ((millis() - lastLoraPacketTime) > 5000) {
        if (DEBUG_MODE) Serial.print("No new LORA date. BLE| ");
        writeStorageToBLE();
        if (DEBUG_MODE) Serial.println(" | end BLE package");
        lastLoraPacketTime = millis();
      }
    }
  }
}


// =======------------------=== READ GPS and SEND TO LORA ==========
void sendGPStoLORA() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  // For one second we parse GPS data and report some key values
  gps_ss.listen();     //switch to gps software serial
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gps_ss.available() > 0)
    if (gps.encode(gps_ss.read())){
      newData = true;
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  
  if (newData && gps.location.isValid() && gps.date.isValid() && gps.time.isValid())
  {
    digitalWrite(LED_BUILTIN, HIGH); // GPS is valid
    sendData( gps.location.lat(), gps.location.lng(), gps.satellites.value(), 
              gps.date.year(), gps.date.month(), gps.date.day(), 
              gps.time.hour(), gps.time.minute(), gps.time.second());
    if (DEBUG_MODE) displayGPSInfo();
  } else {
    //sendData(0, 0, 0);
  }
  
  if (gps.charsProcessed() < 10)
    if (DEBUG_MODE) Serial.println("** No characters received from GPS: check wiring **");
}


// ========== SEND LORA DATA ==========  
void sendData(float lat, float lon, unsigned short sat, 
              unsigned short year, unsigned char month, unsigned char day, 
              unsigned char hour, unsigned char minute, unsigned char second){
  
  // data set
  loraDataPacket.lat = lat;
  loraDataPacket.lon = lon;
  loraDataPacket.sat = sat;

  loraDataPacket.year = year;
  loraDataPacket.month = month;
  loraDataPacket.day = day;

  loraDataPacket.hour = hour;
  loraDataPacket.minute = minute;
  loraDataPacket.second = second;

  strcpy(loraDataPacket.id, MY_NAME);


  // send
  lora_ss.listen();
  loraTransceiver.SendStruct(&loraDataPacket, sizeof(loraDataPacket));
  // debug
  if (DEBUG_MODE) Serial.println(F("SEND LORA DATE"));
}


// =================================== GPS ===========================================
void displayGPSInfo() {
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.print("Satellites in view: ");
  Serial.println(gps.satellites.value()); 
  
  Serial.print(F("Location: ")); 
  
  if (gps.location.isValid())
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print(F("INVALID "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
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
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}





// ================================ BLE SECTION ==================================
void setupBLE(){
  sendCommand("AT");
  sendCommand("AT+NAMEAGLora");
  sendCommand("AT+ROLE0");
//  sendCommand("AT+UUID0xFFE0"); - not working
//  sendCommand("AT+CHAR0xFFE1");
}


void sendCommand(const char * command){
  Serial.println(command);
  delay(200);       //wait some time
  char reply[100];
  int i = 0;
  while (Serial.available()) {
    reply[i] = Serial.read();
    i += 1;
  }
  reply[i] = '\0';  //end the string
}


void writeHeaderToBLE(){

  // AGLoRa BLE protocol
  // HEADER
  Serial.write("AGLoRa");   // signature 6 bytes
  Serial.write(0xAA);       // BLE protocol version, 1 byte

  Serial.write(0x01); // start of heading, 1 byte
  Serial.write(MY_NAME);//NAME_LENGTH bytes
  Serial.write(0x03); // end of text, 1 byte

}


void writePackageToBLE(DATA package) {
  union cvt {
    float val;
    unsigned char b[4];
  } x;

  // AGLoRa BLE protocol
  Serial.write(0x1D); //group separator, 1 byte
  Serial.write(0x02); // start of text, 1 byte
  Serial.write(package.id);   //NAME_LENGTH bytes
  Serial.write(0x03); // end of text, 1 byte

  Serial.write(0x1E); //record separator
  x.val = package.lat;
  Serial.write(x.b, 4);   //4 bytes
  x.val = package.lon;
  Serial.write(x.b, 4);   //4 bytes
  Serial.write(package.sat);      // 1 byte
  Serial.write(package.year);     //2 bytes
  Serial.write(package.month);    //1 byte
  Serial.write(package.day);      //1 byte
  Serial.write(package.hour);     //1 byte
  Serial.write(package.minute);   //1 byte
  Serial.write(package.second);   //1 byte
 
  Serial.write(0x04); // end of transmission, 1 byte
}

void readFromBLE() {
  char reply[100];
  int i = 0;
  byte sreply;

  while (Serial.available()) {
//    reply[i] = mySerial.read();
//    Serial.print(reply[i], HEX);
    i += 1;
    sreply = Serial.read();
    Serial.print(sreply, HEX);
  }
  if(i != 0 ) Serial.println();
}

// =============================== DATA STORAGE ========================================
#define STORAGE_SIZE 5
DATA storage[STORAGE_SIZE];
char storageCounter = 0;

// ADD NEW DATA TO STORAGE
void getNewData(DATA newData) {
  bool isExist = false;
  String newId = newData.id;

  for(int i = 0; i < storageCounter; i++){
    String id = storage[i].id;

    if( id == newId ) {
      storage[i] = newData;
      isExist = true;
      return;
    }
  }

  if( !isExist ) {
    storage[storageCounter] = newData;
    storageCounter++;
    if( storageCounter > STORAGE_SIZE ) storageCounter = STORAGE_SIZE;
  }
}


// Send all data from storage to BLE 
void writeStorageToBLE(){

  writeHeaderToBLE();               //Header
  Serial.write(storageCounter);     //Counter

  if( storageCounter > 0 ) {
    for(int i = 0; i <= storageCounter; i++){
      writePackageToBLE(storage[i]);  //Next tracker DATA
    }
  }
}



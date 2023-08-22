/*
Project AGLoRa (abbreviation of Arduino + GPS + LoRa)
Tiny and chip LoRa GPS tracker
Copyright 2023 Eugeny Shlyagin (shlyagin@gmail.com)

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
Modules used:
- Arduino UNO/Nano (ATMEGA328P, not ATmega168)
- GPS NMEA Module (Generic)
- LoRa EBYTE E220-900T22D (868 MHz) or EBYTE E32-E433T30D (433 MHz)
- Bluetooth BLE AT-09 or HC-05
*/

/* 
HOW THIS SKETCH WORKS :

Most of the time the tracker works in listening mode. 

After receiving the LoRa package from another tracker - 
this data is transferred to the bluetooth serial port.

Once every three minutes, the tracker switches to 
the GPS receiver for a few seconds to get coordinates. 
Then sends a LoRa data packet.

NOTE: GPS is valid, if LED_BUILTIN is HIGH
*/

// HOW TO SETUP:
// ========================================
// ==== Settings LEVEL 1 (required) =======
// ========================================
// The first thing you need is to set up a tracker name and modules connections:

// ========== NAME =======================
#define NAME_LENGTH 6                // The same value for all devices
char MY_NAME[NAME_LENGTH] = "Rick";  // Name of current tracker, NAME_LENGTH characters
// Example:
// char MY_NAME[NAME_LENGTH] = "Morty"; // All names length should be no longer than NAME_LENGTH
// ========== WIRING =====================
#define LORA_PIN_RX 2
#define LORA_PIN_TX 3
#define LORA_PIN_M0 4
#define LORA_PIN_M1 5
#define LORA_PIN_AX 6

#define GPS_PIN_RX 7
#define GPS_PIN_TX 8

// Then install "EByte LoRa E220 by Renzo Mischianty" library
// and "TinyGPSPlus by Mikal Hart" library
// from Arduino IDE library manager ("Tools" -> "Manage Libraries")

// Now you can upload this sketch to Arduino
// and turn on the app "AGLoRa" on your phone

// If something went wrong you can enable
// "debug mode" through the serial port.
// Don't forget to disconnect the bluetooth module.
// Then open “Tools” -> ”Serial monitor” in Arduino IDE.
#define DEBUG_MODE false  // change "false" to "true" to enable

// ========================================
// ==== Settings LEVEL 2 (optional) =======
// ========================================

/*
This is the structure of the LoRa data package.
If you want to send additional data between AGLoRa trackers
you should add it to this section.
Note that all software version must be the same for all trackers.
*/
struct DATA {
  char id[NAME_LENGTH];  // name

  float lat;  // coordinates
  float lon;
  unsigned char sat;

  unsigned char year;  // the Year minus 2000
  unsigned char month;
  unsigned char day;

  unsigned char hour;
  unsigned char minute;
  unsigned char second;

  // Add more data fields here if you need
  // ...
  // unsigned char speed;
  // unsigned char battery;
  // unsigned char sensor1;
  // ...
};



/*
This is a function that sends data to the app.
Data packets are sent using OsmAnd-like protocol:
id=name&lat={0}&lon={1}&timestamp={2}&speed={3}&altitude={4}
*/
void sendPackageToBluetooth(DATA *package) {

  Serial.print(F("&name="));  //other tracker's name
  Serial.print(package->id);  //NAME_LENGTH bytes

  Serial.print(F("&lat="));       // cordinates
  Serial.print(package->lat, 6);  // latitude
  Serial.print(F("&lon="));       // record separator
  Serial.print(package->lon, 6);  // longitute

  //Date and time format: 2023-06-07T15:21:00Z
  Serial.print(F("&timestamp="));      // record separator
  Serial.print(package->year + 2000);  // year
  Serial.print(F("-"));                // data separator
  if (package->month < 10) Serial.print(F("0"));
  Serial.print(package->month);        // month
  Serial.print(F("-"));                // data separator
  if (package->day < 10) Serial.print(F("0"));
  Serial.print(package->day);          // day
  Serial.print(F("T"));                // record separator
  if (package->hour < 10) Serial.print(F("0"));
  Serial.print(package->hour);         // hour
  Serial.print(F(":"));                // time separator
  if (package->minute < 10) Serial.print(F("0"));
  Serial.print(package->minute);       // minute
  Serial.print(F(":"));                // time separator
  if (package->second < 10) Serial.print(F("0"));
  Serial.print(package->second);       // second
  Serial.print(F("Z"));                // UTC

  // Sensors and additional data
  Serial.print(F("&sat="));    //record separator
  Serial.print(package->sat);  // satellites  1 byte

  // Add more data here if you need ...
  // Serial.print("&speed=");       // data's name in app
  // Serial.print(package->speed);   // value

  // Serial.print("&batt=");
  // Serial.print(package->battery);

  // Serial.print("&alienSensor=");
  // Serial.print(package->sensor1);

  Serial.print(F("|"));                // record separator
  Serial.println();  // end of transmission
}


// ========================================
// ==== Settings LEVEL 3 (nightmare) ======
// ========================================
#define USE_EEPROM_MEMORY false  // "false" by default
// set "false" to use SRAM memory, "true" to use EEPROM
// EEPROM is permanent memory, data is not lost even 
// if the system is turned off.
// But the write operation is finite and usually capped at 100,000 cycles.
// Please read: https://docs.arduino.cc/learn/programming/memory-guide
// ============ GPS SETTINGS ============
#define GPS_PACKET_INTERVAL 20000  // milliseconds
// ============ OTHER SETTINGS ==========
#define BLE_CHECK_INTERVAL 30000  // milliseconds
// ============ SRAM STORAGE ==============
// Maximum number of track points (struct DATA) in memory
// Change and check free memory in "Output" after pressing "Verify".
#define SRAM_STORAGE_SIZE 40    // DATA array size
// not used if USE_EEPROM_MEMORY true, may be zero in this case
// ============ EEPROM STORAGE ==============
// EEPROM (non-volatile) memory
// not used if define USE_EEPROM_MEMORY false
#define EEPROM_BEGIN_ADDRESS 0  //bytes
// reserve for storing settings
// not used if USE_EEPROM_MEMORY false
// =========================================
// ========== END OF SETTINGS ==============
// =========================================



/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      Everything below this line is a sketch 
  that will not need to be changed in most cases
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


#include <SoftwareSerial.h>   // standart library
#include <TinyGPSPlus.h>      // install from Arduino IDE
// Docs: http://arduiniana.org/libraries/tinygpsplus/
#include "LoRa_E220.h"        // install from Arduino IDE
// Docs: https://github.com/xreef/EByte_LoRa_E220_Series_Library

// ========== GPS section ==========
TinyGPSPlus gps;
SoftwareSerial gps_ss(GPS_PIN_RX, GPS_PIN_TX);

// ========== LORA section ==========
DATA loraDataPacket;
SoftwareSerial lora_ss(LORA_PIN_RX, LORA_PIN_TX);
LoRa_E220 e220ttl(&lora_ss, 6, 4, 5);  // AUX M0 M1
//LoRa_E220 e220ttl(&lora_ss); // Config without connect AUX and M0 M1

unsigned long _lastLoraPacketTime;


// ========== BEGIN ==========
void setup() {
  Serial.begin(9600);

  agloraGreetings();            // Beautifully print Hello from AGloRa :-)
  
  // Start modules
  if (!DEBUG_MODE) setupBLE();  // BLE initialisation or debug via serial
  setupGPS();
  setupLoRa();
  storageInit();

  #if DEBUG_MODE 
    Serial.println(F("All initializations completed."));
  #endif
}

// ========== MAIN LOOP ==========
unsigned long _nextGPSCheckTime = millis() + GPS_PACKET_INTERVAL;

void loop() {
  if (_nextGPSCheckTime < millis()) {
    gps_ss.listen();   //switch to gps software serial
    sendMyLocationToLoRa();     // send location to other trackers
    lora_ss.listen();  //switch to lora software serial
    _nextGPSCheckTime += GPS_PACKET_INTERVAL;
  }

  if (listenToLORA()) {                       // listening and send to serial
    addNewDataToStorage(&loraDataPacket);
    writeNewPositionToBLE();                  // upload data to app
    _lastLoraPacketTime = millis();  // if you got data, update the checker
  };

  // if the time checker is over some prescribed amount
  // let the user know there is no incoming data
  if ((millis() - _lastLoraPacketTime) > BLE_CHECK_INTERVAL) {
    #if DEBUG_MODE
      Serial.println();
      Serial.println(F("[LoRa: No new LORA data]"));
    #endif
    writeTrackerInfoToBLE();
    _lastLoraPacketTime = millis();
  }

  readFromBluetooth();  // check requests from app
}

void agloraGreetings(){
  #if DEBUG_MODE
    Serial.println();
    for(int i =0; i<25; i++){
      Serial.print(F("-"));
      delay(100);
    }
    Serial.println();
    Serial.println(F("AGLORA tracker started..."));
  #endif
}


// =========================================== Listen LORA =====================================
bool listenToLORA() {
  if (e220ttl.available() > 1) {
    ResponseStructContainer rsc = e220ttl.receiveMessage(sizeof(DATA));
    if (rsc.status.code != 1) {
      Serial.println(rsc.status.getResponseDescription());
    } else {
      loraDataPacket = *(DATA *)rsc.data;
      rsc.close();
    }

    // DEBUG_MODE
    #if DEBUG_MODE   // dump out what was just received
      Serial.println();
      Serial.println(F("[LoRa: New data received.]"));
      Serial.print(F("ID: "));
      Serial.print(loraDataPacket.id);
      Serial.print(F(", lat: "));
      Serial.print(loraDataPacket.lat, 6);
      Serial.print(F(", lon: "));
      Serial.print(loraDataPacket.lon, 6);
      Serial.print(F(", satellites: "));
      Serial.print(loraDataPacket.sat);

      Serial.print(F(", date: "));
      Serial.print(loraDataPacket.year);
      Serial.print(F("/"));
      Serial.print(loraDataPacket.month);
      Serial.print(F("/"));
      Serial.print(loraDataPacket.day);

      Serial.print(F(", time: "));
      Serial.print(loraDataPacket.hour);
      Serial.print(F(":"));
      Serial.print(loraDataPacket.minute);
      Serial.print(F(":"));
      Serial.print(loraDataPacket.second);
      Serial.println(F(" (UTC)"));
    #endif
    return true;
  }
  return false;
}


// ====================== READ GPS and SEND TO LORA ==============================
void sendMyLocationToLoRa() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  // For three seconds we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 3000;) {
    while (gps_ss.available() > 0)
      if (gps.encode(gps_ss.read())) {
        newData = true;
      }
  }

  digitalWrite(LED_BUILTIN, LOW);

  if (newData && gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    digitalWrite(LED_BUILTIN, HIGH);  // GPS is valid


    // data set
    loraDataPacket.lat = gps.location.lat();
    loraDataPacket.lon = gps.location.lng();
    loraDataPacket.sat = gps.satellites.value();

    loraDataPacket.year = gps.date.year() - 2000;
    loraDataPacket.month = gps.date.month();
    loraDataPacket.day = gps.date.day();

    loraDataPacket.hour = gps.time.hour();
    loraDataPacket.minute = gps.time.minute();
    loraDataPacket.second = gps.time.second();

    strcpy(loraDataPacket.id, MY_NAME);
    
    #if DEBUG_MODE 
      displayGPSInfo();
    #endif

    sendLoRaData();
  } else {
    #if DEBUG_MODE 
      Serial.println();
      Serial.println(F("[GPS: No valid data.]"));
    #endif
  }

  if (gps.charsProcessed() < 10){
    #if DEBUG_MODE 
      Serial.println();
      Serial.println(F("[GPS: No characters received from GPS, check wiring!]"));
    #endif
  }
}


// ================================ LORA =================================
void setupLoRa() {
  #if DEBUG_MODE 
    Serial.println(F("[LoRa: Start EByte LoRa configuration.]")); 
  #endif

  lora_ss.begin(9600);
  e220ttl.begin();
  e220ttl.resetModule();

  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  Configuration configuration = *(Configuration *)c.data;
  delay(100);

  configuration.ADDL = 0x00;
  configuration.ADDH = 0x00;
  configuration.CHAN = 0x17;
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;
  configuration.OPTION.subPacketSetting = SPS_200_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;  // monitoring before data transmitted
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

  e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  delay(100);
}

void sendLoRaData() {
  lora_ss.listen();

  #if DEBUG_MODE 
    Serial.println();
    Serial.print(sizeof(loraDataPacket));
    Serial.println(F(" bytes ready to send"));
    Serial.print(F("* "));
    Serial.print(loraDataPacket.id);
    Serial.print(F("/"));
    Serial.print(loraDataPacket.lat, 6);
    Serial.print(F("/"));
    Serial.print(loraDataPacket.lon, 6);
    Serial.print(F(" - "));
    Serial.print(loraDataPacket.year);
    Serial.print(F("-"));
    Serial.print(loraDataPacket.month);
    Serial.print(F("-"));
    Serial.print(loraDataPacket.day);
    Serial.print(F(" "));
    Serial.print(loraDataPacket.hour);
    Serial.print(F(":"));
    Serial.print(loraDataPacket.minute);
    Serial.println(F(" (UTC) ... *"));
    Serial.println(F("[LoRa: sending coordinates.]"));
  #endif

  e220ttl.sendMessage(&loraDataPacket, sizeof(loraDataPacket));
}


// =================================== GPS ===========================================

void setupGPS() {
  #if DEBUG_MODE
    Serial.println(F("[GPS: Start GPS configuration.]")); 
  #endif
  gps_ss.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);  // GPS valid indicator
}

void displayGPSInfo() {
  #if DEBUG_MODE
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println();
    Serial.print(F("[GPS: updating my GPS position. Satellites in view: "));
    Serial.print(gps.satellites.value());
    Serial.println(F("]"));

    Serial.print(F("Location: "));

    if (gps.location.isValid()) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print(F("INVALID "));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
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
  #endif
}





// ================================ BLE SECTION ==================================
void setupBLE() {
  sendCommand(F("AT"));
  sendCommand(F("AT+NAMEAGLoRa"));
  sendCommand(F("AT+ROLE0"));
}


void sendCommand(const String command) {
  Serial.println(command);
  delay(200);  //wait some time
  char reply[100];
  int i = 0;
  while (Serial.available()) {
    reply[i] = Serial.read();
    i += 1;
  }
  reply[i] = '\0';  //end the string
}

void readFromBluetooth() {

  while (Serial.available()) {
    String command = Serial.readString();  //read until timeout
    command.trim();                        // remove any \r \n whitespace at the end of the String
    storageManager(&command);
  }
}







// =============================== DATA STORAGE ========================================
void storageManager(String *command) {
  #if DEBUG_MODE
    Serial.println();
    Serial.print(F("[Bluetooth input: New command received <"));
    Serial.print(*command);
    Serial.println(F(">.]"));
  #endif

  if (command->startsWith(F("info"))) { writeTrackerInfoToBLE(); return;}
  if (command->startsWith(F("all"))) { writeAllPositionsToBLE(); return;}
  if (command->startsWith(F("id="))) { writePositionToBLE(command); return;}
  if (command->startsWith(F("clear"))) { clearAllPositions(); return;}
  if (command->startsWith(F("restart"))) { storageInit(); return;}


  #if DEBUG_MODE
    Serial.println(F("[Storage: Unknown command.]"));
  #endif  
}








#if USE_EEPROM_MEMORY
// ===================== EEPROM MEMORY ===========================
  #include <EEPROM.h>

  struct EEPROMDATA {
     unsigned char counter;
     DATA data;
     unsigned char crc;
  };

  EEPROMDATA EEPROMdata;
  
  unsigned int EEPROMStorageIndex = 0;  // index in memory (address = EEPROMStorageIndex * EEPROMDataSize)  
  unsigned int incrementCounter = 0;             // min 0, max 254 (because default EEPROM is 255)

  const unsigned char EEPROMDataSize = sizeof(EEPROMDATA);
  const unsigned int EEPROMStorageSize = ( EEPROM.length() - EEPROM_BEGIN_ADDRESS ) / EEPROMDataSize;

  void storageInit(){
    EEPROMStorageIndex = 0;
    incrementCounter = 0;

    #if DEBUG_MODE 
      Serial.print(F("[Storage: Start EEPROM initialization. Size of memory: "));
      Serial.print(EEPROM.length());
      Serial.println(F(" bytes]"));
      Serial.print(F("[Storage: "));
      Serial.print(EEPROMStorageSize);
      Serial.println(F(" track points can be saved.]"));
      Serial.print(F("Finding the index of the last record. Read memory... "));
    #endif


    unsigned int prevIncCounter = 0;

    #if DEBUG_MODE 
      for(int i = 0; i < EEPROMStorageSize; i++){
        EEPROM.get(i * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, EEPROMdata);

        unsigned char crc = calculateCRC((unsigned char*)&EEPROMdata.data, sizeof(EEPROMdata.data));

        Serial.print(F(", "));
        if( (i % 10) == 0 ){
          Serial.println(F("*"));
        }
        Serial.print(EEPROMdata.counter);

        if( EEPROMdata.counter != 255 ){
          if( crc == EEPROMdata.crc) {
          Serial.print(F("(CRC OK)"));
          } else {
          Serial.print(F("(CRC ERROR)"));
          }
        }

      }
      Serial.println();
    #endif

    for(int i = 0; i < EEPROMStorageSize; i++){
      EEPROM.get(i * EEPROMDataSize  + EEPROM_BEGIN_ADDRESS, EEPROMdata);

      if( EEPROMdata.counter == 255 ){      // check empty EEPROM cell
        if( i == 0 ) {break;}
        EEPROMStorageIndex = i;
        incrementCounter = prevIncCounter+1;
        if(incrementCounter >= 255) {incrementCounter = 0;}
        break;
      }

      if ( abs(EEPROMdata.counter - prevIncCounter) > 1 ) { // not in sequence
        if( i != 0 ){
          if( prevIncCounter != 254 ){    // exclude the option ...252,253,254,0,1,2...
            EEPROMStorageIndex = i;
            incrementCounter = prevIncCounter+1;
            if(incrementCounter >= 255) {incrementCounter = 0;}
            break;
          }
        }
      }
      prevIncCounter = EEPROMdata.counter;
    }

    #if DEBUG_MODE 
      Serial.println();
      Serial.print(F("Next record will be "));
      Serial.println(EEPROMStorageIndex);
    #endif

  }

  void writeTrackerInfoToBLE() {
    #if DEBUG_MODE
      Serial.println(F("[Bluetooth output: Send AGLoRa tracker's info.]"));
    #endif
    
    Serial.print(F("AGLoRa-info"));    // signature 6 bytes
    Serial.print(F("&ver=2.0"));  // BLE protocol version
    Serial.print(F("&myId="));    // tracker's name
    Serial.print(MY_NAME);
    Serial.print(F("&memorySize="));  // storage length
    Serial.print(EEPROMStorageSize);
    Serial.print(F("&memoryIndex="));  // storage length
    Serial.print(EEPROMStorageIndex);
    Serial.println(F("|"));                // record separator

  }

  // ADD NEW DATA TO STORAGE
  void addNewDataToStorage(DATA *newData) {
    EEPROMdata.counter = incrementCounter;
    EEPROMdata.data = *newData;
    EEPROMdata.crc = calculateCRC((unsigned char*)newData, sizeof(DATA));
    EEPROM.put(EEPROMStorageIndex * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, EEPROMdata);

    #if DEBUG_MODE
      Serial.print(F("[Storage: New data added. Address: "));
      Serial.print(EEPROMStorageIndex * EEPROMDataSize + EEPROM_BEGIN_ADDRESS);
      Serial.print(F(", index: "));
      Serial.print(EEPROMStorageIndex);
      Serial.print(F(", counter: "));
      Serial.print(EEPROMdata.counter);
      Serial.print(F(", CRC: "));
      Serial.print(EEPROMdata.crc);
      Serial.println(F("]"));
    #endif


    EEPROMStorageIndex++;
    if (EEPROMStorageIndex >= EEPROMStorageSize) {
      EEPROMStorageIndex = 0;
    }

    incrementCounter++;
    if ( incrementCounter >= 255 ) {
      incrementCounter = 0;
    }
  }

  void writeNewPositionToBLE() {
    #if DEBUG_MODE 
      Serial.println("[Bluetooth output: Send new data to BLE.]");
    #endif

      Serial.print(F("AGLoRa-newpoint&"));
      Serial.print(F("id="));
      Serial.print(EEPROMStorageIndex - 1);
      sendPackageToBluetooth(&EEPROMdata.data);
  }

  // Send all data from storage to BLE
  void writeAllPositionsToBLE() {
    #if DEBUG_MODE 
      Serial.println(F("[Bluetooth output: Response to <all> command.]"));
    #endif
    unsigned char crc = 0;
    for (int i = 0; i < EEPROMStorageSize; i++) {
      EEPROM.get(i * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, EEPROMdata);
      if( EEPROMdata.counter != 255 ){
        #if DEBUG_MODE 
          Serial.print(EEPROMdata.counter);
          Serial.print(F(" - "));
        #endif
        Serial.print(F("AGLoRa-point&"));
        Serial.print(F("id="));
        Serial.print(i);
        crc = calculateCRC((unsigned char*)&EEPROMdata.data, sizeof(EEPROMdata.data));
        if (EEPROMdata.crc == crc) {
          Serial.print(F("&CRC_OK"));
        } else {
          Serial.print(F("&CRC_ERROR"));
        }
        Serial.print(F("&"));
        sendPackageToBluetooth(&EEPROMdata.data);  //Next tracker DATA
        delay(10);
      }
    }
  }

  void writePositionToBLE(String *command) {
    int index = command->substring(3).toInt();  // length of command "id="

    #if DEBUG_MODE 
      Serial.println(F("[Bluetooth output: Response to <id=X> command.]"));
    #endif

    Serial.print(F("AGLoRa-point&"));

    if(index > EEPROMStorageSize - 1 ){Serial.println(F("ErrorIndexOutOfRange")); return;}

    EEPROM.get(index * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, EEPROMdata);

    Serial.print(F("id="));
    Serial.print(index);
    const unsigned char crc = calculateCRC((unsigned char*)&EEPROMdata.data, sizeof(EEPROMdata.data));
    if (EEPROMdata.crc == crc) {
      Serial.print(F("&CRC_OK"));
    } else {
      Serial.print(F("&CRC_ERROR"));
    }
    Serial.print(F("&"));
    sendPackageToBluetooth(&EEPROMdata.data);  //Next tracker DATA
    delay(10);
  }

  void clearAllPositions(){
  #if DEBUG_MODE 
      Serial.println(F("[Storage: clearing memory.]"));
    #endif

    unsigned char memoryCell = 0;
    for (int i = 0; i < EEPROM.length(); i++) {
      EEPROM.get(i + EEPROM_BEGIN_ADDRESS, memoryCell);
      if( memoryCell != 255 ){  // 255 - default value
        memoryCell = 255;
        EEPROM.put(i + EEPROM_BEGIN_ADDRESS, memoryCell);
      }
    }

    EEPROMStorageIndex = 0;
    incrementCounter = 0;
    storageInit();
  }

#else
// ===================== SRAM MEMORY ===========================

  DATA storage[SRAM_STORAGE_SIZE];
  unsigned int storageIndex = 0;
  bool storageOverflow = false;


  void storageInit(){
    #if DEBUG_MODE 
      Serial.println(F("[Storage: Start initialization.]"));
    #endif
  }

  void writeTrackerInfoToBLE() {
    #if DEBUG_MODE
      Serial.println(F("[Bluetooth output: Send AGLoRa tracker's info.]"));
    #endif
    
    Serial.print(F("AGLoRa-info"));    // signature 6 bytes
    Serial.print(F("&ver=2.0"));  // BLE protocol version
    Serial.print(F("&myId="));    // tracker's name
    Serial.print(MY_NAME);
    Serial.print(F("&memorySize="));  // storage length
    Serial.print(SRAM_STORAGE_SIZE);
    Serial.print(F("&memoryIndex="));  // storage length
    Serial.println(storageIndex);

  }

  // ADD NEW DATA TO STORAGE
  void addNewDataToStorage(DATA *newData) {
    storageIndex++;
    if (storageIndex >= SRAM_STORAGE_SIZE) {
      storageIndex = 0;
      storageOverflow = true;
    }
    storage[storageIndex] = *newData;

    #if DEBUG_MODE
      Serial.println(F("[Storage: New data added.]"));
    #endif
  }

  void writeNewPositionToBLE() {
    #if DEBUG_MODE 
      Serial.println("[Bluetooth output: Send new data to BLE.]");
    #endif

      Serial.print(F("AGLoRa-newpoint&"));
      Serial.print(F("id="));
      Serial.print(storageIndex);
      sendPackageToBluetooth(&storage[storageIndex]);
  }

  // Send all data from storage to BLE
  void writeAllPositionsToBLE() {
    #if DEBUG_MODE 
      Serial.println(F("[Bluetooth output: Response to <all> command.]"));
    #endif

    unsigned int maxIndex = storageOverflow?(SRAM_STORAGE_SIZE-1):storageIndex;

    for (int i = 0; i <= maxIndex; i++) {

      Serial.print(F("AGLoRa-point&"));
      Serial.print(F("id="));
      Serial.print(i);
      Serial.print(F("&"));
      sendPackageToBluetooth(&storage[i]);  //Next tracker DATA
      delay(10);
    }
  }

  void writePositionToBLE(String *command) {
    int index = command->substring(3).toInt();  // length of command "id="

    #if DEBUG_MODE 
      Serial.println(F("[Bluetooth output: Response to <id=X> command.]"));
    #endif

    if(index == 0 ){Serial.println(F("ErrorZeroIndex")); return;}
    if(index > storageIndex ){Serial.println(F("ErrorIndexOutOfRange")); return;}

    Serial.print(F("AGLoRa-point&"));
    Serial.print(F("id="));
    Serial.print(index);
    Serial.print(F("&"));
    sendPackageToBluetooth(&storage[index]);  //Next tracker DATA
    delay(10);
  }

  void clearAllPositions(){
  #if DEBUG_MODE 
      Serial.println(F("[Storage: clearing memory.]"));
    #endif

    storageIndex = 0;
    storageOverflow = false;
  }

#endif









// ======================== UTILITES ===============================

// CRC for AVR only
unsigned char calculateCRC(unsigned char *buffer, unsigned char size) {
  unsigned char crc = 0;
  for (unsigned char i = 0; i < size; i++) {
    unsigned char data = buffer[i];
    uint8_t counter;
    uint8_t buffer;
    asm volatile (
      "EOR %[crc_out], %[data_in] \n\t"
      "LDI %[counter], 8          \n\t"
      "LDI %[buffer], 0x8C        \n\t"
      "_loop_start_%=:            \n\t"
      "LSR %[crc_out]             \n\t"
      "BRCC _loop_end_%=          \n\t"
      "EOR %[crc_out], %[buffer]  \n\t"
      "_loop_end_%=:              \n\t"
      "DEC %[counter]             \n\t"
      "BRNE _loop_start_%="
      : [crc_out]"=r" (crc), [counter]"=d" (counter), [buffer]"=d" (buffer)
      : [crc_in]"0" (crc), [data_in]"r" (data)
    );
  }
  return crc;
}

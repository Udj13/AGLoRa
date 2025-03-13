/*
Project AGLoRa (abbreviation of Arduino + GPS + LoRa)
Tiny and chip LoRa GPS tracker

https://github.com/Udj13/AGLoRa/

Copyright © 2021-2024 Eugeny Shlyagin. Contacts: <shlyagin@gmail.com>
License: http://opensource.org/licenses/MIT

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty.

Modules used:
- Arduino UNO/Nano (ATMEGA328P, not ATmega168)
- GPS NMEA Module (Generic)
- LoRa EBYTE E220-900T22D (868 MHz) or EBYTE E32-E433T30D (433 MHz)
- Bluetooth BLE AT-09 or HC-05
*/

/*
HOW THIS SKETCH WORKS:

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

#include <SoftwareSerial.h>
#include <EEPROM.h>

// First of all, select EBYTE module:
#define EBYTE_E220 // EBYTE_E220 or EBYTE_E32
//#define EBYTE_E32 // EBYTE_E220 or EBYTE_E32

// Here are the libraries that I used in this project, thanks to their author Renzo Mischianti!
// Please, give him a donation if you also think he did a great job!
// https://github.com/xreef
// https://mischianti.org/

#ifdef EBYTE_E32
#include "LoRa_E32.h"
// Docs: https://github.com/xreef/LoRa_E32_Series_Library
#endif
//or
#ifdef EBYTE_E220
#include "LoRa_E220.h"
// Docs: https://github.com/xreef/EByte_LoRa_E220_Series_Library
#endif

// After selecting a module, you need to set up a tracker name and module connections:

// ========== NAME =======================
#define NAME_LENGTH 12             // The same value for all devices
const char NAME[NAME_LENGTH] = "Morty";               // Name of current tracker, NAME_LENGTH characters
// Example:
// #define NAME = "Rick"; // All names length should be no longer than NAME_LENGTH
// ========== WIRING =====================
//UART LORA
#define LORA_PIN_RX 2
#define LORA_PIN_TX 3
#define LORA_PIN_M0 4
#define LORA_PIN_M1 5
#define LORA_PIN_AX 6

//UART GPS
#define GPS_PIN_RX 7
#define GPS_PIN_TX 8

// Leds
#define LORA_LED LED_BUILTIN
#define GPS_LED LED_BUILTIN
#define BLE_LED LED_BUILTIN
#define MEMORY_LED LED_BUILTIN
// ========== MODULES SETTING=============
#define GPS_SPEED 9600
#define LORA_START_SPEED 9600

// Then install "EByte LoRa E220 by Renzo Mischianty" library
// or "EByte LoRa E32 by Renzo Mischianty"
// from Arduino IDE library manager ("Tools" -> "Manage Libraries")

// Now you can upload this sketch to Arduino
// and turn on the app "AGLoRa" on your phone

// If something went wrong you can enable
// "debug mode" through the serial port.
// Don't forget to disconnect the bluetooth module.
// Then open "Tools" -> "Serial monitor" in Arduino IDE.
#define DEBUG_MODE false  // change "false" to "true" to enable
// Next, logs levels for comfortable deallbugging,
// if DEBUG_MODE == false, logs level are not important
#define DEBUG_BLE false  // bluetooth low energy
#define DEBUG_GPS true  // print GPS logs
#define DEBUG_LORA true  // print GPS logs
#define DEBUG_MEMORY true  // print GPS logs
#define DEBUG_AGLORA true  // print GPS logs

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
  char name[NAME_LENGTH];  // name

  float lat;  // coordinates
  float lon;

  unsigned char year;  // the Year minus 2000
  unsigned char month;
  unsigned char day;

  unsigned char hour;
  unsigned char minute;
  unsigned char second;

  bool gpsValid;

  // Add more data fields here if you need
  // ...
  //unsigned char sat;
  //unsigned char hdop;
  unsigned char battery;
  // unsigned char speed;
  // unsigned char sensor1;
  // unsigned char sensor2;
  // ...
};

// Forward declarations for functions that are defined later
String sendToPhone(DATA *package);
String sendBatteryToPhone();

// ========================================
// ==== Settings LEVEL 3 (nightmare) ======
// ========================================
#define USE_EEPROM_MEMORY false  // "false" by default
// set "false" to use SRAM memory, "true" to use EEPROM
// EEPROM is permanent memory, data is not lost even
// if the system is turned off.
// But the write operation is finite and usually capped at 100,000 cycles.
// Please read: https://docs.arduino.cc/learn/programming/memory-guide
// ============ LORA NETWORK SETTINGS ============
#define I_WANT_TO_SEND_MY_LOCATION true  // "true" by default
#define DATA_SENDING_INTERVAL 30000  // milliseconds (seconds * 1000)

#define MESH_MODE true  // "true" by default
#define TTL 3  // Data packet lifetime (for transfer between devices)
// ============ OTHER SETTINGS ==========
#define USE_BLE true  // use BLE output
#define BLE_UPDATE_INTERVAL 50000  // milliseconds (seconds * 1000)
// ============ SRAM STORAGE ==============
// Maximum number of track points (struct DATA) in memory
// Change and check free memory in "Output" after pressing "Verify".
#define SRAM_STORAGE_SIZE 15    // DATA array size
// not used if USE_EEPROM_MEMORY true, may be zero in this case
// NOTE: Don't use all free memory! It's may broke BLE output.
// You should hold free memory for return String from "sendToPhone"
// ============ EEPROM STORAGE ==============
// EEPROM (non-volatile) memory
// not used if define USE_EEPROM_MEMORY false
#define EEPROM_BEGIN_ADDRESS 0  //bytes
// reserve for storing settings
// not used if USE_EEPROM_MEMORY false
// ================ TESTS ==================
#define TEST_LORA_DATA false
#define OTHER_NAME "Summer" // virtual tracker's name, don't forget about NAME_LENGTH
// =========================================
// ========== END OF SETTINGS ==============
// =========================================

// ====================== INDICATION SECTION =======================
/*
  ___   _   _   ____    ___    ____      _      _____   ___    ___    _   _
 |_ _| | \ | | |  _ \  |_ _|  / ___|    / \    |_   _| |_ _|  / _ \  | \ | |
  | |  |  \| | | | | |  | |  | |       / _ \     | |    | |  | | | | |  \| |
  | |  | |\  | | |_| |  | |  | |___   / ___ \    | |    | |  | |_| | | |\  |
 |___| |_| \_| |____/  |___|  \____| /_/   \_\   |_|   |___|  \___/  |_| \_|
*/

enum class GPSStatuses
{
    correct,
    invalid,
    connectionError
};

enum class LoRaStatuses
{
    dataReceived,
    dataTransmitted,
    error
};

enum class BLEStatuses
{
    output,
    input,
    error
};

enum class MemoryStatuses
{
    read,
    write,
    crcError
};

class INDICATION
{
public:
    INDICATION(uint8_t gpsLedPin,
               uint8_t loraLedPin,
               uint8_t bleLedPin,
               uint8_t memoryLedPin);
    void gps(GPSStatuses status);
    void lora(LoRaStatuses status);
    void ble(BLEStatuses status);
    void memory(MemoryStatuses status);
    void loop();

private:
    uint8_t _gpsLedPin;
    uint8_t _loraLedPin;
    uint8_t _bleLedPin;
    uint8_t _memoryLedPin;

    bool loraLedStatus = false;
    const byte loraDelaySec = 1;
    unsigned long lastLoraUpdateTime;
};

INDICATION::INDICATION(uint8_t gpsLedPin, uint8_t loraLedPin, uint8_t bleLedPin, uint8_t memoryLedPin)
{
    _gpsLedPin = gpsLedPin;
    _loraLedPin = loraLedPin;
    _bleLedPin = bleLedPin;
    _memoryLedPin = memoryLedPin;

    pinMode(_gpsLedPin, OUTPUT);  // GPS indicator
    pinMode(_loraLedPin, OUTPUT); // LORA indicator
}

void INDICATION::gps(GPSStatuses status)
{
    switch (status)
    {
    case GPSStatuses::correct:
        digitalWrite(_gpsLedPin, HIGH);
        break;
    case GPSStatuses::invalid:
        digitalWrite(_gpsLedPin, LOW);
        break;
    case GPSStatuses::connectionError:
        digitalWrite(_gpsLedPin, LOW);
        break;
    default:
        digitalWrite(_gpsLedPin, LOW);
    }
}

void INDICATION::lora(LoRaStatuses status)
{
    switch (status)
    {
    case LoRaStatuses::dataReceived:
        digitalWrite(_loraLedPin, HIGH);
        loraLedStatus = true;
        break;
    case LoRaStatuses::dataTransmitted:
        digitalWrite(_loraLedPin, HIGH);
        loraLedStatus = true;
        break;
    case LoRaStatuses::error:
        digitalWrite(_loraLedPin, LOW);
        loraLedStatus = false;
        break;
    default:
        digitalWrite(_loraLedPin, LOW);
        loraLedStatus = false;
    }
    lastLoraUpdateTime = millis();
}

void INDICATION::ble(BLEStatuses status)
{
}

void INDICATION::memory(MemoryStatuses status)
{
}

void INDICATION::loop()
{
    if (loraLedStatus)
    {
        if ((lastLoraUpdateTime + (loraDelaySec * 1000)) < millis())
        {
            digitalWrite(_loraLedPin, LOW);

            loraLedStatus = false;
        }
    }
}

// ======================== UTILITES ===============================
/*
          _     _   _   _   _
  _   _  | |_  (_) | | (_) | |_    ___   ___
 | | | | | __| | | | | | | | __|  / _ \ / __|
 | |_| | | |_  | | | | | | | |_  |  __/ \__ \
  \__,_|  \__| |_| |_| |_|  \__|  \___| |___/
*/

// CRC
unsigned char calculateCRC(unsigned char *buffer, unsigned char size) {
  byte crc = 0;
  for (byte i = 0; i < size; i++) {
    byte data = buffer[i];
    for (int j = 8; j > 0; j--) {
      crc = ((crc ^ data) & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
      data >>= 1;
    }
  }
  return crc;
}

// ========================== BLE SECTION ==============================
/*
  ____    _       _   _   _____   _____    ___     ___    _____   _   _
 | __ )  | |     | | | | | ____| |_   _|  / _ \   / _ \  |_   _| | | | |
 |  _ \  | |     | | | | |  _|     | |   | | | | | | | |   | |   | |_| |
 | |_) | | |___  | |_| | | |___    | |   | |_| | | |_| |   | |   |  _  |
 |____/  |_____|  \___/  |_____|   |_|    \___/   \___/    |_|   |_| |_|
*/

class BLE_HM10
{
public:
    BLE_HM10();
    void setup();
    String read();
    void send(String * package);

private:
    const byte MTU = 22;
    void sendCommand(const String command);
};

BLE_HM10::BLE_HM10()
{
}

void BLE_HM10::setup()
{
#if DEBUG_MODE && DEBUG_BLE
    Serial.print(F("📲[BLE: ready for work ✅. Maximum Transmission Unit (MTU) = "));
    Serial.print(MTU);
    Serial.println(F("]"));
#endif
#if !DEBUG_MODE
    sendCommand(F("AT"));
    sendCommand(F("AT+NAMEAGLoRa"));
    sendCommand(F("AT+ROLE0"));
#endif
}

String BLE_HM10::read()
{
    String result = "";
    while (Serial.available())
    {
        result += Serial.readString(); // read until timeout
    }
    result.trim(); // remove any \r \n whitespace at the end of the String

    return result;
}

void BLE_HM10::send(String *package)
{
#if DEBUG_MODE && DEBUG_BLE
    Serial.print(F("📲[BLE: 📫 Sending: "));
    Serial.print(*package);

    Serial.print(F("\t"));
    for (byte i = 1; i <= MTU; ++i)
    {
        Serial.print(i % 10);
    }
    Serial.print(F(" (MTU = "));
    Serial.print(MTU);
    Serial.println(F(")"));
#endif

    bool isStringNotEmpty = true;
    while (isStringNotEmpty)
    {
#if DEBUG_MODE && DEBUG_BLE
        Serial.print(F("\t"));
#endif
        String nextSendMTU = package->substring(0, MTU);
        package->remove(0, MTU);
        isStringNotEmpty = package->length() != 0;

#if !DEBUG_MODE && !DEBUG_BLE
        // important part
        Serial.print(nextSendMTU); //  here we send data to BLE
        delay(10);
#endif

#if DEBUG_MODE && DEBUG_BLE
        if (isStringNotEmpty)
            Serial.println(F(" ⮐"));
#endif
    }
}

void BLE_HM10::sendCommand(const String command)
{
    Serial.println(command);
    delay(200); // wait some time
    while (Serial.available())
    {
        Serial.read();
    }
}

// ======================= GPS SECTION =====================
/*
   ____   ____    ____
  / ___| |  _ \  / ___|
 | |  _  | |_) | \___ \
 | |_| | |  __/   ___) |
  \____| |_|     |____/
*/

class GPS
{
    SoftwareSerial gpsPort;
public:
    GPS(uint8_t pinRx, uint8_t pinTx, long speed, INDICATION * indication);
    void setup();
    void updateLocation(DATA *dataPackage);

private:
    bool _debugMode;
    INDICATION * _indication;
    void printReadingIndication(unsigned long start, unsigned int delay);
    char _readingIndicator = 0;
    bool processRMCSentence(String sentence, DATA *dataPackage);
    bool processGGASentence(String sentence, DATA *dataPackage);
    float convertDegMinToDecDeg(float degMin);
};

GPS::GPS(uint8_t pinRx, uint8_t pinTx, long speed, INDICATION *indication) : gpsPort(pinRx, pinTx)
{
    gpsPort.begin(speed);
    _indication = indication;
}

void GPS::setup()
{
#if DEBUG_MODE && DEBUG_GPS
    Serial.println(F("📡[GPS: Start GPS configuration.]"));
#endif
}

void GPS::printReadingIndication(unsigned long start, unsigned int delay)
{
#if DEBUG_MODE && DEBUG_GPS
    byte progress = (10 * (millis() - start)) / delay;
    if (progress != _readingIndicator)
    {
        _readingIndicator = progress;
        Serial.print(F("⏳"));
    }
#endif
}

void GPS::updateLocation(DATA *dataPackage)
{
#if DEBUG_MODE && DEBUG_GPS
    Serial.print(F("📡[GPS reading: "));
#endif

    char c;
    String sentence = "";

    // For three seconds we parse GPS data and report some key values
    const unsigned int readingDelay = 3000;
    bool gpsReadingComplited = false;
    bool ggaCatched = false;
    bool rmcCatched = false;

    gpsPort.listen();

    for (unsigned long start = millis(); millis() - start < readingDelay;)
    {
        printReadingIndication(start, readingDelay);
        while (gpsPort.available() > 0)
        {
            c = gpsPort.read();
            if (c == '$')
            {
                // Start of a new sentence
                sentence = "";
            }
            else if (c == '\n')
            {
                // End of a sentence process it
                if (processRMCSentence(sentence, dataPackage))
                    rmcCatched = true;
                if (processGGASentence(sentence, dataPackage))
                    ggaCatched = true;

                if (ggaCatched && rmcCatched)
                {
                    gpsReadingComplited = true;
                    break;
                }
            }
            else
            {
                // Add character to the sentence
                sentence += c;
            }
        }
        if (gpsReadingComplited)
            break;
    }

    if (dataPackage->gpsValid)
    {
#if DEBUG_MODE && DEBUG_GPS
        Serial.println(" ✅ GPS data is correct.]");
#endif
        _indication->gps(GPSStatuses::correct); // GPS is valid
    }
    else
    {
#if DEBUG_MODE && DEBUG_GPS
        Serial.println(F("❌ No valid data.]"));
#endif
        _indication->gps(GPSStatuses::invalid); // GPS is invalid
    }
}

bool GPS::processRMCSentence(String sentence, DATA *dataPackage)
{
    if (sentence.startsWith("GPRMC"))
    {
#if DEBUG_MODE && DEBUG_GPS
        Serial.print("✨");
#endif

        // Process RMC sentence
        int commaIndex = sentence.indexOf(',');
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String status = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        // String latStr = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        // String latDir = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        // String lonStr = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        // String lonDir = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        // String speedKnots = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        // String course = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String dateStr = sentence.substring(0, commaIndex);

        if (status == "A")
        {
            // Convert date to year, month, and day
            dataPackage->day = dateStr.substring(0, 2).toInt();
            ;
            dataPackage->month = dateStr.substring(2, 4).toInt();
            dataPackage->year = dateStr.substring(4, 6).toInt();

            return true;
        }
    }

    return false;
}

bool GPS::processGGASentence(String sentence, DATA *dataPackage)
{
    if (sentence.startsWith("GPGGA"))
    {
#if DEBUG_MODE && DEBUG_GPS
        Serial.print("✨");
#endif

        // Process GGA sentence
        int commaIndex = sentence.indexOf(',');
        String timeStr = sentence.substring(commaIndex + 1, commaIndex + 7);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String latStr = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String latDir = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String lonStr = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String lonDir = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String fixQuality = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String satStr = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);

        commaIndex = sentence.indexOf(',');
        String hdopStr = sentence.substring(0, commaIndex);

        if (fixQuality.toInt() > 0)
        {
            dataPackage->gpsValid = true;
            dataPackage->lat = latStr.toFloat();
            dataPackage->lon = lonStr.toFloat();
            //dataPackage->sat = satStr.toInt();        // optional
            //dataPackage->hdop = hdopStr.toFloat();    // optional

            // Convert latitude and longitude to decimal degrees
            dataPackage->lat = convertDegMinToDecDeg(dataPackage->lat);
            dataPackage->lon = convertDegMinToDecDeg(dataPackage->lon);
            if (latDir == "S")
                dataPackage->lat = -dataPackage->lat;
            if (lonDir == "W")
                dataPackage->lon = -dataPackage->lon;

            // Convert time to hours, minutes, and seconds
            int hour = timeStr.substring(0, 2).toInt();
            int minute = timeStr.substring(2, 4).toInt();
            int second = timeStr.substring(4, 6).toInt();
            dataPackage->hour = hour;
            dataPackage->minute = minute;
            dataPackage->second = second;

            return true;
        }
    }

    return false;
}

float GPS::convertDegMinToDecDeg(float degMin)
{
    float min = fmod(degMin, 100.0);
    float deg = (degMin - min) / 100.0;
    return deg + (min / 60.0);
}

// ======================= LORA SECTION =====================
/*
  _   _      _      ____    _____     _        ___    ____       _
 | | | |    / \    |  _ \  |_   _|   | |      / _ \  |  _ \     / \
 | | | |   / _ \   | |_) |   | |     | |     | | | | | |_) |   / _ \
 | |_| |  / ___ \  |  _ <    | |     | |___  | |_| | |  _ <   / ___ \
  \___/  /_/   \_\ |_| \_\   |_|     |_____|  \___/  |_| \_\ /_/   \_\
*/

struct LORADATA
{
    DATA data;
    unsigned char ttl;   // time to live (for mesh network)
};

#ifdef EBYTE_E220
class LORA
{
    SoftwareSerial loraPort;
    LoRa_E220 e220ttl;
public:
    LORA(uint8_t pinRx, uint8_t pinTx, uint8_t aux, uint8_t m0, uint8_t m1, INDICATION * indication);
    void setup();
    void send(LORADATA *loraDataPackage);
    bool hasNewData(LORADATA *loraDataPackage);

private:
    bool _debugMode;
    uint8_t _ledPin;
    INDICATION * _indication;
    ResponseStructContainer rsc;
};

LORA::LORA(uint8_t pinRx, uint8_t pinTx, uint8_t aux, uint8_t m0, uint8_t m1, INDICATION *indication) : loraPort(pinRx, pinTx),
                                                                                                        e220ttl(&loraPort, aux, m0, m1)
{
    loraPort.begin(LORA_START_SPEED);
    _indication = indication;
}

void LORA::setup()
{
#if DEBUG_MODE && DEBUG_LORA
    Serial.println(F("🛜 [LORA: Start configuration]"));
#endif

    e220ttl.begin();
    e220ttl.resetModule();

    ResponseStructContainer c;
    c = e220ttl.getConfiguration();
    Configuration configuration = *(Configuration *)c.data;
    delay(100);

    configuration.ADDL = 0x00;
    configuration.ADDH = 0x00;
    configuration.CHAN = 0x17;
    configuration.SPED.uartBaudRate = UART_BPS_57600;
    configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
    configuration.SPED.uartParity = MODE_00_8N1;

    configuration.OPTION.subPacketSetting = SPS_200_00;
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
    configuration.OPTION.transmissionPower = POWER_22;

    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
    configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED; // monitoring before data transmitted
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

    e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    delay(100);
#if DEBUG_MODE && DEBUG_LORA
    Serial.print(F("\t🛜 [LORA current config: channel = "));
    Serial.print(configuration.getChannelDescription());
    Serial.print(F(" , airDataRate = "));
    Serial.print(configuration.SPED.getAirDataRateDescription());
    Serial.print(F(" , transmissionPower = "));
    Serial.print(configuration.OPTION.getTransmissionPowerDescription());
    Serial.println(F("]"));
#endif

    loraPort.end();
    loraPort.begin(57600);
}

void LORA::send(LORADATA *loraDataPacket)
{
    loraPort.listen();
    const byte LORADATASIZE = sizeof(LORADATA);

#if DEBUG_MODE && DEBUG_LORA
    Serial.print(F("🛜 [LoRa: Sending 📫, "));
    Serial.print(LORADATASIZE);
    Serial.print(F(" bytes are ready to send"));
    Serial.print(F(" ➜ "));
    Serial.print(loraDataPacket->data.name);
    Serial.print(F(" / "));
    Serial.print(loraDataPacket->data.lat, 6);
    Serial.print(F(" "));
    Serial.print(loraDataPacket->data.lon, 6);
    Serial.print(F(" / "));
    Serial.print(loraDataPacket->data.year);
    Serial.print(F("-"));
    if (loraDataPacket->data.month < 10)
        Serial.print(F("0"));
    Serial.print(loraDataPacket->data.month);
    Serial.print(F("-"));
    if (loraDataPacket->data.day < 10)
        Serial.print(F("0"));
    Serial.print(loraDataPacket->data.day);
    Serial.print(F(" "));
    Serial.print(loraDataPacket->data.hour);
    Serial.print(F(":"));
    if (loraDataPacket->data.minute < 10)
        Serial.print(F("0"));
    Serial.print(loraDataPacket->data.minute);
    Serial.print(F(" (UTC)"));
    Serial.print(F(" TTL="));
    Serial.print(loraDataPacket->ttl);
    Serial.print(F("] ➜ "));

#endif

    ResponseStatus rs = e220ttl.sendMessage(loraDataPacket, LORADATASIZE);

#if DEBUG_MODE && DEBUG_LORA
    Serial.print(F("[Status: "));
    Serial.print(rs.getResponseDescription());
#endif

    if (rs.code == 1)
    {
#if DEBUG_MODE && DEBUG_LORA
        Serial.print(F(" 🆗"));
#endif
        _indication->lora(LoRaStatuses::dataTransmitted);
    }
    else
    {
#if DEBUG_MODE && DEBUG_LORA
        Serial.print(F(" 🚨"));
#endif
        _indication->lora(LoRaStatuses::error);
    }

#if DEBUG_MODE && DEBUG_LORA
    Serial.println(F("]"));
    Serial.println();
#endif
}

bool LORA::hasNewData(LORADATA *loraDataPacket)
{
    if (e220ttl.available() > 1)
    {
#if DEBUG_MODE && DEBUG_LORA
        Serial.println(F("🛜 [LORA: we have new data 🥳]"));
#endif

        rsc = e220ttl.receiveMessage(sizeof(LORADATA));
        if (rsc.status.code != 1)
        {
#if DEBUG_MODE && DEBUG_LORA
            Serial.println(F("🛜 [LORA error: ❌ status - "));
            Serial.println(rsc.status.getResponseDescription());
            Serial.println(F("]"));
#endif
            _indication->lora(LoRaStatuses::error);
            return false;
        }
        else
        {
            memcpy(loraDataPacket, (LORADATA *)rsc.data, sizeof(LORADATA));
            rsc.close();
        }
        _indication->lora(LoRaStatuses::dataReceived);
        return true;
    }
    return false;
}
#endif

#ifdef EBYTE_E32
class LORA
{
    SoftwareSerial loraPort;
    LoRa_E32 e32ttl;
public:
    LORA(uint8_t pinRx, uint8_t pinTx, uint8_t aux, uint8_t m0, uint8_t m1, INDICATION * indication);
    void setup();
    void send(LORADATA *loraDataPackage);
    bool hasNewData(LORADATA *loraDataPackage);

private:
    bool _debugMode;
    uint8_t _ledPin;
    INDICATION * _indication;
    ResponseStructContainer rsc;
};

LORA::LORA(uint8_t pinRx, uint8_t pinTx, uint8_t aux, uint8_t m0, uint8_t m1, INDICATION *indication) : loraPort(pinRx, pinTx),
                                                                                                        e32ttl(&loraPort, aux, m0, m1)
{
    loraPort.begin(LORA_START_SPEED);
    _indication = indication;
}

void LORA::setup()
{
#if DEBUG_MODE && DEBUG_LORA
    Serial.println(F("🛜 [LORA: Start configuration]"));
#endif

    e32ttl.begin();
    e32ttl.resetModule();

    ResponseStructContainer c;
    c = e32ttl.getConfiguration();
    Configuration configuration = *(Configuration *)c.data;
    delay(100);

    configuration.ADDL = 0x0;
    configuration.ADDH = 0x1;
    configuration.CHAN = 0x17;                                             // Channel. (410 + CHAN*1MHz) MHz. Default 17H (433MHz)
    configuration.OPTION.fec = FEC_1_ON;                                   // FEC_0_OFF / FEC_1_ON (default)  - Forward Error Correction Switch
    configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;  // FT_TRANSPARENT_TRANSMISSION (default) / FT_FIXED_TRANSMISSION
    configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;      // IO_D_MODE_OPEN_COLLECTOR / IO_D_MODE_PUSH_PULLS_PULL_UPS
    configuration.OPTION.transmissionPower = POWER_20;                   // 21/24/27/30 dBm if define E32_TTL_1W
    configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;                 // 250 (default)/500/750/1000/1250/1500/1750/2000
    configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;                 // AIR_DATA_RATE_000_03 - 0.3kbps
                                                                           // AIR_DATA_RATE_001_12 - 1.2kbps
                                                                           // AIR_DATA_RATE_010_24 - 2.4kbps (default)
                                                                           // AIR_DATA_RATE_011_48 - 4.8kbps
                                                                           // AIR_DATA_RATE_100_96 - 9.6kbps
                                                                           // AIR_DATA_RATE_101_192 - 19.2kbps
                                                                           // AIR_DATA_RATE_110_192 - 19.2kbps (same 101)
                                                                           // AIR_DATA_RATE_111_192 - 19.2kbps (same 101)
    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.uartParity = MODE_00_8N1;

    e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    delay(100);
#if DEBUG_MODE && DEBUG_LORA
    Serial.print(F("\t🛜 [LORA current config: channel = "));
    Serial.print(configuration.getChannelDescription());

    Serial.print(F(" , airDataRate = "));
    Serial.print(configuration.SPED.getAirDataRate());

    Serial.print(F(" , transmissionPower = "));
    Serial.print(configuration.OPTION.getTransmissionPowerDescription());
    Serial.println(F("]"));
#endif

    loraPort.end();
    loraPort.begin(57600);
}

void LORA::send(LORADATA *loraDataPacket)
{
    loraPort.listen();
    const byte LORADATASIZE = sizeof(LORADATA);

#if DEBUG_MODE && DEBUG_LORA
    Serial.print(F("🛜 [LoRa: Sending 📫, "));
    Serial.print(LORADATASIZE);
    Serial.print(F(" bytes are ready to send"));
    Serial.print(F(" ➜ "));
    Serial.print(loraDataPacket->data.name);
    Serial.print(F(" / "));
    Serial.print(loraDataPacket->data.lat, 6);
    Serial.print(F(" "));
    Serial.print(loraDataPacket->data.lon, 6);
    Serial.print(F(" / "));
    Serial.print(loraDataPacket->data.year);
    Serial.print(F("-"));
    if (loraDataPacket->data.month < 10)
        Serial.print(F("0"));
    Serial.print(loraDataPacket->data.month);
    Serial.print(F("-"));
    if (loraDataPacket->data.day < 10)
        Serial.print(F("0"));
    Serial.print(loraDataPacket->data.day);
    Serial.print(F(" "));
    Serial.print(loraDataPacket->data.hour);
    Serial.print(F(":"));
    if (loraDataPacket->data.minute < 10)
        Serial.print(F("0"));
    Serial.print(loraDataPacket->data.minute);
    Serial.print(F(" (UTC)"));
    Serial.print(F(" TTL="));
    Serial.print(loraDataPacket->ttl);
    Serial.print(F("] ➜ "));
#endif

    ResponseStatus rs = e32ttl.sendMessage(loraDataPacket, LORADATASIZE);

#if DEBUG_MODE && DEBUG_LORA
    Serial.print(F("[Status: "));
    Serial.print(rs.getResponseDescription());
#endif

    if (rs.code == 1)
    {
#if DEBUG_MODE && DEBUG_LORA
        Serial.print(F(" 🆗"));
#endif
        _indication->lora(LoRaStatuses::dataTransmitted);
    }
    else
    {
#if DEBUG_MODE && DEBUG_LORA
        Serial.print(F(" 🚨"));
#endif
        _indication->lora(LoRaStatuses::error);
    }

#if DEBUG_MODE && DEBUG_LORA
    Serial.println(F("]"));
    Serial.println();
#endif
}

bool LORA::hasNewData(LORADATA *loraDataPacket)
{
    if (e32ttl.available() > 1)
    {
#if DEBUG_MODE && DEBUG_LORA
        Serial.println(F("🛜 [LORA: we have new data 🥳]"));
#endif

        rsc = e32ttl.receiveMessage(sizeof(LORADATA));
        if (rsc.status.code != 1)
        {
#if DEBUG_MODE && DEBUG_LORA
            Serial.println(F("🛜 [LORA error: ❌ status - "));
            Serial.println(rsc.status.getResponseDescription());
            Serial.println(F("]"));
#endif
            _indication->lora(LoRaStatuses::error);
            return false;
        }
        else
        {
            memcpy(loraDataPacket, (LORADATA *)rsc.data, sizeof(LORADATA));
            rsc.close();
        }
        _indication->lora(LoRaStatuses::dataReceived);
        return true;
    }
    return false;
}
#endif

// ================= TESTS SECTION =================
/*
 _____   _____   ____    _____   ____
 |_   _| | ____| / ___|  |_   _| / ___|
   | |   |  _|   \___ \    | |   \___ \
   | |   | |___   ___) |   | |    ___) |
   |_|   |_____| |____/    |_|   |____/
*/

class TESTS
{
public:
    bool hasNewDataEveryXSec(LORADATA *loraDataPacket, GPS *gps, byte interval);

private:
    unsigned long _timeOfLastSendedPacket = 0;
};

bool TESTS::hasNewDataEveryXSec(LORADATA *loraDataPacket, GPS *gps, byte interval)
{
        int _intervarSec = interval * 1000;
        if (_timeOfLastSendedPacket + _intervarSec < millis())
        {
#if DEBUG_MODE
                Serial.println();
                Serial.println(F("💛💛💛 [TEST: virtual tracker transmitted the data] 💛💛💛"));
#endif

                gps->updateLocation(&loraDataPacket->data); // we need an actual time
                strcpy(loraDataPacket->data.name, OTHER_NAME);
                loraDataPacket->data.lat = 45.455631;
                loraDataPacket->data.lon = 54.084960;
                loraDataPacket->ttl = TTL;

#if DEBUG_MODE
                Serial.println();
#endif

                _timeOfLastSendedPacket = millis();
                return true;
        }

        return false;
}

/*

  _ __ ___     ___   _ __ ___     ___    _ __   _   _
 | '_ ` _ \   / _ \ | '_ ` _ \   / _ \  | '__| | | | |
 | | | | | | |  __/ | | | | | | | (_) | | |    | |_| |
 |_| |_| |_|  \___| |_| |_| |_|  \___/  |_|     \__, |
                                                |___/
*/

class IMemory {         // interface
    public:
        virtual void setup() = 0;

        virtual void clearAllPositions() = 0;
        virtual bool checkUnique(DATA *newPoint) = 0;
        virtual unsigned int save(DATA *newData) = 0;
        virtual DATA * load(unsigned int index) = 0;

        virtual bool checkCRC() = 0;    // all memory
        virtual bool checkCRC(unsigned int index) = 0;

        virtual unsigned int getSize() = 0;
        virtual unsigned int getIndex() = 0;
        virtual bool getStorageOverwrite() = 0;
};

struct EEPROMDATA {
     unsigned char counter;
     DATA data;
     unsigned char crc;
};

class EEPROMAglora : public IMemory
{
public:
    EEPROMAglora();
    void setup();
    bool checkUnique(DATA *newPoint);
    unsigned int save(DATA *newData);
    DATA *load(unsigned int index);
    void clearAllPositions();
    bool checkCRC();
    bool checkCRC(unsigned int index);
    unsigned int getSize();
    unsigned int getIndex();
    bool getStorageOverwrite();

private:
    EEPROMDATA eepromdata;
    unsigned int EEPROMStorageIndex = 0;  // index in memory (address = EEPROMStorageIndex * EEPROMDataSize)
    unsigned int incrementCounter = 0;             // min 0, max 254 (because default EEPROM is 255)

    unsigned int EEPROMStorageSize;
    byte dataSize;
    bool storageOverwrite = false;

    const unsigned char EEPROMDataSize = sizeof(EEPROMDATA);
};

EEPROMAglora::EEPROMAglora()
{
    dataSize = sizeof(DATA);
    EEPROMStorageSize = (EEPROM.length() - EEPROM_BEGIN_ADDRESS) / EEPROMDataSize;
}

void EEPROMAglora::setup()
{
    EEPROMStorageIndex = 0;
    incrementCounter = 0;
    storageOverwrite = false;

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("📀[EEPROM storage: Start EEPROM initialization. Size of memory: "));
    Serial.print(EEPROM.length());
    Serial.println(F(" bytes]"));
    Serial.print(F("📀[EEPROM storage: "));
    Serial.print(EEPROMStorageSize);
    Serial.println(F(" track points can be saved.]"));
    Serial.println(F("\tCRC check symbols: ✅ CRC OK, ⛔️ CRC ERROR, ⬜ empty memory cell, 👉 - current cell"));
    Serial.println(F("\tFormat: {COUNTER}➜ {CRC} {CRC check symbol}, 255 - default EEPROM value "));
    Serial.print(F("\tFinding the index of the last record. Read memory... "));
#endif

    unsigned int prevIncCounter = 0;

    for (unsigned int i = 0; i < EEPROMStorageSize; i++)
    {
        EEPROM.get(i * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, eepromdata);

#if DEBUG_MODE && DEBUG_MEMORY
        Serial.print(F("."));
        if ((i + 1) % 50 == 0)
            Serial.println();
#endif

        if (eepromdata.counter == 255)
        { // check empty EEPROM cell
            if (i == 0)
            {
                break;
            }
            EEPROMStorageIndex = i;
            incrementCounter = prevIncCounter + 1;
            if (incrementCounter >= 255)
            {
                incrementCounter = 0;
            }
            break;
        }

        if (abs(eepromdata.counter - prevIncCounter) > 1)
        { // not in sequence
            if (i != 0)
            {
                if (prevIncCounter != 254)
                { // exclude the option ...252,253,254,0,1,2...
                    EEPROMStorageIndex = i;
                    incrementCounter = prevIncCounter + 1;
                    if (incrementCounter >= 255)
                    {
                        incrementCounter = 0;
                    }
                    if (eepromdata.counter != 255)
                    {
                        storageOverwrite = true;
                    }
                    break;
                }
            }
        }
        prevIncCounter = eepromdata.counter;
    }

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println();
    Serial.print(F("\tNext record will be "));
    Serial.print(EEPROMStorageIndex + 1);
    Serial.print(F(",\tstorageOverwrite = "));
    Serial.println(storageOverwrite);
#endif
}

bool EEPROMAglora::checkUnique(DATA *newPoint)
{
    if (strcmp(newPoint->name, NAME) == 0)
    {
#if DEBUG_MODE && DEBUG_MEMORY
        Serial.println(F("📀[EEPROM storage: returned package 🔄 ]"));
#endif
        return false;
    }

    for (unsigned int i = 0; i < EEPROMStorageSize; i++)
    {
        DATA *eepromPoint = load(i);
        if ((newPoint->name == eepromPoint->name) &&
            (newPoint->year == eepromPoint->year) &&
            (newPoint->month == eepromPoint->month) &&
            (newPoint->day == eepromPoint->day) &&
            (newPoint->hour == eepromPoint->hour) &&
            (newPoint->minute == eepromPoint->minute) &&
            (newPoint->second == eepromPoint->second))
        {
#if DEBUG_MODE && DEBUG_MEMORY
            Serial.println(F("📀[EEPROM storage: data already exist‼️⛔️]"));
#endif
            return false;
        }
    }
#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println(F("📀[EEPROM storage: new data checked ✅]"));
#endif
    return true;
}

/// @brief Save data to storage
/// @param newData that needs to be added to the storage
/// @return the index of added data
unsigned int EEPROMAglora::save(DATA *newData)
{
    eepromdata.counter = incrementCounter;
    eepromdata.data = *newData;
    eepromdata.crc = calculateCRC((unsigned char *)newData, sizeof(DATA));
    EEPROM.put(EEPROMStorageIndex * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, eepromdata);

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("📀[EEPROM storage: New data added. Address: "));
    Serial.print(EEPROMStorageIndex * EEPROMDataSize + EEPROM_BEGIN_ADDRESS);
    Serial.print(F(", index: "));
    Serial.print(EEPROMStorageIndex);
    Serial.print(F(", counter: "));
    Serial.print(eepromdata.counter);
    Serial.print(F(", CRC: "));
    Serial.print(eepromdata.crc);
    Serial.println(F("]"));
#endif

    EEPROMStorageIndex++;
    if (EEPROMStorageIndex >= EEPROMStorageSize)
    {
        EEPROMStorageIndex = 0;
    }

    incrementCounter++;
    if (incrementCounter >= 255)
    {
        incrementCounter = 0;
    }

    return incrementCounter;
}

DATA *EEPROMAglora::load(unsigned int index)
{
    EEPROM.get(index * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, eepromdata);
    return &eepromdata.data;
}

void EEPROMAglora::clearAllPositions()
{
#if DEBUG_MODE && DEBUG_MEMORY
    const byte rowLength = 120;
    int cellsCounter = 0;
    Serial.println(F("📀[EEPROM storage: clearing memory.]"));
    Serial.print(F("\t"));
#endif

    unsigned char memoryCell = 0;
    for (unsigned int i = 0; i < EEPROM.length(); i++)
    {
        EEPROM.get(i + EEPROM_BEGIN_ADDRESS, memoryCell);
        if (memoryCell != 255)
        { // 255 - default value in EEPROM
            memoryCell = 255;
            EEPROM.put(i + EEPROM_BEGIN_ADDRESS, memoryCell);
#if DEBUG_MODE && DEBUG_MEMORY
            Serial.print(F("#"));
            if (++cellsCounter % rowLength == 0)
            {
                Serial.println();
                Serial.print(F("\t"));
            }
#endif
        }
    }
    EEPROMStorageIndex = 0;
    incrementCounter = 0;
    storageOverwrite = false;

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println();
#endif
}

bool EEPROMAglora::checkCRC()
{
    const byte rowLength = 12; // how many characters will be printed in a row
    const byte rowDivider = 4; // split string for better view
    bool result = true;

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("📀[EEPROM storage: checking CRC, "));
    Serial.print(EEPROMStorageIndex);
    Serial.print(F("/"));
    Serial.print(EEPROMStorageSize);
    Serial.print(F(" cells are used, storageOverwrite is "));
    Serial.print(storageOverwrite);
    Serial.println(F("]"));
#endif

#if DEBUG_MODE && DEBUG_MEMORY // Memory visualisation
    Serial.print(F("\t "));
    for (unsigned int i = 0; i < rowLength; ++i)
    {
        Serial.print(F("🔻"));
        if (i < 10)
            Serial.print(F("0"));
        Serial.print(i + 1);
        Serial.print(F("⎻⎻⎻⎻⎻⎻⎻"));

        if ((i + 1) % rowDivider == 0)
            Serial.print(F("   "));
    }
    Serial.println();
    Serial.print(F("\t"));
#endif

    for (unsigned int i = 0; i < EEPROMStorageSize; i++)
    {
        EEPROM.get(i * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, eepromdata);

        unsigned char crc = calculateCRC((unsigned char *)&eepromdata.data, sizeof(eepromdata.data));

#if DEBUG_MODE && DEBUG_MEMORY
        if (eepromdata.counter < 100)
            Serial.print(F(" "));
        if (eepromdata.counter < 10)
            Serial.print(F(" "));

        Serial.print(F(" "));
        Serial.print(eepromdata.counter);

        if (i == EEPROMStorageIndex - 1)
        {
            Serial.print(F("👉"));
        }
        else
        {
            Serial.print(F("➜ "));
        }

        if (eepromdata.crc < 100)
            Serial.print(F("0"));
        if (eepromdata.crc < 10)
            Serial.print(F("0"));

        Serial.print(eepromdata.crc);
#endif

        if (eepromdata.counter != 255)
        {
            if (crc == eepromdata.crc)
            {
#if DEBUG_MODE && DEBUG_MEMORY
                Serial.print(F("✅"));
#endif
            }
            else
            {
#if DEBUG_MODE && DEBUG_MEMORY
                Serial.print(F("⛔️"));
#endif
                result = false;
            }
        }
        else
        {
#if DEBUG_MODE && DEBUG_MEMORY
            Serial.print(F("⬜"));
#endif
        }

#if DEBUG_MODE && DEBUG_MEMORY // Memory visualisation
        if ((i + 1) % rowLength == 0)
        {
            Serial.println();
            Serial.println();
            Serial.print(F("\t"));
        }
        else
        {
            if ((i + 1) % rowDivider == 0)
                Serial.print(F(" · "));
        }
#endif
    }

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println();
#endif

    return result;
}

bool EEPROMAglora::checkCRC(unsigned int index)
{
    EEPROM.get(index * EEPROMDataSize + EEPROM_BEGIN_ADDRESS, eepromdata);
    const byte crc = calculateCRC((unsigned char *)&eepromdata.data, dataSize);
    if (eepromdata.crc == crc)
        return true;
    return false;
}

unsigned int EEPROMAglora::getSize()
{
    return EEPROMStorageSize;
}

unsigned int EEPROMAglora::getIndex()
{
    return EEPROMStorageIndex;
}

bool EEPROMAglora::getStorageOverwrite()
{
    return storageOverwrite;
}

struct SRAMDATA
{
    DATA data;
    unsigned char crc;
};

class SRAM: public IMemory
{
public:
    SRAM();
    void setup();
    bool checkUnique(DATA *newPoint);
    unsigned int save(DATA *newData); //function returns the index
    DATA * load(unsigned int index);
    void clearAllPositions();
    bool checkCRC();
    bool checkCRC(unsigned int index);
    unsigned int getSize();
    unsigned int getIndex();
    bool getStorageOverwrite();

private:
    SRAMDATA storage[SRAM_STORAGE_SIZE];
    unsigned int storageIndex = 0;
    bool storageOverwrite = false;
    byte dataSize;
    bool checkCRC(SRAMDATA *point);
};

SRAM::SRAM()
{
    dataSize = sizeof(DATA);
}

void SRAM::setup()
{
#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("💾[SRAM storage: memory is ready. SRAM_STORAGE_SIZE="));
    Serial.print(SRAM_STORAGE_SIZE + 1);
    Serial.print(F(" ("));
    Serial.print((SRAM_STORAGE_SIZE + 1) * sizeof(DATA));
    Serial.print(F(" bytes)"));
    Serial.println(F("]"));
    Serial.println(F("\t CRC check symbols: ✅ CRC OK, ⛔️ CRC ERROR, ⬜ empty memory cell, underlined CRC\u0332 - current cell"));
#endif
}

/// @brief Checking new data for uniqueness
/// @param loraDataPacket
/// @return true if the same data is not exist
bool SRAM::checkUnique(DATA *newPoint)
{
#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("💾[SRAM storage: checking data for uniqueness 📦 ↔ 📦 ↔ 📦. First check, is <"));
    Serial.print(newPoint->name);
    Serial.print(F("> equal to <"));
    Serial.print(NAME);
    Serial.println(F(">]"));
#endif

    if (strcmp(newPoint->name, NAME) == 0)
    {
#if DEBUG_MODE && DEBUG_MEMORY
        Serial.print(F("💾[SRAM storage: returned package 🔄, because "));
        Serial.print(newPoint->name);
        Serial.println(F(" it's me!]"));
#endif
        return false;
    }

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println(F("💾[SRAM storage: checking data for uniqueness 📦 ↔ 📦 ↔ 📦. Second check, find the coordinates and time duplication "));
    Serial.print(F("\t"));
#endif

    const unsigned int maxIndex = storageOverwrite ? SRAM_STORAGE_SIZE : storageIndex;
    for (unsigned int i = 0; i < maxIndex; ++i)
    {
#if DEBUG_MODE && DEBUG_MEMORY
        Serial.print(F("#"));
#endif

        if ((strcmp(newPoint->name, storage[i].data.name) == 0) &&
            (newPoint->year == storage[i].data.year) &&
            (newPoint->month == storage[i].data.month) &&
            (newPoint->day == storage[i].data.day) &&
            (newPoint->hour == storage[i].data.hour) &&
            (newPoint->minute == storage[i].data.minute) &&
            (newPoint->second == storage[i].data.second))
        {
#if DEBUG_MODE && DEBUG_MEMORY
            Serial.println();
            Serial.println(F("💾[SRAM storage: data already exist‼️⛔️]"));
#endif
            return false;
        }
    }
#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println();
    Serial.println(F("💾[SRAM storage: new data ✅]"));
#endif
    return true;
}

/// @brief Save data to storage
/// @param newData data that needs to be added to the storage
/// @return the index of added data
unsigned int SRAM::save(DATA *newData)
{
    storage[storageIndex].data = *newData;
    storage[storageIndex].crc = calculateCRC((unsigned char *)newData, dataSize);

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("💾[SRAM storage saving: data from "));
    Serial.print(storage[storageIndex].data.name);
    Serial.print(F(" was added. Memory: "));
    Serial.print(storageIndex + 1);
    Serial.print(F("/"));
    Serial.print(SRAM_STORAGE_SIZE + 1);
    Serial.print(F(", CRC "));
    Serial.print(storage[storageIndex].crc);
    Serial.println(F(" ✅]"));
#endif

    unsigned int addedIindex = storageIndex;
    storageIndex++;
    if (storageIndex >= SRAM_STORAGE_SIZE)
    {
        storageIndex = 0;
        storageOverwrite = true;
    }

    return addedIindex;
}

/// @brief Loading data from memory to loraDataPacket by index
/// @param loraDataPacket pointer
/// @param index index of data in memory
/// @return true if success
DATA *SRAM::load(unsigned int index)
{
    return &storage[index].data;
}

void SRAM::clearAllPositions()
{
#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println(F("💾[SRAM storage: clearing memory 🫙]"));
#endif
    storageIndex = 0;
    storageOverwrite = false;
}

bool SRAM::checkCRC()
{
#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("💾[SRAM storage: checking CRC, "));
    Serial.print(storageIndex);
    Serial.print(F("/"));
    Serial.print(SRAM_STORAGE_SIZE);
    Serial.print(F(" cells are used, storageOverwrite is "));
    Serial.print(storageOverwrite);
    Serial.println(F("]"));
#endif

    const byte rowLength = 12; // how many characters will be printed in a row
    const byte rowDivider = 4; // split string for better view
    bool result = true;
    byte crc = 0;

    if ((storageIndex == 0) && (!storageOverwrite))
    {
#if DEBUG_MODE && DEBUG_MEMORY
        Serial.println(F("💾[SRAM storage: memory is empty]"));
#endif
        return result;
    }

    const unsigned int maxIndex = storageOverwrite ? (SRAM_STORAGE_SIZE - 1) : (storageIndex - 1);

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.print(F("\t"));
#endif

    for (unsigned int i = 0; i < SRAM_STORAGE_SIZE; ++i)
    {
        if (i <= maxIndex)
        {
            crc = calculateCRC((unsigned char *)&storage[i], dataSize);
            if (storage[i].crc == crc)
            {
#if DEBUG_MODE && DEBUG_MEMORY
                Serial.print(F(" ✅"));
#endif
            }
            else
            {
#if DEBUG_MODE && DEBUG_MEMORY
                Serial.print(F(" ⛔️"));
#endif
                result = false;
            }
#if DEBUG_MODE && DEBUG_MEMORY
            if (crc < 100)
                Serial.print(F("0"));
            if (crc < 10)
                Serial.print(F("0"));
            Serial.print(crc);
            if ((i == storageIndex - 1) ||
                ((i == 0) && (storageOverwrite)))
            {
                Serial.print(F("\u0332")); // underline active memory cell
            }
#endif
        }
        else
        {
#if DEBUG_MODE && DEBUG_MEMORY
            Serial.print(F(" ⬜"));
            Serial.print(F("   "));
#endif
        }

#if DEBUG_MODE && DEBUG_MEMORY // Memory visualisation
        if ((i + 1) % rowLength == 0)
        {
            Serial.println();
            Serial.println();
            Serial.print(F("\t"));
        }
        else
        {
            if ((i + 1) % rowDivider == 0)
                Serial.print(F(" · "));
        }
#endif
    }

#if DEBUG_MODE && DEBUG_MEMORY
    Serial.println();
#endif

    return result;
}

bool SRAM::checkCRC(SRAMDATA *point)
{
    const byte crc = calculateCRC((unsigned char *)point, dataSize);
    if (point->crc == crc)
        return true;
    return false;
}

bool SRAM::checkCRC(unsigned int index)
{
    const byte crc = calculateCRC((unsigned char *)&storage[index], dataSize);
    if (storage[index].crc == crc)
        return true;
    return false;
}

unsigned int SRAM::getSize()
{
    return SRAM_STORAGE_SIZE;
}

unsigned int SRAM::getIndex()
{
    return storageIndex;
}

bool SRAM::getStorageOverwrite()
{
    return storageOverwrite;
}

/*****************************************************
     _       ____   _        ___    ____       _
    / \     / ___| | |      / _ \  |  _ \     / \
   / _ \   | |  _  | |     | | | | | |_) |   / _ \
  / ___ \  | |_| | | |___  | |_| | |  _ <   / ___ \
 /_/   \_\  \____| |_____|  \___/  |_| \_\ /_/   \_\
******************************************************/

class AGLORA
{
public:
  AGLORA(IMemory * memory, BLE_HM10 * ble);
  void hello();
  void checkMemoryToBLE();
  void clearDataPacket(DATA * trackerData);
  void updateSensors(DATA * trackerData);
  void printPackage(LORADATA * loraDataPacket);
  void getRequest(String request);
  void sendPackageToBLE(DATA * trackerData, int index);

private:
  IMemory * _memory;
  BLE_HM10 * _ble;
  void sendAllPackagesToBLE();
  void sendLastPackagesToBLE();
  void sendPackageToBLEFromStorage(unsigned int index);
  bool isDataMoreRecent(DATA * newData, DATA * oldData);
};

void AGLORA::updateSensors(DATA *loraDataPacket)
{
    loraDataPacket->battery = 100; // just for example

#if DEBUG_MODE && DEBUG_AGLORA
    Serial.print(F("🟢[AGLoRa sensors: "));
    Serial.print(F("🔋 - "));
    Serial.print(loraDataPacket->battery);
    Serial.println(F("]"));
#endif
}

const String bleProtocolPrefix = "AGLoRa-";
const String bleProtocolTypePoint = "point";
const String bleProtocolTypeMemory = "memory";
const String bleProtocolVersion = "&ver=2.2";
const String bleProtocolParamCRC = "&crc=";
const String bleProtocolOK = "ok";
const String bleProtocolError = "error";

const String bleProtocolParamMemorySize = "&memsize=";
const String bleProtocolParamMemoryOverwrite = "&overwrite=";
const String bleProtocolParamMemoryIndex = "&index=";

const String bleProtocolDeviceName = "&dev_name=" + String(NAME);

const String bleProtocolDivider = "\r\n";

AGLORA::AGLORA(IMemory *memory, BLE_HM10 *ble)
{
  _ble = ble;
  _memory = memory;
}

void AGLORA::hello()
{
#if DEBUG_MODE && DEBUG_AGLORA
  Serial.println(F("[power on]"));

  Serial.print(F("Waiting | "));
  for (int i = 0; i < 50; i++)
  {
    Serial.print(F("#"));
    delay(50);
  }
  Serial.println();
  Serial.println(F("AGLORA tracker started..."));
#endif
}

/// @brief
/// 1. clear
/// 2. set name
/// 3. set default ttl
/// @param loraDataPacket
void AGLORA::clearDataPacket(DATA *trackerData)
{
  memset(trackerData, 0, sizeof(&trackerData));
  strcpy(trackerData->name, NAME);
#if DEBUG_MODE && DEBUG_AGLORA
  Serial.println(F("🟢[AGLoRa: time to send your location📍, new loraDataPacket prepared 📦]"));
#endif
}

void AGLORA::printPackage(LORADATA *loraDataPacket)
{
  // DEBUG_MODE
#if DEBUG_MODE && DEBUG_AGLORA // dump out what was just received
  Serial.println(F("🟢[AGLoRa: loraDataPacket now contains ↴]"));
  Serial.print(F("     Name: "));
  Serial.print(loraDataPacket->data.name);
  Serial.print(F(", 🌎 lat: "));
  Serial.print(loraDataPacket->data.lat, 6);
  Serial.print(F(", lon: "));
  Serial.print(loraDataPacket->data.lon, 6);

  if (loraDataPacket->data.gpsValid)
    Serial.print(F(", GPS 🆗"));
  else
    Serial.print(F(", GPS ❌"));

  // Serial.print(F(", sat: "));
  // Serial.print(loraDataPacket->data.sat);   // example
  // Serial.print(F(", hdop: "));
  // Serial.print(loraDataPacket->data.hdop);  // example

  Serial.print(F(", 📆 date: "));
  Serial.print(loraDataPacket->data.year);
  Serial.print(F("/"));
  if (loraDataPacket->data.month < 10)
    Serial.print(F("0"));
  Serial.print(loraDataPacket->data.month);
  Serial.print(F("/"));
  if (loraDataPacket->data.day < 10)
    Serial.print(F("0"));
  Serial.print(loraDataPacket->data.day);

  Serial.print(F(", 🕰️ time: "));
  Serial.print(loraDataPacket->data.hour);
  Serial.print(F(":"));
  if (loraDataPacket->data.minute < 10)
    Serial.print(F("0"));
  Serial.print(loraDataPacket->data.minute);
  Serial.print(F(":"));
  if (loraDataPacket->data.second < 10)
    Serial.print(F("0"));
  Serial.print(loraDataPacket->data.second);
  Serial.print(F(" (UTC)"));

  Serial.print(F(" 📦 TTL="));
  Serial.print(loraDataPacket->ttl);

  Serial.println();
#endif
}

void AGLORA::getRequest(String request)
{
  if (request.length() == 0)
  {
    return;
  }

#if DEBUG_MODE && DEBUG_AGLORA
  Serial.println();
  Serial.print(F("🟢[AGLoRa: 📭 BLE request received <<"));
  Serial.print(request);
  Serial.println(F(">> received]"));
  Serial.println();
#endif

  if (request.startsWith(F("crc")))
  {
    checkMemoryToBLE();
    return;
  }

  if (request.startsWith(F("clear")))
  {
    _memory->clearAllPositions();
    checkMemoryToBLE();
    return;
  }

  if (request.startsWith(F("all")))
  {
    checkMemoryToBLE();
    sendAllPackagesToBLE();
    return;
  }

  if (request.startsWith(F("id")))
  {
    request.remove(0, 2);
    unsigned int index = request.toInt();
    sendPackageToBLEFromStorage(index);

    return;
  }
}

void AGLORA::checkMemoryToBLE()
{
  String response = bleProtocolPrefix +
                    bleProtocolTypeMemory +
                    bleProtocolVersion;
  response += bleProtocolParamCRC;
  response += _memory->checkCRC() ? bleProtocolOK : bleProtocolError;
  response += bleProtocolDeviceName;
  response += sendBatteryToPhone();
  response += bleProtocolParamMemorySize + _memory->getSize();
  response += bleProtocolParamMemoryIndex + _memory->getIndex();
  response += bleProtocolParamMemoryOverwrite + _memory->getStorageOverwrite();
  response += bleProtocolDivider;
  _ble->send(&response);
}

void AGLORA::sendPackageToBLE(DATA *trackerData, int index)
{
  String response = bleProtocolPrefix +
                    bleProtocolTypePoint +
                    bleProtocolVersion;

  response += sendToPhone(trackerData);
  response += sendBatteryToPhone();
  response += bleProtocolParamMemoryIndex + index;
  response += bleProtocolParamCRC;
  response += _memory->checkCRC(index) ? bleProtocolOK : bleProtocolError;
  response += bleProtocolDivider;

#if DEBUG_MODE && DEBUG_AGLORA
  Serial.print(F("🟢AGLoRa: send point 📦 to BLE → "));
  Serial.print(response);
#endif

  _ble->send(&response);
}

void AGLORA::sendAllPackagesToBLE()
{
  unsigned int maxIndex = _memory->getStorageOverwrite() ? _memory->getSize() : _memory->getIndex();
  for (unsigned int i = 0; i < maxIndex; ++i)
  {
#if DEBUG_MODE && DEBUG_AGLORA
    Serial.print(F("🟢[AGLoRa: loading "));
    Serial.print(i + 1);
    Serial.print(F("/"));
    Serial.print(maxIndex);
    Serial.print(F(" 📦 from memory ]"));
#endif

    sendPackageToBLE(_memory->load(i), i);
  }

#if DEBUG_MODE && DEBUG_AGLORA
  Serial.println();
#endif
}

void AGLORA::sendPackageToBLEFromStorage(unsigned int index)
{
#if DEBUG_MODE && DEBUG_AGLORA
  Serial.print(F("🟢[AGLoRa: loading 📦  from index "));
  Serial.print(index);
  Serial.print(F("]"));
#endif

  if ((_memory->getStorageOverwrite() == false) && (_memory->getIndex() == 0))
  {
#if DEBUG_MODE && DEBUG_AGLORA
    Serial.println(F("- error 🚨 empty memory 🚨"));
#endif

    String response = bleProtocolPrefix +
                      bleProtocolTypeMemory +
                      bleProtocolVersion;
    response += bleProtocolParamMemorySize + _memory->getSize();
    response += bleProtocolParamMemoryIndex + _memory->getIndex();
    response += bleProtocolParamMemoryOverwrite + _memory->getStorageOverwrite();
    response += bleProtocolDivider;
    _ble->send(&response);

    return;
  }

  unsigned int maxIndex = _memory->getStorageOverwrite() ? _memory->getSize() : _memory->getIndex();
  if (index > maxIndex - 1)
  {
#if DEBUG_MODE && DEBUG_AGLORA
    Serial.println(F("- error 🚨 index out of range 🚨"));
#endif
    return;
    // TODO: send error
  }

  sendPackageToBLE(_memory->load(index), index);
}

void AGLORA::sendLastPackagesToBLE()
{
    const unsigned int MAX_TRACKERS = 10; // Max trackers in memory
    struct TrackerData {
        char name[NAME_LENGTH];
        DATA* lastData;
    };
    TrackerData lastDataArray[MAX_TRACKERS];
    unsigned int trackerCount = 0;

    unsigned int maxIndex = _memory->getStorageOverwrite() ? _memory->getSize() : _memory->getIndex();

    for (unsigned int i = 0; i < maxIndex; ++i)
    {
        DATA* data = _memory->load(i);
        if (data == nullptr) continue;

        bool found = false;
        for (unsigned int j = 0; j < trackerCount; ++j)
        {
            if (strncmp(lastDataArray[j].name, data->name, NAME_LENGTH) == 0)
            {
                if (isDataMoreRecent(data, lastDataArray[j].lastData))
                {
                    lastDataArray[j].lastData = data;
                }
                found = true;
                break;
            }
        }

        if (!found && trackerCount < MAX_TRACKERS)
        {
            strncpy(lastDataArray[trackerCount].name, data->name, NAME_LENGTH);
            lastDataArray[trackerCount].lastData = data;
            trackerCount++;
        }
    }

    for (unsigned int i = 0; i < trackerCount; ++i)
    {
        sendPackageToBLE(lastDataArray[i].lastData, 0);
    }
}

bool AGLORA::isDataMoreRecent(DATA * newData, DATA * oldData)
{
    if (newData->year > oldData->year) return true;
    if (newData->year < oldData->year) return false;

    if (newData->month > oldData->month) return true;
    if (newData->month < oldData->month) return false;

    if (newData->day > oldData->day) return true;
    if (newData->day < oldData->day) return false;

    if (newData->hour > oldData->hour) return true;
    if (newData->hour < oldData->hour) return false;

    if (newData->minute > oldData->minute) return true;
    if (newData->minute < oldData->minute) return false;

    if (newData->second > oldData->second) return true;
    if (newData->second < oldData->second) return false;

    return false;
}

/*
This is a function that sends data to the app.
Data packets are sent using OsmAnd-like protocol:
id=name&lat={0}&lon={1}&timestamp={2}&speed={3}&altitude={4}
*/
String sendToPhone(DATA *package) {
  String result;
  result += F("&dev_batt=");  //battery of your device
  result += F("100"); //implement voltage acquisition

  result += F("&name=");  //other tracker's name
  result += package->name;  //NAME_LENGTH bytes

  result += F("&lat=");       // cordinates
  result += String(package->lat, 6);  // latitude
  result += F("&lon=");       // record separator
  result += String(package->lon, 6);  // longitute

  //Date and time format: 2023-06-07T15:21:00Z
  result += F("&timestamp=");      // record separator
  result += package->year + 2000;  // year
  result += F("-");                // data separator
  if (package->month < 10) result += F("0");
  result += package->month;        // month
  result += F("-");                // data separator
  if (package->day < 10) result += F("0");
  result += package->day;          // day
  result += F("T");                // record separator
  if (package->hour < 10) result += F("0");
  result += package->hour;         // hour
  result += F(":");                // time separator
  if (package->minute < 10) result += F("0");
  result += package->minute;       // minute
  result += F(":");                // time separator
  if (package->second < 10) result += F("0");
  result += package->second;       // second
  result += F("Z");                // UTC

  // Sensors and additional data
  result += F("&gpsValid=");
  result += package->gpsValid;  // validity of coordinates  bool

  // Add more data here if you need ...
  // result += "&speed=";
  // result += package->speed;

  result += "&battery=";
  result += package->battery;

  // result += "&C-137-level=";  // data's name in app
  // result += package->sensor2; // value

  return result;
}

String sendBatteryToPhone() {
  String result;

  result += "&dev_bat=";
  result += 100;

  return result;
}

// =========================================================================
// ========================== MAIN PROGRAM =================================
// =========================================================================

TESTS tests;
INDICATION indication(GPS_LED, LORA_LED, BLE_LED, MEMORY_LED);
GPS gps(GPS_PIN_RX, GPS_PIN_TX, GPS_SPEED, &indication);
LORA lora(LORA_PIN_RX, LORA_PIN_TX, LORA_PIN_AX, LORA_PIN_M0, LORA_PIN_M1, &indication);
BLE_HM10 ble;

#if USE_EEPROM_MEMORY
EEPROMAglora memory;
#else
SRAM memory;
#endif

LORADATA loraDataPackage;
AGLORA aglora(&memory, &ble);

// ========== BEGIN ==========
void setup()
{
  Serial.begin(9600);
  aglora.hello(); // Beautifully print Hello from AGloRa :-)
  // Start modules
  gps.setup();    // GPS
  lora.setup();   // LoRa
  memory.setup(); // SRAM or EEPROM
  ble.setup();    // Bluetooth Low Energy
}

// ========== MAIN LOOP ==========
unsigned long _timeToSendMyLocation = millis() + DATA_SENDING_INTERVAL;
unsigned long _timeOfLastReceivedPacket;
unsigned int addedMemoryIndex;
byte ttl = 0;

void processNewData(LORADATA *loraDataPackage);

void loop()
{
  if (_timeToSendMyLocation < millis())
  {
#if I_WANT_TO_SEND_MY_LOCATION
    aglora.clearDataPacket(&loraDataPackage.data); // clear structure before reading new data
    aglora.updateSensors(&loraDataPackage.data);   // add sensors
    gps.updateLocation(&loraDataPackage.data);     // add locations
    loraDataPackage.ttl = TTL;                     // time to live (for mesh network)

    aglora.printPackage(&loraDataPackage);

    lora.send(&loraDataPackage); // send location to other trackers
#endif

    _timeToSendMyLocation += DATA_SENDING_INTERVAL;
  }

  // checking for new data
  if (lora.hasNewData(&loraDataPackage))
  {
    processNewData(&loraDataPackage);
  }

#if TEST_LORA_DATA
  if (tests.hasNewDataEveryXSec(&loraDataPackage, &gps, 10))
  {
    processNewData(&loraDataPackage);
  }
#endif

  // if the time checker is over some prescribed amount
  // let the user know there is no incoming data
 if ((_timeOfLastReceivedPacket + BLE_UPDATE_INTERVAL) < millis())
  {
    aglora.checkMemoryToBLE();
    _timeOfLastReceivedPacket = millis();
  }

  aglora.getRequest(ble.read()); // check requests from app

  indication.loop(); // make an indication
}

/**
 * Processes new data from a LORADATA package.
 *
 * @param loraDataPackage The LORADATA package containing the new data to be processed.
 *
 * @return void
 *
 * @throws None
 */
void processNewData(LORADATA *loraDataPackage)
{
  if (memory.checkUnique(&loraDataPackage->data)) // Check the name and time of the point
  {
    ttl = loraDataPackage->ttl;

    addedMemoryIndex = memory.save(&loraDataPackage->data);
    memory.checkCRC();

#if USE_BLE
    aglora.sendPackageToBLE(&loraDataPackage->data, addedMemoryIndex); // upload data to app
#endif

// resend data to other trackers
#if MESH_MODE
    if (--ttl > 0)
    {
      loraDataPackage->ttl = ttl;
      lora.send(loraDataPackage);
    }
#endif
  }

  _timeOfLastReceivedPacket = millis(); // if you got data, update the checker
}

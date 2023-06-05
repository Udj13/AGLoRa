/*
Ebyte LoRa TEST for AGLoRa device
https://github.com/Udj13/AGLoRa/
*/
#include "Arduino.h"

// Please setup library "EByte LoRa E220 by Renzo Mischianty" from Arduino IDE library manager
#include "LoRa_E220.h"

// Then, set pins

static const byte LORA_PIN_TX = 2, LORA_PIN_RX = 3;
static const byte LORA_PIN_M0 = 4, LORA_PIN_M1 = 5; 
static const byte LORA_PIN_AUX = 6;

// Ready, you can start sketch!

// Your device will be send periodic packets
// also you can enter any text and send it

static const unsigned int PERIODIC_PACKET_INTERVAL = 10000;  // milliseconds

#include <SoftwareSerial.h>
SoftwareSerial mySerial(LORA_PIN_TX, LORA_PIN_RX);
LoRa_E220 e220ttl(&mySerial, LORA_PIN_AUX, LORA_PIN_M1, LORA_PIN_M0);
//LoRa_E220 e220ttl(&mySerial); // Config without connect AUX and M0 M1

void setLoRaParameters(struct Configuration configuration);
void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);
unsigned long lastLoraPacketTime;

void setup() {
  // Serial init
  Serial.begin(9600);
  while (!Serial) {};
  delay(500);
  Serial.println();

  // LoRa init
  e220ttl.begin();
  ResponseStructContainer c;

  // Printing current LoRa parameters
  Serial.println("---------------------------------------------");
  Serial.println("------ Get current LoRa configuration -------");
  c = e220ttl.getConfiguration();
  Configuration configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
  printParameters(configuration);

  // Apply new parameters
  Serial.println("---------------------------------------------");
  Serial.println("--------- Set new LoRa configuration --------");
  setLoRaParameters(&configuration);  // <- check parameters in function
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);

  // Check of changes
  c = e220ttl.getConfiguration();
  configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
  printParameters(configuration);

  c.close();
}

void loop() {
  // if something available
  if (e220ttl.available() > 1) {
    Serial.println("New data received");
    ResponseContainer rc = e220ttl.receiveMessage();
    // is something goes wrong print error
    if (rc.status.code != 1) {
      Serial.println(rc.status.getResponseDescription());
    } else {
      // print the data received
      Serial.print("<");Serial.print(rc.data);Serial.println(">");
      Serial.println(rc.status.getResponseDescription());
      Serial.println();
    }
  }

  // data for transmitting is available
  if (Serial.available()) {
    Serial.println();

    String input = Serial.readString();
    Serial.print("Sending ->: ");
    Serial.print(input);

    Serial.print("sendMessage - ");
    ResponseStatus rs = e220ttl.sendMessage(input);
    Serial.println(rs.getResponseDescription());
  }

  // periodic transmitting
  if ((millis() - lastLoraPacketTime) > PERIODIC_PACKET_INTERVAL) {
    Serial.println("Sending ->: Test data");
    ResponseStatus rs = e220ttl.sendMessage("Test data");
    Serial.println(rs.getResponseDescription());
    lastLoraPacketTime = millis();
  }
}


void setLoRaParameters(struct Configuration *configuration) {
  configuration->ADDL = 0x00;
  configuration->ADDH = 0x00;

  configuration->CHAN = 0x17;

  configuration->SPED.uartBaudRate = UART_BPS_9600;
  configuration->SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration->SPED.uartParity = MODE_00_8N1;

  configuration->OPTION.subPacketSetting = SPS_200_00;
  configuration->OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration->OPTION.transmissionPower = POWER_22;


  configuration->TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration->TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration->TRANSMISSION_MODE.enableLBT = LBT_DISABLED;  // monitoring before data transmitted
  configuration->TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
}


void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(configuration.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));
  Serial.print(configuration.OPTION.subPacketSetting, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));
  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));
  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));
  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));
  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));
  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


  Serial.println("----------------------------------------");
}

void printModuleInformation(struct ModuleInformation moduleInformation) {
  Serial.println("----------------------------------------");
  Serial.print(F("HEAD: "));
  Serial.print(moduleInformation.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(moduleInformation.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(moduleInformation.LENGHT, DEC);

  Serial.print(F("Model no.: "));
  Serial.println(moduleInformation.model, HEX);
  Serial.print(F("Version  : "));
  Serial.println(moduleInformation.version, HEX);
  Serial.print(F("Features : "));
  Serial.println(moduleInformation.features, HEX);
  Serial.println("----------------------------------------");
}

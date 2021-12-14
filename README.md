# AGLoRa

Project AGLoRa - easy opensource LoRa GPS tracker.
Created by Eugeny Shlyagin (shlyagin@gmail.com)

[![AGLoRa video](http://img.youtube.com/vi/bU68tQFBxwA/0.jpg)](http://www.youtube.com/watch?v=bU68tQFBxwA)

Click to open video.

![AGLoRa](https://bitlite.ru/wp-content/uploads/2021/11/aglora-prototype.jpg)
 
AGLoRa is an acronym for "Arduino GPS LoRa".
LoRa (from "long range") is a proprietary low-power wide-area network modulation technique. https://en.wikipedia.org/wiki/LoRa

AGLoRa is a simple open-source satellite tracking system for hiking, sailing, pet finding, and other outdoor activities.
All components widely available on marketplaces (aliexpress etc.).

![Program block diagram](https://bitlite.ru/wp-content/uploads/2021/11/lora-tracker.drawio.png)


AGLoRa receives the coordinates from other trackers (via LoRa) and immediately transmits them to the phone app.
By default the tracker sends its coordinates via LoRa every 10 seconds, when its GPS data is valid.

## ABOUT THE PROJECT

We are going to test E32-E433T30D. 
It is a wireless transceiver module, operates at 433 MHz based on original RFIC SX1278 from SEMTECH.
Aglora broadcast coordinates to other trackers.

![AGLoRa working diagram](https://bitlite.ru/wp-content/uploads/2021/11/Project-proposal-1.jpg)
 
 
### COMPONENTS AND SUPPLIES
- Arduino Nano or Arduino UNO (ATMEGA328P, not ATmega168)
- LoRa Module (EBYTE E220-900T22D (868 MHz), E433T30D (433 MHz))
- GPS Module (Generic)
- HC-05 (AT-09) Bluetooth Low Energy Module
- iOS or Android device
 
### APPS
- Arduino IDE
 
# Wiring.

Let’s Start Building. The circuit is so simple, there are a few connections to be made.

![AGLoRa device diagram](https://bitlite.ru/wp-content/uploads/2021/11/Project-proposal.jpg)


### Сonnecting the LoRa module
```
Arduino Pins	 	  LoRa Pins
Pin 2		——>		RX
Pin 3		——>		TX
Pin 4		——>		M0   
Pin 5		——>		M1  
Pin 6		——>		AX    
5V		——>		VCC
GND		——>		GND
``` 

### Connecting the GPS module

```
Arduino Pins	 	  GPS Pins
Pin 7		——>		TX
Pin 8		——>		RX
5V		——>		VCC
GND		——>		GND
```
 
### Connecting the BLE module (optional)
```
Arduino Pins		  Bluetooth Pins
RX (Pin 0)	——>		TX
TX (Pin 1)	——>		RX
5V		——>		VCC
GND		——>		GND
```

![AGLoRA wiring](https://bitlite.ru/wp-content/uploads/2021/11/aglora-on-green.jpg)

Add a 5V stabilizer for battery power (like LM7805).
 
# Uploading Sketch

Three steps:

1. Download or copy a Sketch from here: https://github.com/Udj13/AGLoRa/blob/main/AGLoRa-tracker.ino Just one file! Easy!

2. Change the “MY_NAME” setting. Сheck the debug mode is off.

```
char MY_NAME[NAME_LENGTH] = "Morty";
#define DEBUG_MODE false
```

3. Upload a Sketch to an Arduino.

**NOTES:**

**Value “NAME_LENGTH” must be same for all your devices**

**Remove Bluetooth module Tx-Rx connection before uploading the program!**


 
# Install the app on your phone.

- Source code is available for free on github: https://github.com/Udj13/AGLoRa-client-flutter
- You can download the iOs app from here (in progress)
- You can download the Android app from here: https://shlyagin.ru/aglora.apk

## How to use the AGLoRa Client App?

![AGLoRa client](https://bitlite.ru/wp-content/uploads/2021/11/aglora-test-3km.jpg)

- Install application on your device
- Turn on the AGLoRa trackers. When GPS data is correct, the built-in LED is on.
- Scan for available device
- Select your Bluetooth module from the List (“AGLoRa”)
- Waiting receive data from other trackers



Description in Russian: https://bitlite.ru/aglora-lora-gps-tracker/



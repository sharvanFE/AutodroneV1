/*

Demonstrates simple RX and TX operation.
Any of the Basic_RX examples can be used as a receiver.
Please read through 'NRFLite.h' for a description of all the methods available in the library.

Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND

*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "NRFLite.h"

const static uint8_t RADIO_ID = 0;             // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 1; // Id of the radio we will transmit to.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

int VRx1 = A0;
int VRy1 = A1;
int SW1 = 2;
int VRx2 = A2;
int VRy2 = A3;
int SW2 = 3;

struct RadioPacket {
  uint8_t FromRadioId;
  uint32_t x1;
  uint32_t y1;
  uint32_t b1;
  uint32_t x2;
  uint32_t y2;
  uint32_t b2;
};

NRFLite _radio;
RadioPacket _radioData;

void setup()
{
    Serial.begin(115200);
    

    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
    
    _radioData.FromRadioId = RADIO_ID;

  pinMode(VRx1, INPUT);
  pinMode(VRy1, INPUT);
  pinMode(SW1, INPUT_PULLUP); 

  pinMode(VRx2, INPUT);
  pinMode(VRy2, INPUT);
  pinMode(SW2, INPUT_PULLUP); 
  
}

void loop() {
  
  _radioData.x1 = analogRead(VRx1);
  _radioData.y1 = analogRead(VRy1);
  _radioData.b1 = digitalRead(SW1);

  _radioData.x2 = analogRead(VRx2);
  _radioData.y2 = analogRead(VRy2);
  _radioData.b2 = digitalRead(SW2);

  Serial.println("");
  Serial.print(" X1: ");
  Serial.print(_radioData.x1);
  Serial.print(" ,Y1: ");
  Serial.print(_radioData.y1);
  Serial.print(" ,B1:  ");
  Serial.print(_radioData.b1);
  Serial.print(" ,X2:  ");
  Serial.print(_radioData.x2);
  Serial.print(" ,Y2:  ");
  Serial.print(_radioData.y2);
  Serial.print(", B2: ");
  Serial.print(_radioData.b2);

  delay(100);
  
   if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) // Note how '&' must be placed in front of the variable name.
    {
        Serial.println("...Success");
    }
}

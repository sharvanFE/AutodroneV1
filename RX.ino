#include "SPI.h"
#include "NRFLite.h"

const static uint8_t RADIO_ID = 1;             // Our radio's id.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

// struct
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
}

void loop()
{
    while (_radio.hasData())
    {
        _radio.readData(&_radioData); // Note how '&' must be placed in front of the variable name.

        String msg = "X1:";
        msg += _radioData.x1;
        msg += "Y1:";
        msg += _radioData.y1;
        msg += " , B1: ";
        msg += _radioData.b1;
        msg += " | X2:";
        msg += _radioData.x2;
        msg += "Y2:";
        msg += _radioData.y2;
        msg += " , B2: ";
        msg += _radioData.b2;
        
        Serial.println(msg);
    }
    //Serial.println("End");
}

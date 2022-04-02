#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/

const int pinCE = 9; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const int pinCSN = 10; //This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out

RF24 wirelessSPI(pinCE, pinCSN); // Create your nRF24 object or wireless SPI connection
const uint64_t pAddress = 0xB00B1E5000LL; // Radio pipe addresses for the 2 nodes to communicate.

int VRx1 = A0;
int VRy1 = A1;
int SW1 = 2;
int VRx2 = A2;
int VRy2 = A3;
int SW2 = 3;

struct RadioPacket {
  uint32_t x1;
  uint32_t y1;
  uint32_t b1;
  uint32_t x2;
  uint32_t y2;
  uint32_t b2;
};

struct RadioAckPacket {
  uint32_t pitch;
  uint32_t roll;
  uint32_t yaw;
  uint32_t fr;
  uint32_t rr;
  uint32_t fl;
  uint32_t rl;
};

RadioPacket _radioData;
RadioAckPacket _radioAckData;
void setup() 
{
  Serial.begin(57600); //start serial to communicate process
  wirelessSPI.begin(); //Start the nRF24 module
  wirelessSPI.setAutoAck(1); // Ensure autoACK is enabled so rec sends ack packet to let you know it got the transmit packet payload
  wirelessSPI.enableAckPayload(); // Allow optional ack payloads
  wirelessSPI.setPALevel(RF24_PA_LOW);
  wirelessSPI.openWritingPipe(pAddress); // pipe address that we will communicate over, must be the same for each nRF24 module
  wirelessSPI.stopListening(); //transmitter so stop listening for data

  pinMode(VRx1, INPUT);
  pinMode(VRy1, INPUT);
  pinMode(SW1, INPUT_PULLUP); 
  
  pinMode(VRx2, INPUT);
  pinMode(VRy2, INPUT);
  pinMode(SW2, INPUT_PULLUP); 

  PCICR |= (1 << PCIE1);                                                                // set PCIE0 to enable PCMSK0 scan.
  PCMSK1 |= (1 << PCINT8);
  PCMSK1 |= (1 << PCINT9);
  PCMSK1 |= (1 << PCINT10);
  PCMSK1 |= (1 << PCINT11);
}

void loop() {
   delay(100); 
   sendData();
}


void sendData(){
    Serial.println("Sending packet"); 
    Serial.println("");

  _radioData.x1 = map(analogRead(VRx1), 0, 1023, 0, 255);
  _radioData.y1 = map(analogRead(VRy1), 0, 1023, 0, 255);
  _radioData.b1 = digitalRead(SW1);

  _radioData.x2 = map(analogRead(VRx2), 0, 1023, 0, 255);
  _radioData.y2 = map(analogRead(VRy2), 0, 1023, 0, 255);
  _radioData.b2 = digitalRead(SW2);
    
   if (wirelessSPI.write( &_radioData, sizeof(_radioData) )){ 
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

        if (wirelessSPI.isAckPayloadAvailable()) {
                // read ack payload and copy data to relevant remoteNodeData array
                wirelessSPI.read(&_radioAckData, sizeof(_radioAckData));
                Serial.println(" ");
                Serial.print("RL: ");
                Serial.print(_radioAckData.rl);
                Serial.print(" | RR: ");
                Serial.print(_radioAckData.rr);
                Serial.println("FL: ");
                Serial.println(_radioAckData.fl);
                Serial.println(" | FR: ");
                Serial.println(_radioAckData.fr);
         }
   }else{
      Serial.println("packet delivery failed"); 
   }  
}

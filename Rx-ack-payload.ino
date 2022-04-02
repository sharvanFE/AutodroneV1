#include <SPI.h> //Call SPI library so you can communicate with the nRF24L01+
#include <RF24.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <nRF24L01.h> //nRF2401 libarary found at https://github.com/tmrh20/RF24/
#include <avr/sleep.h> //library needed to use AVR based sleep API

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
  uint32_t pwm_pitch;
  uint32_t pwm_roll;
  uint32_t pwm_yaw;
  uint32_t pwm_throttle;
};

RadioPacket _radioData;
RadioAckPacket _radioAckData;

const int pinCE = 9; //This pin is used to set the nRF24 to standby (0) or active mode (1)
const int pinCSN = 10; //This pin is used to tell the nRF24 whether the SPI communication is a command or message to send out
int IRQ_PIN = 3;

RF24 wirelessSPI(pinCE, pinCSN); // Declare object from nRF24 library (Create your wireless SPI) 
const uint64_t pAddress = 0xB00B1E5000LL; //Create a pipe addresses for the 2 nodes to communicate over, the "LL" is for LongLong type

//Declaring Global Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte lowByte, highByte, type, gyro_address, error, clockspeed_ok;
byte channel_1_assign, channel_2_assign, channel_3_assign, channel_4_assign;
byte roll_axis, pitch_axis, yaw_axis;
byte receiver_check_byte, gyro_check_byte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1, high_channel_2, high_channel_3, high_channel_4;
int low_channel_1, low_channel_2, low_channel_3, low_channel_4;
int address, cal_int;
unsigned long timer, timer_1, timer_2, timer_3, timer_4, current_time;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;


void setup(){
 pinMode(IRQ_PIN, INPUT);
 wirelessSPI.begin(); //Start the nRF24 module
 wirelessSPI.setAutoAck(1); // Ensure autoACK is enabled so rec sends ack packet to let you know it got the transmit packet payload
 wirelessSPI.enableAckPayload(); //allows you to include payload on ack packet
 wirelessSPI.maskIRQ(1,1,0); //mask all IRQ triggers except for receive (1 is mask, 0 is no mask)
 wirelessSPI.setPALevel(RF24_PA_LOW); //Set power level to low, won't work well at higher levels (interfer with receiver)
 wirelessSPI.openReadingPipe(1,pAddress); //open pipe o for recieving meassages with pipe address
 wirelessSPI.startListening(); // Start listening for messages
 attachInterrupt(digitalPinToInterrupt(IRQ_PIN), interruptFunction, FALLING); //Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt
 Serial.begin(115200); //start serial to communicate process
 delay(250);
}

void loop(){
}

void interruptFunction() {
 bool tx_ds, tx_df, rx_dr;
 wirelessSPI.whatHappened(tx_ds, tx_df, rx_dr);
 if(wirelessSPI.available()) { //get data sent from transmit
     wirelessSPI.read( &_radioData, sizeof(_radioData) ); //read one byte of data and store it in gotByte variable
     receiver_input_channel_1 = _radioData.x1;
     receiver_input_channel_2 = _radioData.y1;
     receiver_input_channel_3 = _radioData.x2;
     receiver_input_channel_4 = _radioData.y2;

     wirelessSPI.writeAckPayload(1, &_radioAckData, sizeof(_radioAckData));

 }
}

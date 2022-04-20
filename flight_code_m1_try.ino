#include <Wire.h>               
#include <EEPROM.h>           
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <avr/sleep.h> 
#include <Servo.h>

Servo FL_ESC;
Servo FR_ESC;
Servo BL_ESC;
Servo BR_ESC;

const int PIN_RADIO_CE = 9;
const int PIN_RADIO_CSN = 10;

unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

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
  uint8_t f_status;
};

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);   // nRF24L01 (CE, CSN)
RadioPacket _radioData;
RadioAckPacket _radioAckData;
int IRQ_PIN = 3;
const uint64_t pAddress = 0xB00B1E5000LL;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 3.55;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.003;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 1.05;              //Gain setting for the roll D-controller
int pid_max_roll = 1000;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 3.55;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 1000;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = false;                 //Auto level on (true) or off (false)


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, d_throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;


int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, acc_total_vector;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time;
int i;
float rad_to_deg = 180/3.141592654;

float pitch_PID,roll_PID,yaw_PID;
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i;
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle; 
double twoX_kp=5;      
double twoX_ki=0.003;
double twoX_kd=2;     
double yaw_kp=3;    
double yaw_ki=0.002;


int ESCout_1 ,ESCout_2 ,ESCout_3 ,ESCout_4;
int input_PITCH = 50;
int input_ROLL = 50;
int input_YAW;
int input_THROTTLE=1100;
int state1,state2,state3,state4;



void setup() {

  // put your setup code here, to run once:
   Serial.begin(115200); //start serial to communicate process
   pinMode(IRQ_PIN, INPUT);
   radio.begin(); //Start the nRF24 module
   radio.setAutoAck(1); // Ensure autoACK is enabled so rec sends ack packet to let you know it got the transmit packet payload
   radio.enableAckPayload(); //allows you to include payload on ack packet
   radio.maskIRQ(1,1,0); //mask all IRQ triggers except for receive (1 is mask, 0 is no mask)
   radio.setPALevel(RF24_PA_LOW); //Set power level to low, won't work well at higher levels (interfer with receiver)
   radio.openReadingPipe(1,pAddress); //open pipe o for recieving meassages with pipe address
   radio.startListening(); // Start listening for messages
   attachInterrupt(digitalPinToInterrupt(IRQ_PIN), interruptFunction, FALLING); //Create interrupt: 0 for pin 2 or 1 for pin 3, the name of the interrupt function or ISR, and condition to trigger interrupt

   FL_ESC.attach(4);
   BL_ESC.attach(5);
   BR_ESC.attach(6);
   FR_ESC.attach(7);

    //Copy the EEPROM data for fast access data.
    for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
    start = 0;                                                                //Set start back to zero.
    gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.
  
    Wire.begin();                                                             //Start the I2C as master.
  
   TWBR = 12;      
   DDRD |= B11110000; 

   //Check the EEPROM signature to make sure that the setup program is executed.
   while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

   
  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  
  set_gyro_registers(); 

  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }

   gyroCalibrate();


   while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
     // digitalWrite(12, !digitalRead(12));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }
  start = 0; 

  battery_voltage = (1023 + 65) * 1.2317;

  //loop_timer = micros();                                                    //Set the timer for the next loop.
}

void loop(){

  /////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
    receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
       //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    //Reset the PID controllers for a bumpless start.
//    pid_i_mem_roll = 0;
//    pid_last_roll_d_error = 0;
//    pid_i_mem_pitch = 0;
//    pid_last_pitch_d_error = 0;
//    pid_i_mem_yaw = 0;
//    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950) start = 0;

  gyro_signalen();
  
  calculate_setpoint();
  
  calculate_error();
  
  calculate_pid();
  
  calculate_esc();

  send_esc_signals();

  // stopping motors on loss of connection
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }


}

void calculate_error(){
  roll_error =  angle_roll_output - roll_desired_angle;
  pitch_error = angle_pitch_output - pitch_desired_angle;  
  yaw_error = angle_yaw - yaw_desired_angle;    
}

void gyroCalibrate(){
  Wire.begin();    
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                  
  Wire.endTransmission();         
  Wire.beginTransmission(0x68);                                      
  Wire.write(0x1C);                                                   
  Wire.write(0x10);                                                   
  Wire.endTransmission();             
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1B);                                                 
  Wire.write(0x08);                                                   
  Wire.endTransmission();  
  delay(1000);
for (int cal_int = 0; cal_int < 2000 ; cal_int ++){  
  if(cal_int % 125 == 0)Serial.print(".");                                           
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                               
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                              
  gyro_x_cal += gyro_x;                                              
  gyro_y_cal += gyro_y;                                             
  gyro_z_cal += gyro_z;                                             
  yield();                                                    
}
gyro_x_cal /= 2000;                                                 
gyro_y_cal /= 2000;                                                 
gyro_z_cal /= 2000;

Time = micros();                                                                            

}

void resetData(){
   start = 0;
}

void calculate_setpoint(){

  roll_desired_angle = 3*((float)input_ROLL/(float)10 - (float)5);
  pitch_desired_angle =3*((float)input_PITCH/(float)10 - (float)5);
  
}

void calculate_esc(){

  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  //d_throttle = 1200;
  if (start == 2){                                                          //The motors are started.
//    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
//    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
//    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
//    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
//    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)
//
//    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
//      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
//      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
//      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
//      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
//    } 
//
//    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
//    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
//    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
//    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.
//
//    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
//    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
//    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
//    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  

    ESCout_1 = input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
    ESCout_2 = input_THROTTLE + roll_PID - pitch_PID + yaw_PID;
    ESCout_3 = input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
    ESCout_4 = input_THROTTLE - roll_PID + pitch_PID + yaw_PID;
    
    if(ESCout_1>2000) ESCout_1=2000;
    else if(ESCout_1<1100) ESCout_1=1100;
    if(ESCout_2>2000) ESCout_2=2000;
    else if(ESCout_2<1100) ESCout_2=1100;
    if(ESCout_3>2000) ESCout_3=2000;
    else if(ESCout_3<1100) ESCout_3=1100;
    if(ESCout_4>2000) ESCout_4=2000;
    else if(ESCout_4<1100) ESCout_4=1100;
    
    roll_previous_error = roll_error;
    pitch_previous_error = pitch_error;

  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
}

void calculate_pid(){

  roll_pid_p = twoX_kp*roll_error;
  pitch_pid_p = twoX_kp*pitch_error;
  yaw_pid_p = yaw_kp*yaw_error;
  
  if(-3 < roll_error <3){roll_pid_i = roll_pid_i+(twoX_ki*roll_error);}
  if(-3 < pitch_error <3){pitch_pid_i = pitch_pid_i+(twoX_ki*pitch_error);}
  if(-3 < yaw_error <3){yaw_pid_i = yaw_pid_i+(yaw_ki*yaw_error);}
  
  roll_pid_d = twoX_kd*((roll_error - roll_previous_error)/elapsedTime);
  pitch_pid_d = twoX_kd*((pitch_error - pitch_previous_error)/elapsedTime);
  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID = yaw_pid_p + yaw_pid_i;
  
  if(roll_PID < -400){roll_PID=-400;}
  else if(roll_PID > 400) {roll_PID=400;}
  if(pitch_PID < -400){pitch_PID=-400;}
  else if(pitch_PID > 400) {pitch_PID=400;}
  if(yaw_PID < -400){yaw_PID=-400;}
  else if(yaw_PID > 400) {yaw_PID=400;}
}

void gyro_signalen(){
  
  timePrev = Time;                   
  Time = micros();  
  elapsedTime = (float)(Time - timePrev) / (float)1000000;
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);                                        
  acc_x = Wire.read()<<8|Wire.read();                               
  acc_y = Wire.read()<<8|Wire.read();                               
  acc_z = Wire.read()<<8|Wire.read();                                 
  temperature = Wire.read()<<8|Wire.read();                           
  gyro_x = Wire.read()<<8|Wire.read();                                
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read(); 
  
  gyro_x -= gyro_x_cal;                                              
  gyro_y -= gyro_y_cal;                                               
  gyro_z -= gyro_z_cal;     
                                                    
  angle_pitch += gyro_x * elapsedTime * 0.01526717557;                                 
  angle_roll += gyro_y * elapsedTime * 0.01526717557;
  angle_yaw += gyro_z * elapsedTime * 0.01526717557;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066); 
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  angle_pitch_acc += 0;                                              
  angle_roll_acc += 0;       
                                        
  if(set_gyro_angles){                                                
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{                                                               
    angle_pitch = angle_pitch_acc;                                     
    angle_roll = angle_roll_acc;                                       
    set_gyro_angles = true;                                            
  }
  
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
//        if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
//          pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
//          roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
//        }

}

void send_esc_signals(){
     FR_ESC.writeMicroseconds(ESCout_1);
     BR_ESC.writeMicroseconds(ESCout_2);
     BL_ESC.writeMicroseconds(ESCout_3);
     FL_ESC.writeMicroseconds(ESCout_4);
} 

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(12,HIGH);                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

  }  
}


int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void interruptFunction() {
 bool tx_ds, tx_df, rx_dr;
 radio.whatHappened(tx_ds, tx_df, rx_dr);
 if(radio.available()) { //get data sent from transmit
     radio.read( &_radioData, sizeof(_radioData) ); //read one byte of data and store it in gotByte variable
     lastReceiveTime = millis(); // At this moment we have received the data

     receiver_input[1] = map(_radioData.x1, 0, 255, 900, 2100);
     receiver_input[2] = map(_radioData.y1, 0, 255, 900, 2100);
     receiver_input[3] = map(_radioData.x2, 0, 255, 900, 2100);
     receiver_input[4] = map(_radioData.y2, 0, 255, 900, 2100);

     _radioAckData.fr = esc_1;
     _radioAckData.rr = esc_2;
     _radioAckData.fl = esc_4;
     _radioAckData.rl = esc_3;
     _radioAckData.f_status = start;

     radio.writeAckPayload(1, &_radioAckData, sizeof(_radioAckData));
 }
}

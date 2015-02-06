/*
working servo controller 
change MY_ADDRESS to the correct value
todo
changed to PD loop op is stable but
values are never reached due ti no I
motor controller has a dead band up to 70 pwm
so motor controller is disabled below 70

    error
    current
    voltage
    is moving


*/
#include <PID_v1.h>
#include <Wire.h>
#include "Arduino.h"
#include <I2C_Anything.h>
#define Right_tilt_pwm			3// old 3
#define Right_tilt_in1			4
#define Right_tilt_in2			5
// change to actuator
int MinVal = 100;
int MaxVal = 980;
double target_pos = 512;
int angle_pub;
const byte MY_ADDRESS = 42;
double angle ;
double Encoder_Position  = 0;
double Right_pwm = 0;
double Right_Gap =0;
double servo_pos =0;

unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;

//volatile boolean haveData = false;
volatile long Target = 8;//why 550 

PID Right_tilt(&angle, &Right_pwm, &target_pos, 4,0,1, DIRECT);
/*
##############################################################################
##############################################################################


set up


####################################################################################
################################################################################
*/
void setup() 
{
  Wire.begin (MY_ADDRESS);
  Serial.begin (115200);
  Wire.onReceive (receiveEvent); //recieve target from master
  Wire.onRequest(requestEvent); // send data to master
  //TCCR1B = TCCR1B & 0b11111000 | 0x01 ;// for pin 9
  TCCR2B = TCCR2B & 0b11111000 | 0x01;//timer 2 pin 3

  pinMode(Right_tilt_in1, OUTPUT);
  pinMode(Right_tilt_in2, OUTPUT);
  pinMode(Right_tilt_pwm, OUTPUT);
  Right_tilt.SetMode(AUTOMATIC);
}  // end of setup
/*
###################################################################################
#####################################################################################

                                          LOOP
                                          
########################################################################################
#####################################################################################

*/


void loop()
{
  Encoder_Position =  analogRead(0);
  angle  = Encoder_Position;//map(Encoder_Position, 0, 1023, 0, 2750);
  angle_pub = map(Encoder_Position, 0, 1023, 0, 2750);
  //int angleT  = map(target_pos, 0, 1023, 0, 300);
  Right_Gap = (target_pos - angle)*0.1;
  //int error = angleT - angle;
  
  
  Serial.print(MY_ADDRESS);
  Serial.print(" ");
  Serial.print(target_pos);
  Serial.print(" ");
  Serial.print(angle_pub);
  Serial.print(" ");
  Serial.print(angle);
 
 
  Serial.println(" ");

  
  //pid timing loop
  milliSecsSinceLastUpdate = millis() - CurrentTime;
  if(milliSecsSinceLastUpdate >= FrameRate)
  {
  Right_tilt.Compute();
  Right_tilt.SetOutputLimits(-250 ,250);
  Right_drive();
  CurrentTime = millis();
  }
    

}  // end of loop

// called by interrupt service routine when incoming target data arrives
void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof Target))
   {
     
   I2C_readAnything (Target); 
   target_pos = Target; 
   if (target_pos < MinVal){
    target_pos == MinVal;}
   if (target_pos > MaxVal){
    target_pos == MaxVal;}
   
   
   //haveData = true;     
   }  // end if have enough data
 }  // end of receiveEvent

void requestEvent()
{
 
  sendSensor (A0);//sends current position
 
}
// sends data back to master
void sendSensor (const byte which)
  {
  int val = analogRead (which);
  byte buf [2];
  
    buf [0] = val >> 8;
    buf [1] = val & 0xFF;
    Wire.write (buf, 2);
  }  // end of sendSensor

void Right_drive()
{
  
  //Right_tilt.Compute();
  //Right_tilt.SetOutputLimits(-255 ,255);
  
  if (Right_pwm < 0){
    digitalWrite(Right_tilt_in1, HIGH);
    digitalWrite(Right_tilt_in2, LOW);
    int right_power = abs(Right_pwm);
    if (right_power < 70) 
    {right_power = 0;}
    analogWrite(Right_tilt_pwm, right_power);
  }
  
  if (Right_pwm > 0){
    digitalWrite(Right_tilt_in1, LOW);
    digitalWrite(Right_tilt_in2, HIGH);
    int right_power = abs(Right_pwm);
    if (right_power < 70) 
    {right_power = 0;}
    analogWrite(Right_tilt_pwm, right_power);
  }
}
  


/*
working servo controller 
change MY_ADDRESS to the correct value
todo
fine tune PID
remove motor noise
change to phoenix_servo.h driver
add extra data for the return
turn off motor driver when not moving

    error
    current
    voltage
    is moving


*/
#include <PID_v1.h>
#include <Wire.h>
#include "Arduino.h"
#include <I2C_Anything.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

// change to actuator
int MinVal = 500;
int MaxVal = 1750;
double angle ;
double target_pos = 900;
const byte MY_ADDRESS = 40;

double Encoder_Position  = 0;
double Right_pwm = 0;
double Right_Gap =0;
double servo_pos =0;

unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;

//volatile boolean haveData = false;
volatile long Target = 8;//why 550 

PID Right_tilt(&angle, &Right_pwm, &target_pos, 10,3,0, DIRECT);
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
  Serial.begin (57600);
  Wire.onReceive (receiveEvent); //recieve target from master
  Wire.onRequest(requestEvent); // send data to master
  
  md.init();
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
  angle  = map(Encoder_Position, 0, 1023, 0, 1800);
  //int angleT  = map(target_pos, 0, 1023, 0, 300);
  Right_Gap = (target_pos - angle)*0.1;
  //int error = target_pos - angle;
  
  
  Serial.print(MY_ADDRESS);
  Serial.print(" ");
  Serial.print(target_pos);
  Serial.print(" ");
  Serial.print(angle);
  Serial.print(" ");
  Serial.print(Right_Gap);
  Serial.print(" ");
  Serial.print(Encoder_Position);
 
  Serial.println(" ");

  
  //pid timing loop
  milliSecsSinceLastUpdate = millis() - CurrentTime;
  if(milliSecsSinceLastUpdate >= FrameRate)
  {
  
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
  //angle;
 
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
  int gap = abs(Right_Gap);
  if (gap < 50)
  {
  Right_tilt.SetTunings(3, 0.1, 0);
  }
  else
  {
    Right_tilt.SetTunings(5, 0, 0);
  }
  Right_tilt.Compute();
  Right_tilt.SetOutputLimits(-400 ,400);
  
    md.setM1Speed(Right_pwm);
  }

  


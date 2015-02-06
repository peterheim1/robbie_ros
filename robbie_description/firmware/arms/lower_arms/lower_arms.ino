/*
i2c bus has problems with stalling


*/

#include <Servo.h>
#include <Wire.h>
#include "Arduino.h"
#include <I2C_Anything.h>
#include <Messenger.h>

int mapped_left_rotate;
int mapped_right_rotate;
int mapped_right_elbow;
//tilt
int r_max = 120;
int r_min = 80;
int l_max = 120;
int l_min = 80;
//lift
int R_L_min = 100;
int R_L_max = 1000;
int L_L_min = 100;
int L_L_max = 1000;
//rotate
int R_R_min = 400;
int R_R_max = 1000;
int L_R_min = 400;
int L_R_max = 1000;
//ELBOW
int R_E_min = 400;
int R_E_max = 1000;
int L_E_min = 90;//SERVO
int L_E_max = 170;//SERVO

//position target values

int right_rotate_target = 100;
int left_rotate_target = 200;
int right_elbow_target = 512;
int left_elbow_target;


// change to actuator
int MinVal = 100;
int MaxVal = 2700;
double target_pos = 0;
int target_pos_servo = 80;

double angle ;
int mapped_Right_Lift;
int mapped_Left_Lift;
double Encoder_Right_Lift;
double Encoder_Left_Lift;
double Right_pwm = 0;
double left_pwm = 0;
double Right_Gap =0;
double servo_pos =0;
unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;
//volatile boolean haveData = false;
volatile long Target = 8;//why 550

Messenger _Messenger = Messenger();
/*
##############################################################################
##############################################################################
set up
####################################################################################
################################################################################
*/
void setup()
{
Wire.begin();
Serial.begin (115200);
_Messenger.attach(OnMssageCompleted);
//Wire.onReceive (receiveEvent); //recieve target from master
//Wire.onRequest(requestEvent); // send data to master

} // end of setup
/*
###################################################################################
#####################################################################################
LOOP
########################################################################################
#####################################################################################
*/
void loop()
{


//pid timing loop
milliSecsSinceLastUpdate = millis() - CurrentTime;
if(milliSecsSinceLastUpdate >= FrameRate)
{
  
  ReadSerial();
  //target_pos = analogRead(A0);
  mapped_left_rotate = map(RequestData(42), 0, 1023, 0, 2700);
  mapped_right_rotate = map(RequestData(44), 0, 1023, 0, 2700);
  mapped_right_elbow = map(RequestData(40), 0, 1023, 0, 1800);
  
  //angle = map(RequestData(42), 0, 1023, 0, 2700);

//target_pos_servo = map(target_pos, 0, 1023, 0, 180);

  Ser_print1();
/*servo control

right_tilt_target = target_pos_servo;//map(Encoder_Position, 0, 1023, 0, 180); 
left_tilt_target = target_pos_servo;//map(Encoder_Position, 0, 1023, 0, 180); 
  //right_tilt_target = map(Encoder_Position, 0, 1023, 0, 180);   
  if (right_tilt_target < r_min){ right_tilt_target = r_min;}
  if (right_tilt_target > r_max){ right_tilt_target = r_max;}
  right.write(right_tilt_target);    // sets the servo position according to the scaled value 
  if (left_tilt_target < l_min){ left_tilt_target = l_min;}
  if (left_tilt_target > l_max){ left_tilt_target = l_max;}
  left.write(left_tilt_target); 
  delay(1);
// end servo  control
*/

CurrentTime = millis();
}

} // end of loop





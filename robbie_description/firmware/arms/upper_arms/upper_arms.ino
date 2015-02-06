/*
i2c bus is now on arms lower
due to stalling I2C on the due


*/
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include "Arduino.h"
#include <I2C_Anything.h>
#include <Messenger.h>

#define Right_lift_pwm 10//9
#define Right_lift_in1 7//2
#define Right_lift_in2 8//4
//left channel
#define left_lift_pwm 9//10
#define left_lift_in1 2//8
#define left_lift_in2 4//7

Servo right;  
Servo left;
Servo left_elbow;
//tilt
int r_max = 120;
int r_min = 80;
int l_max = 120;
int l_min = 80;
//lift
int R_L_min = 400;
int R_L_max = 1000;
int L_L_min = 400;
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
int right_tilt_target = 83;
int left_tilt_target= 88;
double right_lift_target= 520;
double left_lift_target = 420;
int right_rotate_target = 500;
int left_rotate_target = 500;
int right_elbow_target;
int left_elbow_target= 63;

int Left_Tilt_raw;
int Right_Tilt_raw;
// change to actuator
int MinVal = 100;
int MaxVal = 2700;
double target_pos = 0;
int target_pos_servo = 80;
const byte MY_ADDRESS = 42;
double angle ;
int mapped_Right_Lift;
int mapped_Left_Lift;
int mapped_left_elbow;
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
PID Right_lift(&Encoder_Right_Lift, &Right_pwm, &right_lift_target, 1,0,0, DIRECT);
PID Left_lift(&Encoder_Left_Lift, &left_pwm, &left_lift_target, 1,0,0, DIRECT);

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
//Wire.begin();
Serial.begin (115200);
_Messenger.attach(OnMssageCompleted);
//Wire.onReceive (receiveEvent); //recieve target from master
//Wire.onRequest(requestEvent); // send data to master
left.attach(22);  // attaches the servo on pin 9 to the servo object
right.attach(23);
left_elbow.attach(24);
//move to safe start position
left_elbow.write(63);
left.write(81);
right.write(81);


pinMode(Right_lift_in1, OUTPUT);
pinMode(Right_lift_in2, OUTPUT);
pinMode(Right_lift_pwm, OUTPUT);
Right_lift.SetMode(AUTOMATIC);

pinMode(left_lift_in1, OUTPUT);
pinMode(left_lift_in2, OUTPUT);
pinMode(left_lift_pwm, OUTPUT);
Left_lift.SetMode(AUTOMATIC);
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
  target_pos = analogRead(A0);
  Encoder_Right_Lift = analogRead(A2);
  Encoder_Left_Lift = analogRead(A3);
  mapped_Left_Lift = map(Encoder_Left_Lift, 0, 1023, 2620, -2620)+1570;
  mapped_Right_Lift = map(Encoder_Right_Lift, 0, 1023, 2620, -2620)+1570;
  mapped_left_elbow = map(getFeedback(10), 255, 550, 0, 1250);
  if (mapped_left_elbow < 0){mapped_left_elbow = 0;}
  Left_Tilt_raw = analogRead(A9);
  Right_Tilt_raw = analogRead(A8);
  
  //angle = map(Encoder_Right_Lift, 0, 1023, 0, 1023);

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
Right_drive();
left_drive();
CurrentTime = millis();
}

} // end of loop

void Right_drive()
{
Right_lift.Compute();
Right_lift.SetOutputLimits(-150 ,200);
if (Right_pwm < 0){
digitalWrite(Right_lift_in1, HIGH);
digitalWrite(Right_lift_in2, LOW);
int right_power = abs(Right_pwm);
analogWrite(Right_lift_pwm, right_power);
}
if (Right_pwm > 0){
digitalWrite(Right_lift_in1, LOW);
digitalWrite(Right_lift_in2, HIGH);
int right_power = abs(Right_pwm);
analogWrite(Right_lift_pwm, right_power);
}
}

void left_drive()
{
Left_lift.Compute();
Left_lift.SetOutputLimits(-250 ,250);
if (left_pwm < 0){
digitalWrite(left_lift_in1, HIGH);
digitalWrite(left_lift_in2, LOW);
int left_power = abs(left_pwm);
analogWrite(left_lift_pwm, left_power);
}
if (left_pwm > 0){
digitalWrite(left_lift_in1, LOW);
digitalWrite(left_lift_in2, HIGH);
int left_power = abs(left_pwm);
analogWrite(left_lift_pwm, left_power);
}
}

///feedback
int getFeedback(int a){
int j;
int mean;
int result;
int test;
int reading[20];
boolean done;

for (j=0; j<20; j++){
reading[j] = analogRead(a); //get raw data from servo potentiometer
delay(3);
} // sort the readings low to high in array
done = false; // clear sorting flag
while(done != true){ // simple swap sort, sorts numbers from lowest to highest
done = true;
for (j=0; j<20; j++){
if (reading[j] > reading[j + 1]){ // sorting numbers here
test = reading[j + 1];
reading [j+1] = reading[j] ;
reading[j] = test;
done = false;
}
}
}
mean = 0;
for (int k=6; k<14; k++){ //discard the 6 highest and 6 lowest readings
mean += reading[k];
}
result = mean/8; //average useful readings
return(result);
}    // END GET FEEDBACK





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
#include "DualMotorShield.h"
/* Include definition of serial commands */
#include "commands.h"
/* Serial port baud rate */
#define BAUDRATE     57600
DualMotorShield md;

// change to actuator
int MinVal_lift = 100;
int MaxVal_Lift = 2700;
int MinVal_tilt = 100;
int MaxVal_tilt = 2700;
double angle_lift ;
double angle_tilt ;
double target_pos_lift = 250;
double target_pos_tilt = 900;
double target_pos_pan = 900;  
double target_pos_rot = 900; 
double target_pos_elbow = 900; 
const byte MY_ADDRESS = 50;

double Encoder_Position_tilt  = 0;
double tilt_pwm = 0;
double tilt_Gap =0;
double Encoder_Position_lift  = 0;
double lift_pwm = 0;
double lift_Gap =0;
double servo_pos =0;
double pan;
double pan1;
double rot;
double rot1;
double elbow;
double elbow1;

unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;

//volatile boolean haveData = false;
volatile long Target = 8;//why 550 

PID tilt(&angle_tilt, &tilt_pwm, &target_pos_tilt, 10,3,0, DIRECT);
PID lift(&angle_lift, &lift_pwm, &target_pos_lift, 10,3,0, DIRECT);


// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case READ_ENCODERS:
    pan = RequestData(44);
    pan1 = map(pan, 0, 1023, 0, 300);
    rot = RequestData(42);
    rot1 = map(rot, 0, 1023, 0, 300);
    elbow = RequestData(40);
    elbow1 = map(elbow, 0, 1023, 0, 180);
    Serial.print("pan");
    Serial.print("    ");
    Serial.print("tilt");
    Serial.print("    ");
    Serial.print("lift");
    Serial.print("    ");
    Serial.print("rot");
    Serial.print("    ");
    Serial.println("elbow");
    Serial.print(pan1);
    Serial.print("  ");
    Serial.print(angle_tilt*0.1);
    Serial.print("  ");
    Serial.print(angle_lift*0.1);
    Serial.print("  ");
    Serial.print(rot1);
    Serial.print("  ");
    Serial.println(elbow1);
    break;
  case TILT:
    target_pos_tilt = (arg1*10);
    if (target_pos_tilt < 900){
       target_pos_tilt =900;}
    if (target_pos_tilt >1600){
       target_pos_tilt =1600;}
    Serial.println(target_pos_tilt);
    break;
  case LIFT:
    target_pos_lift= (arg1*10);
    if (target_pos_lift <60){
       target_pos_lift =60;}
    if (target_pos_lift > 1900){
       target_pos_lift =1900;}
    Serial.println(target_pos_lift); 
    break;
  case PAN:
    SetTarget(44, (arg1*10)); 
    Serial.println("pan "); 
    break;
  case ROT:
    SetTarget(42, (arg1*10)); 
    Serial.println(arg1); 
    break;
  case ELBOW:
    SetTarget(40, (arg1*10)); 
    Serial.println(arg1); 
    break;
  }}
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
  Serial.begin (BAUDRATE);
  Wire.onReceive (receiveEvent); //recieve target from master
  Wire.onRequest(requestEvent); // send data to master
  
  md.init();
  tilt.SetMode(AUTOMATIC);
  lift.SetMode(AUTOMATIC);
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
  Encoder_Position_tilt =  analogRead(2);
  Encoder_Position_lift =  analogRead(3);
  angle_tilt  = map(Encoder_Position_tilt, 0, 1023, 0, 2750);
  angle_lift  = map(Encoder_Position_lift, 0, 1023, 0, 2750);
  tilt_Gap = (target_pos_tilt - angle_tilt)*0.1;
  lift_Gap = (target_pos_lift - angle_lift)*0.1;
  //return_data();
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
/*  
  Serial.print(MY_ADDRESS);
  Serial.print(" ");
  Serial.print(target_pos_tilt);
  Serial.print(" ");
  Serial.print(angle_tilt);
  Serial.print(" ");
  Serial.print(tilt_Gap);
  Serial.print(" ");
  Serial.print(Encoder_Position_tilt);
  Serial.print(" ");
  */
  //Serial.print(target_pos_lift);
  //Serial.print(" ");
  //Serial.print(angle_lift);
  //Serial.print(" ");
  //Serial.print(lift_Gap);
  //Serial.print(" ");
  //Serial.print(Encoder_Position_lift);
 
  //Serial.println(" ");

  
  //pid timing loop
  milliSecsSinceLastUpdate = millis() - CurrentTime;
  if(milliSecsSinceLastUpdate >= FrameRate)
  {
  //md.setM2Speed(200);
  
  tilt_drive();
  lift_drive();
  CurrentTime = millis();
  }
    

}  // end of loop

// called by interrupt service routine when incoming target data arrives
void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof Target))
   {
     
   I2C_readAnything (Target); 
   target_pos_tilt = Target; 
   if (target_pos_tilt < MinVal_tilt){
    target_pos_tilt == MinVal_tilt;}
   if (target_pos_tilt > MaxVal_tilt){
    target_pos_tilt == MaxVal_tilt;}
   
   
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

void tilt_drive()
{
  int gap = abs(tilt_Gap);
  if (gap < 50)
  {
  tilt.SetTunings(1, 0.1, 0);
  }
  else
  {
    tilt.SetTunings(5, 0, 0);
  }
  tilt.Compute();
  tilt.SetOutputLimits(-400 ,400);
  
    md.setM1Speed(tilt_pwm);
  }

void lift_drive()
{
  int gap = abs(lift_Gap);
  if (gap < 50)
  {
  lift.SetTunings(1, 0.1, 0);
  }
  else
  {
    lift.SetTunings(5, 0, 0);
  }
  lift.Compute();
  lift.SetOutputLimits(-250 ,250);
  
    md.setM2Speed(lift_pwm);
  }
  
int RequestData(int address)
{
  int val;
  Wire.requestFrom(address, 2);    // request 2 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
  val = Wire.read ();
  val <<= 8;
  val |= Wire.read ();
  int result = val;
  return result;
  }

}

// set servo target on slave servo
void SetTarget(int address, int target)
{
    long foo = target;
    Wire.beginTransmission (address);
    //I2C_writeAnything (enc);
    I2C_writeAnything (foo);
    Wire.endTransmission ();
       
}

void tilt_drive1()
{
  int gap = abs(tilt_Gap);
  if (gap < 50)
  {
  tilt.SetTunings(1, 0.1, 0);
  }
  else
  {
    tilt.SetTunings(5, 0, 0);
  }
  tilt.Compute();
  tilt.SetOutputLimits(-400 ,400);
  
    md.setM1Speed(tilt_pwm);
  }
  
void return_data()
{
  
  int pan_a = 2700;
  int tilt_a = 1800;
  int lift_a = 1800;
  int rot_a = 2700;
  int elbow_a = 1800;
  
  pan = RequestData(44);
  pan1 = map(pan, 0, 1023, 0, pan_a);
  rot = RequestData(42);
  rot1 = map(rot, 0, 1023, 0, rot_a);
  elbow = RequestData(40);
  elbow1 = map(elbow, 0, 1023, 0, elbow_a);
  
  
  Serial.print("P1");// joint message arm pan
  Serial.print("\t");
  Serial.print(pan1);//tilt);//current position
  Serial.print("\t");
  Serial.print(target_pos_pan);// joint target
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P2");// joint message arm tilt
  Serial.print("\t");
  Serial.print(angle_tilt);//tilt);//current position
  Serial.print("\t");
  Serial.print(target_pos_tilt);// joint target
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P3");// joint message arm lift
  Serial.print("\t");
  Serial.print(angle_lift);//lift);//current position
  Serial.print("\t");
  Serial.print(target_pos_lift);// joint target
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P4");// joint message arm rot
  Serial.print("\t");
  Serial.print(rot1);//rot);//current position
  Serial.print("\t");
  Serial.print(target_pos_rot);// joint target
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P5");// joint message arm elbow
  Serial.print("\t");
  Serial.print(elbow1);//elbow);//current position
  Serial.print("\t");
  Serial.print(target_pos_elbow);// joint target
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  


}



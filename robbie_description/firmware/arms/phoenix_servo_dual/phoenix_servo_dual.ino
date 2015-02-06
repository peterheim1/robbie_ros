/*
servo controller 
for hector mod to robbie
right arm
*/
#include <PID_v1.h>
#include <Wire.h>
#include "Arduino.h"
#include <Messenger.h>
#include <I2C_Anything.h>
#include "DualMotorShield.h"
/* Include definition of serial commands */

/* Serial port baud rate */
#define BAUDRATE     115200
DualMotorShield md;

// change to actuator
int MinVal_lift = 100;
int MaxVal_Lift = 1800;
int MinVal_tilt = 100;
int MaxVal_tilt = 1800;
double angle_lift ;
double angle_tilt ;
double target_pos_lift = 350;
double target_pos_tilt =512;
double target_pos_pan = 900;  
double target_pos_rot = 900; 
double target_pos_elbow = 900; 
const byte MY_ADDRESS = 50;
int target_angle_tilt = 0 ;


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

PID tilt(&Encoder_Position_tilt, &tilt_pwm, &target_pos_tilt, 4,0,0, DIRECT);
PID lift(&angle_lift, &lift_pwm, &target_pos_lift, 4,0,0, DIRECT);

// Instantiate Messenger object with the message function and the default separator (the space character)
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
  Wire.begin (MY_ADDRESS);
  Serial.begin (BAUDRATE);
  _Messenger.attach(OnMssageCompleted);
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

  ReadSerial();
  Encoder_Position_tilt =  analogRead(2);
  Encoder_Position_lift =  1023 - analogRead(3);
  angle_tilt  = map(Encoder_Position_tilt, 0, 1023, 0, 3000);
  angle_lift  = map(Encoder_Position_lift, 0, 1023, 0, 3000);
  tilt_Gap = (target_pos_tilt - angle_tilt)*0.1;
  lift_Gap = (target_pos_lift - angle_lift)*0.1;
  
  

  
  //pid timing loop
  milliSecsSinceLastUpdate = millis() - CurrentTime;
  if(milliSecsSinceLastUpdate >= FrameRate)
  {
  //md.setM2Speed(200);
  
  tilt_drive();
  lift_drive();
  return_data();
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
  /*int gap = abs(tilt_Gap);
  if (gap < 50)
  {
  tilt.SetTunings(1, 0.1, 0);
  }
  else
  {
    tilt.SetTunings(5, 0, 0);
  }*/
  tilt.Compute();
  tilt.SetOutputLimits(-400 ,400);
  
    md.setM1Speed(tilt_pwm);
  }

void lift_drive()
{
  /*int gap = abs(lift_Gap);
  if (gap < 50)
  {
  lift.SetTunings(1, 0.1, 0);
  }
  else
  {
    lift.SetTunings(5, 0, 0);
  }*/
  lift.Compute();
  lift.SetOutputLimits(-400 ,400);
  
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
  Serial.print(pan);// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P2");// joint message arm tilt
  Serial.print("\t");
  Serial.print(angle_tilt);//tilt);//current position
  Serial.print("\t");
  Serial.print(target_angle_tilt);// joint target
  Serial.print("\t");
  Serial.print(Encoder_Position_tilt);// velocity
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
  Serial.print(Encoder_Position_lift);// velocity
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
  Serial.print(rot);// velocity
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


void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
}

// Define messenger function
void OnMssageCompleted()
{
  if (_Messenger.checkString("j1"))
  {
    J1();
    
    
    return;
    
  }   
  
  if (_Messenger.checkString("j2"))
  {
    J2();
    
    
    return;
    
  }   
  
  if (_Messenger.checkString("j3"))
  {
    J3();
    
    
    return;
    
  }   
  
  if (_Messenger.checkString("j4"))
  {
    J4();
    
    
    return;
    
  }   
  
  if (_Messenger.checkString("j5"))
  {
    J5();
    
    
    return;
    
  }   
  

    
  

  
// clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}


  void J1()
{
  target_pos_pan = _Messenger.readInt(); 
  SetTarget(44, target_pos_pan);
}

void J2()
{
  target_pos_tilt = _Messenger.readInt(); 
  target_angle_tilt = map(target_pos_tilt, 0, 1023, 0, 3000);
  
}

void J3()
{
  target_pos_lift = _Messenger.readInt(); 
   
}

void J4()
{
  target_pos_rot = _Messenger.readInt(); 
  SetTarget(42, target_pos_rot); 
}

void J5()
{
  target_pos_elbow = _Messenger.readInt();
  SetTarget(40, target_pos_elbow); 
}



float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


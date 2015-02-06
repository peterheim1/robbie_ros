/*

todo
auto dock
pid needs to be faster
use PID from pi robot


*/
#define ROBOGAIA

#include <math.h>
#include <Servo.h>
#include <PID_v1.h>
#include "commands.h"
#include "encoder_driver.h"

#include <Messenger.h>
#include "Arduino.h"

#define PI 3.14159265
#define TwoPI 6.28318531
//TimeInfo _TimeInfo = TimeInfo();
unsigned long milliSecsSinceLastUpdate;
unsigned long CurrentTime = 0;
int FrameRate = 30;
  
float Voltage = 0;
float Amps = 0;

/* Serial port baud rate */
#define BAUDRATE  57600   


Servo LeftWheel;
Servo RightWheel;
double LeftPower =0;
double RightPower = 0;
               
int voltage = 0;                                // in mV
int current = 0;                                // in mA
long count_L = 0;                        // rev counter
long count_R = 0;                       // rev counter
//set speed in tickes per frame
double SpeedRight = 0;
double SpeedLeft = 0;
double SpeedRight_req = 0;
double SpeedLeft_req = 0;

int RearBumper = 2;
int RearBumperState = 0;

int FrontLeftIR = 2;
int FrontRightIR = 3;
float FrontRightDistance =0;
float FrontLeftDistance =0;


double DistancePerCount = 0.000062832;//.0001944;   pi*dia/encoder res
double RadiansPerCount = 0.000146121;//.000611320754717;  distpercount/trackwidth

double X;  // x coord in global frame
double Y;  // y coord in global frame
double Heading;  // heading (radians) in the global frame. The value lies in (-PI, PI]
	
double VLeft;   // left motor speed
double VRight;  // right motor speed
double V;  // forward speed
double Omega;  // angular speed (radians per sec)

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

bool _IsInitialized = false;
//arduino pid*******************************************************************************************************************PID  PID
PID Front_right(&SpeedRight, &RightPower, &SpeedRight_req, 0.2,0.5,0.01, DIRECT);
PID Front_left(&SpeedLeft, &LeftPower, &SpeedLeft_req, 0.2,0.5,0.01, DIRECT);


/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

                 Set up
                 
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

*/
void setup()
{
  //initialize the variables we're linked to
  Serial.begin(BAUDRATE);
  _Messenger.attach(OnMssageCompleted);
 
  pinMode(2, INPUT);
  
  
  //hb25
  LeftWheel.attach(7, 1000, 2000); //minPulse, maxPulse);//10
  RightWheel.attach(8, 1000, 2000);// minPulse, maxPulse);//11
  
 
  //arduino pid
  Front_right.SetMode(AUTOMATIC);
  Front_left.SetMode(AUTOMATIC);
  Front_right.SetOutputLimits(-90, 90);
  Front_left.SetOutputLimits(-90, 90); 
  
}

/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

timing loop
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

*/
void loop()
{
   
   ReadSerial();
   Voltage = (analogRead(0)*0.00488)*3.75;
   Amps = (analogRead(1)*0.0133);
   //FrontRightDistance = analogRead(FrontRightIR);
   //FrontLeftDistance = read_gp2d12_range(FrontLeftIR);
   RearBumperState = digitalRead(RearBumper);
   
   
   //timing loop
   milliSecsSinceLastUpdate = millis() - CurrentTime;
  if(milliSecsSinceLastUpdate >= FrameRate)
  {
  
    move_R();
    move_L();
    Pose();
    DoWork();

    CurrentTime = millis();
    
  }
}
/*
##################################################################################################3

                   End Timing loop
                   

##################################################################################################
*/
// from messenger

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
  if (_Messenger.checkString("s"))
  {
    SetSpeed();

    
    return;
    
  }   
  
  if (_Messenger.checkString("d"))
  {
    //InitializeDriveGeometry();
    return;
  }
  if (_Messenger.checkString("j"))
  {
    //SetSpeed();
    void(* resetFunc) (void) = 0; //declare reset function @ address 0
    resetFunc();
    return;
    
  }
  

 

  // clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}
 

 

float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void SetSpeed()
{

  
  SpeedLeft_req = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  SpeedRight_req = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
}

float read_gp2d12_range(byte pin) {
	int tmp;

	tmp = analogRead(pin);
	if (tmp < 3)
		return -1; // invalid value

	return (6787.0 /((float)tmp - 3.0)) - 4.0;

}



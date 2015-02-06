/*


Joints
Right_lift_target       motor
Right_tilt_target       Servo
Right_rotate_target     Servo
Right_elbow_target      Servo


Left_lift_target       motor
Left_tilt_target       Servo
Left_rotate_target     Servo
Left_elbow_target      Servo






*/


#include <math.h>
#include <Servo.h>

#include <Messenger.h>
#include "Arduino.h"
#include <Wire.h>
#include <I2C_Anything.h>

double Right_lift_enc  = 0;
//double Right_tilt_target  = 30;


//Servos on arm total 9
//Servo Right_Shoulder_tilt;
//Servo Right_Shoulder_Rotate;
Servo Right_Elbow_lift;
Servo Wrist_tilt;
Servo Thumb;
Servo Index;
Servo Middle;
Servo Ring;
Servo Pinky;

int Servo_pos= 90;

//set  hand to open

double Right_lift_target        = 550; //sholder lift  motor position
int Right_tilt_target        = 240;  // sholder tilt  motor position
int Right_rotate_target      = 140; //works
int Right_elbow_target       = 160;
int Wrist_tilt_target = 20;
int Thumb_target = 30;//works
int Index_target = 20;
int Middle_target = 20;
int Ring_target = 60;
int Pinky_target = 140;

//position controll
//int Right_tilt_position      = 0;
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
   Wire.begin();
   Serial.begin(115200);
  _Messenger.attach(OnMssageCompleted);
  analogReference(INTERNAL2V56); // use AREF for reference voltage

// Attach Servos
//Right_Shoulder_tilt.attach(49);//not realy working
//Right_Shoulder_Rotate.attach(53);
//Right_Elbow_lift.attach(37,1000, 1300);// need to set limits 0 80 for start
Wrist_tilt.attach(33);//no 2
Thumb.attach(29);//no 3
Index.attach(27);//ono 4
Middle.attach(31);//no 5
Ring.attach(35);//no 6
Pinky.attach(23);//no 7


}
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

//Right_Shoulder_Rotate.write(Right_rotate_target);
//Right_Elbow_lift.write(Right_elbow_target);
Wrist_tilt.write(Wrist_tilt_target);
Thumb.write(Thumb_target);
Index.write(Index_target);
Middle.write(Middle_target);
Ring.write(Ring_target);
Pinky.write(Pinky_target);

Ser_print();
 
}

void Ser_print()
{
  
  Serial.print("P1");// joint message arm tilt
  Serial.print("\t");
  Serial.print(RequestData(42));//tilt);//current position
  Serial.print("\t");
  Serial.print(Right_tilt_target);// joint target
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P2"); // joint message arm lift
  Serial.print("\t");
  Serial.print(RequestData(40));//current position
  Serial.print("\t");
  Serial.print(Right_lift_target);//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P3"); // joint message arm rotate
  Serial.print("\t");
  Serial.print(RequestData(44));//current position
  Serial.print("\t");
  Serial.print(Right_rotate_target);//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P4");// joint message arm elbow
  Serial.print("\t");
  Serial.print(RequestData(46));//current position
  Serial.print("\t");
  Serial.print(Right_elbow_target);//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P5"); // joint message wrist tilt
  Serial.print("\t");
  Serial.print(Wrist_tilt.read() );//current position
  Serial.print("\t");
  Serial.print(Wrist_tilt.read());//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P6");// joint message thumb
  Serial.print("\t");
  Serial.print(Thumb.read());//current position
  Serial.print("\t");
  Serial.print(Thumb.read());//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P7"); // joint message index
  Serial.print("\t");
  Serial.print(Index.read());//current position
  Serial.print("\t");
  Serial.print(Index.read());//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P8");// joint message middle
  Serial.print("\t");
  Serial.print(Middle.read());//current position
  Serial.print("\t");
  Serial.print(Middle.read());//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P9"); // joint message ring
  Serial.print("\t");
  Serial.print(Ring.read());//current position
  Serial.print("\t");
  Serial.print(Ring.read());//target
  Serial.print("\t");
  Serial.print("0");// current in amps
  Serial.print("\t");
  Serial.print("0");// velocity
  Serial.print("\t");
  Serial.print("0");// is the joint moving
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("P10");// joint message pinky
  Serial.print("\t");
  Serial.print(Pinky.read());//current position
  Serial.print("\t");
  Serial.print(Pinky.read());//target
  Serial.print("\t");
  Serial.print("0");// current in amps
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
  
  if (_Messenger.checkString("j6"))
  {
    J6();
    
    
    return;
    
  }   
  
  if (_Messenger.checkString("j7"))
  {
    J7();
    
    
    return;
    
  }   
  
  if (_Messenger.checkString("j8"))
  {
    J8();
    
    
    return;
    
  }  
 
 if (_Messenger.checkString("j9"))
  {
    J9();
    
    
    return;
    
  }   
 
 if (_Messenger.checkString("j10"))
  {
    J10();
    
    
    return;
    
  }    
  
    
  if (_Messenger.checkString("DriveGeometry"))
  {
    //InitializeDriveGeometry();
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
  Right_lift_target = _Messenger.readInt(); 
  SetTarget(40, Right_lift_target);
}

void J2()
{
  Right_tilt_target = _Messenger.readInt(); 
  SetTarget(42, Right_tilt_target); 
}

void J3()
{
  Right_rotate_target = _Messenger.readInt(); 
  SetTarget(44, Right_rotate_target); 
}

void J4()
{
  Right_elbow_target = _Messenger.readInt(); 
  SetTarget(46, Right_elbow_target); 
}

void J5()
{
  Wrist_tilt_target = _Messenger.readInt();  
}

void J6()
{
  Thumb_target = _Messenger.readInt();  
}

void J7()
{
  Index_target = _Messenger.readInt();  
}

void J8()
{
  Middle_target = _Messenger.readInt();  
}

void J9()
{
  Ring_target = _Messenger.readInt();  
}

void J10()
{
  Pinky_target = _Messenger.readInt();  
}

float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


///feedback
int getFeedback(int a){
int j;
int mean;
int result;
int result1;
int test;
int reading[20];
boolean done;

for (j=0; j<20; j++){
reading[j] = analogRead(a); //get raw data from Servo potentiometer
delay(1);
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
result1 = map(result , 0, 1023, 0, 179);
return(result1);
}    // END GET FEEDBACK

// set servo target on slave servo
void SetTarget(int address, int target)
{
    long foo = target;
    Wire.beginTransmission (address);
    //I2C_writeAnything (enc);
    I2C_writeAnything (foo);
    Wire.endTransmission ();
       
}
// request position from slave
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

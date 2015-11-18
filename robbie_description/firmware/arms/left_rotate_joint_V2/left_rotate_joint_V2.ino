

/* Serial port baud rate */
#define BAUDRATE     115200
#include "Arduino.h"
#include <Wire.h>
#include <I2C_Anything.h>
const byte MY_ADDRESS = 42;
int MinVal = 100;
int MaxVal = 980;
volatile long Target = 10;//why 550


/* Include definition of serial commands */
#include "commands.h"

int ADC_SetPoint = 400;
int ADC_SetPointOld = 0;
int ADC_ServoPoti = 0;
int ADC_ServoPotiOld = 0;
int dutyCycle = 70; // 10 - 255
int ADCdiff = 0;
int timeDiff = 0;
int mapped_Right_Lift;
int tmp;
int offset = 60;
int angle = 0;

//Change values below to adapt your motor
//Set MAX_DUTYCYCLE to 75 for the first test run!

#define P_FRACTION 2        //0.0 - 10.0 (0.3)
#define I_FRACTION 2.0         //0.0 - 10.0 (0.3)
#define D_FRACTION 4.0         //0.0 - 10.0 (4.0)
#define V_WINDOW 5            //10 - 1000 (25) stop tollerance
#define MIN_DUTYCYCLE 70       //0 - 255 (25)
#define MAX_DUTYCYCLE 200      //0 - 255 (255)
#define SOFT_START 0.3        //0.00 - 1.00 (0.30) 1.00 = OFF
#define D_FRACTION_DEMO 0      //1 - consider only D_FRACTION for servo movement (0 - OFF)
#define EMERGENCY_SHUTDOWN 4   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE 10 //Prevent H bridge from shoot through whenever the direction pin is changed

#define SERVO_DIRECTION_PIN_1 4 //Direction pin servo motor
#define SERVO_DIRECTION_PIN_2 5 //Direction pin servo motor
#define SERVO_PWM_PIN 3       //PWM pin servo motor
#define SERVO_SENSOR A0       //Analog input servo sensor      



/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index1 = 0;

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
  index1 = 0;
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
  case RIGHT_TILT://a
    Serial.println(mapped_Right_Lift);
    
    break;
  case LEFT_TILT://b
    //tmp =map(arg1, , 300, 0, 1023);
    //Serial.println(tmp);
    
    break;
  case RIGHT_LIFT://c
    //arg1= arg1+offset;
    arg2 = map(arg1, 130, -130, 0, 1023)+ offset;
    Serial.println(arg2);
    ADC_SetPoint = arg2;
    
    break;
  case LEFT_LIFT://d
    Serial.print("p1");// joint message left_tilt
    Serial.print("\t");
    Serial.print(mapped_Right_Lift);//tilt);//current position
    Serial.print("\t");
    Serial.print(angle);// joint target input
    Serial.print("\t");
    Serial.print(ADC_SetPoint);// setpoint in ticks
    Serial.print("\t");
    Serial.print("pan");// is the joint moving
    Serial.print("\t");
    Serial.print("\n");
     
    break;
  case LEFT_ELBOW:
    Serial.println(arg1);
    
    break;
  

 
    

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent); //recieve target from master
  Wire.onRequest(requestEvent); // send data to master
  pinMode(SERVO_DIRECTION_PIN_1, OUTPUT);  
  pinMode(SERVO_DIRECTION_PIN_2, OUTPUT);   
  pinMode(SERVO_PWM_PIN, OUTPUT);     

  pinMode(SERVO_SENSOR, INPUT);     
     

}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  ADC_ServoPoti = analogRead(SERVO_SENSOR);     // reads the servo sensor (between 0 and 1023) 
  mapped_Right_Lift = map((ADC_ServoPoti-offset), 0, 1023, 130, -130);
  right_motor();
  Ser_print();
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index1] = NULL;
      else if (arg == 2) argv2[index1] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index1] = NULL;
        arg = 2;
        index1 = 0;
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
        argv1[index1] = chr;
        index1++;
      }
      else if (arg == 2) {
        argv2[index1] = chr;
        index1++;
      }
    }
  }
  

}


int right_motor(){
  ADCdiff = ADC_SetPoint - ADC_ServoPoti;
  
  dutyCycle = abs(ADCdiff) * P_FRACTION;
  dutyCycle += timeDiff * I_FRACTION;
  dutyCycle += abs(ADC_SetPointOld - ADC_SetPoint) * D_FRACTION;
  
  if(D_FRACTION_DEMO == 1){
    dutyCycle = abs(ADC_SetPointOld - ADC_SetPoint) * D_FRACTION;
  }
  
  if(SOFT_START * timeDiff < 1){
    dutyCycle = dutyCycle * (SOFT_START * timeDiff);
  }
  
  timeDiff++;
  
  if(dutyCycle < MIN_DUTYCYCLE && dutyCycle > 0){
    dutyCycle = MIN_DUTYCYCLE;
  }
  
  if(dutyCycle > MAX_DUTYCYCLE){
    dutyCycle = MAX_DUTYCYCLE;
  }
  
  if(dutyCycle < 0){
    dutyCycle = 0;
  }
  
  
  
  if(D_FRACTION_DEMO == 1){
    if(abs(ADC_SetPointOld - ADC_SetPoint) < 2){
      analogWrite(SERVO_PWM_PIN, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE);
      digitalWrite(SERVO_DIRECTION_PIN_1, 0);
      digitalWrite(SERVO_DIRECTION_PIN_2, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE);
    }
    else{
      if(ADC_SetPointOld - ADC_SetPoint < 0){
        analogWrite(SERVO_PWM_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        digitalWrite(SERVO_DIRECTION_PIN_1, 0);//forward
        digitalWrite(SERVO_DIRECTION_PIN_2, 1);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(SERVO_PWM_PIN, dutyCycle);
      }
      if(ADC_SetPointOld - ADC_SetPoint > 0){
        analogWrite(SERVO_PWM_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        digitalWrite(SERVO_DIRECTION_PIN_1, 1);// reverse
        digitalWrite(SERVO_DIRECTION_PIN_2, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(SERVO_PWM_PIN, dutyCycle);
      }
    }
  }
  else{
    if(abs(ADCdiff) < V_WINDOW){
      dutyCycle = 0;
      timeDiff = 0;
    }

    if(abs(ADC_ServoPotiOld - ADC_ServoPoti) < EMERGENCY_SHUTDOWN && dutyCycle == MAX_DUTYCYCLE && timeDiff > 50){
      analogWrite(SERVO_PWM_PIN, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE);
      digitalWrite(SERVO_DIRECTION_PIN_1, 0);
      digitalWrite(SERVO_DIRECTION_PIN_2, 0);//stop
      delay(1000);
      timeDiff = 0;
    }
    else{  
      if(ADCdiff > 0){
        analogWrite(SERVO_PWM_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        digitalWrite(SERVO_DIRECTION_PIN_1, 0);
        digitalWrite(SERVO_DIRECTION_PIN_2, 1);//forward
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(SERVO_PWM_PIN, dutyCycle);
      }
      if(ADCdiff < 0){
        analogWrite(SERVO_PWM_PIN, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        digitalWrite(SERVO_DIRECTION_PIN_1, 1);
        digitalWrite(SERVO_DIRECTION_PIN_2, 0);//reverse
        delayMicroseconds(SHOOT_THROUGH_PAUSE);
        analogWrite(SERVO_PWM_PIN, dutyCycle);
      }
    }
  }
  

  ADC_SetPointOld = ADC_SetPoint;
  ADC_ServoPotiOld = ADC_ServoPoti;


 


  delay(60);                             // waits for the servo to get there 
}

void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof Target))
   {
     
   I2C_readAnything (Target); 
   angle = Target;
   Target = map(Target, 130, -130, 0, 1023)+ offset;
   if (Target < MinVal){
    Target = MinVal;}
   if (Target > MaxVal){
    Target = MaxVal;}
   ADC_SetPoint = Target;
   
   
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



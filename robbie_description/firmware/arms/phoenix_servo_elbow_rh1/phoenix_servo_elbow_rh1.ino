//Wiper motor software v 1.3:
//Servo sensor to Analog 0
//Setpoint potentiometer to Analog 1
//H bridge direction pin to Digital 2
//H bridge PWM signal to Digital 3
//LEDs at digital pins 4 - 13 are optional (indicating setpoint as LED bar)
//
//For the first test run, set MAX_DUTYCYCLE to 75 in order to lower the maximum torque in case something goes wrong.
//When connecting the servo to the microcontroller for the first time, you have to consider the rotational direction of your wiper motor!
//Adjust the potentiometer defining the setpoint to the center position.
//If the servo moves to neutral position, too after connecting the circuit to the supply voltage, the polarity of the servo sensor is correct.
//If the wiper motor starts spinning away from the center position, the polarity of the servo sensor (+ and - connector) has to be swapped.
//
//
//Source & info: www.HomoFaciens.de/technics-computer-arduino-uno_en_navion.htm


//#include <avr/io.h>
//#include <util/delay.h>
#include <Wire.h>
#include <I2C_Anything.h>

int ADC_SetPoint = 430;
int ADC_SetPointOld = 0;
int ADC_ServoPoti = 0;
int ADC_ServoPotiOld = 0;
int dutyCycle = 90; // 10 - 255
int ADCdiff = 0;
int timeDiff = 0;
const byte MY_ADDRESS = 40;
int MinVal = 430;
int MaxVal = 1000;
volatile long Target = 10;//why 550
int angle_pub;
//Change values below to adapt your motor
//Set MAX_DUTYCYCLE to 75 for the first test run!

#define P_FRACTION 3        //0.0 - 10.0 (0.3)
#define I_FRACTION 2.0         //0.0 - 10.0 (0.3)
#define D_FRACTION 4.0         //0.0 - 10.0 (4.0)
#define V_WINDOW 17            //10 - 1000 (25) stop tollerance
#define MIN_DUTYCYCLE 70       //0 - 255 (25)
#define MAX_DUTYCYCLE 240      //0 - 255 (255)
#define SOFT_START 0.3        //0.00 - 1.00 (0.30) 1.00 = OFF
#define D_FRACTION_DEMO 0      //1 - consider only D_FRACTION for servo movement (0 - OFF)
#define EMERGENCY_SHUTDOWN 0   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE 10 //Prevent H bridge from shoot through whenever the direction pin is changed

#define SERVO_DIRECTION_PIN_1 4 //Direction pin servo motor
#define SERVO_DIRECTION_PIN_2 5 //Direction pin servo motor
#define SERVO_PWM_PIN 9       //PWM pin servo motor
#define SERVO_SENSOR A0       //Analog input servo sensor
#define POTI A1               //Analog input potentiometer



void setup(){
  Serial.begin (115200);
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent); //recieve target from master
  Wire.onRequest(requestEvent); // send data to master
  pinMode(SERVO_DIRECTION_PIN_1, OUTPUT);  
  pinMode(SERVO_DIRECTION_PIN_2, OUTPUT);   
  pinMode(SERVO_PWM_PIN, OUTPUT);     

  pinMode(SERVO_SENSOR, INPUT);     
  pinMode(POTI, INPUT);     

  
  
} 

 
void loop(){ 
  ADC_ServoPoti = analogRead(0);     // reads the servo sensor (between 0 and 1023) 
  angle_pub = map(ADC_ServoPoti, 0, 1023, 0, 2750);
  Serial.print(MY_ADDRESS);
  Serial.print(" ");
  Serial.print(ADC_SetPoint);
  Serial.print(" ");
  Serial.print(angle_pub);
  Serial.print(" ");
  Serial.print(ADC_SetPoint);
  Serial.print(" ");
  Serial.print("  motor     ");  
  Serial.print(ADC_ServoPoti); 
  Serial.print("  input  "); 
  Serial.println(ADC_SetPoint); 
  
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


 


  delay(30);                             // waits for the servo to get there 

}

void receiveEvent (int howMany)
 {
 if (howMany >= (sizeof Target))
   {
     
   I2C_readAnything (Target); 
    
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



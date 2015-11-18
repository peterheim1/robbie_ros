

/* Serial port baud rate */
#define BAUDRATE     115200
#include "Arduino.h"
#include <Servo.h>
/* Include definition of serial commands */
#include "commands.h"

Servo right;  
Servo left;
Servo left_elbow;
int Encoder_Right_Lift;
int Encoder_Left_Lift;
int Left_Tilt_raw;
int Right_Tilt_raw;
int right_tilt_target = 82;
int left_tilt_target= 81;
int left_elbow_target= 28;


int Right_tilt_MaxVal = 180;// in degrees
int Right_tilt_MinVal = 80;// in degrees
int Right_tilt_offset = 80;// differance betweeen requested and actual
int Right_tilt_mapped ;
int Right_tilt1 = 81;

int Left_tilt_MaxVal = 180;// in degrees
int Left_tilt_MinVal = 80;// in degrees
int Left_tilt_offset = 80;// differance betweeen requested and actual
int Left_tilt_mapped ;


int Left_elbow_MaxVal = 170;// in degrees
int Left_elbow_MinVal = 10;// in degrees
int Left_elbow_offset = 28;// differance betweeen requested and actual
int mapped_left_elbow;


//Right motor
int ADC_SetPoint_right = 285;// at ros 1.57
int ADC_SetPointOld_right = 0;
int ADC_ServoPoti_right = 0;
int ADC_ServoPotiOld_right = 0;
int dutyCycle_right = 50; // 10 - 255
int ADCdiff_right = 0;
int timeDiff_right = 0;
int mapped_Right_Lift;
int right_lift_target = 0;
int tmp;
int Right_MaxVal = 1000;// in degrees
int Right_MinVal = 200;// in degrees
int Right_offset = 285;// differance betweeen requested and actual
// Left motor
int ADC_SetPoint_left = 376;// at ros 1.57
int ADC_SetPointOld_left = 0;
int ADC_ServoPoti_left = 0;
int ADC_ServoPotiOld_left = 0;
int dutyCycle_left = 50; // 10 - 255
int ADCdiff_left = 0;
int timeDiff_left = 0;
int mapped_Left_Lift;
int left_lift_target = 0;
int Left_MaxVal = 1000;// in degrees
int Left_MinVal = 200;// in degrees
int Left_offset = 376;// differance betweeen requested and actual


//Change values below to adapt your motor
//Set MAX_dutyCycle_right to 75 for the first test run!
// right motor
#define P_FRACTION_right 2        //0.0 - 10.0 (0.3)
#define I_FRACTION_right 0.3//2.0         //0.0 - 10.0 (0.3)
#define D_FRACTION_right 5//4.0         //0.0 - 10.0 (4.0)
#define V_WINDOW_right 10            //10 - 1000 (25) stop tollerance
#define MIN_dutyCycle_right 50       //0 - 255 (25)
#define MAX_dutyCycle_right 200      //0 - 255 (255)
#define SOFT_START_right 0.3        //0.00 - 1.00 (0.30) 1.00 = OFF
#define D_FRACTION_DEMO_right 0      //1 - consider only D_FRACTION for servo movement (0 - OFF)
#define EMERGENCY_SHUTDOWN_right 4   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE_right 10 //Prevent H bridge from shoot through whenever the direction pin is changed

#define SERVO_DIRECTION_PIN_1_right 7 //Direction pin servo motor
#define SERVO_DIRECTION_PIN_2_right 8 //Direction pin servo motor
#define SERVO_PWM_PIN_right 10       //PWM pin servo motor
#define SERVO_SENSOR_right A2       //Analog input servo sensor
#define POTI A1               //Analog input potentiometer  NOT USED

//Left Motor
#define P_FRACTION_left 1.5        //0.0 - 10.0 (0.3)
#define I_FRACTION_left 0.3//2.0         //0.0 - 10.0 (0.3)
#define D_FRACTION_left 5//4.0         //0.0 - 10.0 (4.0)
#define V_WINDOW_left 10            //10 - 1000 (25) stop tollerance
#define MIN_dutyCycle_left 50       //0 - 255 (25)
#define MAX_dutyCycle_left 200      //0 - 255 (255)
#define SOFT_START_left 0.3        //0.00 - 1.00 (0.30) 1.00 = OFF
#define D_FRACTION_DEMO_left 0      //1 - consider only D_FRACTION for servo movement (0 - OFF)
#define EMERGENCY_SHUTDOWN_left 4   //0 - 100 (4), 0 - OFF, Stops motor if blocked
#define SHOOT_THROUGH_PAUSE_left 10 //Prevent H bridge from shoot through whenever the direction pin is changed

#define SERVO_DIRECTION_PIN_1_left 2 //Direction pin servo motor
#define SERVO_DIRECTION_PIN_2_left 4 //Direction pin servo motor
#define SERVO_PWM_PIN_left 9       //PWM pin servo motor
#define SERVO_SENSOR_left A3       //Analog input servo sensor


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
    arg2 = (arg1*1.5) + 80;
    if (arg2 < Right_tilt_MinVal){
      arg2 = Right_tilt_MinVal;}
    if (arg2 > Right_tilt_MaxVal){
      arg2 = Right_tilt_MaxVal;}
    right_tilt_target = 0 - arg1;
    //Serial.println(right_tilt_target);
    Right_tilt1 = arg2;
    right.write(Right_tilt1);
    
    break;
  case LEFT_TILT://b
    arg2 = (arg1*1.4) + 80;
    if (arg2 < Left_tilt_MinVal){
    arg2 = Left_tilt_MinVal;}
    if (arg2 > Left_tilt_MaxVal){
    arg2 = Left_tilt_MaxVal;}
    //Serial.println(arg2);
    left_tilt_target = 0 - arg1;
    left.write(arg2);
    
    break;
  case RIGHT_LIFT://c
    right_lift_target = arg1;
    arg2 = map(arg1, 0, 260, 0, 1023)+ Right_offset;
    if (arg2 < Right_MinVal){
    arg2 = Right_MinVal;}
   if (arg2 > Right_MaxVal){
    arg2 = Right_MaxVal;}
    
    //Serial.println(arg2);//in ticks
    ADC_SetPoint_right = arg2;
    
    break;
  case LEFT_LIFT://d
    left_lift_target=arg1;
    arg2 = map(arg1, 0, 260, 0, 1023)+ Left_offset;
    if (arg2 < Left_MinVal){
    arg2 = Left_MinVal;}
   if (arg2 > Left_MaxVal){
    arg2 = Left_MaxVal;}
    
    //Serial.println(arg2);//in ticks
    ADC_SetPoint_left = arg2;
     
    break;
  case LEFT_ELBOW://e
    arg2 = (arg1*1.2) + 28;
    if (arg2 < Left_elbow_MinVal){
    arg1 = Left_elbow_MinVal;}
    if (arg2 > Left_elbow_MaxVal){
    arg2 = Left_elbow_MaxVal;}
    left_elbow_target = arg2;
    //Serial.println(left_elbow_target);
    //Serial.println(arg2);
    left_elbow.write(arg2);
    
    break;
    
  case MAPPED://m
    Serial.print(mapped_left_elbow);
    Serial.print("   ");
    Serial.print(left_elbow_target);
    Serial.print("   ");
    Serial.println(analogRead(A10));
    break;
    
  case RAW://r
  
    Serial.print(analogRead(A8));
    Serial.print("   ");
    Serial.print(analogRead(A9));
    Serial.print("   ");
    Serial.print(analogRead(A10));
    Serial.print("   ");
    Serial.print(Right_tilt_mapped);
    Serial.print("   ");
    Serial.println(Right_Tilt_raw);
    break;
  

 
    

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  pinMode(SERVO_DIRECTION_PIN_1_right, OUTPUT);  
  pinMode(SERVO_DIRECTION_PIN_2_right, OUTPUT);   
  pinMode(SERVO_PWM_PIN_right, OUTPUT);
  
  pinMode(SERVO_DIRECTION_PIN_1_left, OUTPUT);  
  pinMode(SERVO_DIRECTION_PIN_2_left, OUTPUT);   
  pinMode(SERVO_PWM_PIN_left, OUTPUT);     

  pinMode(SERVO_SENSOR_left, INPUT);      

  pinMode(SERVO_SENSOR_right, INPUT);     
  pinMode(POTI, INPUT);     
  
  left.attach(22);  // attaches the servo on pin 9 to the servo object
  right.attach(23);
  left_elbow.attach(24);
//move to safe start position
  left_elbow.write(left_elbow_target);
  left.write(left_tilt_target);
  right.write(right_tilt_target);

}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  Encoder_Right_Lift = analogRead(A2);
  Encoder_Left_Lift = analogRead(A3);
  
  ADC_ServoPoti_right = analogRead(SERVO_SENSOR_right);     // reads the servo sensor (between 0 and 1023) 
  mapped_Right_Lift = 0-(map((ADC_ServoPoti_right - Right_offset), 0, 1023, 0, 2600))+900;
  
  ADC_ServoPoti_left = analogRead(SERVO_SENSOR_left);     // reads the servo sensor (between 0 and 1023) 
  mapped_Left_Lift = 0-(map((ADC_ServoPoti_left - Left_offset), 0, 1023, 0, 2600))+900;
  
  mapped_left_elbow = map((analogRead(A10)-298), 0, 1023, 0, 2000);
  //if (mapped_left_elbow < 0){mapped_left_elbow = 0;}
  
  Left_Tilt_raw = (analogRead(A9)-525);///5;//map(analogRead(A11), 0, 1023, 0, 260)-127;
  if (Left_Tilt_raw < 4 ){Left_Tilt_raw = 0;}
  Left_tilt_mapped = 0 - Left_Tilt_raw;
  
  Right_Tilt_raw = (analogRead(A8)-525);///5;//map(analogRead(A11), 0, 1023, 0, 260)-127;
  //if (Right_Tilt_raw >= 3 && Right_Tilt_raw < 40){Right_Tilt_raw = Right_Tilt_raw + 3;}
  if (Right_Tilt_raw < 4 ){Right_Tilt_raw = 0;}
  Right_tilt_mapped = 0 - Right_Tilt_raw ;
  right.write(Right_tilt1);
  right_motor();
  left_motor();
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
  ADCdiff_right = ADC_SetPoint_right - ADC_ServoPoti_right;
  
  dutyCycle_right = abs(ADCdiff_right) * P_FRACTION_right;
  dutyCycle_right += timeDiff_right * I_FRACTION_right;
  dutyCycle_right += abs(ADC_SetPointOld_right - ADC_SetPoint_right) * D_FRACTION_right;
  
  if(D_FRACTION_DEMO_right == 1){
    dutyCycle_right = abs(ADC_SetPointOld_right - ADC_SetPoint_right) * D_FRACTION_right;
  }
  
  if(SOFT_START_right * timeDiff_right < 1){
    dutyCycle_right = dutyCycle_right * (SOFT_START_right * timeDiff_right);
  }
  
  timeDiff_right++;
  
  if(dutyCycle_right < MIN_dutyCycle_right && dutyCycle_right > 0){
    dutyCycle_right = MIN_dutyCycle_right;
  }
  
  if(dutyCycle_right > MAX_dutyCycle_right){
    dutyCycle_right = MAX_dutyCycle_right;
  }
  
  if(dutyCycle_right < 0){
    dutyCycle_right = 0;
  }
  
  
  
  if(D_FRACTION_DEMO_right == 1){
    if(abs(ADC_SetPointOld_right - ADC_SetPoint_right) < 2){
      analogWrite(SERVO_PWM_PIN_right, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
      digitalWrite(SERVO_DIRECTION_PIN_1_right, 0);
      digitalWrite(SERVO_DIRECTION_PIN_2_right, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
    }
    else{
      if(ADC_SetPointOld_right - ADC_SetPoint_right < 0){
        analogWrite(SERVO_PWM_PIN_right, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        digitalWrite(SERVO_DIRECTION_PIN_1_right, 0);//forward
        digitalWrite(SERVO_DIRECTION_PIN_2_right, 1);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        analogWrite(SERVO_PWM_PIN_right, dutyCycle_right);
      }
      if(ADC_SetPointOld_right - ADC_SetPoint_right > 0){
        analogWrite(SERVO_PWM_PIN_right, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        digitalWrite(SERVO_DIRECTION_PIN_1_right, 1);// reverse
        digitalWrite(SERVO_DIRECTION_PIN_2_right, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        analogWrite(SERVO_PWM_PIN_right, dutyCycle_right);
      }
    }
  }
  else{
    if(abs(ADCdiff_right) < V_WINDOW_right){
      dutyCycle_right = 0;
      timeDiff_right = 0;
    }

    if(abs(ADC_ServoPotiOld_right - ADC_ServoPoti_right) < EMERGENCY_SHUTDOWN_right && dutyCycle_right == MAX_dutyCycle_right && timeDiff_right > 50){
      analogWrite(SERVO_PWM_PIN_right, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
      digitalWrite(SERVO_DIRECTION_PIN_1_right, 0);
      digitalWrite(SERVO_DIRECTION_PIN_2_right, 0);//stop
      delay(1000);
      timeDiff_right = 0;
    }
    else{  
      if(ADCdiff_right > 0){
        analogWrite(SERVO_PWM_PIN_right, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        digitalWrite(SERVO_DIRECTION_PIN_1_right, 0);
        digitalWrite(SERVO_DIRECTION_PIN_2_right, 1);//forward
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        analogWrite(SERVO_PWM_PIN_right, dutyCycle_right);
      }
      if(ADCdiff_right < 0){
        analogWrite(SERVO_PWM_PIN_right, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        digitalWrite(SERVO_DIRECTION_PIN_1_right, 1);
        digitalWrite(SERVO_DIRECTION_PIN_2_right, 0);//reverse
        delayMicroseconds(SHOOT_THROUGH_PAUSE_right);
        analogWrite(SERVO_PWM_PIN_right, dutyCycle_right);
      }
    }
  }
  

  ADC_SetPointOld_right = ADC_SetPoint_right;
  ADC_ServoPotiOld_right = ADC_ServoPoti_right;


 


  delay(60);                             // waits for the servo to get there 
}

int left_motor(){
  ADCdiff_left = ADC_SetPoint_left - ADC_ServoPoti_left;
  
  dutyCycle_left = abs(ADCdiff_left) * P_FRACTION_left;
  dutyCycle_left += timeDiff_left * I_FRACTION_left;
  dutyCycle_left += abs(ADC_SetPointOld_left - ADC_SetPoint_left) * D_FRACTION_left;
  
  if(D_FRACTION_DEMO_left == 1){
    dutyCycle_left = abs(ADC_SetPointOld_left - ADC_SetPoint_left) * D_FRACTION_left;
  }
  
  if(SOFT_START_left * timeDiff_left < 1){
    dutyCycle_left = dutyCycle_left * (SOFT_START_left * timeDiff_left);
  }
  
  timeDiff_left++;
  
  if(dutyCycle_left < MIN_dutyCycle_left && dutyCycle_left > 0){
    dutyCycle_left = MIN_dutyCycle_left;
  }
  
  if(dutyCycle_left > MAX_dutyCycle_left){
    dutyCycle_left = MAX_dutyCycle_left;
  }
  
  if(dutyCycle_left < 0){
    dutyCycle_left = 0;
  }
  
  
  
  if(D_FRACTION_DEMO_left == 1){
    if(abs(ADC_SetPointOld_left - ADC_SetPoint_left) < 2){
      analogWrite(SERVO_PWM_PIN_left, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
      digitalWrite(SERVO_DIRECTION_PIN_1_left, 0);
      digitalWrite(SERVO_DIRECTION_PIN_2_left, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
    }
    else{
      if(ADC_SetPointOld_left - ADC_SetPoint_left < 0){
        analogWrite(SERVO_PWM_PIN_left, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        digitalWrite(SERVO_DIRECTION_PIN_1_left, 0);//forward
        digitalWrite(SERVO_DIRECTION_PIN_2_left, 1);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        analogWrite(SERVO_PWM_PIN_left, dutyCycle_left);
      }
      if(ADC_SetPointOld_left - ADC_SetPoint_left > 0){
        analogWrite(SERVO_PWM_PIN_left, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        digitalWrite(SERVO_DIRECTION_PIN_1_left, 1);// reverse
        digitalWrite(SERVO_DIRECTION_PIN_2_left, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        analogWrite(SERVO_PWM_PIN_left, dutyCycle_left);
      }
    }
  }
  else{
    if(abs(ADCdiff_left) < V_WINDOW_left){
      dutyCycle_left = 0;
      timeDiff_left = 0;
    }

    if(abs(ADC_ServoPotiOld_left - ADC_ServoPoti_left) < EMERGENCY_SHUTDOWN_left && dutyCycle_left == MAX_dutyCycle_left && timeDiff_left > 50){
      analogWrite(SERVO_PWM_PIN_left, 0);
      delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
      digitalWrite(SERVO_DIRECTION_PIN_1_left, 0);
      digitalWrite(SERVO_DIRECTION_PIN_2_left, 0);//stop
      delay(1000);
      timeDiff_left = 0;
    }
    else{  
      if(ADCdiff_left > 0){
        analogWrite(SERVO_PWM_PIN_left, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        digitalWrite(SERVO_DIRECTION_PIN_1_left, 0);
        digitalWrite(SERVO_DIRECTION_PIN_2_left, 1);//forward
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        analogWrite(SERVO_PWM_PIN_left, dutyCycle_left);
      }
      if(ADCdiff_left < 0){
        analogWrite(SERVO_PWM_PIN_left, 0);
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        digitalWrite(SERVO_DIRECTION_PIN_1_left, 1);
        digitalWrite(SERVO_DIRECTION_PIN_2_left, 0);//reverse
        delayMicroseconds(SHOOT_THROUGH_PAUSE_left);
        analogWrite(SERVO_PWM_PIN_left, dutyCycle_left);
      }
    }
  }
  

  ADC_SetPointOld_left = ADC_SetPoint_left;
  ADC_ServoPotiOld_left = ADC_ServoPoti_left;


 


  delay(60);                             // waits for the servo to get there 
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






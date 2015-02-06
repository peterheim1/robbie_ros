void getMotorData_L()  {                                                        // calculate speed, 
static long countAnt_L = 0;                                                   // last count
  SpeedLeft = (readEncoder(LEFT) - countAnt_L);//*(60*(1000/c_UpdateInterval)))/(2480);              // 2500 counts per output shaft rev
  countAnt_L = readEncoder(LEFT);                  
 
}

void getMotorData_R()  {                                                        // calculate speed, 
static long countAnt_R = 0;                                                   // last count
  SpeedRight = (readEncoder(RIGHT) - countAnt_R);//*(60*(1000/c_UpdateInterval)))/(2480);              // 2500 counts per output shaft rev
  countAnt_R = readEncoder(RIGHT);                  
  
}



void move_R(){
    
    getMotorData_R();
    Front_right.Compute();
    //Front_right.SetOutputLimits(0, 80); 
    if (RightPower >= -5 && RightPower <= 5){RightPower = 0;}
    RightWheel.write(90 + RightPower); 
}

void move_L(){
    
    getMotorData_L();
    Front_left.Compute();
    //Front_left.SetOutputLimits(-90, 90); 
    if (LeftPower >= -5 && LeftPower <= 5){LeftPower = 0;}
    LeftWheel.write(90 + LeftPower); 
}

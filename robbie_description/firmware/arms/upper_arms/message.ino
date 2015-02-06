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
 
  
// clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}


  void J1()
{
  
  //Right lift target
  int tar = 512;
  tar = _Messenger.readInt(); 
  if (tar < R_L_min){ tar = R_L_min;}
  if (tar > R_L_max){ tar = R_L_max;}
  right_lift_target = tar; 
}

void J2()
{
  //Right tilt target
  int tar = 83;
  tar = _Messenger.readInt(); 
  if (tar < r_min){ tar = r_min;}
  if (tar > r_max){ tar = r_max;}
  right_tilt_target = tar;
  right.write(tar); 
  delay(1);
  
}

void J3()
{
  //Right rotate target
  int tar = 512;
  tar = _Messenger.readInt(); 
  if (tar < R_L_min){ tar = R_L_min;}
  if (tar > R_L_max){ tar = R_L_max;}
  right_rotate_target = tar;
  SetTarget(44, tar); 
}

void J4()
{
  //Right_elbow_target 
  int tar = 83;
  tar = _Messenger.readInt();  
  if (tar < R_L_min){ tar = R_L_min;}
  if (tar > R_L_max){ tar = R_L_max;}
  right_elbow_target = tar;
  SetTarget(46, tar);
}
void J5()
{
  
  //Left lift target
  int tar = 512;
  tar = _Messenger.readInt();  
  if (tar < L_L_min){ tar = L_L_min;}
  if (tar > L_L_max){ tar = L_L_max;}
  left_lift_target = tar; 
}

void J6()
{
  //Left tilt target
  int tar = 83;
  tar = _Messenger.readInt();  
  if (tar < l_min){ tar = l_min;}
  if (tar > l_max){ tar = l_max;}
  left_tilt_target = tar;
  left.write(tar); 
  delay(1); 
  
}

void J7()
{
  //Left rotate target
  int tar = 512;
  tar = _Messenger.readInt(); 
  if (tar < L_L_min){ tar = L_L_min;}
  if (tar > L_L_max){ tar = L_L_max;}
  left_rotate_target = tar;
  SetTarget(42, tar); 
}

void J8()
{
  //Left_elbow_target
  int tar = 61;
  tar = _Messenger.readInt();  
  if (tar < 61){ tar = 61;}
  if (tar > 167){ tar = 167;}
  left_elbow_target = tar;
  left_elbow.write(tar); 
  delay(1);
}
float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

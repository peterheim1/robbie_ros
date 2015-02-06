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


 
void J5()
{
  
  //right rotate
  int tar = _Messenger.readInt(); 
  if (tar < L_L_min){ tar = L_L_min;}
  if (tar > L_L_max){ tar = L_L_max;}
  right_rotate_target = tar;
  SetTarget(44, tar);  
}

void J6()
{
  //right elbow target
  int tar = _Messenger.readInt(); 
  if (tar < 100){ tar = 100;}
  if (tar > 980){ tar = 980;}
  right_elbow_target = tar;
  SetTarget(40, tar);
  
}

void J7()
{
  //Left rotate target
  int tar = _Messenger.readInt(); 
  //if (tar < L_L_min){ tar = L_L_min;}
  //if (tar > L_L_max){ tar = L_L_max;}
  int Targ = map(tar, 0, 2700, 0, 1023);
  left_rotate_target = Targ;
  SetTarget(42, Targ); 
}

void J8()
{
  //Left_elbow_target
  int tar = _Messenger.readInt(); 
  if (tar < L_L_min){ tar = L_L_min;}
  if (tar > L_L_max){ tar = L_L_max;}
  //left_elbow_target = tar;
}
float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

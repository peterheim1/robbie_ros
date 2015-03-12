void DoWork()
{

  Serial.print("o"); // o indicates odometry message
  Serial.print("\t");
  Serial.print(X, 3);
  Serial.print("\t");
  Serial.print(Y, 3);
  Serial.print("\t");
  Serial.print(Heading, 3);
  Serial.print("\t");
  Serial.print(V);//vel x
  Serial.print("\t");
  Serial.print(Omega);//vel y
  Serial.print("\t");
  Serial.print(readEncoder(RIGHT));//vel in z in radians
  Serial.print("\t");
  Serial.print(readEncoder(LEFT));
  Serial.print("\t");
  Serial.print(SpeedRight_req);
  Serial.print("\t");
  Serial.print(SpeedLeft_req);
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("m"); // o indicates battery message
  Serial.print("\t");
  Serial.print(SpeedRight);
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("n"); // o indicates battery message
  Serial.print("\t");
  Serial.print(SpeedLeft);
  Serial.print("\t");
  Serial.print("\n");
 
  
  Serial.print("b"); // o indicates battery message
  Serial.print("\t");
  Serial.print(Voltage);
  Serial.print("\t");
  Serial.print(Amps);
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("d"); // o indicates docking message
  Serial.print("\t");
  Serial.print(Right_Ir_State);
  Serial.print("\t");
  Serial.print(Left_Ir_State);
  Serial.print("\t");
  Serial.print(Rear_Bumper_State);
  Serial.print("\t");
  Serial.print(Auto_Dock_Cmd);
  Serial.print("\t");
  Serial.print("\n");
  
 
}

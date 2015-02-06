void Ser_print1()
{

Serial.print("p1");// joint message left_tilt
Serial.print("\t");
Serial.print(getFeedback(9));//tilt);//current position
Serial.print("\t");
Serial.print(left_tilt_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("left_tilt");// is the joint moving
Serial.print("\t");
Serial.print("\n");

Serial.print("p2");// joint message right_tilt
Serial.print("\t");
Serial.print(getFeedback(8));//tilt);//current position
Serial.print("\t");
Serial.print(right_tilt_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("right_tilt");// is the joint moving
Serial.print("\t");
Serial.print("\n");

Serial.print("p3");// joint message left_lift
Serial.print("\t");
Serial.print(mapped_Left_Lift);//tilt);//current position
Serial.print("\t");
Serial.print(left_lift_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("left_lift");// is the joint moving
Serial.print("\t");
Serial.print("\n");

Serial.print("p4");// joint message right_lift
Serial.print("\t");
Serial.print(mapped_Right_Lift);//tilt);//current position
Serial.print("\t");
Serial.print(right_lift_target);// joint target
Serial.print("\t");
Serial.print("O");// velocity
Serial.print("\t");
Serial.print("right_lift");// is the joint moving
Serial.print("\t");
Serial.print("\n");


Serial.print("p5");// joint message left elbow
Serial.print("\t");
Serial.print(mapped_left_elbow);//tilt);//current position
Serial.print("\t");
Serial.print(left_elbow_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("left_elbow");// is the joint moving
Serial.print("\t");
Serial.print("\n");
/*
Serial.print("p6");// joint message left_rotate
Serial.print("\t");
Serial.print(RequestData(42));//tilt);//current position
Serial.print("\t");
Serial.print(left_rotate_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("left_rotate");// is the joint moving
Serial.print("\t");
Serial.print("\n");

Serial.print("p7");// joint message right_elbow
Serial.print("\t");
Serial.print(RequestData(40));//tilt);//current position
Serial.print("\t");
Serial.print(right_rotate_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("right_elbow");// is the joint moving
Serial.print("\t");
Serial.print("\n");

Serial.print("p8");// joint message left_elbow
Serial.print("\t");
Serial.print(RequestData(42));//tilt);//current position
Serial.print("\t");
Serial.print(left_rotate_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("left_elbow");// is the joint moving
Serial.print("\t");
Serial.print("\n");
*/

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

// set servo target on slave servo
void SetTarget(int address, int target)
{
    long foo = target;
    Wire.beginTransmission (address);
    //I2C_writeAnything (enc);
    I2C_writeAnything (foo);
    Wire.endTransmission ();
       
}

void Ser_print1()
{



Serial.print("p5");// joint message right_rotate
Serial.print("\t");
Serial.print(mapped_right_rotate);//tilt);//current position
Serial.print("\t");
Serial.print(right_rotate_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("right_rotate");// is the joint moving
Serial.print("\t");
Serial.print("\n");

Serial.print("p6");// joint message left_rotate
Serial.print("\t");
Serial.print(mapped_left_rotate);//tilt);//current position
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
Serial.print(mapped_right_elbow);//tilt);//current position
Serial.print("\t");
Serial.print(right_elbow_target);// joint target
Serial.print("\t");
Serial.print("0");// velocity
Serial.print("\t");
Serial.print("right_elbow");// is the joint moving
Serial.print("\t");
Serial.print("\n");
/*
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

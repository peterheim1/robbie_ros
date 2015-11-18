void Ser_print()
{

Serial.print("42");// joint message left_tilt
Serial.print("\t");
Serial.print(mapped_Right_Lift);//tilt);//current position
Serial.print("\t");
Serial.print(angle);// joint target input
Serial.print("\t");
Serial.print(ADC_SetPoint);// setpoint in ticks
Serial.print("\t");
Serial.print("left_rotate");// is the joint moving
Serial.print("\t");
Serial.print("\n");

}



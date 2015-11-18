void Ser_print()
{

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

}



void Pose(){
    static long PreviousLeftEncoderCounts = 0;
    static long PreviousRightEncoderCounts= 0;
    float SecondsSinceLastUpdate =  FrameRate/1000;
    long deltaLeft = readEncoder(LEFT) - PreviousLeftEncoderCounts;
    long deltaRight = readEncoder(RIGHT) - PreviousRightEncoderCounts;
    
    
    
    VLeft = deltaLeft * DistancePerCount_left / SecondsSinceLastUpdate;
    VRight = deltaRight * DistancePerCount_right / SecondsSinceLastUpdate;// FrameRate is time since last update
    
    double deltaDistance = 0.5f * (double)((deltaLeft * DistancePerCount_left) + (deltaRight * DistancePerCount_right));
    double deltaX = deltaDistance * (double)cos(Heading);
    double deltaY = deltaDistance * (double)sin(Heading);

    double deltaHeading = (double)(deltaRight - deltaLeft) * RadiansPerCount;
    
    X += deltaX;
    Y += deltaY;
    Heading += deltaHeading;
    // limit heading to -Pi <= heading < Pi
    if (Heading > PI)
	{
	Heading -= TwoPI;
	}
	else
	{
	if (Heading <= -PI)
	{
	Heading += TwoPI;
			}
	}
	//Velocity in meters per second and rotatation in rads per second
	V = deltaDistance / 0.03;// / SecondsSinceLastUpdate;
	Omega = deltaHeading / 0.03;// / SecondsSinceLastUpdate;

	PreviousLeftEncoderCounts = readEncoder(LEFT);
	PreviousRightEncoderCounts = readEncoder(RIGHT);
  
  
  
  
  
  
  
  
}

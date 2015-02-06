void Pose(){
    static long PreviousLeftEncoderCounts = 0;
    static long PreviousRightEncoderCounts= 0;
    float SecondsSinceLastUpdate =  FrameRate/1000;
    long deltaLeft = readEncoder(LEFT) - PreviousLeftEncoderCounts;
    long deltaRight = readEncoder(RIGHT) - PreviousRightEncoderCounts;
    
    VLeft = deltaLeft * DistancePerCount / SecondsSinceLastUpdate;
    VRight = deltaRight * DistancePerCount / SecondsSinceLastUpdate;// FrameRate is time since last update
    
    double deltaDistance = 0.5f * (double)(deltaLeft + deltaRight) * DistancePerCount;
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
		
	V = deltaDistance / SecondsSinceLastUpdate;
	Omega = deltaHeading / SecondsSinceLastUpdate;

	PreviousLeftEncoderCounts = readEncoder(LEFT);
	PreviousRightEncoderCounts = readEncoder(RIGHT);
  
  
  
  
  
  
  
  
}

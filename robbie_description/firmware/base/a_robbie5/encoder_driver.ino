#if defined ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  //#include "MegaEncoderCounter.h"
  #include <Encoder.h>
  
  Encoder Left_Encoder(19, 25);
  Encoder Right_Encoder(18, 24);
  /* Create the encoder shield object */
  //MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return Left_Encoder.read();
    else return Right_Encoder.read();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return Left_Encoder.write(0);
    else return Right_Encoder.write(0);
  }
//#else
  //#error A encoder driver must be selected!
//#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  Left_Encoder.write(0);
  Right_Encoder.write(0);
}

#endif


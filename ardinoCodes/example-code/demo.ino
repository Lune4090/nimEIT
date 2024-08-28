#include <EITKitArduino.h>

EITKitArduino *eit = nullptr;

void setup() {
  //eit.take_measurements(AD, AD, 16);
  eit = new EITKitArduino(16,1,4, AD, AD, true);
}

void loop() {
  // Example for taking measurements 
  // Should return origin frame and frame measurements
  // in Serial
  delay(5);
}

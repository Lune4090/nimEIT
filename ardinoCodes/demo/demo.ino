#include </home/lune/nimEIT/ardinoCodes/EITKitArduino_mod/EITKitArduino_mod.h>
#include </home/lune/nimEIT/ardinoCodes/EITKitArduino_mod/EITKitArduino_mod.cpp>

EITKitArduino *eit = nullptr;

void setup() {
  eit = new EITKitArduino();
}

void loop() {
  // Example for taking measurements 
  // Should return origin frame and frame measurements
  // in Serial
  delay(100);
  uint32_t ans = eit -> read_ADCBits();
  Serial.println(ans);
}

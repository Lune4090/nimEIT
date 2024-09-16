#include </home/lune/nimEIT/ardinoCodes/EITSpresense/EITSpresense.h>
#include </home/lune/nimEIT/ardinoCodes/EITSpresense/EITSpresense.cpp>

uint8_t PIN_ANALOG_0 = 0;
uint8_t PIN_ANALOG_1 = 2;
uint8_t PIN_ANALOG_V = 4;
uint8_t PIN_ANALOG_GND = 5;
uint16_t data0 = 0;
uint16_t data1 = 0;
uint16_t V = 1023;
uint16_t GND = 0;

void setup() {
  EITSpresense *eit = nullptr;
  eit = new EITSpresense();
  
  Serial.println("Starting initial setting...");
  uint16_t c = 0;
  // 14/14, FREQ0, PHASE0, RESET, Sin波出力
  c = eit -> AD9833_send_and_recv(0b0000000100000000);
  Serial.println(c, DEC);
  c = eit -> AD9833_send_and_recv(0b0000000000000000);
  Serial.println(c, DEC);
  // FREQ = f_mclk(16M)/2^28 (=0.05) * FREQ0
  c = eit -> AD9833_send_and_recv(0b010000100000000);
  Serial.println(c, DEC);
  c = eit -> AD9833_send_and_recv(0b1000000000000000);
  Serial.println(c, DEC);
  
  Serial.println("Initial setting finished!");
}

void loop() {
  delay(10);
  data0 = analogRead(PIN_ANALOG_0);
  data1 = analogRead(PIN_ANALOG_1);
  V = analogRead(PIN_ANALOG_V);
  GND = analogRead(PIN_ANALOG_GND);
  Serial.print(data0);
  Serial.print(",");
  Serial.print(data1);
  Serial.print(",");
  Serial.print(V);
  Serial.print(",");
  Serial.print(GND);
  Serial.println("");
}

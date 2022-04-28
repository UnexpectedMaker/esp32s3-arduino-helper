#include <UMS3.h>

UMS3 ums3;

void setup() {
  Serial.begin(115200);

  // Initialize all board peripherals, call this first
  ums3.begin();
}

void loop() {
  // Light sensor voltage goes up to about 3.3v
  float light = ums3.getLightSensorVoltage();

  // View this with the arduino serial plotter (in the tools menu)
  Serial.println(light);

  delay(50);
}
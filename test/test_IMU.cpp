#include "Arduino.h"
#include "IMU.h"

void setup() {
  Serial.begin(115200);
  setup_IMU();
}

void loop() {
  loop_IMU();
}
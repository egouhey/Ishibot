#include "Arduino.h"   
#include "screen.h"


void setup() {
  Serial.begin(115200);
  setup_screen();
}

void loop() {
  screen();
}

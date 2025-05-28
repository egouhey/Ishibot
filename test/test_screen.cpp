#include "Arduino.h"   
#include "screen.h"


void setup() {
  Serial.begin(115200);
  init_ecran();
  // setup_screen();
}

void loop() {
  // screen();
  print_oled_Position(400, -4000, 40);
  vTaskDelay(2000/portTICK_PERIOD_MS);
}

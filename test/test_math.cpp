#include <cmath>
#include <Arduino.h>

float min_float(float a, float b){
  if(a<b){return a;}
  else{return b;};
}

float max_float(float a, float b){
  if(a<b){return b;}
  else{return a;};
}

void setup() {
    Serial.begin(115200);

    Serial.println(min_float(1.0, 2.0));
    Serial.println(max_float(1.0, 2.0));
    Serial.println(min_float(-1.0, -2.0));
    Serial.println(max_float(-1.0, -2.0));
}

// entry 0x400805e4
// 1.00
// 2.00
// -2.00
// -1.00

void loop() {
    // Votre code ici
}

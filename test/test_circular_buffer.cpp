#include <Arduino.h>
#include <circular_buffer.h>

CIRCULAR_BUFFER c_buffer;

void setup() {
    Serial.begin(115200);
    c_buffer.put(1.0);
    c_buffer.put(2.0);
    c_buffer.put(3.0);
}

void loop() {
    Serial.print("sum = ");
    Serial.println(c_buffer.sum_buffer());
    delay(1000);
    c_buffer.put(3.0);

}

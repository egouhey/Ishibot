#include "lidar.h"

#define RXS1 17
#define TXS1 18

LIDAR lidar(Serial1, RXS1, TXS1);

void setup(){
    Serial.begin(115200);
    lidar.start_interrupt();
}

void loop(){
    vTaskDelay(1000/portTICK_PERIOD_MS);
}
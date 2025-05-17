#include "lidar.h"
#include "screen.h"
#include "config.h"

LIDAR lidar(Serial1, RX_LIDAR, TX_LIDAR);

void setup(){
    Serial.begin(115200);
    init_ecran();
    delay(1000);
    lidar.start_interrupt();
}

void loop(){
    if(lidar.stop){
        print_oled_STOP();
        lidar.stop=false;
    }
    else{
        print_oled_GO();
        lidar.stop=false;
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
}
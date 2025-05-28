#include "lidar.h"
#include "screen.h"
#include "config.h"
#include "motor.h"

LIDAR lidar(Serial1, RX_LIDAR, TX_LIDAR);

void setup(){
    Serial.begin(115200);
    setup_motor();
    init_ecran();
    vTaskDelay(100/portTICK_PERIOD_MS);
    lidar.start_interrupt();
    // vTaskDelay(100/portTICK_PERIOD_MS);
    setup_control_motor();
    control_motor_speed(10, HIGH, 10, HIGH, 10, HIGH);
}

bool change = true;

void loop(){
    if(lidar.stop){
        print_oled_STOP();
        lidar.stop=false;
        control_motor_speed(0, HIGH, 0, HIGH, 0, HIGH);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    else{
        control_motor_speed(10, HIGH, 10, HIGH, 10, HIGH);
        print_oled_GO();
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
}
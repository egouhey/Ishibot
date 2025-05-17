#include <robot.h>
#include "config.h"
#include "motor.h"

ROBOT robot(Serial2, RX_ENCODER, TX_ENCODER);

void setup(){
    Serial.begin(115200);
    setup_motor();
    setup_control_motor();
    // const byte message[] = {0xe0, 0x8b, 0x02, 0x6d}; 
    // robot.robotSerial->write(message, sizeof(message));
    // delay(1000);
    control_motor_speed(0, HIGH, 10, LOW, 10, HIGH);
    // const byte message[] = {0xe0, 0x8b, 0x02, 0x6d}; 
    // robot.robotSerial->write(message, sizeof(message));
    // while(robot.robotSerial->available()){
    //     Serial.println(robot.robotSerial->read());
    // }
}

int res=0;

void loop(){

    res = robot.ReadEncoderMotor(0xe0);
    Serial.print("0 = ");
    Serial.println(res);
    vTaskDelay(1000/portTICK_PERIOD_MS);

    res = robot.ReadEncoderMotor(0xe1);
    Serial.print("1 = ");
    Serial.println(res);
    vTaskDelay(1000/portTICK_PERIOD_MS);


    res = robot.ReadEncoderMotor(0xe2);
    Serial.print("2 = ");
    Serial.println(res);
    vTaskDelay(1000/portTICK_PERIOD_MS);


    // Serial.print("3 = ");
    // Serial.println(robot.ReadEncoderMotor(0xe3));
    // vTaskDelay(1000/portTICK_PERIOD_MS);
    // Serial.print("X = ");
    // Serial.println(robot.ReadEncoderMotor(0xe0));
    // vTaskDelay(1000/portTICK_PERIOD_MS);
    // const byte message[] = {0xe0, 0x8b, 0x01, 0x6d}; 
    // robot.robotSerial->write(message, sizeof(message));
    // while(robot.robotSerial->available()){
    //     Serial.println(robot.robotSerial->read());
    // }
    // vTaskDelay(10000/portTICK_PERIOD_MS);
    // const byte message1[] = {0xe0,0x36,0x16}; 
    // robot.robotSerial->write(message1, sizeof(message1));
    // while(robot.robotSerial->available()){
    //     Serial.println(robot.robotSerial->read());
    // }
}
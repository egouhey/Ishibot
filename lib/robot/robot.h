#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

struct Position {
  int X = 0;
  int Y = 0;
  int theta = 0;
};

struct Objective {
  int X = 0;
  int Y = 0;
  int theta = 0;
  bool hit = true;
};

class ROBOT{
    public:
    ROBOT(HardwareSerial &serial, uint8_t RxPin, uint8_t TxPin);
    void lauch_task_encoder();
    Position position;
    SemaphoreHandle_t mutex_position;
    Objective obj;

    int ReadEncoderMotor(int addr_motor);
    int addr_motor_1 = 0xe1;
    int addr_motor_2 = 0xe2;
    int addr_motor_3 = 0xe3;
    HardwareSerial *robotSerial;

    private:
    void setup_uart();
    void wheel2position(int d_motor_1, int d_motor_2, int d_motor_3);
    int position2wheel();
    void asserv_robot();
    static void task_robot_encoder(void *pvParameters);

    uint8_t Rxpin;
    uint8_t Txpin;
    const unsigned long baudrate = 9600;    
};

#endif
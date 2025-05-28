#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <screen.h>
#include <esp32-hal-ledc.h> 
#include <cmath>
#include <config.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

const float r_robot =   162.253;
const float r_wheel =   30.0;
// const float rstep   =  360/65536;

struct Position {
  float X = 0;
  float Y = 0;
  float A = 0;
};

struct Speed {
  float vX = 0;
  float vY = 0;
  float W = 0;
};

struct Omni {
  float x1 = 0;   // cote actionneur
  float x2 = 0;   // cote numerique
  float x3 = 0;   // cote puissance
};

class ROBOT{
    public:
    ROBOT(HardwareSerial &serial, uint8_t RxPin, uint8_t TxPin);
    void setup_motor();
    void setup_control_motor();
    void lauch_task_encoder();
    void lauch_task_motor();
    void clear_position();

    void control_motor_speed(float speed_1, float speed_2, float speed_3);
    
    Speed speed_local_from_omni(Omni omni);
    Speed speed_world_from_speed_local(Speed local_speed);
    void  position_world_from_speed_world(Speed world_speed);

    Speed speed_world_from_delta_position(Position delta_positon);
    Speed speed_local_from_speed_world(Speed world_speed);
    Omni  omni_from_speed_local(Speed local_speed);

    Position  objectif_world;
    Position  position_world;
    Speed     speed_world;
    Speed     speed_local;
    Omni      omni;
    SemaphoreHandle_t mutex_position;

    int ReadEncoderMotor(int addr_motor);
    int addr_motor_1 = 0xe1;
    int addr_motor_2 = 0xe2;
    int addr_motor_3 = 0xe3;
    HardwareSerial *robotSerial;

    private:
    void setup_uart();
    static void task_robot_encoder(void *pvParameters);
    static void task_robot_motor(void *pvParameters);

    uint8_t Rxpin;
    uint8_t Txpin;
    const unsigned long baudrate = 115200;   
    const int read_step_per_turn = 65536; 

    int interval=100;

    float Kp_l = 3.5;
    // float Kp_r = 0.003;
    float Kp_r = 0.01;

    int previous_speed_1=0;
    int previous_speed_2=0;
    int previous_speed_3=0;

    int step_ramp=5;
    float max_speed_motor=1000.0;
    float tresh_l = 100.0;
    float tresh_r = 1.0;

    // void wheel2position(int step_motor_1, int step_motor_2, int step_motor_3);
    // int position2wheel(float moveX, float moveY, float movetheta);
    // void asserv_robot();
};

float cos_deg(float theta);

float sin_deg(float theta);

float min_float(float a, float b);

float max_float(float a, float b);

#endif
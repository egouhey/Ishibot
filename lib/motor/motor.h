#ifndef MOTOR_H
#define MOTOR_H

// motor 1 : puissance
// motor 2 : actionneur
// motor 3 : numérique

#include <Arduino.h>
#include "config.h"

void setup_motor();                 // setup motor pin

void setup_control_motor();         // setup motor pwm channel

void control_motor_speed(           // control motor speed 
    uint32_t    freq_1, 
    bool        dir_1, 
    uint32_t    freq_2, 
    bool        dir_2, 
    uint32_t    freq_3, 
    bool        dir_3
);

void control_motor_position(        // control motor position 
    int step_motor1,
    int step_motor2,
    int step_motor3,
    int time
);

void test_motor(                    // basic movment
    const char* cstr
);

void setup_encoder();

int ReadEncoderMotor1();

#endif
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include "config.h"
#include "motor.h"

void turn(
    float w,
    int time
);

void move(
    float x,
    float y,
    int time
);

void move_regular_old(
    float x,
    float y
);

void move_regular(
    float x,
    float y
);

void movement(
    float x, 
    float y, 
    float w
);

#endif
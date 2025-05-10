#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "config.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern int delta_x, delta_y, delta_z;
extern int current_accelx, current_accely, current_accelz;
extern int vel_x, vel_y, vel_z;
extern int pos_x, pos_y, pos_z;
extern unsigned long lastTime;
extern int epsilon;

void setup_IMU();

void loop_IMU();


#endif
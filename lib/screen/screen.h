#ifndef SCREEN_H
#define SCREEN_H

#include <Arduino.h>
#include "config.h"

#include <Adafruit_SSD1306.h>    

extern volatile bool blue_team;
extern volatile bool yellow_team;
extern volatile bool carre;
extern volatile bool go;

extern volatile bool state_bouton1;
extern volatile bool state_bouton2;
extern volatile bool state_bouton3;
extern volatile bool old_state_bouton1;
extern volatile bool old_state_bouton2;
extern volatile bool old_state_bouton3;

extern volatile bool state_menu;
extern volatile bool state_coupe;
extern volatile bool state_position;
extern volatile bool state_connect;
 
extern volatile int choice_menu;
extern volatile int choice_coupe;
extern volatile int choice_position;
extern volatile int choice_connect;

extern volatile bool flag_next;
extern volatile bool flag_enter;
extern volatile bool flag_menu;

void rectangle(int x1, int y1, int x2, int y2);

void init_ecran();

void menu();

void coupe();

void position();

void connect();

void setup_screen(); // warning 

void IRAM_ATTR make_menu();

void IRAM_ATTR make_enter();

void IRAM_ATTR make_next();

void screen();

void print_oled_GO();

void print_oled_STOP();

void print_oled_Position(float x, float y, float W);


#endif



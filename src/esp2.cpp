#include "kinematics.h"
#include "motor.h"
// #include "IMU.h"
#include "screen.h"

void yellow_game(){
  move_regular_old(0,300);
  delay(100);
  move_regular_old(-425, 0);
  delay(100);
  move_regular_old(0, -250);
  delay(100);
  move_regular_old(0,300);
  delay(100);
  move_regular_old(-125,0);
  delay(100);
  move_regular_old(0,550);
  delay(100);
  move_regular_old(400,0);
  delay(100);
  move_regular_old(200,-700);
  delay(100);
  // move_regular_old(-850,1150);
  // delay(100);
}

void blue_game(){
  move_regular_old(0,300);
  delay(100);
  move_regular_old(425, 0);
  delay(100);
  move_regular_old(0, -250);
  delay(100);
  move_regular_old(0,300);
  delay(100);
  move_regular_old(125,0);
  delay(100);
  move_regular_old(0,550);
  delay(100);
  move_regular_old(-400,0);
  delay(100);
  move_regular_old(-200,-700);
  delay(100);
  // move_regular_old(850,1150);
  // delay(100);
}

void carre_game(){
  while(1){
    move_regular_old(0,300);
    delay(100);
    move_regular_old(300, 0);
    delay(100);
    move_regular_old(0, -300);
    delay(100);
    move_regular_old(-300,0);
    delay(100);
  }
}


void setup() {
  Serial.begin(115200);
  setup_motor();
  setup_control_motor();
  Serial.println("setup");
  setup_screen();
}

void loop() {
  if (go && yellow_team){
    go=false;
    yellow_game();
  }
  else if (go && blue_team){
    go=false;
    blue_game();
  }
  else if (go && carre){
    go=false;
    carre_game();
  }
  else{screen();}
  // Serial.println("hello");
  vTaskDelay(100);
}

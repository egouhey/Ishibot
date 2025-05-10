#include "kinematics.h"

void turn_old(float w, int time){
    int step1= (int) step_per_turn*peri_robot/peri_wheel*w;
    int step2= (int) step_per_turn*peri_robot/peri_wheel*w;
    int step3= (int) step_per_turn*peri_robot/peri_wheel*w;

    control_motor_position(step1, step2, step3, time );
}

void move_old(float x, float y, int time){
    int step1 = (int) x*(-1)*step_per_turn/(2*peri_wheel) + y*step_per_turn/peri_wheel;
    int step2 = (int) x*step_per_turn/peri_wheel;
    int step3 = (int) x*(-1)*step_per_turn/(2*peri_wheel) + y*(-1)*step_per_turn/peri_wheel;

    Serial.println(step1);
    Serial.println(step2);
    Serial.println(step3);
    control_motor_position(step1, step2, step3, time );
}

void move_regular_old(float x, float y){
    int step1 = (int) x*(-1)*step_per_turn/(2*peri_wheel) + y*step_per_turn/peri_wheel;
    int step2 = (int) x*step_per_turn/peri_wheel;
    int step3 = (int) x*(-1)*step_per_turn/(2*peri_wheel) + y*(-1)*step_per_turn/peri_wheel;

    Serial.println(step1);
    Serial.println(step2);
    Serial.println(step3);
    if(x<0){
        x=-x;
    }
    if(y<0){
        y=-y;
    }
    control_motor_position(step1, step2, step3, (15*(x+y) ));
}


void turn(float w, int time){
    int step1= (int) step_per_turn*peri_robot/peri_wheel*w/360.0;
    int step2= (int) step_per_turn*peri_robot/peri_wheel*w/360.0;
    int step3= (int) step_per_turn*peri_robot/peri_wheel*w/360.0;

    control_motor_position(step1, step2, step3, time );
}

void move(float x, float y, int time){
    int step1 = (int) ( x*(-1)*step_per_turn/(2*peri_wheel) + 0.866*y*step_per_turn/peri_wheel );
    int step2 = (int) ( x*step_per_turn/peri_wheel );
    int step3 = (int) ( x*(-1)*step_per_turn/(2*peri_wheel) + 0.866*y*(-1)*step_per_turn/peri_wheel );

    Serial.println(step1);
    Serial.println(step2);
    Serial.println(step3);
    control_motor_position(step1, step2, step3, time );
}

void move_regular(float x, float y){
    int step1 = (int) ( x*(-1)*step_per_turn/(2*peri_wheel) + 0.866*y*step_per_turn/peri_wheel );
    int step2 = (int) ( x*step_per_turn/peri_wheel );
    int step3 = (int) ( x*(-1)*step_per_turn/(2*peri_wheel) + 0.866*y*(-1)*step_per_turn/peri_wheel );

    Serial.println(step1);
    Serial.println(step2);
    Serial.println(step3);
    if(x<0){
        x=-x;
    }
    if(y<0){
        y=-y;
    }
    control_motor_position(step1, step2, step3, 10*(x+y) );
    // control_motor_position(5*step1/100, 5*step2/100, 5*step3/100, (x+y)/2 );        // (5*(x+y))*(5/100)*2
    // control_motor_position((9/10)*step1, (9/10)*step2, (9/10)*step3, 4.5*(x+y) );   //(9/10)*5
    // control_motor_position(5*step1/100, 5*step2/100, 5*step3/100, (x+y)/2 );        // (5*(x+y))*(5/100)*2

}

void movement(float x, float y, float w){
    int step1 = (int) ( x*(-1)*step_per_turn/(2*peri_wheel) + 0.866*y*step_per_turn/peri_wheel - step_per_turn*peri_robot/peri_wheel*w/360.0);
    int step2 = (int) ( x*step_per_turn/peri_wheel - step_per_turn*peri_robot/peri_wheel*w/360.0);
    int step3 = (int) ( x*(-1)*step_per_turn/(2*peri_wheel) + 0.866*y*(-1)*step_per_turn/peri_wheel - step_per_turn*peri_robot/peri_wheel*w/360.0);

    Serial.println(step1);
    Serial.println(step2);
    Serial.println(step3);
    if(x<0){
        x=-x;
    }
    if(y<0){
        y=-y;
    }
    control_motor_position(step1, step2, step3, 5*(x+y) );
    // control_motor_position(5*step1/100, 5*step2/100, 5*step3/100, (x+y)/2 );        // (5*(x+y))*(5/100)*2
    // control_motor_position((9/10)*step1, (9/10)*step2, (9/10)*step3, 4.5*(x+y) );   //(9/10)*5
    // control_motor_position(5*step1/100, 5*step2/100, 5*step3/100, (x+y)/2 );        // (5*(x+y))*(5/100)*2

}
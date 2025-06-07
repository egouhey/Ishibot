#include <robot.h>
#include "config.h"
#include "screen.h"

ROBOT robot(Serial2, RX_ENCODER, TX_ENCODER);

void setup(){
    Serial.begin(115200);
    robot.setup_motor();
    robot.setup_control_motor();
    init_ecran();
    print_oled_GO();
    robot.lauch_task_encoder();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    robot.clear_position();
    vTaskDelay(50/portTICK_PERIOD_MS);
    robot.lauch_task_motor();
    // robot.control_motor_speed(200.0, -100.0, -100.0);
}

void test_odometry_with_angle();
void test_odometry_simple();
void test_odometry_speed();

void loop(){ 
    // test_odometry_simple();
    // print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
    vTaskDelay(3000/portTICK_PERIOD_MS);
}

void test_odometry_speed(){
    Speed command_speed_world;
    command_speed_world.vY=100;
    command_speed_world.W=0;
    Speed command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    Omni command_omni = robot.omni_from_speed_local(command_speed_local);
    robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    for(int i=0; i<50; i++){
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void test_odometry_with_angle(){
    robot.control_motor_speed(100.0, 100.0, 100.0);
    while(robot.position_world.A<180){
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(2/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(0.0, 100.0, -100.0);
    while(robot.position_world.Y<200){
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(-200.0, 100.0, 100.0);
    while(robot.position_world.X<200){
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(0.0, -100.0, 100.0);
    while(robot.position_world.Y>0){
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(200.0, -100.0, -100.0);
    while(robot.position_world.X>0){
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(0.0, 0.0, 0.0);
    robot.clear_position();

    vTaskDelay(2000/portTICK_PERIOD_MS);
}

void test_odometry_simple(){
    while(robot.position_world.Y<250){
        robot.control_motor_speed(0.0, -50.0, 50.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.X<250){
        robot.control_motor_speed(100.0, -50.0, -50.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.Y>0){
        robot.control_motor_speed(0.0, 50.0, -50.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.X>0){
        robot.control_motor_speed(-100.0, 50.0, 50.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(0.0, 0.0, 0.0);
    robot.clear_position();

    vTaskDelay(250/portTICK_PERIOD_MS);
}
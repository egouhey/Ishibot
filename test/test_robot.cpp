#include <robot.h>
#include <lidar.h>
#include "config.h"
#include "screen.h"

// void carre_game(){
//   while(1){
//     move_regular_old(0,300);
//     delay(100);
//     move_regular_old(300, 0);
//     delay(100);
//     move_regular_old(0, -300);
//     delay(100);
//     move_regular_old(-300,0);
//     delay(100);
//   }
// }

LIDAR lidar(Serial1, RX_LIDAR, TX_LIDAR);
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
    // robot.lauch_task_motor();
    // robot.control_motor_speed(10.0, 0.0, -10.0);
}

void test_odometry_with_angle();
void test_odometry_simple();
void test_odometry_speed();

void loop(){ 
    // test_odometry_speed();

    // if(lidar)
    robot.control_motor_speed(100.0, 100.0, 100.0);
    print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
    vTaskDelay(350/portTICK_PERIOD_MS);
}

void test_odometry_speed(){
    Speed command_speed_world;
    // command_speed_world.vX=0;
    // command_speed_world.vY=0;
    // command_speed_world.W=10;
    // Speed command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    // Omni  command_omni = robot.omni_from_speed_local(command_speed_local);
    // robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    // vTaskDelay(5000/portTICK_PERIOD_MS);
    // command_speed_world.vX=0;
    // command_speed_world.vY=0;
    // command_speed_world.W=-10;
    // command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    // command_omni = robot.omni_from_speed_local(command_speed_local);
    // robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    // vTaskDelay(5000/portTICK_PERIOD_MS);
    command_speed_world.vX=0;
    command_speed_world.vY=0;
    command_speed_world.W=10;
    Speed command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    Omni command_omni = robot.omni_from_speed_local(command_speed_local);
    robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    vTaskDelay(5000/portTICK_PERIOD_MS);
    // command_speed_world.vX=10;
    // command_speed_world.vY=0;
    // command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    // command_omni = robot.omni_from_speed_local(command_speed_local);
    // robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    // vTaskDelay(5000/portTICK_PERIOD_MS);
    // command_speed_world.vX=0;
    // command_speed_world.vY=-10;
    // command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    // command_omni = robot.omni_from_speed_local(command_speed_local);
    // robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    // vTaskDelay(5000/portTICK_PERIOD_MS);
    // command_speed_world.vX=-10;
    // command_speed_world.vY=0;
    // command_speed_local = robot.speed_local_from_speed_world(command_speed_world);
    // command_omni = robot.omni_from_speed_local(command_speed_local);
    // robot.control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    // vTaskDelay(5000/portTICK_PERIOD_MS);
}

void test_odometry_with_angle(){
    while(robot.position_world.Y<360){
        robot.control_motor_speed(0.0, 10.0, -10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.X<360){
        robot.control_motor_speed(-20.0, 10.0, 10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.Y>0){
        robot.control_motor_speed(0.0, -10.0, 10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.X>0){
        robot.control_motor_speed(20.0, -10.0, -10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(0.0, 0.0, 0.0);
    robot.clear_position();

    vTaskDelay(100/portTICK_PERIOD_MS);
}

void test_odometry_simple(){
    while(robot.position_world.Y<250){
        robot.control_motor_speed(0.0, -10.0, 10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.X<250){
        robot.control_motor_speed(20.0, -10.0, -10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.Y>0){
        robot.control_motor_speed(0.0, 10.0, -10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    while(robot.position_world.X>0){
        robot.control_motor_speed(-20.0, 10.0, 10.0);
        print_oled_Position(robot.position_world.X, robot.position_world.Y, robot.position_world.A);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    robot.control_motor_speed(0.0, 0.0, 0.0);
    robot.clear_position();

    vTaskDelay(250/portTICK_PERIOD_MS);
}
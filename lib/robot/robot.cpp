#include "robot.h"

ROBOT::ROBOT(HardwareSerial &serial, uint8_t RxPin, uint8_t TxPin)
{
  this->robotSerial=&serial;
  this->Rxpin=RxPin;
  this->Txpin=TxPin;
  this->setup_uart();
  mutex_position = xSemaphoreCreateMutex();
}

void ROBOT::setup_uart()
{
  robotSerial->begin(this->baudrate, SERIAL_8N1, this->Rxpin, this->Txpin);
}

void ROBOT::lauch_task_encoder()
{
    xTaskCreate(ROBOT::task_robot_encoder, "task_robot_motor", 1024, this, 1, NULL);
}

void ROBOT::task_robot_encoder(void *pvParameters)
{
  ROBOT *robot = static_cast<ROBOT*>(pvParameters);
  int d_motor_1=0;
  int d_motor_2=0;
  int d_motor_3=0;
  int last_d_motor_1=0;
  int last_d_motor_2=0;
  int last_d_motor_3=0;

  while(1){
    last_d_motor_1=d_motor_1;
    last_d_motor_2=d_motor_2;
    last_d_motor_3=d_motor_3;
    d_motor_1 = robot->ReadEncoderMotor(robot->addr_motor_1);
    d_motor_2 = robot->ReadEncoderMotor(robot->addr_motor_2);
    d_motor_3 = robot->ReadEncoderMotor(robot->addr_motor_3);
    robot->wheel2position(d_motor_1-last_d_motor_1, d_motor_2-last_d_motor_2, d_motor_3-last_d_motor_3);
    // Serial.println("hello world");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

int ROBOT::ReadEncoderMotor(int addr_motor) 
{
  const byte message[] = {static_cast<byte>(addr_motor & 0xFF),0x36,static_cast<byte>((addr_motor + 0x36) & 0xFF) }; 
  // const byte message[] = (addr_motor<<32) && (0x36<<16) && 0x16; 
  this->robotSerial->write(message, sizeof(message));
  uint16_t motorData[8];                                   
  if (this->robotSerial->available() > 0) 
  {
    int numBytes = this->robotSerial->available();
    for (int i = 0; i < numBytes; i++)
    {
      motorData[i] = this->robotSerial->read();

      // if (motorData[0] != addr_motor) 
      // {
      //   Serial.println("Wrong adress");
      //   break;
      // }
    }
    Serial.println(addr_motor);
    Serial.println(motorData[0]);
    if (numBytes >= 3) 
    {
      uint32_t encoderValue = (motorData[1] << 24) | (motorData[2] << 16) | (motorData[3] << 8) | motorData[4];
      vTaskDelay(pdMS_TO_TICKS(5));
      if(encoderValue>2147483648){
        int res=encoderValue-4294967296;
        return res;
      }
      else{
        return encoderValue;
      }
    } 
  }
  return 0;
}

void ROBOT::wheel2position(int d_motor_1, int d_motor_2, int d_motor_3) // TODO
{
  if (xSemaphoreTake(mutex_position, pdMS_TO_TICKS(10)) == pdTRUE){
    position.X=0;
    position.Y=0;
    position.theta=0;
    xSemaphoreGive(mutex_position);
  }
}

int ROBOT::position2wheel()  // TODO
{
  return 0;
}

void ROBOT::asserv_robot()
{
 int vX=0;
 int vY=0;
 int vtheta=0;

}
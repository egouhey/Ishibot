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

void ROBOT::setup_motor()
{
  // motor 1
  pinMode(enPin1, OUTPUT);    digitalWrite(enPin1, LOW);      
  pinMode(stpPin1, OUTPUT);   digitalWrite(stpPin1, LOW);    
  pinMode(dirPin1, OUTPUT);   digitalWrite(dirPin1, LOW);    

  // motor 2
  pinMode(enPin2, OUTPUT);    digitalWrite(enPin2, LOW);     
  pinMode(stpPin2, OUTPUT);   digitalWrite(stpPin2, LOW);    
  pinMode(dirPin2, OUTPUT);   digitalWrite(dirPin2, LOW);  

  // motor 3
  pinMode(enPin3, OUTPUT);    digitalWrite(enPin3, LOW);     
  pinMode(stpPin3, OUTPUT);   digitalWrite(stpPin3, LOW);
  pinMode(dirPin3, OUTPUT);   digitalWrite(dirPin3, LOW); 
}

void ROBOT::setup_control_motor()
{
  // setup pwm channel
  ledcSetup(pwmChannel_1, 0, resolution_pwm);
  ledcSetup(pwmChannel_2, 0, resolution_pwm);
  ledcSetup(pwmChannel_3, 0, resolution_pwm);

  // attach pin to pwm channel
  ledcAttachPin(stpPin1, pwmChannel_1);
  ledcAttachPin(stpPin2, pwmChannel_2);
  ledcAttachPin(stpPin3, pwmChannel_3);

  // setup duty cycle for pwm channel
  ledcWrite(pwmChannel_1, 128);
  ledcWrite(pwmChannel_2, 128);
  ledcWrite(pwmChannel_3, 128);
}

void ROBOT::lauch_task_encoder()
{
    xTaskCreate(ROBOT::task_robot_encoder, "task_robot_encoder", 1024, this, 1, NULL);
}

void ROBOT::task_robot_encoder(void *pvParameters){
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

    while (robot->robotSerial->available() > 0) {
      char c = robot->robotSerial->read();
    }

    d_motor_1 = robot->ReadEncoderMotor(0xe1);
    d_motor_2 = robot->ReadEncoderMotor(0xe2);
    d_motor_3 = robot->ReadEncoderMotor(0xe3);

    // Serial.print("d_motor_1 = ");
    // Serial.println(d_motor_1);
    // Serial.print("d_motor_2 = ");
    // Serial.println(d_motor_2);
    // Serial.print("d_motor_3 = ");
    // Serial.println(d_motor_3);

    robot->omni.x1=((d_motor_1-last_d_motor_1)/robot->interval);
    robot->omni.x2=((d_motor_2-last_d_motor_2)/robot->interval);
    robot->omni.x3=((d_motor_3-last_d_motor_3)/robot->interval);

    // Serial.print("x1 = ");
    // Serial.println(robot->omni.x1);
    // Serial.print("x2 = ");
    // Serial.println(robot->omni.x2);
    // Serial.print("x3 = ");
    // Serial.println(robot->omni.x3);


    robot->speed_local=robot->speed_local_from_omni(robot->omni);
    robot->speed_world=robot->speed_world_from_speed_local(robot->speed_local);
    if (xSemaphoreTake(robot->mutex_position, pdMS_TO_TICKS(10)) == pdTRUE){
      robot->position_world_from_speed_world(robot->speed_world);
      xSemaphoreGive(robot->mutex_position);
    }

    // Serial.print("x = ");
    // Serial.println(robot->position_world.X);
    // Serial.print("y = ");
    // Serial.println(robot->position_world.Y);
    // Serial.print("a = ");
    // Serial.println(robot->position_world.A);
    vTaskDelay(robot->interval/portTICK_PERIOD_MS);
    }
}

void ROBOT::lauch_task_motor()
{
  xTaskCreate(ROBOT::task_robot_motor, "task_robot_motor", 8*1024, this, 1, NULL);
}

void ROBOT::task_robot_motor(void *pvParameters)
{
  ROBOT *robot = static_cast<ROBOT*>(pvParameters);
  int i=1;
  while(1){
    robot->delta.X = robot->objectif_world.X - robot->position_world.X;
    robot->delta.Y = robot->objectif_world.Y - robot->position_world.Y;
    robot->delta.A = robot->objectif_world.A - robot->position_world.A;
    // Serial.print("robot->delta_x = ");
    // Serial.println(robot->delta.X);
    // Serial.print("robot->delta_y = ");
    // Serial.println(robot->delta.Y);
    // Serial.print("robot->delta_a = ");
    // Serial.println(robot->delta.A);

    if(abs(robot->delta.X)>=3.0 || abs(robot->delta.Y)>=3.0 || abs(robot->delta.A)>=3.0  ){
      Speed command_speed_world = robot->speed_world_from_delta_position(robot->delta);
      Speed command_speed_local = robot->speed_local_from_speed_world(command_speed_world);
      Omni  command_omni = robot->omni_from_speed_local(command_speed_local);
      robot->control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    }
    else{
      robot->control_motor_speed(0, 0, 0);
      // if (i==0){
      //   robot->objectif_world.X=0;
      //   robot->objectif_world.Y=300;
      //   robot->objectif_world.A=180;
      //   i++;
      //   // i++;
      //   // i++;
      //   // i++;
      // }

      if (i==1){
        robot->objectif_world.X=0;
        robot->objectif_world.Y=250;
        robot->objectif_world.A=0;
        i++;
        // i++;
        // i++;
      }
      else if (i==2){
        robot->objectif_world.X=250;
        robot->objectif_world.Y=250;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==3){
        robot->objectif_world.X=250;
        robot->objectif_world.Y=0;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==4){
        robot->objectif_world.X=0;
        robot->objectif_world.Y=0;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==5){
        vTaskDelay(100/portTICK_PERIOD_MS);
        i=1;
      }
    }
    vTaskDelay(robot->interval/portTICK_PERIOD_MS);
  }
}

int ROBOT::ReadEncoderMotor(int addr_motor)
{
  const byte message[] = {static_cast<byte>(addr_motor & 0xFF),0x36,static_cast<byte>((addr_motor + 0x36) & 0xFF) };
  // const byte message[] = (addr_motor<<32) && (0x36<<16) && 0x16;
  this->robotSerial->write(message, sizeof(message));
  uint16_t motorData[8];
  vTaskDelay(5/portTICK_PERIOD_MS);
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
    // Serial.println(addr_motor);
    // Serial.println(motorData[0]);
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

/////////////////////////////////

Speed ROBOT::speed_local_from_omni(Omni omni)
{
  Speed temp;
  temp.vX=(2.0 * omni.x1 - omni.x2 - omni.x3) / 3;
  temp.vY= (sqrt(3.0) * omni.x3 - sqrt(3.0) * omni.x2) / 3;
  temp.W=(omni.x1 + omni.x2 + omni.x3) / (3);
  return temp;
}

Speed ROBOT::speed_world_from_speed_local(Speed local_speed)
{
  Speed temp;
  temp.vX=cos_deg(position_world.A)*local_speed.vX - sin_deg(position_world.A)*local_speed.vY;
  temp.vY=sin_deg(position_world.A)*local_speed.vX + cos_deg(position_world.A)*local_speed.vY ;
  temp.W=local_speed.W;

  Serial.print("vx = ");
  Serial.println(temp.vX);
  Serial.print("vy = ");
  Serial.println(temp.vY);
  Serial.print("vw = ");
  Serial.println(temp.W);

  return temp;
}

void ROBOT::position_world_from_speed_world(Speed world_speed){
  this->position_world.X-=(2.0*PI*r_wheel*world_speed.vX*interval)/read_step_per_turn;
  this->position_world.Y-=(2.0*PI*r_wheel*world_speed.vY*interval)/read_step_per_turn;
  this->position_world.A-=(360.0*(r_wheel/r_robot)*(world_speed.W*interval))/read_step_per_turn;

  // Serial.print("X = ");
  // Serial.println(position_world.X);
  // Serial.print("Y = ");
  // Serial.println(position_world.Y);
  // Serial.print("A = ");
  // Serial.println(position_world.A);
}

/////////////////////////////////

Speed ROBOT::speed_world_from_delta_position(Position delta_positon)
{
  Speed temp;

  temp.W=Kp_r*delta_positon.A;
  temp.vX=Kp_l*delta_positon.X;
  temp.vY=Kp_l*delta_positon.Y;

  if(abs(this->previous_speed_x-temp.vX)>this->max_speed_world){
    if(this->previous_speed_x<temp.vX){
      temp.vX=this->previous_speed_x + this->speed_ramp;
    }
    else{
      temp.vX=this->previous_speed_x - this->speed_ramp;
    }
  }

  if(abs(this->previous_speed_y-temp.vY)>this->max_speed_world){
    if(this->previous_speed_y<temp.vY){
      temp.vY=this->previous_speed_y + this->speed_ramp;
    }
    else{
      temp.vY=this->previous_speed_y - this->speed_ramp;
    }
  }

  if(abs(this->previous_speed_w-temp.W)>this->max_speed_world){
    if(this->previous_speed_w<temp.W){
      temp.W=this->previous_speed_w + this->speed_ramp;
    }
    else{
      temp.W=this->previous_speed_w - this->speed_ramp;
    }
  }

  this->previous_speed_x=temp.vX;
  this->previous_speed_y=temp.vY;
  this->previous_speed_w=temp.W;



  // Serial.print("command word vx = ");
  // Serial.println(temp.vX);
  // Serial.print("command word vy = ");
  // Serial.println(temp.vY);
  // Serial.print("command word vw = ");
  // Serial.println(temp.W);

  return temp;
}

Speed ROBOT::speed_local_from_speed_world(Speed world_speed)
{
  Speed temp;
  temp.vX= cos_deg(position_world.A)*world_speed.vX + sin_deg(position_world.A)*world_speed.vY;
  temp.vY=-sin_deg(position_world.A)*world_speed.vX + cos_deg(position_world.A)*world_speed.vY;
  temp.W=world_speed.W;

  // Serial.print("temp.vX local = ");
  // Serial.println(temp.vX);
  // Serial.print("temp.vY local = ");
  // Serial.println(temp.vY);
  // Serial.print("temp.W local = ");
  // Serial.println(temp.W);

  return temp;
}

Omni ROBOT::omni_from_speed_local(Speed local_speed){
  Omni temp;
  temp.x1= local_speed.vX     + 0                    + local_speed.W;
  temp.x2=-local_speed.vX/2.0 - 0.866*local_speed.vY + local_speed.W;
  temp.x3=-local_speed.vX/2.0 + 0.866*local_speed.vY + local_speed.W;


  float max = max_float(max_float(abs(temp.x1), abs(temp.x2)), abs(temp.x3));

  if(max>max_speed_motor){
    temp.x1=-(temp.x1/max)*max_speed_motor;
    temp.x2=-(temp.x2/max)*max_speed_motor;
    temp.x3=-(temp.x3/max)*max_speed_motor;
    // Serial.println(1);
  }
  else{
    temp.x1=-temp.x1;
    temp.x2=-temp.x2;
    temp.x3=-temp.x3;   
  }

  // Serial.print("temp.x1 omni = ");
  // Serial.println(temp.x1);
  // Serial.print("temp.x2 omni = ");
  // Serial.println(temp.x2);
  // Serial.print("temp.x3 omni = ");
  // Serial.println(temp.x3);
  // Serial.println();
  return temp;
}

void ROBOT::control_motor_speed(float speed_1, float speed_2, float speed_3)
{

  if(speed_1>0.0f){digitalWrite(dirPin1, LOW);} 
  else{digitalWrite(dirPin1, HIGH);}
  if(speed_2>0.0f){digitalWrite(dirPin2, LOW);} 
  else{digitalWrite(dirPin2, HIGH);}
  if(speed_3>0.0f){digitalWrite(dirPin3, LOW);} 
  else{digitalWrite(dirPin3, HIGH);}

  speed_1=abs(speed_1)*10.0;
  speed_2=abs(speed_2)*10.0;
  speed_3=abs(speed_3)*10.0;

  int int_speed_1 = (int)round(speed_1);
  int int_speed_2 = (int)round(speed_2);
  int int_speed_3 = (int)round(speed_3);

  // control speed
  ledcWriteTone(pwmChannel_1, int_speed_1);
  ledcWriteTone(pwmChannel_2, int_speed_2);
  ledcWriteTone(pwmChannel_3, int_speed_3);
}

/////////////////////////////////

void ROBOT::clear_position()
{
  if (xSemaphoreTake(mutex_position, pdMS_TO_TICKS(10)) == pdTRUE){
    position_world.X=0;
    position_world.Y=0;
    position_world.A=0;
    xSemaphoreGive(mutex_position);
  }
}

float cos_deg(float theta){
  return cos(theta*(PI/180));
}

float sin_deg(float theta){
  return sin(theta*(PI/180));
}

float min_float(float a, float b){
  if(a<b){return a;}
  else{return b;};
}

float max_float(float a, float b){
  if(a<b){return b;}
  else{return a;};
}

float norme_carre(float a, float b){
  return sqrt(a*a+b*b);
}

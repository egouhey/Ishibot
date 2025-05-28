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
    robot->ReadEncoderMotor(0xe2); //+ 20693
    vTaskDelay(5/portTICK_PERIOD_MS);
    d_motor_3 = robot->ReadEncoderMotor(0xe3);
    vTaskDelay(5/portTICK_PERIOD_MS);
    d_motor_2 = robot->ReadEncoderMotor(0xe1);
    vTaskDelay(5/portTICK_PERIOD_MS);
    d_motor_1 = robot->ReadEncoderMotor(0xe9);
    robot->omni.x1=((d_motor_1-last_d_motor_1)/robot->interval);
    robot->omni.x2=((d_motor_2-last_d_motor_2)/robot->interval);
    robot->omni.x3=((d_motor_3-last_d_motor_3)/robot->interval);

    // Serial.print("x1 = ");
    // Serial.println(d_motor_1);
    // Serial.print("x2 = ");
    // Serial.println(d_motor_2);
    // Serial.print("x3 = ");
    // Serial.println(d_motor_3);


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
  int i=0;
  while(1){
    Position delta;
    delta.X = robot->objectif_world.X - robot->position_world.X;
    delta.Y = robot->objectif_world.Y - robot->position_world.Y;
    delta.A = robot->objectif_world.A - robot->position_world.A;
    // Serial.print("delta_x = ");
    // Serial.println(delta.X);
    // Serial.print("delta_y = ");
    // Serial.println(delta.Y);
    // Serial.print("delta_a = ");
    // Serial.println(delta.A);

    if(abs(delta.X)>2 || abs(delta.Y)>2 || abs(delta.A)>2  ){
      Speed command_speed_world = robot->speed_world_from_delta_position(delta);
      Speed command_speed_local = robot->speed_local_from_speed_world(command_speed_world);
      Omni  command_omni = robot->omni_from_speed_local(command_speed_local);
      robot->control_motor_speed(command_omni.x1, command_omni.x2, command_omni.x3);
    }
    else{

      if (i==0){
        robot->objectif_world.X=250;
        robot->objectif_world.Y=0;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==1){
        robot->objectif_world.X=250;
        robot->objectif_world.Y=250;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==2){
        robot->objectif_world.X=0;
        robot->objectif_world.Y=250;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==3){
        robot->objectif_world.X=0;
        robot->objectif_world.Y=0;
        robot->objectif_world.A=0;
        i++;
      }
      else if (i==4){
        vTaskDelay(100/portTICK_PERIOD_MS);
        i=0;
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

  // Serial.print("vx = ");
  // Serial.println(temp.vX);
  // Serial.print("vy = ");
  // Serial.println(temp.vY);
  // Serial.print("vw = ");
  // Serial.println(temp.W);

  return temp;
}

void ROBOT::position_world_from_speed_world(Speed world_speed){
  this->position_world.X+=2.0*PI*r_wheel*world_speed.vX*interval/read_step_per_turn;
  this->position_world.Y+=2.0*PI*r_wheel*world_speed.vY*interval/read_step_per_turn;
  this->position_world.A+=(world_speed.W*interval/read_step_per_turn);

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
  temp.vX=Kp_l*delta_positon.X;
  if(temp.vX>0){
    temp.vX=min_float(temp.vX, (this->tresh_l));
  }
  else{
    temp.vX=max_float(temp.vX, -(this->tresh_l));
  }
  
  temp.vY=Kp_l*delta_positon.Y;
  if(temp.vY>0){
    temp.vY=min_float(temp.vY, (this->tresh_l));
  }
  else{
    temp.vY=max_float(temp.vY, -(this->tresh_l));
  }

  temp.W=Kp_r*delta_positon.A;
  if(temp.W>0){
    temp.W=min_float(temp.W, (this->tresh_r));
  }
  else{
    temp.W=max_float(temp.W, -(this->tresh_r));
  }

  // Serial.print("vx = ");
  // Serial.println(temp.vX);
  // Serial.print("vy = ");
  // Serial.println(temp.vY);
  // Serial.print("vw = ");
  // Serial.println(temp.W);


  return temp;
}

Speed ROBOT::speed_local_from_speed_world(Speed world_speed)
{
  // cos(pos.a) * world_vel.vx + sin(pos.a) * world_vel.vy,
  //           -sin(pos.a) * world_vel.vx + cos(pos.a) * world_vel.vy,
  //           world_vel.w)
  Speed temp;
  temp.vX= cos_deg(position_world.A)*world_speed.vX + sin_deg(position_world.A)*world_speed.vY;
  temp.vY=-sin_deg(position_world.A)*world_speed.vX + cos_deg(position_world.A)*world_speed.vY;
  temp.W=world_speed.W;

  // Serial.print("temp.vX = ");
  // Serial.println(temp.vX);
  // Serial.print("temp.vY = ");
  // Serial.println(temp.vY);
  // Serial.print("temp.W = ");
  // Serial.println(temp.W);

  return temp;
}

Omni ROBOT::omni_from_speed_local(Speed local_speed)
{
            //   -vel.vx / 2.0 - sqrt(3.0) * vel.vy/2.0 + ROBOT_L * vel.w,
            // vel.vx + ROBOT_L * vel.w,
            // -vel.vx / 2.0 + sqrt(3.0) * vel.vy / 2.0 + ROBOT_L * vel.w
  Omni temp;
  temp.x1= local_speed.vX     + 0                    - local_speed.W; // + r_robot/r_wheel*
  temp.x2=-local_speed.vX/2.0 - 0.866*local_speed.vY - local_speed.W; // + r_robot/r_wheel*
  temp.x3=-local_speed.vX/2.0 + 0.866*local_speed.vY - local_speed.W; // + r_robot/r_wheel*

  Serial.print("temp.x1 = ");
  Serial.println(temp.x1);
  Serial.print("temp.x2 = ");
  Serial.println(temp.x2);
  Serial.print("temp.x3 = ");
  Serial.println(temp.x3);


  Serial.println();
  Serial.println();

  return temp;
}

void ROBOT::control_motor_speed(float speed_1, float speed_2, float speed_3)
{

  if(speed_1<0.0f){digitalWrite(dirPin1, LOW);} 
  else{digitalWrite(dirPin1, HIGH);}
  if(speed_2<0.0f){digitalWrite(dirPin2, LOW);} 
  else{digitalWrite(dirPin2, HIGH);}
  if(speed_3<0.0f){digitalWrite(dirPin3, LOW);} 
  else{digitalWrite(dirPin3, HIGH);}

  speed_1=abs(speed_1);
  speed_2=abs(speed_2);
  speed_3=abs(speed_3);

  // speed_1=min_float(abs(speed_1), this->max_speed_motor);
  // speed_2=min_float(abs(speed_2), this->max_speed_motor);
  // speed_3=min_float(abs(speed_3), this->max_speed_motor);

  // if(speed_1>10){
  // Serial.print("speed_1 = ");
  // Serial.println(speed_1);
  // Serial.print("speed_2 = ");
  // Serial.println(speed_2);
  // Serial.print("speed_3 = ");
  // Serial.println(speed_3);

  // Serial.println();
  // Serial.println();

  // }
  
  int int_speed_1 = (int)(speed_1);
  int int_speed_2 = (int)(speed_2);
  int int_speed_3 = (int)(speed_3);

  // if(int_speed_1>this->previous_speed_1+this->step_ramp){
  //   int_speed_1=previous_speed_1+this->step_ramp;
  // }
  // if(int_speed_2>this->previous_speed_2+this->step_ramp){
  //   int_speed_2=previous_speed_2+this->step_ramp;
  // }
  // if(int_speed_3>this->previous_speed_3+this->step_ramp){
  //   int_speed_3=previous_speed_3+this->step_ramp;
  // }

  // this->previous_speed_1 = int_speed_1;
  // this->previous_speed_2 = int_speed_2;
  // this->previous_speed_3 = int_speed_3;

  // control speed
  ledcWriteTone(pwmChannel_1, granularity*int_speed_1);
  ledcWriteTone(pwmChannel_3, granularity*int_speed_2);
  ledcWriteTone(pwmChannel_2, granularity*int_speed_3);
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


// void ROBOT::wheel2position(int step_motor_1, int step_motor_2, int step_motor_3) // TODO
// {
//   step_motor_1 = (int)step_motor_1 / 1.0;
//   step_motor_2 = (int)step_motor_2 / 1.0;
//   step_motor_3 = (int)step_motor_3 / 1.0;

//   // Serial.print("1 = ");
//   // Serial.println(step_motor_1);
//   // Serial.print("2 = ");
//   // Serial.println(step_motor_2);
//   // Serial.print("3 = ");
//   // Serial.println(step_motor_3);

//   float d_motor_A = (float)step_motor_1 / 65536; // ou (float)step_motor_1 / (float)65536;
//   float d_motor_B = (float)step_motor_2 / 65536; // ou (float)step_motor_2 / (float)65536;
//   float d_motor_C = (float)step_motor_3 / 65536; // ou (float)step_motor_3 / (float)65536;
//   d_motor_A = 2*PI*r_wheel*d_motor_A;
//   d_motor_B = 2*PI*r_wheel*d_motor_B; 
//   d_motor_C = 2*PI*r_wheel*d_motor_C;

//   // Serial.print("1 = ");
//   // Serial.println(d_motor_A);
//   // Serial.print("2 = ");
//   // Serial.println(d_motor_B);
//   // Serial.print("3 = ");
//   // Serial.println(d_motor_C);


//   if (xSemaphoreTake(mutex_position, pdMS_TO_TICKS(10)) == pdTRUE){
//     position.theta+=4*r_wheel*((-1/(3*r_robot))*d_motor_A + ((-cos_deg(position.theta)+sin_deg(position.theta))/(3*r_robot))*d_motor_B + ((-cos_deg(position.theta)-sin_deg(position.theta))/(3*r_robot))*d_motor_C);
//     position.X+=r_wheel*((2/3)*d_motor_A + ((-cos_deg(position.theta)+sin_deg(position.theta))/3)*d_motor_B + ((-cos_deg(position.theta)-sin_deg(position.theta))/3)*d_motor_C);
//     position.Y+=r_wheel*((0)*d_motor_A + ((-cos_deg(position.theta)+sin_deg(position.theta))/(2*sin(PI/3)))*d_motor_B + ((-cos_deg(position.theta)+sin_deg(position.theta))/(2*sin(PI/3)))*d_motor_C);
//     Serial.print("theta = ");
//     Serial.println(position.theta);
//     Serial.print("x = ");
//     Serial.println(position.X);
//     Serial.print("y = ");
//     Serial.println(position.Y);
//     Serial.println();
//     xSemaphoreGive(mutex_position);
//   }
// }


// void ROBOT::task_robot_encoder(void *pvParameters)
// {
//   ROBOT *robot = static_cast<ROBOT*>(pvParameters);
//   int d_motor_1=0;
//   int d_motor_2=0;
//   int d_motor_3=0;
//   int last_d_motor_1=0;
//   int last_d_motor_2=0;
//   int last_d_motor_3=0;

//   while(1){
//     last_d_motor_1=d_motor_1;
//     last_d_motor_2=d_motor_2;
//     last_d_motor_3=d_motor_3;

//     while (robot->robotSerial->available() > 0) {
//       char c = robot->robotSerial->read();
//     }
//     robot->ReadEncoderMotor(0xe2); //+ 20693
//     vTaskDelay(5/portTICK_PERIOD_MS);
//     d_motor_3 = robot->ReadEncoderMotor(0xe3);
//     vTaskDelay(5/portTICK_PERIOD_MS);
//     d_motor_2 = robot->ReadEncoderMotor(0xe1);
//     vTaskDelay(5/portTICK_PERIOD_MS);
//     d_motor_1 = robot->ReadEncoderMotor(0xe9);

//     // Serial.print("1 = ");
//     // Serial.println(d_motor_1-last_d_motor_1);
//     // Serial.print("2 = ");
//     // Serial.println(d_motor_2-last_d_motor_2);
//     // Serial.print("3 = ");
//     // Serial.println(d_motor_3-last_d_motor_3);

//     robot->wheel2position(d_motor_1-last_d_motor_1, d_motor_2-last_d_motor_2, d_motor_3-last_d_motor_3);
//     vTaskDelay(20/portTICK_PERIOD_MS);

//     // Serial.println();
//     // vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }

// _Alpha     =  _Alpha_Save+ (((-1.0/(3.0*RADIUS))*StepperB->getPosition()) + ((-1.0/(3.0*RADIUS))*StepperA->getPosition()) + ((-1.0/(3.0*RADIUS))*StepperC->getPosition()))*KSTP/(PI/180.0);
// _positionX = _positionX_Save+ ((-cos((PI/180.0)*_Alpha)/6.0)*StepperA->getPosition() + ((cos((PI/180.0)*_Alpha)-sqrt(3)*sin((PI/180.0)*_Alpha))/12.0)*StepperB->getPosition() + ((sqrt(3)*sin((PI/180.0)*_Alpha)+cos((PI/180.0)*_Alpha))/12.0)*StepperC->getPosition())*KSTP*4;
// _positionY = _positionY_Save+ ((sin((PI/180.0)*_Alpha)/6.0)*StepperA->getPosition() - ((sin((PI/180.0)*_Alpha)+sqrt(3)*cos((PI/180.0)*_Alpha))/12.0)*StepperB->getPosition() + ((sqrt(3)*cos((PI/180.0)*_Alpha)-sin((PI/180.0)*_Alpha))/12.0)*StepperC->getPosition())*KSTP*4;

// RADUIS = L = r_robot
// StepperA->move(int(((-RADIUS*_MoveAlpha*(PI/180.0)) - cos((PI/180.0)*_Alpha)*_MovepositionX + sin((PI/180.0)*_Alpha)*_MovepositionY)/KSTP));
// StepperB->move(int(((-RADIUS*_MoveAlpha*(PI/180.0)) + cos((PI/180.0)*(THETA+_Alpha))*_MovepositionX - sin((PI/180.0)*(THETA+_Alpha))*_MovepositionY)/KSTP));
// StepperC->move(int(((-RADIUS*_MoveAlpha*(PI/180.0)) + cos((PI/180.0)*(THETA-_Alpha))*_MovepositionX + sin((PI/180.0)*(THETA-_Alpha))*_MovepositionY)/KSTP));

//  distance_angulaire_roue_1 = (-m.sin(initial_theta)*deplacement_x+m.cos(initial_theta)*deplacement_y +distance_roue_centre*deplacement_angle)*(1/rayon_roue); #en Rad
//     distance_angulaire_roue_2 = (-m.sin(m.pi/3-initial_theta)*deplacement_x - m.cos(m.pi/3-initial_theta)*deplacement_y +distance_roue_centre*deplacement_angle)*(1/rayon_roue); # en Rad
//     distance_angulaire_roue_3 = (m.sin(m.pi/3+initial_theta)*deplacement_x - m.cos(m.pi/3+initial_theta)*deplacement_y +distance_roue_centre*deplacement_angle)*(1/rayon_roue); # en Rad


// int ROBOT::position2wheel(float moveX, float moveY, float movetheta)  // TODO
// {
//   if (xSemaphoreTake(mutex_position, pdMS_TO_TICKS(10)) == pdTRUE){
//     int step_motor_1 = (int) (-r_robot*movetheta - cos_deg(position.theta)*moveX + sin_deg(position.theta)*moveY)/Kstp;
//     int step_motor_2 = (int) (-r_robot*movetheta + cos_deg(position.theta)*moveX - sin_deg(position.theta)*moveY)/Kstp;
//     int step_motor_3 = (int) (-r_robot*movetheta + cos_deg(position.theta)*moveX + sin_deg(position.theta)*moveY)/Kstp;
//     xSemaphoreGive(mutex_position);
//     return step_motor_1, step_motor_2, step_motor_3;
//   }
//   else{
//     return 0, 0, 0;
//   }
// }
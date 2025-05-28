#include "motor.h"

void setup_motor()
{
  // motor 1
  pinMode(enPin1, OUTPUT);    digitalWrite(enPin1, HIGH);      
  pinMode(stpPin1, OUTPUT);   digitalWrite(stpPin1, LOW);    
  pinMode(dirPin1, OUTPUT);   digitalWrite(dirPin1, LOW);    

  // motor 2
  pinMode(enPin2, OUTPUT);    digitalWrite(enPin2, HIGH);     
  pinMode(stpPin2, OUTPUT);   digitalWrite(stpPin2, LOW);    
  pinMode(dirPin2, OUTPUT);   digitalWrite(dirPin2, LOW);  

  // motor 3
  pinMode(enPin3, OUTPUT);    digitalWrite(enPin3, HIGH);     
  pinMode(stpPin3, OUTPUT);   digitalWrite(stpPin3, LOW);
  pinMode(dirPin3, OUTPUT);   digitalWrite(dirPin3, LOW); 
}

void setup_control_motor()
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

void control_motor_speed( uint32_t freq_1, bool dir_1, uint32_t freq_2, bool dir_2, uint32_t freq_3, bool dir_3)
{

    // control direction
  digitalWrite(dirPin1, dir_1);
  digitalWrite(dirPin2, dir_2);
  digitalWrite(dirPin3, dir_3);

  // control speed
  ledcWriteTone(pwmChannel_1, granularity*freq_1);
  ledcWriteTone(pwmChannel_2, granularity*freq_2);
  ledcWriteTone(pwmChannel_3, granularity*freq_3);
}

void control_motor_position(int step_motor1, int step_motor2, int step_motor3,int time)
{
  if(step_motor1<0){
    step_motor1=(-1)*step_motor1;
    digitalWrite(dirPin1, LOW); 
  }
  else{
    digitalWrite(dirPin1, HIGH); 
  }

  if(step_motor2<0){
    step_motor2=(-1)*step_motor2;
    digitalWrite(dirPin2, LOW); 
  }
  else{
    digitalWrite(dirPin2, HIGH); 
  }

  if(step_motor3<0){
    step_motor3=(-1)*step_motor3;
    digitalWrite(dirPin3, LOW); 
  }
  else{
    digitalWrite(dirPin3, HIGH); 
  }

  uint32_t freq_1 = 500*step_motor1/time;
  uint32_t freq_2 = 500*step_motor2/time;
  uint32_t freq_3 = 500*step_motor3/time;

  ledcWriteTone(pwmChannel_1,freq_1);
  ledcWriteTone(pwmChannel_2,freq_2);
  ledcWriteTone(pwmChannel_3,freq_3);

  uint32_t startMillis = millis();
  while(millis()-startMillis<time){}

  ledcWriteTone(pwmChannel_1, 0);
  ledcWriteTone(pwmChannel_2, 0);
  ledcWriteTone(pwmChannel_3, 0);
}

void test_motor(const char* cstr)
{
  if(cstr=="carre"){
    control_motor_speed(10, LOW, 20, HIGH, 10, LOW);
    delay(2000);
    control_motor_speed(10, LOW, 0, HIGH, 10, HIGH);
    delay(2000);
    control_motor_speed(10, HIGH, 20, LOW, 10, HIGH);
    delay(2000);
    control_motor_speed(10, HIGH, 0, LOW, 10, LOW);
    delay(2000);
  }
  else if (cstr=="tourne"){
    control_motor_speed(10, HIGH, 10, HIGH, 10, HIGH);
    delay(20000);
  }
  else{
    control_motor_speed(0, HIGH, 0, HIGH, 0, HIGH);
  }
}

void setup_encoder()
{
  Serial1.begin(38400, SERIAL_8N1, 12, 14);
}

int ReadEncoderMotor1() 
{
  const byte message[] = {0xe0,0x36,0x16}; 
  Serial1.write(message, sizeof(message));
  vTaskDelay(pdMS_TO_TICKS(10));
  uint16_t motorData[8];                                   
  if (Serial1.available() > 0) 
  {
    int numBytes = Serial1.available();
    for (int i = 0; i < numBytes; i++)
    {
      motorData[i] = Serial1.read();
      if (motorData[0] != 0xE0) 
      {
        break;
      }
    }
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

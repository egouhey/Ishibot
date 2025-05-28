#include "motor.h"

volatile int encoder = 0;
volatile int last_encoder = 0;

void setup() {
  Serial.begin(115200);
  setup_motor();
  setup_control_motor();
  setup_encoder();
  control_motor_speed(10, HIGH, 0, LOW, 0, LOW);
}

void loop() {
  // control_motor_speed(5, LOW, 0, LOW, 0, LOW);
  // encoder=ReadEncoderMotor1();
  // Serial.println(encoder-last_encoder);
  // vTaskDelay(pdMS_TO_TICKS(5));
  // while (encoder-last_encoder<65535){
  //   encoder=ReadEncoderMotor1();
  //   //Serial.println(encoder-last_encoder);
  // }
  // // control_motor_speed(0, LOW, 0, LOW, 0, LOW);
  // // Serial.println(encoder-last_encoder);
  // last_encoder=encoder;
  delay(1000);
}
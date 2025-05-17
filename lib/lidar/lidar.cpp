#include <lidar.h>

LIDAR::LIDAR(HardwareSerial &serial, uint8_t RxPin, uint8_t TxPin = 255){
  this->lidarSerial=&serial;
  this->Rxpin=RxPin;
  this->Txpin=TxPin;
  this->setup_uart();
}

void LIDAR::setup_uart(){
  lidarSerial->setRxBufferSize(1024); // min (376)
  lidarSerial->setRxTimeout(1); 
  lidarSerial->begin(this->baudrate, SERIAL_8N1, this->Rxpin, this->Txpin);
}

void IRAM_ATTR LIDAR::onUARTReceive(){
  this->read_data();
}

void Lidar_start_interrupt(LIDAR *lidar){
  lidar->lidarSerial->onReceive( [lidar]() { (lidar)->onUARTReceive(); }, true); 
}

void LIDAR::start_interrupt(){
  Lidar_start_interrupt(this);
}

void LIDAR::stop_interrupt(){
  this->lidarSerial->onReceive(NULL); 
}

bool LIDAR::read_data(){
  while (lidarSerial->available()) {
    uint8_t current = lidarSerial->read();
    this->currentbuffer->tab[this->currentbuffer->index]=current;
    this->currentbuffer->index++;
    if (this->currentbuffer->index == length_tab-1){
      this->swapBuffers();
      xTaskCreate(LIDAR::task_lidar, "task_lidar", 1024, this, 1, NULL);
      return true;
    }
  }
  return false;
}

void LIDAR::analyse_data(){
  int i=0;
  Serial.println();
  while(this->previousbuffer->tab[i]!=LIDAR_HEADER && this->previousbuffer->tab[i+1]!=LIDAR_VER_SIZE && i<length_tab-2){
      i++;
  }
  while(i<length_tab-48){
      if ( (this->previousbuffer->tab[i]==LIDAR_HEADER && this->previousbuffer->tab[i+1]==LIDAR_VER_SIZE) 
      && (this->previousbuffer->tab[i+LIDAR_PACKET_SIZE]==LIDAR_HEADER && this->previousbuffer->tab[i+1+LIDAR_PACKET_SIZE]==LIDAR_VER_SIZE)){
        int angle = (int) (0.01*(((this->previousbuffer->tab[i+5])<<8) + (this->previousbuffer->tab[i+4])));
        int distance = ((this->previousbuffer->tab[i+7])<<8) + (this->previousbuffer->tab[i+6]);
        if (20<distance && distance < 100){
          Serial.print("angle = ");
          Serial.println(angle);
          Serial.print("distance = ");
          Serial.println(distance);
          this->stop=true;
        }
      }
      i++;
  }
}

void LIDAR::task_lidar(void *pvParameters){
  LIDAR *lidar = static_cast<LIDAR*>(pvParameters);
  lidar->analyse_data();
  vTaskDelay(pdMS_TO_TICKS(50)); // 50 millis secondes
  vTaskDelete(NULL);
}

void LIDAR::swapBuffers() {
  if (change_buffer) {
    currentbuffer = &bufferB;
    previousbuffer = &bufferA;
  } else {
    currentbuffer = &bufferA;
    previousbuffer = &bufferB;
  }
  change_buffer = !change_buffer;
  currentbuffer->index = 0;
}
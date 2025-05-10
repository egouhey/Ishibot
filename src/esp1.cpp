#include "Arduino.h"

#define LIDAR_TX 25  // RX2 (LD06 TX)
#define LIDAR_RX 27  // TX2 (LD06 RX)


#include <HardwareSerial.h>

#define LIDAR_SERIAL     Serial1
#define LIDAR_BAUDRATE   230400

#define LD06_HEADER      0x54
#define LD06_VER_SIZE    0x2C
#define PACKET_SIZE      47

uint8_t packet[PACKET_SIZE];

void setup() {
  Serial.begin(115200);
  LIDAR_SERIAL.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX, LIDAR_TX); // GPIO16 (RX), GPIO17 (TX)
  Serial.println("LD06 Lidar démarré...");
}

bool readLidarPacket() {
  static int index = 0;

  while (LIDAR_SERIAL.available()) {
    uint8_t byte = LIDAR_SERIAL.read();

    if (index == 0 && byte != LD06_HEADER) return false;
    if (index == 1 && byte != LD06_VER_SIZE) { index = 0; return false; }

    packet[index++] = byte;

    if (index == PACKET_SIZE) {
      index = 0;
      return true;
    }
  }
  return false;
}

void printLidarData() {
  uint16_t startAngle = packet[2] | (packet[3] << 8);
  uint16_t endAngle   = packet[42] | (packet[43] << 8);
  float fsa = startAngle / 100.0;
  float esa = endAngle / 100.0;
  float angleStep = (esa - fsa) / 11.0;

  for (int i = 0; i < 12; i++) {
    int offset = 4 + i * 3;
    uint16_t distance = packet[offset] | ((packet[offset + 1] & 0x1F) << 8);
    uint8_t  intensity = packet[offset + 2];
    float angle = fsa + angleStep * i;

    Serial.print("Angle: ");
    Serial.print(angle, 1);
    Serial.print("°\tDistance: ");
    Serial.print(distance);
    Serial.print(" mm\tIntensité: ");
    Serial.println(intensity);
  }

  Serial.println("--------------------");
}

void printLidarTeleplot() {
  uint16_t startAngle = packet[2] | (packet[3] << 8);
  uint16_t endAngle   = packet[42] | (packet[43] << 8);
  float fsa = startAngle / 100.0;
  float esa = endAngle / 100.0;
  float angleStep = (esa - fsa) / 11.0;

  Serial.print(">lidar:");  // <- Début du message Teleplot
  for (int i = 0; i < 12; i++) {
    int offset = 4 + i * 3;
    uint16_t distance = packet[offset] | ((packet[offset + 1] & 0x1F) << 8);
    float angle = fsa + angleStep * i;

    // Calcul en XY (coordonnées cartésiennes)
    float rad = angle * PI / 180.0;
    float x = distance * cos(rad);
    float y = distance * sin(rad);

    Serial.print(x, 1);
    Serial.print(":");
    Serial.print(y, 1);
    Serial.print(";");
  }
  Serial.println("|xy");  // <- Fin du message Teleplot
}

void loop() {
  if (readLidarPacket()) {
    printLidarTeleplot();
  }
}

// #include "kinematics.h"
// #include "motor.h"
// // #include "IMU.h"
// #include "screen.h"

// void yellow_game(){
//   move_regular_old(0,300);
//   delay(100);
//   move_regular_old(-425, 0);
//   delay(100);
//   move_regular_old(0, -250);
//   delay(100);
//   move_regular_old(0,300);
//   delay(100);
//   move_regular_old(-125,0);
//   delay(100);
//   move_regular_old(0,550);
//   delay(100);
//   move_regular_old(400,0);
//   delay(100);
//   move_regular_old(200,-700);
//   delay(100);
//   // move_regular_old(-850,1150);
//   // delay(100);
// }

// void blue_game(){
//   move_regular_old(0,300);
//   delay(100);
//   move_regular_old(425, 0);
//   delay(100);
//   move_regular_old(0, -250);
//   delay(100);
//   move_regular_old(0,300);
//   delay(100);
//   move_regular_old(125,0);
//   delay(100);
//   move_regular_old(0,550);
//   delay(100);
//   move_regular_old(-400,0);
//   delay(100);
//   move_regular_old(-200,-700);
//   delay(100);
//   // move_regular_old(850,1150);
//   // delay(100);
// }


// void setup() {
//   Serial.begin(115200);
//   setup_motor();
//   setup_control_motor();
//   Serial.println("setup");
//   setup_screen();
// }

// void loop() {
//   if (go && yellow_team){
//     go=false;
//     yellow_game();
//   }
//   else if (go && blue_team){
//     go=false;
//     blue_game();
//   }
//   else{screen();}
//   vTaskDelay(100);
// }

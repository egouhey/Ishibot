#include "BluetoothSerial.h"
#include "All.h"

String device_name = "ESP32-BT-Slave";

BluetoothSerial SerialBT;

void setup(){
    Serial.begin(115200);
    SerialBT.begin(device_name); 
    setup_motor();
    setup_control_motor();
}

void loop(){
  if (SerialBT.available()) {
    char data = SerialBT.read();
    Serial.println(data);
    if(data=='1'){ // avant
        while(data!='5'){
            move_regular_old(0, 10);
            data = SerialBT.read();
        }
    }

    else if(data=='2'){ // gauche
        while(data!='5'){
            move_regular_old(10, 0);
            data = SerialBT.read();
        }
    }

    else if(data=='3'){   // droite
        while(data!='5'){
            move_regular_old(-10, 0);
            data = SerialBT.read();
        }
    }

    else if(data=='4'){ // arrière
        while(data!='5'){
            move_regular_old(0, -10);
            data = SerialBT.read();
        }
    }
  }
  delay(20);
}
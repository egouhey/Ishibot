#include "IMU.h"

Adafruit_MPU6050 mpu;

signed int delta_x= 0, delta_y= 0, delta_z = 0;
signed int current_accelx= 0, current_accely= 0, current_accelz=0;
signed int vel_x= 0, vel_y= 0, vel_z=0;
signed int pos_x= 0, pos_y= 0, pos_z=0;
unsigned long lastTime=0;
int epsilon=50;

void setup_IMU() {
    Serial.begin(115200);
    while (!Serial) delay(10);  // Attente de l'ouverture du moniteur série

    // Initialisation du MPU6050
    if (!mpu.begin()) {
        Serial.println("Impossible de trouver le MPU6050. Vérifiez les connexions !");
        while (1) {
        delay(10);
        }
    }

    // Configuration des réglages pour détecter les mouvements lents
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);    // Plage de ±2g pour plus de précision
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);         // Plage de ±250°/s pour les rotations lentes
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);       // Réduction du bruit avec filtre passe-bas

    for(int i=0; i<2000; i++){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        delta_x+= a.acceleration.x*1000;
        delta_y+= a.acceleration.y*1000;
        delta_z+= a.acceleration.z*1000;
        delay(1);
    }
    delta_x/=2000;
    delta_y/=2000;
    delta_z/=2000;

    Serial.println("MPU6050 prêt !");
    delay(100);
    lastTime = millis();
}

void loop_IMU() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convertir en secondes
    lastTime = currentTime;
    Serial.println(deltaTime);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    signed int accelx = ((signed int) (a.acceleration.x*1000)) - delta_x;
    signed int accely = ((signed int) (a.acceleration.y*1000)) - delta_y;
    signed int accelz = ((signed int) (a.acceleration.z*1000)) - delta_z;

  // Affichage des données d'accélération sur les 3 axes
    Serial.print(" Acceleration mm/s², X: ");
    Serial.print(accelx);
    Serial.print(" mm/s², Y: ");
    Serial.print(accely);
    Serial.print(" mm/s², Z: ");
    Serial.print(accelz);
    Serial.println(" mm/s²");

    if ( accelx>epsilon || -epsilon>accelx ){
        vel_x += accelx * deltaTime;
        pos_x += vel_x * deltaTime + 0.5 * accelx * deltaTime * deltaTime;
    }
    if ( accely>epsilon || -epsilon>accely ){
        vel_y += accely * deltaTime;
        pos_y += vel_y * deltaTime + 0.5 * accely * deltaTime * deltaTime;
    }
    if ( accelz>epsilon || -epsilon>accelz ){
        vel_z += accelz * deltaTime;
        pos_z += vel_z * deltaTime + 0.5 * accelz * deltaTime * deltaTime;
    }

    Serial.print(" Position X: ");
    Serial.print(pos_x);
    Serial.print(" mm, Y: ");
    Serial.print(pos_y);
    Serial.print(" mm, Z: ");
    Serial.print(pos_z);
    Serial.println(" mm");


    delay(1000);  // Délai pour stabiliser les lectures
}

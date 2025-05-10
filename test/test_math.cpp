#include <math.h>
#include <Arduino.h>

hw_timer_t *timer = NULL;  // Déclaration du timer hardware
volatile int compteur = 0; // Compteur global (volatile pour être modifié dans l'ISR)

void IRAM_ATTR onTimer() {
    compteur++; // Incrémente le compteur à chaque overflow
}

void setup() {
    Serial.begin(115200);
    timer = timerBegin(0, 80, true); // Timer 0, prescaler de 80 (1 MHz, soit 1 µs par tick)
    timerAttachInterrupt(timer, &onTimer, true); // Lier l'ISR au timer
    timerAlarmWrite(timer, 5, true); // Déclenchement tous les 1000 cycles (1ms)
    timerAlarmEnable(timer); // Activer l'interruption
}

void loop() {
    delay(3000);
    double angle_degrees = 45; // Angle en degrés
    double angle_radians = angle_degrees * (M_PI / 180.0); // Conversion en radians

    int time_b_ms=compteur;
    double sinus_val = sin(angle_radians);
    int time_e_ms=compteur;

    Serial.print("Sinus de ");
    Serial.print(angle_degrees);
    Serial.print("° : ");
    Serial.println(sinus_val);  

    Serial.print("time end ");
    Serial.println(time_e_ms);
        Serial.print("time begin ");
    Serial.println(time_b_ms);
    Serial.print("compteur ");
    Serial.println(compteur );
}
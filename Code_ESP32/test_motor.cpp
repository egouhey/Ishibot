#include <Arduino.h>

#define   enPin1     15    
#define   stpPin1    2    
#define   dirPin1    0  

#define   enPin2     4   
#define   stpPin2    16  
#define   dirPin2    17 

#define   enPin3     19  
#define   stpPin3    18  
#define   dirPin3    5   


int i = 0;  
int DELAY_STEPPER = 200;

void setup() {
  Serial.begin(115200);

  pinMode(enPin1, OUTPUT);  digitalWrite(enPin1, LOW);  
  pinMode(stpPin1, OUTPUT);  digitalWrite(stpPin1, LOW);  
  pinMode(dirPin1, OUTPUT);  digitalWrite(dirPin1, LOW);  

  pinMode(enPin2, OUTPUT);  digitalWrite(enPin2, LOW);  
  pinMode(stpPin2, OUTPUT);  digitalWrite(stpPin2, LOW);  
  pinMode(dirPin2, OUTPUT);  digitalWrite(dirPin2, LOW);  


  pinMode(enPin3, OUTPUT);  digitalWrite(enPin3, LOW);  
  pinMode(stpPin3, OUTPUT);  digitalWrite(stpPin3, LOW); 
  pinMode(dirPin3, OUTPUT);  digitalWrite(dirPin3, LOW); 
}

void loop() {

  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  digitalWrite(dirPin3, LOW);
  Serial.println(1);
  while(i < 6400){
    digitalWrite(stpPin1, !digitalRead(stpPin1));
    digitalWrite(stpPin2, !digitalRead(stpPin2));
    digitalWrite(stpPin3, !digitalRead(stpPin3));
    delayMicroseconds(DELAY_STEPPER);
    i++;
  }
  delay(1000); 
  

  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);
  digitalWrite(dirPin3, HIGH);
  while(i > 0){
    digitalWrite(stpPin1, !digitalRead(stpPin1));
    digitalWrite(stpPin2, !digitalRead(stpPin2));
    digitalWrite(stpPin3, !digitalRead(stpPin3));
    delayMicroseconds(DELAY_STEPPER);
    i--;
  }
  delay(1000); 

}
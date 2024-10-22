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

volatile int motorSteps_1 = 0; 
volatile int motorSteps_2 = 0;  
volatile int motorSteps_3 = 0;  
TaskHandle_t TaskHandle_motor_1 = NULL;
TaskHandle_t TaskHandle_motor_2 = NULL;
TaskHandle_t TaskHandle_motor_3 = NULL;

bool ready_1 = true;
bool ready_2 = true;
bool ready_3 = true;

volatile float Delay_stepper_choose = 200.0;
volatile float A_choose = 0.2;
//volatile float step_total = 3200;
volatile float X_choose= 5.0;

int trapeze(int step, float step_total, float X, float A, float Delay_stepper) {

    if (step <= (int)(A * step_total)) {
        float res = Delay_stepper*( X + ((1-X)*step)/(A*step_total));
        return (int) res;
    } 
    
    else if ((int)((1.0 - A) * step_total) <= step) {
        float res = Delay_stepper*(X - (X-1)/A + step*(X-1)/(A*step_total));
        return (int) res;
    } 
    
    else if ( ((int)(A * step_total) )< step || step < ((int)((1.0 - A) * step_total)) ) {
        return (int) Delay_stepper;
    } 
    
    else {
        return (int) Delay_stepper*5.0;
    }
}

bool sens_1 = true;

void Task_motor_1(void *pvParameters) {
    while(1) {
        if (motorSteps_1 != 0) {   
            digitalWrite(dirPin1, !digitalRead(dirPin1));
            int stepsToDo = motorSteps_1 * 6400 / 2;

            for (int i = 0; i < stepsToDo; i++) {
                int a = trapeze(i, stepsToDo, X_choose, A_choose, Delay_stepper_choose);
                digitalWrite(stpPin1, HIGH);
                delayMicroseconds(a);
                digitalWrite(stpPin1, LOW);
                delayMicroseconds(a);
            }
            ready_1=true;
            motorSteps_1 = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

bool sens_2=true;

void Task_motor_2(void *pvParameters){
    while(1){
        if (motorSteps_2 != 0) {  
            digitalWrite(dirPin2, !digitalRead(dirPin2));
            int stepsToDo = abs(motorSteps_2) * 6400 / 2;

            for (int i = 0; i < stepsToDo; i++) {
                int a = trapeze(i, stepsToDo, X_choose, A_choose, Delay_stepper_choose);
                digitalWrite(stpPin2, HIGH);
                delayMicroseconds(a);
                digitalWrite(stpPin2, LOW);
                delayMicroseconds(a);
                if (motorSteps_2 == 0) {
                    break; 
                }
            }
            ready_2 = true;
            motorSteps_2 = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

bool sens_3= true;

void Task_motor_3(void *pvParameters){
    while(1){
        if (motorSteps_3 != 0) {
            
            digitalWrite(dirPin3, !digitalRead(dirPin3));
            int stepsToDo = abs(motorSteps_3) * 6400 / 2;

            for (int i = 0; i < stepsToDo; i++) {
                int a = trapeze(i, stepsToDo, X_choose, A_choose, Delay_stepper_choose);
                digitalWrite(stpPin3, HIGH);
                delayMicroseconds(a);
                digitalWrite(stpPin3, LOW);
                delayMicroseconds(a);
            }
            ready_3 = true;
            motorSteps_3 = 0;             
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

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
    
    xTaskCreate(Task_motor_1, "TaskMotor_1", 2048, NULL, 1, &TaskHandle_motor_1);
    xTaskCreate(Task_motor_2, "TaskMotor_2", 2048, NULL, 1, &TaskHandle_motor_2);
    xTaskCreate(Task_motor_3, "TaskMotor_3", 2048, NULL, 1, &TaskHandle_motor_3);
}

void loop() {
    if(ready_1 & ready_2 & ready_3){
        delay(1000);
        motorSteps_1=1;
        motorSteps_2=1;
        motorSteps_3=1;
        ready_1=false;
        ready_2=false;
        ready_3=false;
    }
}

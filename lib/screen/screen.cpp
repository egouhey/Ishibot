#include "screen.h"

volatile bool blue_team=true;
volatile bool yellow_team=false;
volatile bool carre=false;
volatile bool go=false;

volatile bool state_bouton1=false;
volatile bool state_bouton2=false;
volatile bool state_bouton3=false;
volatile bool old_state_bouton1=false;
volatile bool old_state_bouton2=false;
volatile bool old_state_bouton3=false;

volatile bool state_menu=true;
volatile bool state_coupe=false;
volatile bool state_position=false;
volatile bool state_connect=false;

volatile int choice_menu=3;
volatile int choice_coupe=1;
volatile int choice_position=2;
volatile int choice_connect=3;

volatile bool flag_next=false;
volatile bool flag_enter=false;
volatile bool flag_menu=false;

Adafruit_SSD1306 oled(nombreDePixelsEnLargeur, nombreDePixelsEnHauteur, &Wire, brocheResetOLED);


TaskHandle_t task1_handle = NULL; 

void rectangle(int  x1, int  y1, int  x2, int  y2){
  for(int numeroLigne=y1 ; numeroLigne < y2 ; numeroLigne++) {
    for(int numeroColonne=x1 ; numeroColonne < x2 ; numeroColonne++) {
      oled.drawPixel(numeroColonne, numeroLigne, WHITE);          // On demande l'allumage de chaque pixel, un à un, avec la commande : drawPixel(posX,posY,couleur);
    }
  }
} 

void init_ecran(){
    if(!oled.begin(SSD1306_SWITCHCAPVCC, adresseI2CecranOLED)){
    Serial.println("Erreur ecran");
    return;
  }
  oled.clearDisplay();
  for(int numeroLigne=0 ; numeroLigne < nombreDePixelsEnHauteur ; numeroLigne++) {
    for(int numeroColonne=0 ; numeroColonne < nombreDePixelsEnLargeur ; numeroColonne++) {
      oled.drawPixel(numeroColonne, numeroLigne, WHITE);          // On demande l'allumage de chaque pixel, un à un, avec la commande : drawPixel(posX,posY,couleur);
    }
  }
  oled.display();

  delay(1000);

  oled.clearDisplay();
  oled.display();
  delay(1000);
  oled.setTextSize(2);
  oled.setCursor(0,0);
  oled.setTextColor(SSD1306_WHITE);
}

void menu(){
  choice_menu++;
  if(choice_menu==4){ choice_menu=0;  }
  oled.clearDisplay();

  // oled.setTextSize(2);
  // oled.setCursor(0,0);
  // oled.setTextColor(SSD1306_WHITE);

  oled.println("Coupe");
  oled.println("Position");
  oled.println("Connect");

  if(blue_team){
    oled.println("Go Blue");
  }
  else if(yellow_team){
    oled.println("Go Yellow");
  }
  else if(carre){
    oled.println("Go carre");
  }

  rectangle(96, choice_menu*16, 112, choice_menu*16+16);

  oled.display();
  delay(500);
}

void coupe(){
  choice_coupe++;
  if(choice_coupe==3){ choice_coupe=0;  }
  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setCursor(0,0);
  oled.setTextColor(SSD1306_WHITE);

  oled.println("Jaune");
  oled.println("Bleu");
  oled.println("Carre");

  if(blue_team){
    oled.println("Equipe : B");
  }
  else if(yellow_team){
    oled.println("Equipe : J");
  }
  else if(carre){
    oled.println("Equipe : C");
  }

  rectangle(96, choice_coupe*16, 112, choice_coupe*16+16);

  oled.display();
  delay(500);
}

void position(){
  choice_position++;
  if(choice_position==3){ choice_position=0;  }
  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setCursor(0,0);
  oled.setTextColor(SSD1306_WHITE);

  oled.println("X =");
  oled.println("Y =");
  oled.println("W =");

  rectangle(96, choice_position*16, 112, choice_position*16+16);

  oled.display();
  delay(500);
}

void connect(){
  choice_connect++;
  if(choice_connect==4){ choice_connect=1;  }
  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setCursor(0,0);
  oled.setTextColor(SSD1306_WHITE);

  oled.println("Connected to");
  oled.println("ESP 1");
  oled.println("ESP 2");
  oled.println("ESP 3");

  rectangle(96, choice_connect*16, 112, choice_connect*16+16);

  oled.display();
  delay(500);
}

void IRAM_ATTR make_menu() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {  // 200ms d'antirebond
    detachInterrupt(digitalPinToInterrupt(Pin_Button_1));  
    flag_menu = true;
    lastInterruptTime = interruptTime;
  }
}

void IRAM_ATTR make_next() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {  
    detachInterrupt(digitalPinToInterrupt(Pin_Button_2));
    flag_next = true;
    lastInterruptTime = interruptTime;
  }
}

void IRAM_ATTR make_enter() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {  
    detachInterrupt(digitalPinToInterrupt(Pin_Button_3));
    flag_enter = true;
    lastInterruptTime = interruptTime;
  }
}

void setup_screen() {
  init_ecran();
  menu();

  pinMode(Pin_Button_1, INPUT_PULLUP); 
  pinMode(Pin_Button_2, INPUT_PULLUP); 
  pinMode(Pin_Button_3, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(Pin_Button_1), make_menu, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_Button_2), make_next, RISING);
  attachInterrupt(digitalPinToInterrupt(Pin_Button_3), make_enter, RISING);
}

void screen(){
  if(flag_enter==true){
    flag_enter=false;
    Serial.println("enter");

    if (state_menu==true){
      switch (choice_menu) {
          case 0:
            coupe();
            state_menu=false;
            state_coupe=true;
            break;
          case 1:
            position();
            state_menu=false;
            state_position=true;
            break;
          case 2:
            connect();
            state_menu=false;
            state_connect=true;
            break;   
          case 3:
            go=true;
            menu();
            break;    
      }
    }

    else if (state_coupe==true){
      switch (choice_coupe) {
          case 0:
            blue_team=false;
            yellow_team=true;
            carre=false;
            coupe();
            break;
          case 1:
            blue_team=true;
            yellow_team=false;
            carre=false;
            coupe();
            break;  
          case 2:
            blue_team=false;
            yellow_team=false;
            carre=true;
            coupe();
            break;  
      }
    }
    
    attachInterrupt(digitalPinToInterrupt(Pin_Button_3), make_enter, RISING);
  }

  else if(flag_menu==true){
    flag_menu=false;
    Serial.println("menu");

    menu(); 
    state_menu=true;
    state_coupe=false;
    state_position=false;
    state_connect=false;

    attachInterrupt(digitalPinToInterrupt(Pin_Button_1), make_menu, RISING);
  }
  else if(flag_next==true){
    flag_next=false;
    Serial.println("next");

    if(state_menu==true){menu();}
    else if(state_coupe==true){ coupe(); }
    else if(state_position==true){ position(); }
    else if(state_connect==true){ connect(); }

    attachInterrupt(digitalPinToInterrupt(Pin_Button_2), make_next, RISING);
  }

  vTaskDelay(1);
}

void print_oled_GO(){
  oled.setCursor(0,0);
  oled.clearDisplay();
  oled.println("GO");
  oled.display();
}

void print_oled_STOP(){
  oled.setCursor(0,0);
  oled.clearDisplay();
  oled.println("STOP");
  oled.display();
}


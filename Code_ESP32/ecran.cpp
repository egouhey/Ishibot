#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <ezButton.h> 

#define nombreDePixelsEnLargeur     128         
#define nombreDePixelsEnHauteur     64          
#define brocheResetOLED             -1          
#define adresseI2CecranOLED         0x3C        

ezButton bouton1(25);                       
ezButton bouton2(26);                       
ezButton bouton3(2);                       

bool state_bouton1;
bool state_bouton2;
bool state_bouton3;
bool old_state_bouton1;
bool old_state_bouton2;
bool old_state_bouton3;

bool state_menu=true;
bool state_coupe=false;
bool state_position=false;
bool state_connect=false;

int choice_menu=2;
int choice_coupe=1;
int choice_position=2;
int choice_connect=3;

TaskHandle_t task1_handle = NULL; 

Adafruit_SSD1306 oled(nombreDePixelsEnLargeur, nombreDePixelsEnHauteur, &Wire, brocheResetOLED);

void rectangle(int  x1, int  y1, int  x2, int  y2){
  for(int numeroLigne=y1 ; numeroLigne < y2 ; numeroLigne++) {
    for(int numeroColonne=x1 ; numeroColonne < x2 ; numeroColonne++) {
      oled.drawPixel(numeroColonne, numeroLigne, WHITE);          
    }
  }
} 

void init_ecran(){

  oled.clearDisplay();
  for(int numeroLigne=0 ; numeroLigne < nombreDePixelsEnHauteur ; numeroLigne++) {
    for(int numeroColonne=0 ; numeroColonne < nombreDePixelsEnLargeur ; numeroColonne++) {
      oled.drawPixel(numeroColonne, numeroLigne, WHITE);          
    }
  }
  oled.display();

  delay(1000);

  oled.clearDisplay();
  oled.display();
  delay(1000);
}

void menu(){
  choice_menu++;
  if(choice_menu==3){ choice_menu=0;  }
  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setCursor(0,0);
  oled.setTextColor(SSD1306_WHITE);

  oled.println("Coupe");
  oled.println("Position");
  oled.println("Connect");

  rectangle(96, choice_menu*16, 112, choice_menu*16+16);

  oled.display();
  delay(500);
}


void coupe(){
  choice_coupe++;
  if(choice_coupe==2){ choice_coupe=0;  }
  oled.clearDisplay();

  oled.setTextSize(2);
  oled.setCursor(0,0);
  oled.setTextColor(SSD1306_WHITE);

  oled.println("Jaune");
  oled.println("Bleu");

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

void task1(void*parameters){
  while(1){
    bouton1.loop(); 
    bouton2.loop(); 
    bouton3.loop(); 

    old_state_bouton1 = state_bouton1;                                    //menu
    old_state_bouton2 = state_bouton2;                                    //select
    old_state_bouton3 = state_bouton3;                                    //next

    state_bouton1= bouton1.getState();
    state_bouton2= bouton2.getState();
    state_bouton3= bouton3.getState();

    if(state_bouton1 == HIGH && old_state_bouton1==LOW){                       //menu
      menu(); 
      state_menu=true;
      state_coupe=false;
      state_position=false;
      state_connect=false;
    }     
    else if(state_bouton2 == HIGH && old_state_bouton2==LOW){                  //select
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
                
        }
      }
    }
    else if(state_bouton3 == HIGH && old_state_bouton3==LOW){
      if(state_menu==true){ menu(); }
      else if(state_coupe==true){ coupe(); }
      else if(state_position==true){ position(); }
      else if(state_connect==true){ connect(); }
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  } 
}


void setup() {
  Serial.begin(115200);

  bouton1.setDebounceTime(50);
  bouton2.setDebounceTime(50);
  bouton3.setDebounceTime(50);

  if(!oled.begin(SSD1306_SWITCHCAPVCC, adresseI2CecranOLED)){
    Serial.println("Erreur ecran");
  }
  Serial.println("1");
  init_ecran();
  Serial.println("2");
  Serial.println("3");
}

void loop() {

  bouton1.loop();
  state_bouton1 = bouton1.getState();


  if(state_bouton1 == HIGH){
    Serial.println("ON");
  }
  else{
    Serial.println("OFF");
  }

}


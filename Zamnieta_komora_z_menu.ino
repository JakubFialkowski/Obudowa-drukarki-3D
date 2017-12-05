#include <Wire.h>
#include <LiquidCrystal_I2C.h> 
#include <OneWire.h>
#include <DS18B20.h>
#include <AM2320.h>
#include "HX711.h"
#include <PID_v1.h>
AM2320 th;


LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //definicja wyświetlacza
// definicja przycisków
#define BTN_BACK  11
#define BTN_NEXT  10
#define BTN_PREV  8
#define BTN_OK    9

#define ONEWIRE_PIN 4 // czujnik temp 18b20
#define SENSORS_NUM 3 // ilosc czujnikow 18b20

#define calibration_factor -2400 //Wartosc uzyskana za pomoca SparkFun_HX711_Calibration sketch
#define zero_factor 9959920//16180400 //wartosc uzyska za pomoca SparkFun_HX711_Calibration sketch

//definicja pinow dla HX711
#define DOUT  3
#define CLK  2

HX711 scale(DOUT, CLK);

// Adresy czujnikow DS18B20 dla magistrali one wire
const byte address[SENSORS_NUM][8] PROGMEM = {
  0x28, 0xAC, 0x71, 0x7B, 0x5, 0x0, 0x0, 0x96,
0x28, 0x61, 0x64, 0x11, 0xA9, 0x88, 0x81, 0xA0,
0x28, 0x61, 0x64, 0x11, 0xA9, 0xB7, 0x74, 0xDE
};
OneWire onewire(ONEWIRE_PIN);
DS18B20 sensors(&onewire);
int czujnik[3];
float czujnikAM[2];
//zmienne dla regulatorow PID
 int wentylator_gora = 6;
int wentylator_dol = 12;
 double Setpoint, Input, Output;
 double Setpoint1, Input1, Output1;

//Nastawy regulatorow PID
double Kp=80, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

double Kp1=80, Ki1=5, Kd1=1;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, REVERSE);

//struktura danych dla menu
typedef struct {
  String label;
  int minVal;
  int maxVal;
  int currentVal;
  void (*handler)();
} STRUCT_MENUPOS;

typedef enum {
  BACK, NEXT, PREV, OK, NONE
} ENUM_BUTTON;

STRUCT_MENUPOS menu[6]; //liczba pozycji menu

int currentMenuPos = 0;
int menuSize;
bool isInLowerLevel = false;
int tempVal;

void setup() {
  
  Serial.begin(9600);
  lcd.begin(20, 4);
  lcd.setBacklight(255);
  sensors.begin();
  sensors.request();
  scale.set_scale(calibration_factor); //ustawienie wagi na 0
  scale.set_offset(zero_factor); //ustawienie wagi na 0 przy znanej wartosci

  pinMode(5, OUTPUT);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(wentylator_gora, OUTPUT);
  //Wejscie i wartosc zadana dla PID
  Input = (czujnikAM[1]+czujnik[2])/2; //srednia z dwoch czujnikow wewnatrz strefy wydruku
  Setpoint = (menu[2].currentVal + menu[4].currentVal)/2 ; 

  Input1 = czujnik[1]; 
  Setpoint1 = menu[3].currentVal;
  //wlaczenie PIDow w trybie automatycznym
  myPID.SetMode(AUTOMATIC);
  myPID1.SetMode(AUTOMATIC);
  //definicja Menu
  menu[0] = {"Parametry", 0, 1, 5, parametry};
  menu[1] = {"Oswietlenie LED", 0, 1, 0, oswietlenieLed};
  menu[2] = {"Ustaw temp srodek ", 0, 100, 30, NULL};
  menu[3] = {"Ustaw temp dol", 0, 100, 30, NULL};
  menu[4] = {"Ustaw temp gora", 0, 100, 30, NULL};
  menu[5] = {"Ustaw czas druku", 0, 100, 40, NULL};
  menuSize = sizeof(menu)/sizeof(STRUCT_MENUPOS);
}

void loop() {
  drawMenu();
  if(currentMenuPos == 0) {
    parametry();
  }
  Input = (czujnikAM[1]+czujnik[0])/2;
  Setpoint = (menu[2].currentVal + menu[4].currentVal)/2 ;
  Input1 = czujnik[1];
  Setpoint1 = menu[3].currentVal;
  //Obliczenie PIDow
  myPID.Compute();
  myPID1.Compute();
  //Ograniczenie sygnału PWM wysterowania ( przy małych prędkościach obrotowych wentylatory wydają piskliwe dzwięki)
  if (Output >= 130){
    analogWrite(wentylator_gora,Output);
  }
  else
  {
    analogWrite(wentylator_gora,0);
  }
  if (Output1 >= 130){
    analogWrite(wentylator_dol,Output1);
  }
    else
  {
    analogWrite(wentylator_dol,0);
  }
  //wyswietlanie wartosci regulatorow PID za pomoca portu szeregowego ( w celach diagnostycznych)
  Serial.print(Input);
  Serial.print("  ");
  Serial.print(Setpoint);
  Serial.print("  ");
  Serial.print(Output);
  Serial.print("            ");
  Serial.print(Input1);
  Serial.print("  ");
  Serial.print(Setpoint1);
  Serial.print("  ");
  Serial.println(Output1);

  delay(100);
}
  
ENUM_BUTTON getButton() {
  if(!digitalRead(BTN_BACK)) return BACK;
  if(!digitalRead(BTN_NEXT)) return NEXT;
  if(!digitalRead(BTN_PREV)) return PREV;
  if(!digitalRead(BTN_OK)) return OK;
  return NONE;
}

void drawMenu() {
  //odczyt z czujnika AM2320
  th.Read();
  czujnikAM[0]=th.h;
  czujnikAM[1]=th.t;
  
  for (byte i=0; i<SENSORS_NUM; i++){
    czujnik[i]=sensors.readTemperature(FA(address[i]));
  }
  sensors.request();
    
  static unsigned long lastRead = 0;
  static ENUM_BUTTON lastPressedButton = OK;
  static unsigned int isPressedSince = 0;
  int autoSwitchTime = 500;

  ENUM_BUTTON pressedButton = getButton();

  if(pressedButton == NONE && lastRead != 0) {
    isPressedSince = 0;
    return;
  }
  if(pressedButton != lastPressedButton) {
    isPressedSince = 0;
  }

  if(isPressedSince > 3) autoSwitchTime = 70;
  if(lastRead != 0 && millis() - lastRead < autoSwitchTime && pressedButton == lastPressedButton) return;

  isPressedSince++;
  lastRead = millis();
  lastPressedButton = pressedButton;
  
  switch(pressedButton) {
    case NEXT: handleNext(); break;
    case PREV: handlePrev(); break;
    case BACK: handleBack(); break;
    case OK: handleOk(); break;
  }

  lcd.home();
  lcd.clear();
  if(isInLowerLevel && currentMenuPos != 0  ) {
  lcd.print(menu[currentMenuPos].label);
  
    if(menu[currentMenuPos].handler != NULL) {
      (*(menu[currentMenuPos].handler))();
    } else {
      lcd.setCursor(0, 1);
      lcd.print(tempVal);
    }
  } else if (currentMenuPos != 0) {
    lcd.print(menu[currentMenuPos].label);
  }
  
}

void handleNext() {
  if(isInLowerLevel) {
    tempVal++;
    if(tempVal > menu[currentMenuPos].maxVal) tempVal = menu[currentMenuPos].maxVal;
  } else {
    currentMenuPos = (currentMenuPos + 1) % menuSize;
  }
}

void handlePrev() {
  if(isInLowerLevel) {
    tempVal--;
    if(tempVal < menu[currentMenuPos].minVal) tempVal = menu[currentMenuPos].minVal;
  } else {
    currentMenuPos--;
    if(currentMenuPos < 0) currentMenuPos = menuSize - 1;
  }
}

void handleBack() {
  if(isInLowerLevel) {
    isInLowerLevel = false;
  }
}

void handleOk() {
  if(menu[currentMenuPos].handler != NULL && menu[currentMenuPos].maxVal <= menu[currentMenuPos].minVal) {
    (*(menu[currentMenuPos].handler))();
    return;
  }
  if(isInLowerLevel) {
    menu[currentMenuPos].currentVal = tempVal;
    isInLowerLevel = false;
  } else {
    tempVal = menu[currentMenuPos].currentVal;
    isInLowerLevel = true;
  }
}

void parametry() {
  
  
  lcd.setCursor(0,0);
  lcd.print(menu[2].currentVal);
  lcd.print("|");
  lcd.print(czujnik[0]);
  lcd.setCursor(5,0);
  lcd.print("*C");
  lcd.setCursor(0,2);
  lcd.print(menu[3].currentVal);
  lcd.print("|");
  lcd.print(czujnik[1]);
  lcd.setCursor(5,2);
  lcd.print("*C");
  lcd.setCursor(0,1);
  lcd.print(menu[4].currentVal);
  lcd.print("|");
  lcd.print(czujnikAM[1],0);
  lcd.setCursor(5,1);
  lcd.print("*C");
  lcd.setCursor(9,0);
  lcd.print(czujnikAM[0],0);
  lcd.setCursor(11,0);
  lcd.print("%RH");
  lcd.setCursor(9,1);
  lcd.print(czujnik[2]);
  lcd.setCursor(11,1);
  lcd.print("*C");
  lcd.setCursor(9,2);
  lcd.print(-scale.get_units(), 0);
 // lcd.setCursor(14,2);
  lcd.print("g");
  
  
}

void oswietlenieLed() {
  String dictonary[] = {"ON", "OFF"};
  lcd.setCursor(0,1);
  lcd.print(dictonary[tempVal]);
  if (dictonary[tempVal] == "OFF"){
    digitalWrite(5, HIGH);
  }
    else
    {
      digitalWrite(5, LOW);
    }
  
}



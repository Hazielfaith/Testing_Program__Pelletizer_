
#include <EEPROM.h>               
#include <PID_v1.h>
#include <Ultrasonic.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>    
LiquidCrystal_I2C lcd1(0x27,16,2);  
LiquidCrystal_I2C lcd2(0x26,16,2);  
#include "max6675.h"
#include <SPI.h>


int MAX6675_CS = 23;
int MAX6675_SO = 22;
int MAX6675_SCK = 24;
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);

int MAX6675_CS2 = 15;
int MAX6675_SO2 = 14;
int MAX6675_SCK2 = 16;
MAX6675 thermocouple1(MAX6675_SCK2, MAX6675_CS2, MAX6675_SO2);

//Arduino Pins
int SSR_PIN=7;
int SSR_PIN2=6;
int SSR_PIN3=5;
int CuttingMotor = 26;
int DCmotor = 27;
int Coolingfan = 29;

//BARREL BUTTONS
#define but_up  10
#define but_down  11
#define but_stop  12
//NOZZLE BUTTONS
#define but_up2  50
#define but_down2 51
#define but_stop2  52

//BARREL LED
int LED_BLUE = 30;
int LED_YELLOW = 31;
int LED_RED = 32;
//NOZZLE LED
int led_blue = 34;
int led_yellow = 35;
int led_red = 36;
//Hopper sensor
int led_green = 38;

Ultrasonic ultrasonic1(40, 41);
int USR;

int IN2 = 9; 
int IN1 = 8;

#define TEMP_READ_DELAY 500
#define TEMP_READ_DELAY2 500

//Variables
uint8_t state = 0;
bool D10_state = 1;
bool D11_state = 1;
bool D12_state = 1;

bool D50_state = 1;
bool D51_state = 1;
bool D52_state = 1;

float prev_isr_timeD10, prev_isr_timeD11, prev_isr_timeD12; 
float prev_isr_timeD50, prev_isr_timeD51, prev_isr_timeD52;
float prev_timeD52,prev_timeD12;

long lastDebounceTime = 0; // NozzleButton_Stop
long debounceDelay = 20;   // NozzleButton_Stop
long lastDebounceTime1 = 0;// NozzleButton_down
long debounceDelay1 = 20;  // NozzleButton_down
long lastDebounceTime2 = 0;// NozzleButton_up
long debounceDelay2 = 20;  // NozzleButton_up

unsigned long prev_time;
unsigned long prev_time2;

int onbutton = 3;
int offbutton = 2;
int motorbutton = 4;
bool onvalue = false;
bool offvalue = true;
bool motorstate = 0;
int estop = 1;
bool emergency_stop = 1;



//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double lcd2_Setpoint, output, input;
double kp=35, ki=1.8, kd=35; 
//Specify the links and initial tuning parameters
double Kp=35, Ki=1.5, Kd=40;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID MyPID(&input, &output, &lcd2_Setpoint, kp,ki, kd, DIRECT);


float real_temp;     //temperature sensor_barrel
float real_temp2;    //temperature sensor_nozzle
unsigned long lastTempUpdate;
unsigned long lastTempUpdate2;
int Outputmax = 255;

unsigned long prev_time_lcd, now_time_lcd;
unsigned long prev_time1_lcd, now_time1_lcd;
unsigned long prev_time2_lcd, now_time2_lcd;
unsigned long prev_time3_lcd, now_time3_lcd;
unsigned long prev_time4_lcd, now_time4_lcd;
unsigned long prev_time5_lcd, now_time5_lcd;
unsigned long prev_time_stop, now_time_stop;
unsigned long prev_time_relay, now_time_relay;
unsigned long prev_time1_relay, now_time1_relay;


bool updateTemperature() {
    if ((millis() - lastTempUpdate) > TEMP_READ_DELAY){
      real_temp = thermocouple.readCelsius();
      lastTempUpdate = millis();
      return true;
      }
      return false;
  }
 bool updateTemperature2() {
    if ((millis() - lastTempUpdate2) > TEMP_READ_DELAY2){
      real_temp2 = thermocouple1.readCelsius();
      lastTempUpdate2 = millis();
      return true;
      }
      return false;
  } 


void setup()
{
  
  cli();
  Setpoint = EEPROM.read(0); //we adf 
  lcd2_Setpoint = EEPROM.read(1);
  sei();
   
  myPID.SetOutputLimits(0, Outputmax);
  MyPID.SetOutputLimits(0, Outputmax);
  Serial.begin(9600);
  //RELAY
  pinMode(SSR_PIN, OUTPUT);     
  pinMode(SSR_PIN2, OUTPUT);
  pinMode(SSR_PIN3, OUTPUT); 
  pinMode(CuttingMotor, OUTPUT);
  pinMode(DCmotor, OUTPUT);
  pinMode(Coolingfan, OUTPUT);
  // BUTTONS
  pinMode(but_up, INPUT_PULLUP); 
  pinMode(but_down, INPUT_PULLUP); 
  pinMode(but_stop, INPUT_PULLUP); 
  pinMode(but_up2, INPUT_PULLUP); 
  pinMode(but_down2, INPUT_PULLUP); 
  pinMode(but_stop2, INPUT_PULLUP);
 
  pinMode(onbutton, INPUT_PULLUP);
  pinMode(offbutton, INPUT_PULLUP);
  pinMode(motorbutton, INPUT_PULLUP);
  pinMode(estop,INPUT_PULLUP);
  PCICR |= B00000001;      //Bit2 = 1 -> "PCIE2" enabeled (PCINT16 to PCINT23)
  PCMSK0 |= B01111110;     //PCINT20, CINT21, CINT22 enabeled -> D4, D5, D6 will trigger interrupt

 // INDICATOR LIGHTS
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(led_blue, OUTPUT);
  pinMode(led_yellow, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);

//DC MOTOR
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  real_temp = thermocouple.readCelsius();
  real_temp2 = thermocouple1.readCelsius();
  while(!updateTemperature()){ }
  while(!updateTemperature2()){ }
  
  myPID.SetMode(AUTOMATIC);
  MyPID.SetMode(AUTOMATIC);
  
  
  prev_time = millis();
  prev_time2 = millis();

  prev_time_lcd = millis();
  prev_time1_lcd = millis();
  prev_time2_lcd = millis();
  prev_time3_lcd = millis();
  prev_time4_lcd = millis();
  prev_time5_lcd = millis();
  prev_time_stop = millis();
  prev_time_relay = millis();
  prev_time1_relay = millis();
  
  lcd1.init();
  lcd2.init();
  lcd1.backlight();
  lcd2.backlight();
}

void loop(){
 bool on_currentstate = digitalRead(onbutton);
 bool off_currentstate = digitalRead(offbutton); 
 prev_time = millis();
 prev_time2 = millis();
 updateTemperature();
 updateTemperature2();
 if (on_currentstate == onvalue) {
      onvalue = true;
      offvalue = false;
      updateTemperature();
      updateTemperature2(); 
      ESTOP;
      SensorUpdate();
      MotorControl(); 
      Hopper_level();
      analogWrite(IN1, 0);
      analogWrite(IN2, 20);                                                                                                                                                            
      if (state == 0) 
      { 
         PID_RUN2();
         PID_RUN(); 
         Indicator2();
         Indicator(); 
         Hopper_level();  
      }
      else
      {
        cooldown();
        cooldown2();
        Hopper_level();  
      }
 }
  
 if (off_currentstate == offvalue) {
    onvalue = false;
    /*offvalue = true;*/ //optional
    ESTOP;
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(SSR_PIN2, LOW);
    digitalWrite(SSR_PIN3, LOW);
    
    digitalWrite(CuttingMotor, HIGH);
    digitalWrite(Coolingfan, HIGH);
    digitalWrite(DCmotor, HIGH);
    
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(led_red, LOW);
    digitalWrite(led_yellow, LOW);
    digitalWrite(led_blue, LOW);
    digitalWrite(led_green, LOW);
    
    now_time_stop = millis();
   if (now_time_stop - prev_time_stop > 150)
    lcd1.clear();
    lcd2.clear();
    lcd2.setCursor(0,0);
    lcd2.print("    PelleFeX    ");
    lcd2.setCursor(0,1);
    lcd2.print(" OPERATION STOP ");
    lcd1.setCursor(0,0);
    lcd1.print("    PelleFeX    ");
    lcd1.setCursor(0,1);
    lcd1.print(" OPERATION STOP ");
    prev_time = millis();
    prev_time2 = millis();
    prev_time_stop = now_time_stop;
     }
 }
 
/////////////////////////////////////////////////////
void PID_RUN() 
{
    updateTemperature();
    Input = real_temp;
    myPID.Compute();
   if (millis() - prev_time > Outputmax)
  { prev_time += Outputmax;
  prev_time = millis();
  }
   if (Output > millis() - prev_time){
    digitalWrite(SSR_PIN, HIGH);
    digitalWrite(SSR_PIN2, HIGH);
      
      if (Setpoint - 30 <= real_temp ){
        now_time_relay = millis();
        if (now_time_relay - prev_time_relay >1000)
        {digitalWrite(SSR_PIN, HIGH);
        digitalWrite(SSR_PIN2, LOW);
        prev_time_relay = now_time_relay;}
        now_time1_relay = millis();
        if (now_time1_relay - prev_time1_relay >350)
        {digitalWrite(SSR_PIN2, HIGH);
        prev_time1_relay = now_time1_relay;}
        prev_time = millis();
        }   
    now_time_lcd = millis();
      if  (now_time_lcd - prev_time_lcd > 200)
      {
        lcd1.clear();
        lcd1.setCursor(0,0);           
        lcd1.print("Set: ");
        lcd1.print(Setpoint,1);  
        lcd1.setCursor(0,1); 
        lcd1.print("Real Temp: "); 
        lcd1.print(real_temp,1);  
        Serial.println(real_temp); 
        prev_time = millis();
        prev_time_lcd = now_time_lcd;
      }
    }
    else {
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(SSR_PIN2, LOW);
    now_time1_lcd = millis();
    if  (now_time1_lcd - prev_time1_lcd > 200){
      lcd1.clear();
      lcd1.setCursor(0,0);           
      lcd1.print("Set: ");
      lcd1.print(Setpoint,1);  
      lcd1.setCursor(0,1); 
      lcd1.print("Real Temp: "); 
      lcd1.print(real_temp,1);  
      Serial.println(real_temp); 
      prev_time = millis();
      prev_time1_lcd = now_time1_lcd;
      }
    }
    updateTemperature();
  }
///////////////////////////////////////////////////
void PID_RUN2() 
{
    
    updateTemperature2();
    input = real_temp2;
    MyPID.Compute();
   if (millis() - prev_time > Outputmax)
    { 
      prev_time2 += Outputmax;
      prev_time2 = millis();
    }
   if (output > millis() - prev_time2){
    digitalWrite(SSR_PIN3, HIGH);
    now_time2_lcd = millis();
    if (now_time2_lcd - prev_time2_lcd > 200)
      {
        lcd2.clear();
        lcd2.setCursor(0,0);           
        lcd2.print("Set: ");
        lcd2.print(lcd2_Setpoint,1);  
        lcd2.setCursor(0,1); 
        lcd2.print("Real Temp: "); 
        lcd2.print(real_temp2,1);  
        Serial.println(real_temp2);  
        prev_time2 = millis();
        prev_time2_lcd = now_time2_lcd;
      }
    }
    else {
    digitalWrite(SSR_PIN3, LOW);
    now_time3_lcd = millis();
    if (now_time3_lcd - prev_time3_lcd > 200)
      {
        lcd2.clear();
        lcd2.setCursor(0,0);           
        lcd2.print("Set: ");
        lcd2.print(lcd2_Setpoint,1);  
        lcd2.setCursor(0,1); 
        lcd2.print("Real Temp: "); 
        lcd2.print(real_temp2,1);  
        Serial.println(real_temp2); 
        prev_time2 = millis();
        prev_time3_lcd = now_time3_lcd;
      }
    }
    updateTemperature2();
    digitalWrite(led_blue, LOW);   
  }
////////////////////////////////////////////////////
void cooldown(){
      digitalWrite(SSR_PIN, LOW);
      digitalWrite(SSR_PIN2, LOW);
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED, LOW);
      now_time4_lcd = millis();
      if (now_time4_lcd - prev_time4_lcd > 200)
      {
        lcd1.clear();        
        lcd1.setCursor(0,0);           
        lcd1.print("Set: "); 
        lcd1.print(Setpoint,1); 
        lcd1.setCursor(7,1);   
        lcd1.print("Off");       
        lcd1.display();
        prev_time = millis();
        prev_time4_lcd = now_time4_lcd;
      }
  }
////////////////////////////////////////////////////
void cooldown2(){
      digitalWrite(SSR_PIN3, LOW);
      digitalWrite(led_blue, HIGH);
      digitalWrite(led_red, LOW);
      digitalWrite(led_yellow, LOW);
      now_time5_lcd = millis();
      if (now_time5_lcd - prev_time5_lcd > 200)
      {
        lcd2.clear();        
        lcd2.setCursor(0,0);           
        lcd2.print("Set: "); 
        lcd2.print(lcd2_Setpoint,1); 
        lcd2.setCursor(7,1);   
        lcd2.print("Off");       
        lcd2.display();
        prev_time5_lcd = now_time5_lcd;
        prev_time2 = millis();
      }
  }
////////////////////////////////////////////////////
void Indicator(){
    if (real_temp <= (Setpoint-15)){
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_YELLOW, HIGH);
          digitalWrite(LED_BLUE, LOW);
          }
    if (real_temp >= (Setpoint-15)){
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_YELLOW, LOW);
          digitalWrite(LED_BLUE, LOW);
          }
      updateTemperature();
      updateTemperature2();
  }
////////////////////////////////////////////////////
void Indicator2(){  
     if (real_temp2 <= (lcd2_Setpoint-15)){
        digitalWrite(led_red, LOW);
        digitalWrite(led_blue, LOW);
        digitalWrite(led_yellow, HIGH);
        }
     if (real_temp2 >= (lcd2_Setpoint-15)){
        digitalWrite(led_red, HIGH); 
        digitalWrite(led_yellow, LOW);
        digitalWrite(led_blue, LOW);
        }
      updateTemperature();
      updateTemperature2();
     }
////////////////////////////////////////////////////
void MotorControl(){
  updateTemperature();
  updateTemperature2();
  motorstate = digitalRead(motorbutton); 
  if (motorstate == LOW ){
      digitalWrite(CuttingMotor, LOW);
      digitalWrite(Coolingfan, LOW);
      digitalWrite(DCmotor, LOW);
       updateTemperature();
       updateTemperature2();
    }
  }
///////////////////////////////////////////////////
void Hopper_level(){
  
  if ( USR <= 5)
    {
      digitalWrite(led_green, HIGH);
    }
  
  else
    {
      digitalWrite(led_green, LOW);
    }

  }
//////////////////////////////////////////////////
void SensorUpdate() {
  Serial.println("cm");
  Serial.print("USR: ");
  Serial.print(ultrasonic1.read());
  USR = ultrasonic1.read();
  }
//////////////////////////////////////////////////
void ESTOP (){
  emergency_stop = digitalRead(estop);
  if (emergency_stop == LOW){
    Serial.println("Emergency Stop has been pressed");
    }
  else {
    Serial.println("Do nothing");
    }  
  
  }
///////////////////////////////////////////////////
ISR (PCINT0_vect) 
{
  cli();  
  //1. Check D10 pin HIGH
  if(PINB & B00010000){ 
    if(D10_state == 0){
      pulseIn(D10_state, HIGH);
      D10_state = 1;
      prev_isr_timeD10 = millis();
      Serial.println("Barrel Button_up : HIGH");
    }
      
  }
  else if (D10_state == 1 && (millis() - prev_isr_timeD10 > 2)){
    Setpoint ++;
    int st = Setpoint;
    EEPROM.write(0, st);
    D10_state = 0;
  }


  //2. Check D11 pin HIGH
  if(PINB & B00100000){ 
    if(D11_state == 0){
      pulseIn(D11_state, HIGH);
      D11_state = 1;
      prev_isr_timeD11 = millis();
      Serial.println("Barrel Button_down : HIGH");
    }
      
  }
  else if (D11_state == 1 && (millis() - prev_isr_timeD11 > 2)){
    Setpoint --;
    int st = Setpoint;
    EEPROM.write(0, st);
    D11_state = 0;
  }



  //3. Check D12 pin HIGH
  if(PINB & B01000000){ 
    if(D12_state == 0){
      pulseIn(D12_state, HIGH);
      D12_state = 1;
      prev_isr_timeD12 = millis();
      Serial.println("Barrel Button_stop : HIGH");
    }
      
  }
  else if (D12_state == 1 && (millis() - prev_isr_timeD12 > 2)){    
    if(state == 0 || state == 1){
      state = 2;
    }
    else if (state == 2){
      state = 0;
    }
    D12_state = 0;
  }

   //1. Check D4 pin HIGH
  if(PINB & B00001000){ 
   if ( (millis() - lastDebounceTime2) > debounceDelay2){ 
    if(D50_state == 0){
      pulseIn(D50_state, HIGH);
      D50_state = 1;
      prev_isr_timeD50 = millis();
      Serial.println("Nozzle Button_up : HIGH");
      lastDebounceTime2 = millis();}
      }
      
  }
  else if (D50_state == 1 && (millis() - prev_isr_timeD50 > 2)){
    lcd2_Setpoint ++;
    int st2 = lcd2_Setpoint;
    EEPROM.write(1, st2);
    D50_state = 0;
  }


  //2. Check D5 pin HIGH
  if(PINB & B00000100){ 
  if ( (millis() - lastDebounceTime1) > debounceDelay1){
    if(D51_state == 0){
       pulseIn(D51_state, HIGH);
      D51_state = 1;
      prev_isr_timeD51 = millis();
      Serial.println("Nozzle Button_down : HIGH");
      lastDebounceTime1 = millis();}
    }
      
  }
  else if (D51_state == 1 && (millis() - prev_isr_timeD51 > 2)){
    lcd2_Setpoint --;
    int st2 = lcd2_Setpoint;
    EEPROM.write(1, st2);
    D51_state = 0;
  }




  //3. Check D6 pin HIGH
  if(PINB & B00000010){
   if ( (millis() - lastDebounceTime) > debounceDelay){  
    if(D52_state == 0){
      pulseIn(D52_state, HIGH);
      D52_state = 1;
      prev_isr_timeD52 = millis();
      Serial.println("Nozzle Button_stop : HIGH");
      lastDebounceTime = millis();
    }}   
  }
  else if (D52_state == HIGH && (millis() - prev_isr_timeD52 > 2)){    
    if(state == 0 || state == 1){
      state = 2;
    }
    else if (state == 2){
      state = 0;
    }
    D52_state = 0;
  }
  sei();

  
}  
////////////////////////////////////////////////////////////////////////

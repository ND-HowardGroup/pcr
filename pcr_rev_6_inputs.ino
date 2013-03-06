/* PersonalPCR, code for Arduino based PCR
   Copyright (C) 2011 Chris Templeman <templemanautomation.com>
                 2012 Scott Howard <showard@nd.edu>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  
   If not, see <http://www.gnu.org/licenses/>.
*/

//todo: write function for reused code in settemperature
//do we need "i" in settemperature? just use millis()

#include <PID_v1.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <serLCD.h>
#include <Keypad.h>

const byte ROWS = 6; //six rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = { 
 {'X','3','2','1'},
 {'X','6','5','4'},
 {'X','9','X','X'},
 {'X','0','8','7'},
 {'*','X','X','X'},
 {'#','X','X','X'},
};
byte rowPins[ROWS] = {10, 9, 8, 7, 3, 11}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {2, 4, 5, 6}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );   

// PCR Temperatures in C, to be set by recipe or user with keypad
// Presets set in recipeChoice function
double INITIALIZATION;
double DENATURATION;
double ANNEAL;
double EXTENSION;
double FINAL_ELONGATION;
double HOLDT;

// Number of cycles, input in recipeChoice
int NUMBER_OF_CYCLES;

// PCR cycle times, corresponding to above double temps.
// Set in recipeChoice function.
double INIT_TIME;
double DENAT_TIME;
double ANNL_TIME;
double EXT_TIME;
double FIN_TIME;
double HLD_TIME;

// Definie the serial LCD object and communication pin
int pin = 12;
serLCD lcd(pin);

// core functions
void recipeChoice();
void settemperature(double settemp, long settime);
void printData(double setTemp, int timer);
void readTemps();
void controltemp(boolean fan_on, unsigned long timerinit, unsigned long timer, double settemp);

int cycle = 0; //"global" because used by serial print

const int heater = 13;
const int fan = 0;

double temp1 = 0;
double temp2 = 0;
double temp3 = 0;

unsigned long pulse=0; //will output every one sec

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters, also do so in
//loop() with SetTunings
PID myPID(&Input, &Output, &Setpoint,6.25,.15,0, DIRECT);

void setup() 
{
  Serial.begin(9600);
  pinMode(heater, OUTPUT);
  pinMode(fan, OUTPUT);

  recipeChoice();

  // Print headers on LCDh
  lcd.clear();
  lcd.print(" s  cyc sp   T1");
  lcd.selectLine(2);
  
  // Print to Serial Monitor on Laptop
  Serial.print(" s  cyc sp   T1    T2     T3     output");

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 
}

// Change times!
void loop() 
{
  settemperature(INITIALIZATION, INIT_TIME); 
  for (int i = 0; i < NUMBER_OF_CYCLES; i++)
  {
    cycle++;
    settemperature(DENATURATION, DENAT_TIME);
    settemperature(ANNEAL, ANNL_TIME);
    settemperature(EXTENSION, EXT_TIME);
  }
  cycle++;
  settemperature(FINAL_ELONGATION, FIN_TIME);
  while (1)
  {
    settemperature(HOLDT, HLD_TIME);
  } 
}

// Either choose a preset recipe for temperatures, or create custom
void recipeChoice() {
  // Clear LCD, select line 1
  lcd.clear();
  lcd.selectLine(1);
  
  // Determine if prests or custom wanted
  lcd.print("1 for preset");
  lcd.selectLine(2);
  lcd.print("2 for custom");
  
  // Wait for first key press, get first key press c1
  char c1 = keypad.waitForKey();
  Serial.println(c1);
  
  // If c1 is 1, show presets; gets preset choice c2 ( 1 thru 3 )
  if(c1 == '1') {
    lcd.clear();
    lcd.selectLine(1);
    lcd.print("Enter choice:");
    lcd.selectLine(2);
    lcd.print("(1 thru 3)");
    
    char c2 = keypad.waitForKey();
    
    // Set presets here
    if(c2 == '1') {
      // Edit these fields to change recipe
      INITIALIZATION = 95;
      DENATURATION = 95;
      ANNEAL = 55;
      EXTENSION = 72;
      FINAL_ELONGATION = 72;
      HOLDT = 10;
      NUMBER_OF_CYCLES = 30;
      
      // Change timing, set in seconds
      INIT_TIME = 30;
      DENAT_TIME = 30;
      ANNL_TIME = 60;
      EXT_TIME = 60;
      FIN_TIME = 300;
      HLD_TIME = 20;
      
      lcd.clear();
      lcd.print("Preset 1 selected.");
      delay(1000);
    }
    else if(c2 == '2') {
       // Edit these fields to change recipe
      INITIALIZATION = 90;
      DENATURATION = 96;
      ANNEAL = 50;
      EXTENSION = 74;
      FINAL_ELONGATION = 73;
      HOLDT = 12;
      NUMBER_OF_CYCLES = 20;
      
      // Change timing, set in seconds
      INIT_TIME = 30;
      DENAT_TIME = 30;
      ANNL_TIME = 60;
      EXT_TIME = 60;
      FIN_TIME = 300;
      HLD_TIME = 20;      
      
      lcd.clear();
      lcd.print("Preset 2 selected.");
      delay(1000);
    }
     else if(c2 == '3') {
      // Edit these fields to change recipe
      INITIALIZATION = 97;
      DENATURATION = 97;
      ANNEAL = 52;
      EXTENSION = 74;
      FINAL_ELONGATION = 68;
      HOLDT = 12;
      NUMBER_OF_CYCLES = 45;
      
      // Change timing, set in seconds
      INIT_TIME = 30;
      DENAT_TIME = 30;
      ANNL_TIME = 60;
      EXT_TIME = 60;
      FIN_TIME = 300;
      HLD_TIME = 20;
        
      lcd.clear();
      lcd.print("Preset 3 selected.");
      delay(1000);
    }
  }
  else if(c1 == '2') {
    lcd.clear();
    lcd.selectLine(1);
   // lcd.print("Enter init: ");
    lcd.print("Custom recipe!");
    lcd.print("Coming soon...");
    
   // keypad.waitForKey();
   // char init = keypad.getKey();    
  }
  else {
    lcd.clear();
    lcd.print("Bad choice");
    delay(2500);
  }  
}

void settemperature(double settemp, long settime)
{
  unsigned long timer, time2, timerinit;
  timer = millis();
  timerinit=timer;
  Setpoint = settemp;

  while (abs(temp1-settemp) > 1.0) //ramp
  {
    controltemp(1, timerinit, timer, settemp);
    timer=millis();
    if(timer>pulse)
    {
      pulse+=1000;
      printData(settemp, 0);
    }
  }
  //digitalWrite(fan, LOW);
  timerinit=timer;
  time2=settime*1000+timerinit;
    
  while (timer < time2) //hold at temperature
  {
    controltemp(1, timerinit, timer, settemp);
    timer=millis();
    if(timer>pulse)
    {
      pulse+=1000;
      printData(settemp, (timer-timerinit)/1000.0);
    }
  }
}

void readTemps() 
{ 
  double r1, r2, r3;
  int sensorValue1, sensorValue2, sensorValue3;
  //read all three sensors
  sensorValue1 = analogRead(A0);
  delay(1); //delay to prevent excess ringing, from datasheet
  sensorValue2 = analogRead(A1);
  delay(1);
  sensorValue3 = analogRead(A2);
  
  //convert all thermistors from "counts" voltage to C
  r1 = 2000.0*(1024.0/sensorValue1) - 2000.0;
  temp1 = 3560.0/log(r1/0.0130444106) - 273.15;
  r2 = 2000.0*(1024.0/sensorValue2) - 2000.0;
  temp2 = 3560.0/log(r2/0.0130444106) - 273.15;
  r3 = 2000.0*(1024.0/sensorValue3) - 2000.0;
  temp3 = 3560.0/log(r3/0.0130444106) - 273.15;
}


// NEW:  try only setting buffer for seconds, lcd.print for the rest
void printData(double setTemp, int timer)
{
    char buff[17];
    
    sprintf(buff, "%4.d %2.d ", millis()/1000, cycle);
    
    // Prints out the time in seconds, and the cycle number (always on line 2)
    lcd.selectLine(2);
    lcd.print(buff);
    lcd.print(int(setTemp));
    lcd.print(" ");
    lcd.print(temp1);
    
    // Print to serial monitor as well
    Serial.println();
    Serial.print(buff);
    Serial.print(int(setTemp));
    
    Serial.print(" ");
    
    Serial.print(temp1);
    Serial.print("  ");
    Serial.print(temp2); 
    Serial.print("  "); 
    Serial.print(temp3);
    Serial.print("   ");
    Serial.print(Output);  
}

void controltemp(boolean fan_on, unsigned long timerinit, unsigned long timer, double settemp)
{
  readTemps();
  if (fan_on && temp1>settemp+1)
  {
    digitalWrite(fan, HIGH);
  } else digitalWrite(fan, LOW);
  //display to serial monitor
  Input = temp1;
  myPID.Compute();
  //analogWrite(heater, (settemp-temp1)*3);
  analogWrite(heater, Output);
}

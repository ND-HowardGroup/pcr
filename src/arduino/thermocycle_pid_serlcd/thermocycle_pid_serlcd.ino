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

// PCR Temperatures in C
const double INITIALIZATION = 45;
const double DENATURATION = 45;
const double ANNEAL = 25;
const double EXTENSION = 32;
const double FINAL_ELONGATION = 35;
const double HOLD = 10;
const int NUMBER_OF_CYCLES = 3;

// Definie the serial LCD object and communication pin
int pin = 5;
serLCD lcd(pin);

// core functions
void settemperature(double settemp, long settime);
void printData(double setTemp, int timer);
void readTemps();
void controltemp(boolean fan_on, unsigned long timerinit, unsigned long timer, double settemp);

int cycle = 0; //"global" because used by serial print

const int heater = 3;
const int fan = 2;

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
  digitalWrite(fan, HIGH);
  delay(1000);
  digitalWrite(fan, LOW);

  // Clears the serial LC
  //lcd.clear();

  // Prints out first headers, selects line two.
  lcd.clear();
  lcd.selectLine(1);
  lcd.print(" s  cyc sp   T1");
  //lcd.selectLine(2);
  
  // Print to Serial Monitor on Laptop
  Serial.print(" s  cyc sp   T1    T2     T3     output");

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 
}

void loop() 
{
  settemperature(INITIALIZATION, 30); 
  for (int i = 0; i < NUMBER_OF_CYCLES ;i++)
  {
    cycle++;
    settemperature(DENATURATION, 30);
    settemperature(ANNEAL, 30);
    settemperature(EXTENSION, 30);
  }
  cycle++;
  settemperature(FINAL_ELONGATION, 30);
  while (1)
  {
    settemperature(HOLD, 20);
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
    Serial.print(millis()/1000);
    Serial.print(" ");
    Serial.print(cycle);
    Serial.print(" ");
    Serial.print(timer);
    Serial.print(" ");
    
    Serial.print(setTemp);
    
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
  if (fan_on && temp1>settemp+1.0)
  {
    digitalWrite(fan, HIGH);
  } else digitalWrite(fan, LOW);
  //display to serial monitor
  Input = temp1;
  myPID.Compute();
  //analogWrite(heater, (settemp-temp1)*3);
  analogWrite(heater, Output);
}

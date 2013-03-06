// PersonalPCR v0.2.0
// 11/16/2011
// templemanautomation.com
// coFactorbio.com
//
// Modified by Matt Brittan and Alex Toombs

// FEATURES TO IMPLEMENT:
//
// 1. Set sample recipe
// 2. Implement STOP
// 3. Choose from list of preset recipes
// 4. Optimize code
// 5. PWM?

#include <math.h>
#include <SoftwareSerial.h>
#include <serLCD.h>
#define aref_voltage 5.0
#define TRUE 1
#define FALSE 0


// PCR Temperatures in C
const float INITIALIZATION = 95;
const float DENATURATION = 95;
const float ANNEAL = 55;
const float EXTENSION = 72;
const float FINAL_ELONGATION = 72;
const float HOLD = 10;

// Hardcode number of cycles
int NUMBER_OF_CYCLES = 30;

// Definie the serial LCD object and communication pin
int pin = 2;
serLCD lcd(pin);

// core functions
void settemperature(float settemp, int settime, int duty, int dolastdelay);
void rampup(float temp);
void rampdown(float temp);
void printData(float setTemp, int timer);
void readTemps();

const float TOLERANCE = 1;

const float DENATURATION_OFFSET = 4;
const float ANNEAL_OFFSET = 2;
const float EXTENSION_OFFSET = 3.5;

const int DENATURE_RAMP_OFFSET = 4 + DENATURATION_OFFSET;
const int ANNEAL_RAMP_OFFSET = 3 - ANNEAL_OFFSET;
const int EXTENSION_RAMP_OFFSET = 3 + EXTENSION_OFFSET;

int timer = 0;
unsigned long t = 0; 
int cycle = 0;

int heater = 3;
int fan = 2;

int sensorValue1 = 0;  
int sensorValue2 = 0; 
int sensorValue3 = 0; 

float temp1 = 0; 
float temp2 = 0;  
float temp3 = 0; 

float r1 = 0; 
float r2 = 0; 
float r3 = 0; 

const int analogInPin0 = A0; 
const int analogInPin1 = A1; 
const int analogInPin2 = A2; 


void setup() 
{
  // Clears the serial LC
  lcd.clear();
  
  Serial.begin(9600);
  pinMode(heater, OUTPUT);
  digitalWrite(heater, LOW);
  
  // Prints out first headers, selects line two.
  lcd.print("s  ");
  lcd.print("c   ");
  lcd.print("sp    ");
  lcd.print("T ");
  lcd.selectLine(2);
  
  //Serial.print("seconds");
  //Serial.print(" "); 
  //Serial.print("cycle_num");
  //Serial.print(" "); 
  //Serial.print("cycle_time");
  //Serial.print(" "); 
  //Serial.print("target_temp"); 
  //Serial.print(" "); 
  //Serial.print("probe_1_temp"); 
  //Serial.print(" "); 
  //Serial.print("probe_2_temp"); 
  //Serial.print(" "); 
  //Serial.print("control_temp");
  //Serial.println(" "); 
 
}

void loop() 
{ 
  rampup(INITIALIZATION - DENATURE_RAMP_OFFSET);
  settemperature(INITIALIZATION - DENATURATION_OFFSET, 30, 50, TRUE); 
  for (int i = 0; i < NUMBER_OF_CYCLES ;i++)
  {
    cycle++;
    rampup(DENATURATION - DENATURE_RAMP_OFFSET);
    settemperature(DENATURATION - DENATURATION_OFFSET, 30, 50, TRUE);
    rampdown(ANNEAL + ANNEAL_RAMP_OFFSET);
    settemperature(ANNEAL - ANNEAL_OFFSET, 60, 30, TRUE);
    rampup(EXTENSION - EXTENSION_RAMP_OFFSET);
    settemperature(EXTENSION - EXTENSION_OFFSET, 60, 40, TRUE);
  }
  cycle++;
  settemperature(FINAL_ELONGATION - EXTENSION_OFFSET, 300, 50, TRUE);
  while (1)
  {
    rampdown(HOLD);
    settemperature(HOLD, 20, 50, TRUE);
  } 
}

void settemperature(float settemp, int settime, int duty, int dolastdelay)
{
  int i = 0;
  int timer = 0;
  int time2 = 0;
  time2 = settime * 10;
  int fanDuty = (100 - duty)/10;
  
  while (timer < time2)
  {
    readTemps();
    //display to serial monitor
    i++;
    t++;
    if (i == 10)
    {
      i=0;
      printData(settemp, timer);
    }
    
    if ( temp3 >= settemp)
    {
      digitalWrite(fan, HIGH);
    }
    if(temp3 < settemp + TOLERANCE){
      digitalWrite(heater, HIGH);
      delay(duty);
      digitalWrite(heater, LOW);
    }
    else {
      delay(duty);
    }
    
    if ((temp3 < (settemp + TOLERANCE)) && (temp3 > (settemp - TOLERANCE)))
    {
      timer = timer + 1;  
    }  
    if (dolastdelay)
    {
      delay(98 - duty - 20);
      digitalWrite(fan, LOW);
      delay(20);
    }      
  }
}

void readTemps() 
{ 
  //read all three sensors
  sensorValue1 = analogRead(analogInPin0);
  delay(1);
  sensorValue2 = analogRead(analogInPin1);
  delay(1);
  sensorValue3 = analogRead(analogInPin2);
  
  //convert all thermistors from "counts" voltage to C
  r1 = 2000.0*(1024.0/sensorValue1) - 2000.0;
  temp1 = 3560.0/log(r1/0.0130444106) - 273.15;
  r2 = 2000.0*(1024.0/sensorValue2) - 2000.0;
  temp2 = 3560.0/log(r2/0.0130444106) - 273.15;
  r3 = 2000.0*(1024.0/sensorValue3) - 2000.0;
  temp3 = 3560.0/log(r3/0.0130444106) - 273.15;
}

void printData(float setTemp, int timer)
{
    // Prints out the time in seconds, and the cycle number (always on line 2)
    lcd.selectLine(2);
    lcd.print(t/10);
    lcd.print(" ");
    lcd.print(cycle);
    lcd.print(" ");
    lcd.print(setTemp);
    lcd.print("  ");
    lcd.print(temp1);
    //Serial.print(t / 10);
    //Serial.print(" "); 
    //Serial.print(cycle);
    //Serial.print(" "); 
    //Serial.print(timer / 10);
    //Serial.print(" "); 
    //Serial.print(setTemp);
    //Serial.print(" "); 
    //Serial.print(temp1);
    //Serial.print(" "); 
    //Serial.print(temp2); 
    //Serial.print(" "); 
    //Serial.print(temp3);
    //Serial.println(" "); 
}

void rampup(float temp)
{
   digitalWrite(heater, HIGH);
   int i = 0;
   readTemps();
   while(temp3 < temp) {
     readTemps();
    //display to serial monitor
    i++;
    t++;
    if (i == 10)
    {
      i=0;
      printData(temp, timer);
    }
    delay(100);     
   }
   digitalWrite(heater, LOW);
}

void rampdown(float temp)
{
   digitalWrite(fan, HIGH);
   int i = 0;
   readTemps();
   while(temp3 > temp) {
     readTemps();
    //display to serial monitor
    i++;
    t++;
    if (i == 10)
    {
      i=0;
      printData(temp, timer);
    }
    delay(100);     
   }
   digitalWrite(fan, LOW);
}

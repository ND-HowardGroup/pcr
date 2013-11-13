/* PersonalPCR, code for Arduino based PCR
 Copyright (C) 2011 Chris Templeman <templemanautomation.com>
 2012 Scott Howard <showard@nd.edu>
 
 Authors:  Chris Templeman
 Matt Brittan
 Alex Toombs
 Elizabeth Hunschke
 Frank Kuhny    
 
 Date Last Modified:  5/19/13
 
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

#include <PID_v1.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <serLCD.h>
#include <Keypad.h>

//Four rows
const byte ROWS = 4;
//Three columns
const byte COLS = 3;
//Define the Keymap
char keys[ROWS][COLS] = {
  {
    '1','2','3'      }
  ,
  {
    '4','5','6'      }
  ,
  {
    '7','8','9'      }
  ,
  {
    '*','0','#'      }
};
//connect to the row pinouts of the keypad
byte rowPins[ROWS] = {
  12, 7, 8, 10 };
//connect to the column pinouts of the keypad
byte colPins[COLS] = {
  11, 13, 9 };

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );   

// PCR Temperatures in C, to be set by recipe or user with keypad
// Presets set in recipeChoice function
double INITIALIZATION = 95;
double DENATURATION = 95;
double ANNEAL = 55;
double EXTENSION = 72;
double FINAL_ELONGATION = 72;
double HOLDT = 9;

// Number of cycles, input in recipeChoice
int NUMBER_OF_CYCLES = 35;

// PCR cycle times, corresponding to above double temps.
// Set in recipeChoice function.
double INIT_TIME = 30;
double DENAT_TIME = 30;
double ANNL_TIME = 60;
double EXT_TIME = 60;
double FIN_TIME = 300;
double HLD_TIME = 20;

// Definie the serial LCD object and communication pin
const int pin = 19;
serLCD lcd(pin);

// core function protoypes
// Directs user to select a recipe
void recipeChoice();
// Calls controltemp() to bring temp to a new level, hold it for a set time, calls printData() to display what is happening
void settemperature(double settemp, long settime);
// Prints data to LCD screen and serial monitor (time remaining in minutes, current temp)
void printData(double setTemp, int timer);
// Reads voltages from thermistor/resistor voltage dividers and calculates temperatures
void readTemps();
// Uses PID control with measurements from readTemps to set power to heater and fan
void controltemp(boolean fan_on, unsigned long timerinit, unsigned long timer, double settemp);

// track number of cycles
int cycle = 0; //"global" because used by serial print

// heater, fan, and buzzer pins
const int heater = 3;
const int fan = 5;
const int buzzer = A3;

// temperature for heater 1
double temp1 = 0;
// temperature for heater 2
double temp2 = 0;
// averaged temps
double temp = 0;

// will output every one sec
unsigned long pulse=0;

// Define PID variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters, also do so in
//loop() with SetTunings
// tunings are setup and material specific
PID myPID(&Input, &Output, &Setpoint,33.01,0.35,120.55, DIRECT);

// Set up variables, recipe choice, and headers
void setup() 
{
  // Begin serial communication at 9600 baud
  Serial.begin(9600);
  // set output pins to output
  pinMode(heater, OUTPUT);
  pinMode(fan, OUTPUT);

  // Get recipe to control cycle
  recipeChoice();

  // print delay message until heaters are at INITIALIZATION point
  while(temp > INITIALIZATION) {
    lcd.selectLine(1);
    lcd.print("WAIT FOR HEATERS");
  }
  
  // after heats are heated, wait for input before starting recipe
  lcd.clear();
  lcd.selectLine(1);
  lcd.print("Load vials");
  lcd.selectLine(2);
  lcd.print("Hit 1 to start");

  // wait until any key is pressed.
  keypad.waitForKey();

  // Print headers on LCD
  lcd.clear();
  lcd.print("min_left    T");
  lcd.selectLine(2);

  // Print to Serial Monitor on Laptop
  Serial.print(" s  cyc sp   T1    T2   output  time_rem");

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}

// Doesn't actually loop like it's supposed to.  This will only loop once, in theory
void loop() 
{
  // Set the temperature to remain at INITIALIZATION for INIT_TIME seconds
  settemperature(INITIALIZATION, INIT_TIME); 

  // Loop for PCR recipe while current cycles is less than total number of cycles, as specified in recipe
  // Real loop is here!
  for (int i = 0; i < NUMBER_OF_CYCLES; i++)
  {
    // increment cycle
    cycle++;

    // step through each step of process
    settemperature(DENATURATION, DENAT_TIME);
    settemperature(ANNEAL, ANNL_TIME);
    settemperature(EXTENSION, EXT_TIME);
  }
  cycle++;

  // PCR process loop ended; elongating, holding, then buzzing

  // Final 
  settemperature(FINAL_ELONGATION, FIN_TIME);
  // Hold at HOLD_TEMPERATURE for a long time
  settemperature(HOLDT, HLD_TIME);

  // Play buzzer until button hit, as PCR is done, tubes should be removed
  lcd.clear();
  lcd.selectLine(1);
  lcd.print("REMOVE VIALS!");
  lcd.selectLine(2);
  lcd.print("Hit Key For End");

  // play annoying buzzer until they hit a key to alert for vial removal     
  digitalWrite(buzzer, HIGH);

  // buzzer should play until a key is pressed
  keypad.waitForKey();
  digitalWrite(buzzer, LOW);

  lcd.clear();

  // display end message, loop infinitely
  while(1) {
    lcd.selectLine(1);
    lcd.print("Fridge vials!");
    lcd.selectLine(2);
    lcd.print("Run complete.");
    
    // delay to prevent flicker
    delay(500);
  }
}

// Either choose a preset recipe for temperatures
void recipeChoice() {
  // Clear LCD, select line 1
  lcd.clear();
  lcd.selectLine(1);

  // Welcome message displayed
  delay(1000);
  lcd.print("Welcome to NDPCR");
  delay(2000);
  lcd.clear();
  lcd.clear();

  lcd.selectLine(1);
  lcd.print("Enter choice:");
  lcd.selectLine(2);
  lcd.print("(1 thru 3)");

  // Recipe choice input (char digit)
  char c2 = keypad.waitForKey();

  // Sets recipes, can be 0-9, # and * as chars (up to 12 recipes stored)
  if(c2 == '1') {
    // Edit these fields to change recipe
    INITIALIZATION = 95;
    DENATURATION = 95;
    ANNEAL = 55;
    EXTENSION = 72;
    FINAL_ELONGATION = 72;
    HOLDT = 10;
    NUMBER_OF_CYCLES = 35;

    // Change timing, set in seconds
    INIT_TIME = 30;
    DENAT_TIME = 30;
    ANNL_TIME = 60;
    EXT_TIME = 60;
    FIN_TIME = 300;
    HLD_TIME = 20;

    lcd.clear();
    lcd.print("Rcp 1 Chosen");
    delay(500);
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
    lcd.print("Rcp 2 Chosen");
    delay(500);
  }
  else if(c2 == '3') {
    // Edit these fields to change recipe
    // This recipe currently used to test lcd display
    INITIALIZATION = 35;
    DENATURATION = 30;
    ANNEAL = 25;
    EXTENSION = 27;
    FINAL_ELONGATION = 27;
    HOLDT = 23;
    NUMBER_OF_CYCLES = 3;

    // Change timing, set in seconds
    INIT_TIME = 10;
    DENAT_TIME = 10;
    ANNL_TIME = 10;
    EXT_TIME = 5;
    FIN_TIME = 5;
    HLD_TIME = 5;

    lcd.clear();
    lcd.print("Test RCP!");
    delay(500);
  }
}

// Set temperature of heaters
void settemperature(double settemp, long settime)
{
  unsigned long timer, time2, timerinit;
  timer = millis();
  timerinit=timer;
  Setpoint = settemp;

  // while temperature is off from settemp, keep looping
  while (abs(temp-settemp) > 1.0)
  {
    // control PID and fan with other function
    controltemp(1, timerinit, timer, settemp);

    // get current time in ms
    timer=millis();

    // print to screen every second
    if(timer>pulse)
    {
      // increment by 1 second
      pulse+=1000;

      // output to screen
      printData(settemp, 0);
    }
  }

  // Grabs time to control step duration
  timerinit=timer;
  time2=settime*1000+timerinit;

  // Hold at temperature while at current step  
  while (timer < time2)
  {
    // control PID output and fan on/off by using controltemp function
    controltemp(1, timerinit, timer, settemp);

    // Every second, output to screen
    timer=millis();
    if(timer>pulse)
    {
      pulse+=1000;
      // print data to LCD/serial monitor
      printData(settemp, (timer-timerinit)/1000.0);
    }
  }
}

// read temperatures from digital temperature sensors on aluminum block
void readTemps() 
{ 
  double r1, r2;
  int sensorValue1, sensorValue2;
  //read all three sensors
  sensorValue1 = analogRead(A0);
  delay(1); //delay to prevent excess ringing, from datasheet
  sensorValue2 = analogRead(A7);

  //convert all thermistors from "counts" voltage to C
  r1 = 1000.0/((1023.0/sensorValue1) - 1);
  temp1 = 3560.0/log(r1/0.0130444106) - 273.15;
  r2 = 1000.0/((1023.0/sensorValue2) - 1);
  temp2 = 3560.0/log(r2/0.0130444106) - 273.15;

  // Average temperatures of two sensors to control fan and PID
  temp = (temp1 + temp2) / 2;
}


// displays data to LCD monitor and serial monitor
void printData(double setTemp, int timer)
{
  char buff[17];

  // buffered charString that contains data printed to LCD
  sprintf(buff, "%3.d ", calcTimeRemaining());

  // Prints out time remaining in minutes and current averaged temp (always on line 2)
  lcd.selectLine(2);
  lcd.print(buff);
  lcd.print("     ");
  lcd.print(temp);

  // Print to serial monitor as well
  // Appearance:  ## ##.## ##.## ##.##
  Serial.println();
  Serial.println();
  Serial.print(buff);
  Serial.print("   ");
  Serial.print(int(setTemp));

  Serial.print("   ");

  Serial.print(temp1);
  Serial.print("      ");
  Serial.print(temp2); 
  Serial.print("     "); 
  Serial.print(Output);
  Serial.print("     ");
  Serial.print(calcTimeRemaining());  
}

// Control temperature by calculating PID and turning fan on or off.
void controltemp(boolean fan_on, unsigned long timerinit, unsigned long timer, double settemp)
{
  // Read temperatures into global variables
  readTemps();

  // if temp too high, turn fan on to cool.  else, turn fan off.
  // NOTE:  fan_on is a stupid variable that is always 1 when this function is called
  if (fan_on && temp>settemp+1)
  {
    // turn fan on
    digitalWrite(fan, HIGH);
  } 
  else  digitalWrite(fan, LOW);

  // Send temperature to PID controller for predictive contrl
  Input = temp;

  // Compute next PID step
  myPID.Compute();

  // Control heater based upon PID compute
  analogWrite(heater, Output);
}

// return time remaining on this PCR recipe
// return: time remaining in minutes, int
int calcTimeRemaining() {
  // total process time in seconds, excluding ramp times
  int totTime = INIT_TIME + (NUMBER_OF_CYCLES * (DENAT_TIME + ANNL_TIME + EXT_TIME)) + (FIN_TIME + HLD_TIME);

  // "fudge factor" for ramp times, etc.; needs to be more elegant later
  int fudgeFactor = totTime*.07;

  // Calculate final time
  int totalTime = totTime + fudgeFactor;

  // returns time remaining in minutes as an integer
  return (totalTime - (millis()/1000))/60;
}




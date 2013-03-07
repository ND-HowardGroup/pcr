// PersonalPCR v0.2.0
// 11/16/2011
// templemanautomation.com
// coFactorbio.com
//
// Modified by Matt Brittan and Alex Toombs
// Last modified:  4/1/12
//
// Modified to outut to serial LCD
 
#include <math.h>
#include <SoftwareSerial.h>
#include <serLCD.h>
#define aref_voltage 5.0
#define TRUE 1
#define FALSE 0 
 
// Serial output pin
int pin = 2;

// Initialize the serLCD
serLCD lcd(pin);

// PCR Temperatures in C, all user input
int INITIALIZATION;
int DENATURATION;
int ANNEAL;
int EXTENSION;
int FINAL_ELONGATION;
int HOLD; // is HOLD a temperature too?
 
// Number of times run to produce 2^n more DNA strands, user input
int NUMBER_OF_CYCLES;
 
// Core function prototyping
void settemperature(float settemp, int settime, int duty, int dolastdelay);
void rampup(float temp);
void rampdown(float temp);
void printData(float setTemp, int timer);
void readTemps();
void setInputs(int input);

// Constant modifications
const float TOLERANCE = 1;
const float DENATURATION_OFFSET = 4;
const float ANNEAL_OFFSET = 2;
const float EXTENSION_OFFSET = 3.5;
const int DENATURE_RAMP_OFFSET = 4 + DENATURATION_OFFSET;
const int ANNEAL_RAMP_OFFSET = 3 - ANNEAL_OFFSET;
const int EXTENSION_RAMP_OFFSET = 3 + EXTENSION_OFFSET;

// Set other constants 
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
 
// Run once to establish baud rate, pin assignment, and above temperature/cycle inputs
void setup()
{
  // Clears the serial LCD object
  lcd.clear();
  lcd.print("hello, world!");
  
  // Opens serial port at 9600 baud rate, sets heater pin
  Serial.begin(9600);
  pinMode(heater, OUTPUT);
  digitalWrite(heater, LOW);
  
  Serial.print("Please initialize the thermocycler.");
  Serial.println();
 
  // Sets number of thermo cycles to run
  Serial.print("Please enter the number of cycles you would like to run:  ");
  Serial.println();
  setInput(NUMBER_OF_CYCLES);
  
  // Sets two-digit INITIALIZATON temperature
  Serial.print("Please enter desired two-digit initialization temperature:  ");
  Serial.println();
  setInput(INITIALIZATION);  
  
  // Sets two-digit DENATURATION temperature
  Serial.print("Please enter desired two-digit denaturation temperature:  ");
  Serial.println();
  setInput(DENATURATION); 
 
  // Sets two-digit ANNEAL temperature
  Serial.print("Please enter desired two-digit annealing temperature:  ");
  Serial.println();
  setInput(ANNEAL);
  
  // Sets two-digit EXTENSION temperature
  Serial.print("Please enter desired two-digit extension temperature:  ");
  Serial.println();
  setInput(EXTENSION);
  
  // Sets two-digit FINAL_ELONGATION temperature
  Serial.print("Please enter desired two-digit temperature for final elongation:  ");
  Serial.println();
  setInput(FINAL_ELONGATION);
  
  // Sets two-digit HOLD temperature
  Serial.print("Please enter desired two-digit hold temperature:  ");
  Serial.println();
  setInput(HOLD);
  Serial.println();
    
  // Prints headers of received serial data
  Serial.print("seconds");
  Serial.print(" "); 
  Serial.print("cycle_num");
  Serial.print(" "); 
  Serial.print("cycle_time");
  Serial.print(" "); 
  Serial.print("target_temp"); 
  Serial.print(" "); 
  Serial.print("probe_1_temp"); 
  Serial.print(" "); 
  Serial.print("probe_2_temp"); 
  Serial.print(" "); 
  Serial.print("control_temp");
  Serial.println(" ");
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

void setInput(int input) {
  char tensPlace; //Used to read in input
  int tensPlaceInt; //Used to convert input to integers--looked illegal to atoi(immediate input)
  int number=0; //Used to temporarily store number
  char *tensPtr; //Used for atoi
  char str[3]; //String to enter chars--NOTE: adjust later to make more
  int n=0; //Used as a counter
  int i=0; //Used as counter
  int cyc=0; //Bounds the while loop inside here
  
  // run loop until satisfactory input found
  while(cyc < 1) {
    // When data available on serial port, if string is 'S##E', cycles = ##
    if (Serial.available() > 0) {
      if (Serial.read() == 'S') {
        
        //Reads in string
        while (str[2] != 'E') {
           delay(250); // DO NOT REMOVE; delay is critical [not sure why?]
           str[n]= Serial.read();
           n = n + 1;           
        }
        cyc = cyc + 1;
                                       
        //For loop multiplies through tens place and ones place, adds them together
         for (i=1; i>=0; i--)
         {
            *tensPtr = str[i];
            tensPlaceInt = atoi (tensPtr); 
            
            //Multiplies first digit by 10
            if (i == 0) {
               tensPlaceInt = 10 * tensPlaceInt;
            }
            number = number + tensPlaceInt;
         }       
        
         input = number;
      }
    }
  }
  tensPlace = 0;
  *tensPtr = 0;
  for(int j = 0; j < 3; j++) {
      str[j] = 0;
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
    Serial.print(t / 10);
    Serial.print("          ");
    Serial.print(cycle);
    Serial.print("           ");
    Serial.print(timer / 10);
    Serial.print("         ");
    Serial.print(setTemp);
    Serial.print("        ");
    Serial.print(temp1);
    Serial.print("         ");
    Serial.print(temp2);
    Serial.print("        ");
    Serial.print(temp3);
    Serial.println("           ");
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

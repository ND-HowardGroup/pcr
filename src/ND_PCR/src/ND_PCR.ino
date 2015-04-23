/* PersonalPCR, code for Arduino based PCR
 Copyright (C) 2011 Chris Templeman <templemanautomation.com>
 2012 Scott Howard <showard@nd.edu>

 Authors:
 Chris Templeman
 Matt Brittan
 Alex Toombs
 Elizabeth Hunschke
 Frank Kuhny
 Romeo Kwihangana
 Aamir Ahmed Khan
 Genevieve Vigil
 Andrew Steinbergs
 Christine Joseph
 Steven Waller

 Date Last Modified:  04/22/2015

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

 #Change Log
    - changed fudgefactor in calcTimeRemaining() function making it more accurate
    - added cycle # to output to serial
    - added rise and fall time to calcTimeRemaining for more accuracy

 */

#include <PID_v1.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <serLCD.h>
#include <Keypad.h>


int debugTest = 0;  // Global debug variable
int restartDevice();
void(* resetFunc) (void) = 0;  //declare reset function at address 0

//Four rows
const byte ROWS = 4;
//Three columns
const byte COLS = 3;
//Define the Keymap
char keys[ROWS][COLS] = {
  { '1','2','3'      },
  { '4','5','6'      },
  { '7','8','9'      },
  { '*','0','#'      }
};
byte rowPins[ROWS] = { 8,13,12,10 };    //connect to the row pinouts of the keypad
byte colPins[COLS] = { 9,7,11 };        //connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );


// Recipe struct
struct pcrRecipe {
    int numCycles;
    // setpoints in Celsius
    double spInitz;
    double spDenat;
    double spAnnea;
    double spExten;
    double spFElng;
    double spFHold;
    // hold times in seconds
    double htInitz;
    double htDenat;
    double htAnnea;
    double htExten;
    double htFElng;
    double htFHold;
} pcr;

char recipeChar;

// stage type variable
enum stageType {    DENAT = 1,
                    ANNEA = 2,
                    EXTEN = 3,
                    XXXXX = -1     };


// core function protoypes
// Directs user to select a recipe
int recipeChoice();
// Calls controlTemp() to bring temp to a new level, hold it for a set time, calls printData() to display what is happening
void setTemperature(double setpointTemp, long holdTime, int cycle);
// Prints data to LCD screen and serial monitor (time remaining in minutes, current temp)
void printData(double setpointTemp, int cycle);
// Reads voltages from thermistor/resistor voltage dividers and calculates temperatures
double getTemp();
// Uses PID control with measurements from readTemps to set power to heater and fan
void controlTemp(double setpointTemp);

// heater, fan, and buzzer pins
const int pinHeater = 3;
const int pinFan1 = 5;
const int pinFan2 = 2;
const int pinBuzzer = A3;

// Define the serial LCD object and communication pin
const int pinLcd = 19;
serLCD lcd(pinLcd);

// will output every one sec
unsigned long pulse = 0;
unsigned long startMillis = 0;  // milliseconds count when receipe started

// Define PID variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

// Specify the links and initial tuning parameters, also do so in
// loop() with SetTunings
// tunings are setup and material specific

// (Kp, Ki, Kd) obtained from Autotune [3.4 Ohm heater and two fans]
// (128+/-50, 70+/-5) ==> (3.41, 0.12, 24.46)
   PID myPID(&pidInput, &pidOutput, &pidSetpoint, 3.41, 0.12, 24.46, DIRECT);



// Set up variables, recipe choice, and headers
void setup() {
    // register the custom characters in LCD
    byte upArrow[8] = {     B00000,
                            B01111,
                            B00011,
                            B00101,
                            B01001,
                            B10000,
                            B00000,
                            B00000  };

    byte downArrow[8] = {   B00000,
                            B10000,
                            B01001,
                            B00101,
                            B00011,
                            B01111,
                            B00000,
                            B00000  };

    lcd.createChar(1, upArrow);
    lcd.createChar(2, downArrow);


    // Begin serial communication at 9600 baud
    Serial.begin(9600);
    // set output pins to output
    pinMode(pinHeater, OUTPUT);
    pinMode(pinFan1, OUTPUT);
    pinMode(pinFan2, OUTPUT);
    pinMode(pinBuzzer, OUTPUT);

    // Welcome message displayed
    lcd.setBrightness(30);
    buzz(500);
    lcd.clear();
    delay(1000);
    lcd.selectLine(1);
    lcdAnim("### ND-PCR v2 ##");
    delay(500);
    lcd.selectLine(2);
    lcdAnim(">>> Howard's lab", 20);
    delay(1000);
    buzz(200);

    // Get the recipe to control PCR reaction
    while (recipeChoice() != 0);

    // print delay message until heaters are at pcr.spInitz point
    lcd.clear();
    lcd.selectLine(1);
    // lcd.print("Block too hot!");
    // lcd.selectLine(2);
    lcd.print("Cooling down...");
    digitalWrite(pinFan1, HIGH);
    digitalWrite(pinFan2, HIGH);
    while(getTemp() > pcr.spInitz) {
        lcd.selectLine(2);
        lcd.print("T = "); lcd.print(getTemp()); lcd.print(' '); lcd.print((char)223); lcd.print("C");
        delay(500);
    }
    lcd.clear();

    // Wait for input before starting the cycle
    lcd.clear();
    lcd.selectLine(1);
    lcd.print("Load vials and hit key to start");
    // wait until any key is pressed.
    keypad.waitForKey();
    buzz(500);
    lcd.clear();

    // turn the PID on
    myPID.SetMode(AUTOMATIC);

    // Print to Serial Monitor on Laptop
    Serial.println("SEC\tTIMEREM\tCYC\tTSP\tT1\tT2\tTAVG\tHEATER\tFAN\tSTAGE");
}

// Doesn't actually loop like it's supposed to.  This will only loop once, in theory
void loop() {
    startMillis = millis(); // milliseconds count when receipe started
    // Set the temperature to remain at pcr.spInitz for pcr.htInitz seconds
    setTemperature(pcr.spInitz, pcr.htInitz, 0);
    buzz(200);

    // Loop for PCR recipe while current cycles is less than total number of cycles, as specified in recipe
    // Real loop is here!
    int cycle = 1;
    for ( ; cycle <= pcr.numCycles; cycle++) {
        // step through each step of process
        setTemperature(pcr.spDenat, pcr.htDenat, cycle);
        buzz(200);
        setTemperature(pcr.spAnnea, pcr.htAnnea, cycle);
        buzz(200);
        setTemperature(pcr.spExten, pcr.htExten, cycle);
        buzz(500);
    }

    // PCR process loop ended; elongating, holding, then buzzing

    // Final two steps
    cycle--; // in order to not display extra cycle
    setTemperature(pcr.spFElng, pcr.htFElng, cycle);
    // setTemperature(pcr.spFHold, pcr.htFHold); // no need


    // Turn the heater off and turn the fans on
    myPID.SetMode(MANUAL);
    analogWrite(pinHeater, 0);
    digitalWrite(pinFan1, HIGH);
    digitalWrite(pinFan2, HIGH);

    // Play buzzer until button hit, as PCR is done, tubes should be removed
    tone(pinBuzzer, 2400);
                                                        // noTone(pinBuzzer);
    lcd.clear();
    bool a = 0;
    unsigned long previousMillis = 0;

    while(1) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis > 1000) {
            a = !a;
            previousMillis = currentMillis;
        }
        lcd.selectLine(1);
        if (a)
            lcd.print("PCR run complete");
        else {
            lcd.print((getTemp() > 40) ? "Too hot to remov" : "Remove vials now");
        }
        delay(1000);
        lcd.selectLine(2);
        lcd.print("T = "); lcd.print(getTemp()); lcd.print(' '); lcd.print((char)223); lcd.print("C");
        char ch = keypad.getKey();
        if (ch != NO_KEY)
            noTone(pinBuzzer);
        if (ch == '#')
            restartDevice();
    }
}


// Set temperature of heaters
void setTemperature(double setpointTemp, long holdTime, int cycle) {
    pidSetpoint = setpointTemp;

    // while temperature is off from setpointTemp, keep looping
    while (abs(getTemp() - setpointTemp) > 1.0) {
        // control PID and fan with other function
        controlTemp(setpointTemp);
        if(millis() > pulse) { // print to screen every second
            pulse += 1000;    // increment by 1 second
            printData(setpointTemp, cycle); // print data to LCD/serial monitor
        }
    }

    // Grabs time to control step duration
    unsigned long time2 = holdTime * 1000 + millis();

    // Hold at temperature while at current step
    while (millis() < time2) {
        // control PID output and fan on/off by using controlTemp function
        controlTemp(setpointTemp);
        // Every second, output to screen
        if(millis() > pulse) { // print to screen every second
            pulse += 1000;    // increment by 1 second
            printData(setpointTemp, cycle); // print data to LCD/serial monitor
        }
    }
            // check if restart is requested
            // TODO: not working

            // char ch = keypad.getKey();
            // if (ch != NO_KEY)
                // buzz(100);
            // if (ch == '#') {
                // lcd.clear();
                // lcd.print("Press # again to restart!");
                // ch = keypad.waitForKey();
                // buzz(100);
                // if (ch == '#')
                    // restartDevice();
            // }

            // if (keypad.getState() == HOLD) {
                // if (keypad.waitForKey() == '#') {
                    // buzz(100);
                    // lcd.clear();
                    // lcd.print("Press # again to restart!");
                    // char ch = keypad.waitForKey();
                    // buzz(100);
                    // if (ch == '#')
                        // restartDevice();
                // }
            // }
}

/*
// Set temperature of heaters
void setTemperature(double setpointTemp, long holdTime, int cycle) {
    unsigned long timer, time2, timerInit;
    timer = millis();
    timerInit = timer;
    pidSetpoint = setpointTemp;

    // while temperature is off from setpointTemp, keep looping
    while (abs(getTemp() - setpointTemp) > 1.0) {
        // control PID and fan with other function
        controlTemp(setpointTemp);
        timer = millis();   // get current time in ms
        if(timer > pulse) { // print to screen every second
            pulse += 1000;    // increment by 1 second
            printData(setpointTemp, cycle); // print data to LCD/serial monitor
        }
    }

    // Grabs time to control step duration
    timerInit = timer;
    time2 = holdTime * 1000 + timerInit;

    // Hold at temperature while at current step
    while (timer < time2) {
        // control PID output and fan on/off by using controlTemp function
        controlTemp(setpointTemp);
        // Every second, output to screen
        timer = millis();
        if(timer > pulse) { // print to screen every second
            pulse += 1000;    // increment by 1 second
            printData(setpointTemp, cycle); // print data to LCD/serial monitor

            // check if restart is requested
            // TODO: not working

            // char ch = keypad.getKey();
            // if (ch != NO_KEY)
                // buzz(100);
            // if (ch == '#') {
                // lcd.clear();
                // lcd.print("Press # again to restart!");
                // ch = keypad.waitForKey();
                // buzz(100);
                // if (ch == '#')
                    // restartDevice();
            // }

            // if (keypad.getState() == HOLD) {
                // if (keypad.waitForKey() == '#') {
                    // buzz(100);
                    // lcd.clear();
                    // lcd.print("Press # again to restart!");
                    // char ch = keypad.waitForKey();
                    // buzz(100);
                    // if (ch == '#')
                        // restartDevice();
                // }
            // }
        }
    }
}
*/

// Control temperature by calculating PID and turning fan on or off.
void controlTemp(double setpointTemp) {
    // Read temperatures into global variables
    double temp = getTemp();

    // if temp too high, turn fan on to cool.  else, turn fan off.
    //if (temp>setpointTemp+1) {
    if (temp > setpointTemp) {
        digitalWrite(pinFan1, HIGH);
        digitalWrite(pinFan2, HIGH);
    }
    else {
        digitalWrite(pinFan1, LOW);
        digitalWrite(pinFan2, LOW);
    }

    // Send temperature to PID controller for predictive control
    pidInput = temp;

    // Compute next PID step
    myPID.Compute();

    // Control heater based upon PID compute
    analogWrite(pinHeater, pidOutput);
}


// Either choose a preset recipe for temperatures
int recipeChoice() {
    char buff1[20];
    char buff2[20];
    char ch1 = 'T';
    char ch2 = 'W';

    lcd.clear();
    lcd.selectLine(1);
    lcd.print("Choose PCR recipe");
    lcd.selectLine(2);
    lcd.print("[1 - 9]");
    // Recipe choice input (char digit)
    ch1 = keypad.waitForKey();
    buzz(100);

    // Sets recipes, can be 0-9, # and * as chars (up to 12 recipes stored)
labelSwitch1:
    switch (ch1) {
        // { numCycles,
        //   spInitz, spDenat, spAnnea, spExten, spFElng, spFHold,
        //   htInitz, htDenat, htAnnea, htExten, htFElng, htFHold }
        case '1':
            pcr = {  35,
                     96, 96, 50, 72, 72,   4,
                    600, 20, 20, 30, 30, 120 };
            break;
        case '2':
            pcr = {  55,
                     96, 96, 50, 72, 72,   4,
                    600, 20, 20, 30, 30, 120 };
            break;
        case '3':
            pcr = {  2,
                    90, 96, 50, 72, 60,  4,
                    30, 30, 30, 30, 30, 30 };
            break;
        case '4':
            pcr = { 10,
                    96, 96, 50, 72, 72,   4,
                    60, 20, 20, 30, 30, 120 };
            break;
        case '5':
            pcr = {  1,
                    52, 50, 40, 45, 47, 4,
                     3,  3,  3,  3,  3, 3 };
            break;

        case '#':   // restart
            lcd.clear();
            lcd.print("Press # again to restart!");
            ch2 = keypad.waitForKey();
            buzz(100);
            return (ch2 == '#') ? restartDevice() : -1;
        default:
            lcd.clear();
            lcd.print("Rcp.");
            lcd.print(ch1);
            lcd.print(" not avail!");
            lcd.selectLine(2);
            lcd.print("Select again");
            delay(1000);
            return -1;
    }

    // Display
    int tReq = totalTimeReq();
    int h = (int)(tReq / 3600.0);
    int m = (int)((tReq - 3600*h) / 60.0);
    int s = tReq - 3600*h - 60*m;
    sprintf(buff1, "R%c, %dh:%dm:%ds", ch1, h, m, s);
    sprintf(buff2, "(%d,%d,%d)%cC x%d", int(pcr.spDenat), int(pcr.spAnnea), int(pcr.spExten), (char)223, pcr.numCycles);
    lcd.clear(); lcd.print(buff1); lcd.selectLine(2); lcd.print(buff2);

    // wait for confirmation
    ch2 = keypad.waitForKey();
    buzz(100);
    recipeChar = ch1;
    // return (ch2 == ch1) ? 0 : -1;
    if (ch2 == ch1)
        return 0;
    else {
        ch1 = ch2; // copy the new choice
        goto labelSwitch1;
    }
}


// read temperatures from digital temperature sensors on aluminium block
double getTemp1() {
    int sensorValue1 = analogRead(A0);
    delay(1); //delay to prevent excess ringing, from datasheet
    double r1 = 1.0e3/((1023.0/sensorValue1) - 1);
    double temp1 = 3560.0/log(r1/0.0130444106) - 273.15;
    return temp1;
}

double getTemp2() {
    int sensorValue2 = analogRead(A7);
    delay(1); //delay to prevent excess ringing, from datasheet
    double r2 = 1.0e3/((1023.0/sensorValue2) - 1);
    double temp2 = 3560.0/log(r2/0.0130444106) - 273.15;
    return temp2;
}

double getTemp() {
    // Average temperatures of two sensors to control fan and PID
    return (getTemp1() + getTemp2()) / 2;
}


// displays data to LCD monitor and serial monitor
void printData(double setpointTemp, int cycle) {
    char buff1[20];
    char buff2[20];
    char buff3[20];
    char stageStr[20];
    // byte stageNum;
    stageType stageNum;
    double temp = getTemp();

    int tPas = (millis() - startMillis) / 1000.0;
    int tReq = totalTimeReq();
    int tRem = calcTimeRemaining();
    int h = (int)(tRem / 3600.0);
    int m = (int)((tRem - 3600*h) / 60.0);
    int s = tRem - 3600*h - 60*m;

    if (setpointTemp == pcr.spInitz || setpointTemp == pcr.spDenat) {
        strcpy(stageStr, "DENAT");
        stageNum = DENAT;
    }
    else if (setpointTemp == pcr.spAnnea) {
        strcpy(stageStr, "ANNEA");
        stageNum = ANNEA;
    }
    else if (setpointTemp == pcr.spExten || setpointTemp == pcr.spFElng) {
        strcpy(stageStr, "EXTEN");
        stageNum = EXTEN;
    }
    else {
        strcpy(stageStr, "XXXXX");
        stageNum = XXXXX;
    }

    // buffered charString that contains data printed to LCD
    sprintf(buff1, "R%c,C#%d/%d,%s", (char)recipeChar, cycle, pcr.numCycles, stageStr);
    sprintf(buff2, "%d/%d min, %d%%", int(tPas/60.0), int(tReq/60.0), int(100.0*(tPas*1.0)/(tReq*1.0)));
    sprintf(buff3, "ETA=%dh:%dm:%ds", h, m, s);

    // Prints out time remaining in minutes and current averaged temp (always on line 2)
    lcd.selectLine(1);
    switch ( int(millis()/1000.0) % 6 ) {
        case 0: lcd.clearLine(1); lcd.print(buff1); break;
        case 2: lcd.clearLine(1); lcd.print(buff2); break;
        case 4: lcd.clearLine(1); lcd.print(buff3); break;
    }

    lcd.selectLine(2);
    lcd.print(temp);
    if (temp <= setpointTemp) {
        // lcd.print(' /htr/ ');
        lcd.print(' ');
        lcd.printCustomChar(1);
        sprintf(buff3, "%3d%% ", int(100.0*pidOutput/255.0));
        lcd.print(buff3);
        lcd.printCustomChar(1);
        lcd.print(' ');
    }
    else {
        lcd.print(' ');
        lcd.printCustomChar(2);
        sprintf(buff3, "%3d%% ", int(100.0*pidOutput/255.0));
        lcd.print(buff3);
        lcd.printCustomChar(2);
        lcd.print(' ');
    }
    lcd.print(setpointTemp, 0);

    // Print to serial monitor as well
    // Appearance:  ## ##.## ##.## ##.##
    Serial.print((millis()-startMillis)/1000.0);
    Serial.print("\t");
    Serial.print(calcTimeRemaining());
    Serial.print("\t");
    Serial.print(cycle);
    Serial.print("\t");
    Serial.print(setpointTemp);
    Serial.print("\t");
    Serial.print(getTemp1());
    Serial.print("\t");
    Serial.print(getTemp2());
    Serial.print("\t");
    Serial.print(temp);
    Serial.print("\t");
    Serial.print(int(pidOutput));
    Serial.print("\t");
    if(digitalRead(pinFan2) == HIGH)
        Serial.print(1);
    else
        Serial.print(0);
    Serial.print("\t");
    Serial.print(stageNum);
    Serial.println();
}


// return time remaining on this PCR recipe
// return: time remaining in minutes, int
int calcTimeRemaining() {
    return totalTimeReq() - ((millis()-startMillis)/1000.0);
}


int totalTimeReq() {
    // the ramp slopes (degree/s) were determined experimentally
    int totTime =       0.70 * abs(pcr.spInitz -          25) + pcr.htInitz
                   + (  0.43 * abs(pcr.spDenat - pcr.spExten) + pcr.htDenat
                      + 0.63 * abs(pcr.spAnnea - pcr.spDenat) + pcr.htAnnea
                      + 0.34 * abs(pcr.spExten - pcr.spAnnea) + pcr.htAnnea ) * pcr.numCycles
                   +    0.50 * abs(pcr.htFElng - pcr.spExten) + pcr.htFElng;
    int fudgeFactor = 1.2;
    return totTime * (1 + fudgeFactor);
}

void buzz(int milliseconds) {
    tone(pinBuzzer, 2400);
    for (byte i = 0; i < milliseconds/100; i++)
        delay(100);
    noTone(pinBuzzer);
}

void buzz() {
    buzz(100);
}

int restartDevice() {
    lcd.clear();
    lcd.print("Restarting"); delay(500);
    tone(pinBuzzer, 2400);
    for (byte i = 0; i < 5; i++) {
        lcd.print(".");
        delay(100);
    }
    noTone(pinBuzzer);
    resetFunc(); // restart the device
}

void lcdAnim(char *str, int msDelay) {
    for(byte i = 0; i < strlen(str); i++) {
        lcd.print(str[i]);
        delay(msDelay);
    }
}

void lcdAnim(char *str) {
    lcdAnim(str, 50);
}

// program to autotune the pcr. Set the "setpoint" variable
// and read the optimized PID from the serial port
// use "SetControlType" to 0 for PI or 1 for PID (in changeAutoTune)

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <math.h>

byte ATuneModeRemember = 2;
double input = 25;
double output = 255;
double kp = 26.49;  // at 50 C new box and fan
double ki = 0.68;
double kd = 0;

double kpmodel = 1.5;
double taup = 100;
double theta[50];

double setpoint = 70;
double aTuneNoise = 5;

double aTuneStartValue = 128;
double outputStart = 128;
double aTuneStep = 50;
unsigned int aTuneLookBack = 20;

boolean tuning = true;
unsigned long modelTime;
unsigned long serialTime;

void readTemps(); // Reads voltages from thermistor/resistor voltage dividers and calculates temperatures
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);


//set to false to connect to the real world
boolean useSimulation = false;

const int heater = 3;
const int fan = 5;
const int fan2 = 2;
const int buzzer = A3;

//Initialize temperatures
double temp1 = 0;
double temp2 = 0;
double temp = 0;

void setup() {
    pinMode(heater, OUTPUT);
    pinMode(fan, OUTPUT);
    pinMode(fan2, OUTPUT);
    pinMode(buzzer, OUTPUT);

    aTune.SetControlType(0);  // 0 for PI tuning. 1 for PID tuning
    if (useSimulation) {
        for (byte i = 0; i < 50; i++)
            theta[i] = outputStart;
        modelTime = 0;
    }

    // Setup the pid
    myPID.SetMode(AUTOMATIC);

    if (tuning) {
        tuning = false;
        changeAutoTune();
        tuning = true;
    }

    serialTime = 0;
    Serial.begin(9600);

    if (!useSimulation) { //pull the input in from the real world
        readTemps();
        digitalWrite(heater,HIGH);
        while(temp < setpoint) {
            if (millis() > serialTime)
            {
                readTemps();
                serialTime += 500;
                Serial.print(millis() * 1e-3);
                Serial.print("\t");
                Serial.print("setpoint: "); Serial.print(setpoint); Serial.print(" ");
                Serial.print("input: ");    Serial.print(temp);     Serial.print(" ");
                Serial.print("output: ");   Serial.print(output);   Serial.println();
            }
        }
        digitalWrite(heater,LOW);
    }
}

void loop() {
    unsigned long now = millis();

    if (!useSimulation) { //pull the input in from the real world
        readTemps();
        input=temp;
    }

    if (tuning) {
        if (aTune.Runtime() != 0) {
          tuning = false;
        }
        if (!tuning) { // we're done, set the tuning parameters
            tone(buzzer, 2400, 9000);
            kp = aTune.GetKp();
            ki = aTune.GetKi();
            kd = aTune.GetKd();
            myPID.SetTunings(kp,ki,kd);
            AutoTuneHelper(false);
        }
    }
    else {
        myPID.Compute();
        if (temp > setpoint) {
            digitalWrite(fan, HIGH);
            digitalWrite(fan2, HIGH);
        }
        else {
            digitalWrite(fan, LOW);
            digitalWrite(fan2, LOW);
        }
    }

    if (useSimulation) {
        theta[30]=output;
        if (now >= modelTime)
        {
            modelTime += 100;
            DoModel();
        }
    }
    else {
    //     if (output>=128){
    //       digitalWrite(fan,LOW);
    //       analogWrite(heater,(output-128)*2);
    //       //Serial.print("heater ");Serial.print((output-128)*2);Serial.println();
    //     }
    //     if (output<=127){
    //       digitalWrite(heater,LOW);
    //       analogWrite(fan,(127-output)*2);
    //       //Serial.print("fan ");Serial.print((127-output)*2);Serial.println();
    //     }
        analogWrite(heater,output);
        if (aTune.Runtime() == 0) {   // if still tuning
            // if (temp > setpoint) {
            // if (output == 1) {
            if (output < outputStart) {
                digitalWrite(fan, HIGH);
                digitalWrite(fan2, HIGH);
            }
            else {
                digitalWrite(fan, LOW);
                digitalWrite(fan2, LOW);
            }
        }
    }

    //send-receive with processing if it's time
    if (millis() > serialTime) {
        SerialReceive();
        SerialSend();
        serialTime += 500;
    }
}

void changeAutoTune() {
    if (!tuning) { //Set the output to the desired starting frequency.
        output=aTuneStartValue;
        aTune.SetNoiseBand(aTuneNoise);
        aTune.SetOutputStep(aTuneStep);
        aTune.SetControlType(1);    // 0 for PI tuning. 1 for PID tuning
        aTune.SetLookbackSec((int)aTuneLookBack);
        AutoTuneHelper(true);
        tuning = true;
    }
    else { //cancel autotune
        aTune.Cancel();
        tuning = false;
        AutoTuneHelper(false);
    }
}

void AutoTuneHelper(boolean start)
{
    if (start)
        ATuneModeRemember = myPID.GetMode();
    else
        myPID.SetMode(ATuneModeRemember);
}


void SerialSend() {
    Serial.print(millis() * 1e-3);
    Serial.print("\t");
    Serial.print("setpoint: "); Serial.print(setpoint); Serial.print(" ");
    Serial.print("input: ");    Serial.print(input);    Serial.print(" ");
    Serial.print("output: ");   Serial.print(output);   Serial.print(" ");
    if (tuning){
        Serial.println("tuning mode");
    }
    else {
        Serial.print("kp: "); Serial.print(myPID.GetKp()); Serial.print(" ");
        Serial.print("ki: "); Serial.print(myPID.GetKi()); Serial.print(" ");
        Serial.print("kd: "); Serial.print(myPID.GetKd()); Serial.println();
    }
}

void SerialReceive()
{
  if (Serial.available())
  {
   char b = Serial.read();
   Serial.flush();
   if ((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for (byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

void readTemps()
{
  int sensorValue1, sensorValue2;
  double r1, r2;

  //read voltage of sensors (voltage across thermistors)- gives a count value from 0-1023 representing 0-5 V
  sensorValue1 = analogRead(A0);
  delay(1); //delay to prevent excess ringing, from datasheet
  sensorValue2 = analogRead(A7);

  //Calculate resistance of thermistor based on measured voltage in "counts"- equation from layout of voltage divider
  r1 = 1000.0/((1023.0/sensorValue1) - 1);
  r2 = 1000.0/((1023.0/sensorValue2) - 1);

  //Calculte temperatures in degrees C- standard thermistor equation with values from thermistor datasheet
  temp1 = 3560.0/log(r1/0.0130444106) - 273.15;
  temp2 = 3560.0/log(r2/0.0130444106) - 273.15;
  temp = (temp1+temp2)/2.0;
}



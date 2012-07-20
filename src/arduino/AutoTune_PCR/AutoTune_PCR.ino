//program to autotune the pcr. Set the "setpoint" variable
//and read the optimized PID from the serial port
//use "SetControlType" to 0 for PI or 1 for PID (in changeAutoTune)

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <math.h>

byte ATuneModeRemember=2;
double input=25, output=255, setpoint=90;
//double kp=5.11,ki=.5,kd=0; //at 50 C
//double kp=6.25,ki=.15,kd=0; //at 100 C
//double kp=22.19,ki=.4,kd=0; //at 50 C new box
double kp=26.49,ki=.68,kd=0; //at 50 C new box and fan

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=127, aTuneNoise=1, aTuneStartValue=128;
unsigned int aTuneLookBack=20;

boolean tuning = true;
unsigned long  modelTime, serialTime;

PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&input, &output);


//set to false to connect to the real world
boolean useSimulation = false;

const int heater=3;
const int fan = 5;

void setup()
{
  double sensorValue1;
  double r1;
  double temp;
  pinMode(heater, OUTPUT);
  pinMode(fan, OUTPUT);
  //digitalWrite(2,HIGH);
  aTune.SetControlType(1);//set for PID extraction instead of PI
  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);

  if(!useSimulation)
  { //pull the input in from the real world
    sensorValue1 = analogRead(0);
    r1 = 1000.0/((1023.0/sensorValue1)-1);
    temp = 3560.0/log(r1/0.0130444106) - 273.15;
    digitalWrite(heater,HIGH);
    while(temp<setpoint){
      sensorValue1 = analogRead(0);
      r1 = 1000.0/((1023.0/sensorValue1)-1);
      temp = 3560.0/log(r1/0.0130444106) - 273.15;
      Serial.print(temp);Serial.println();
    }
    digitalWrite(heater,LOW);
    
  }
}

void loop()
{
  double sensorValue1;
  double r1;

  unsigned long now = millis();

  if(!useSimulation)
  { //pull the input in from the real world
    sensorValue1 = analogRead(0);
    r1 = 1000.0/((1023.0/sensorValue1)-1);
    input = 3560.0/log(r1/0.0130444106) - 273.15;
  }
  
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
//     if(output>=128){
//       digitalWrite(fan,LOW);
//       analogWrite(heater,(output-128)*2);
//       //Serial.print("heater ");Serial.print((output-128)*2);Serial.println();
//     }
//     if(output<=127){
//       digitalWrite(heater,LOW);
//       analogWrite(fan,(127-output)*2);
//       //Serial.print("fan ");Serial.print((127-output)*2);Serial.println();
//     }
     analogWrite(heater,output);
     if(input>setpoint+1){
       digitalWrite(fan,HIGH);
     } else digitalWrite(fan,LOW);
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetControlType(1);//set for PID extraction instead of PI
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + input*(1-1/taup) + ((float)random(-10,10))/100;

}

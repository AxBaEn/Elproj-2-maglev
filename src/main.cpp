#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

//Define pins
const int outPin = 0;

//Define Variables we'll be connecting to
double setpoint, input, output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  input = analogRead(0);
  setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
}

void loop()
{
  input = analogRead(0);

  double Error = abs(setpoint-input); //distance away from setpoint
  /*
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  */

  myPID.Compute();
  analogWrite(3,output);
}
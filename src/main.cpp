#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
 
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

//VL53L0X sensor;

//Define pins
const int outPin = 0;

//Define Variables we'll be connecting to
double setpoint, input, output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);


////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("start");
  //setupCall();

  //initialize the variables we're linked to
  input = analogRead(0);
  setpoint = 15; //15mm

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  Serial.println("Adafruit VL53L0X test");
  if (!sensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }

  //Wire.begin();

  /*sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  sensor.startContinuous();
  */
}

void loop() {
  /*
  input = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  else {Serial.print(input);}

  double error = abs(setpoint-input); //distance away from setpoint
  /*
  if(error<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  analogWrite(3,output);
  */
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
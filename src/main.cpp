#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>

 
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

//VL53L0X sensor;

//define pins
const int pwmPin = 1;

//define constants
const int pwmFreq = 100000;
//uint?
uint sliceNum; //PWM channel
int pwmWrapPoint, pwmDutyCycle;

//define Variables we'll be connecting to
double setpoint, input, output;

//define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);

////////////////////////////////////////////////////////////////////////////////////////////////////////

//set wrap point on the 125MHz (8ns period) 65536 value PWM counter
//wrap = T/(8*10^-9), T = 1/f
void setupPwm(int pwmFreq){
  gpio_set_function(pwmPin, GPIO_FUNC_PWM);
  sliceNum = pwm_gpio_to_slice_num(pwmPin); //get PWM channel
  pwm_set_enabled(sliceNum, true);
  pwmWrapPoint = 10^9/(pwmFreq*8);
  pwm_set_wrap(sliceNum, pwmWrapPoint); //set wrap point
}

//sets PWM duty cycle with 0 <= D <= 1
void setPwmDutyCycle(int pwmDutyCycle){
  pwm_set_chan_level(sliceNum, PWM_CHAN_A, pwmDutyCycle * pwmWrapPoint);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("start");
  //pinMode(1, OUTPUT);
  //digitalWrite(1, HIGH);

  pwmDutyCycle = 0.5;
  setupPwm(pwmFreq);
  setPwmDutyCycle(pwmDutyCycle);

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
  delay(1000);
  //Serial.println("loop");
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
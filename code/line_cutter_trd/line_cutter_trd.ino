#include <Wire.h>
#include "Melon_MS5607.h"
#include "MovingAvg.h"


// states
int state = 0;
const int WAITING = 0;  // anytime before parachute deployment
const int DEPLOYED = 1;
const int PARTIAL_DISREEF = 2;  // after first line is cut
const int FULL_DISREEF = 3;  // after second line is cut
const int LANDED = 4;

// pins
const int VOLTAGE_DIVIDER = A0;
const int NICHROME_PIN1 = A1;
const int NICHROME_PIN2 = A2;

// requirements for state transitions
const double LIMIT_VELOCITY = -0.4;  // m/s
const double ALTITUDE1 = 7;  // disreefing altitudes, in meters
const double ALTITUDE2 = 3.5;

// PWM settings
const double PWM_VOLTAGE1 = 1.4;  // voltage applied to nichrome for line cuts
const double PWM_VOLTAGE2 = 1.4;
const int PWM_DURATION = 1000;  // length of pwm in milliseconds

// altitude calculation
const double SEALEVEL = 102100.0;
double initialAltitude;

// barometer and moving averages
Melon_MS5607 baro{};
int ARRAY_SIZE = 12;
MovingAvg altitudeReadings(ARRAY_SIZE);  // store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // store differences between recent avgs calculated using ^

// variables used in loop()
const int DELAY = 250;  // milliseconds
unsigned long loopStart;
unsigned long loopEnd;
int32_t pressure;  // pascals
double altitude;  // meters
double previousAltitudeAvg;
double currentAltitudeAvg;
double delta;  // meters/second
double currentDeltaAvg;


/*****************************
 *        SETUP/LOOP         *
 ****************************/
 
void setup() {
  Serial.begin(115200);

  // connect to barometer
  while ( !baro.begin(0x76) ) {
    Serial.println("Error connecting to barometer...");
    delay(500);
  }
  Serial.println("Barometer connected with calibration:");
  baro.printCalibData();

  // initialize moving averages
  baro.getPressureBlocking();
  initialAltitude = pressureToAltitude(baro.getPressure());
  Serial.print("Initial altitude [m]: ");
  Serial.println(initialAltitude);
  altitudeReadings.begin();
  altitudeReadings.reading(pressureToAltitude(baro.getPressure()) - initialAltitude);
  altitudeAvgDeltas.begin();
  altitudeAvgDeltas.reading(0.0);

  // analog pins should give a number from 0 to 1023 when read
  analogReadResolution(10);  // 10 bits
}


void loop() {
  loopStart = millis();
 
  // read pressure, calculate altitude
  baro.getPressureBlocking();
  pressure = baro.getPressure();
  altitude = pressureToAltitude(pressure) - initialAltitude;

  // update moving averages
  previousAltitudeAvg = altitudeReadings.getAvg();
  currentAltitudeAvg = altitudeReadings.reading(altitude);  // update and return new avg
  delta = (currentAltitudeAvg - previousAltitudeAvg) * (1000.0 / DELAY);
  currentDeltaAvg = altitudeAvgDeltas.reading(delta);  // update and return new avg

  // printData();
  
  switch(state) {
    case WAITING:
      if (currentAltitudeAvg > ALTITUDE1 && currentDeltaAvg < LIMIT_VELOCITY) {
        Serial.print("Parachute deployed at time ");
        Serial.println(millis());
        printData();
        state = DEPLOYED;
      }
      break;
    case DEPLOYED:
      if (currentAltitudeAvg < ALTITUDE1) {
        pwmExecute(NICHROME_PIN1, PWM_VOLTAGE1);
        Serial.print("First line cut at time ");
        Serial.println(millis());
        printData();
        state = PARTIAL_DISREEF;
      }
      break;
    case PARTIAL_DISREEF:
      if (currentAltitudeAvg < ALTITUDE2) {
        pwmExecute(NICHROME_PIN2, PWM_VOLTAGE2);
        Serial.print("Second line cut at time ");
        Serial.println(millis());
        printData();
        state = FULL_DISREEF;
      }
      break;
    case FULL_DISREEF:
      if (abs(currentDeltaAvg) < 0.05) {
        Serial.print("Landed at time ");
        Serial.println(millis());
        printData();
        state = LANDED;        
      }
      break;
    case LANDED:
      // this doesn't do anything right now.
      // if anyone has thoughts on whether the line cutter should do anything after landing, lmk.
      break;
  }
  
  loopEnd = millis();
  // loop should take a consistent amount of time (will be longer when PWM happens)
  delay(DELAY - constrain(loopEnd - loopStart, 0, DELAY));
}


/*****************************
 *          HELPERS          *
 ****************************/

// convert pressure (in pascals) to altitude (in meters) using sea-level pressure
// int32_t -> double
double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / SEALEVEL, 1.0 / 5.25588));
}

// PWM pin to heat nichrome and cut parachute line
// int, double -> _
void pwmExecute(int pin, double targetVoltage) {
  Serial.print("Starting PWM on pin ");
  Serial.println(pin);
  analogWrite(pin, pwmLevel(targetVoltage));
  delay(PWM_DURATION);
  analogWrite(pin, 0);
  Serial.println("Done.");
  return;
}

// calculate number to be used for analogWrite() of nichrome pin
// double -> int
int pwmLevel(double targetVoltage) {
  // get reading from voltage divider and multiply by 2
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  // to get current vbat  
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  // find proportion of vbat needed to apply target voltage, then make it out of 255 for analogWrite
  return (targetVoltage / vbat) * 255;
}

void printData() {
  Serial.print("State: ");
  Serial.println(state);
  Serial.print("Pressure [Pa]: ");
  Serial.println(pressure);
  Serial.print("Altitude [m]: ");
  Serial.println(altitude);
  Serial.print("Averaged altitude [m]: ");
  Serial.println(currentAltitudeAvg);
  Serial.print("Change in altitude [m/s]: ");
  Serial.println(delta);
  Serial.print("Averaged change in altitude [m/s]: ");
  Serial.println(currentDeltaAvg);
  Serial.println("------------");
  return;
}

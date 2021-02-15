#include <algorithm>
#include <iterator>
#include <Wire.h>
#include <"Melon_MS5607.h">
#include <"MovingAvg.h">


// states
int state = 0;
const int WAITING = 0;  // anytime before parachute deployment
const int DEPLOYED = 1;
const int PARTIAL_DISREEF = 2;  // after first line is cut
const int FULL_DISREEF = 3;  // after second line is cut
const int LANDED = 4;

// requirements for state transitions
const int DELAY = 20  // milliseconds
const int MIN_SPEED = 3;  // m/s
const double MIN_DELTA = MIN_SPEED * -(DELAY / 1000.0)
const int ALTITUDE1 = 200;  // disreefing altitudes, in meters
const int ALTITUDE2 = 150;

// PWM settings
const double PWM_VOLTAGE1 = 1.4;  // voltage applied to nichrome for line cuts
const double PWM_VOLTAGE2 = 1.4;
const int PWM_DURATION = 2000;  // length of pwm in milliseconds

// pins
const int VOLTAGE_DIVIDER = A0;
const int NICHROME_PIN1 = A1;
const int NICHROME_PIN2 = A2;

// pressure calculation
const double SEALEVEL = 101325.0;

// variables for loop()
unsigned long loopStart;
unsigned long loopEnd;
int32_t pressure;  // pascals
double altitude;  // meters
int previousAvg;
int currentAvg;
int delta;
int* deltaArray;

// create barometer and moving averages
Melon_MS5607 baro{};
movingAvg altitudeReadings(10);  // store last 10 altitude readings
movingAvg altitudeAvgDeltas(10);  // store differences between last 10 avgs calculated using ^


/*************************
 *      SETUP/LOOP       *
 ************************/
 
void setup() {
  Serial.begin(9600);

  // connect to barometer
  while ( !baro.begin(0x76) ) {
    Serial.println("Error connecting to barometer...");
    delay(500);
  }
  Serial.println("Barometer connected with calibration:");
  baro.printCalibData();

  // initialize moving averages
  altitudeReadings.begin();
  altitudeAvgDeltas.begin();

  // analog pins should give a number from 0 to 1023 when read
  analogReadResolution(10);  // 10 bits
}


void loop() {
  loopStart = millis();
  Serial.println(loopStart + "-----------");
 
  // read pressure, calculate altitude
  baro.getPressureBlocking();
  pressure = baro.getPressure();
  Serial.print("Pressure [Pa]: ");
  Serial.println(pressure);
  altitude = pressureToAltitude(pressure);

  // update moving averages
  previousAvg = altitudeReadings.getAvg();
  currentAvg = altitudeReadings.reading(altitude);  // update and return new avg
  Serial.print("Altitude [m]: ");
  Serial.println(currentAvg);
  delta = currentAvg - previousAvg;
  altitudeAvgDeltas.reading(delta);
  Serial.print("Change in altitude [m]: ");
  Serial.println(delta);
  
  switch(state) {
    case WAITING:
      deltaArray = altitudeAvgDeltas.getReadings();
      if (currentAvg > ALTITUDE1 &&
          all_of(std::begin(deltaArray), std::end(deltaArray), 
            // all recent deltas should be lower negative numbers than limit
            [](int delta){ return delta < MIN_DELTA; })) {
        state = DEPLOYED;
      }
      break;
    case DEPLOYED:
      if (currentAvg < ALTITUDE1) {
        pwmExecute(NICHROME_PIN1, PWM_VOLTAGE1);
        state = PARTIAL_DISREEF;
      }
      break;
    case PARTIAL_DISREEF:
      if (currentAvg < ALTITUDE2) {
        pwmExecute(NICHROME_PIN2, PWM_VOLTAGE2);
        state = FULL_DISREEF;
      }
      break;
    case FULL_DISREEF:
      deltaArray = altitudeAvgDeltas.getReadings();
      if (all_of(std::begin(deltaArray), std::end(deltaArray),
            // all recent deltas should be very small if rocket has landed
            [](int delta){ return abs(delta) < 0.05; }) {
        state = LANDED;        
      }
      break;
    case LANDED:
      // this doesn't do anything right now.
      // if anyone has thoughts on whether the line cutter should do anything after landing, lmk.
      break;
  }
  
  loopEnd = millis();
  // loop should take a consistent amount of time
  delay(DELAY - constrain(loopEnd - loopStart, 0, DELAY));
}


/*************************
 *        HELPERS        *
 ************************/

double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / SEALEVEL, 1.0 / 5.25588));
}


void pwmExecute(int pin, double targetVoltage) {
  Serial.println("Starting PWM ...");
  analogWrite(pin, pwmLevel(targetVoltage));
  delay(PWM_DURATION);
  analogWrite(pin, 0);
  Serial.println("Done.");
  return;
}


int pwmLevel(double targetVoltage) {
  // get reading from voltage divider and multiply by 2
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  // to get current vbat
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  // find proportion of vbat needed to apply target voltage, then make it out of 255 for analogWrite
  return (targetVoltage / vbat) * 255.0;
}

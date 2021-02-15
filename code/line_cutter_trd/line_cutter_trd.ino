#include <Wire.h>
#include "Melon_MS5607.h"

int state = 0;

// states
const int WAITING = 0;  // anytime before parachute deployment
const int DEPLOYED = 1;
const int PARTIAL_DISREEF = 2;  // after first parachute line is cut
const int FULL_DISREEF = 3;  // after second parachute line is cut
const int LANDED = 4;

// PWM settings
const double PWM_VOLTAGE1 = 1.4;  // voltage applied to nichrome for first cut
const double PWM_VOLTAGE2 = 1.4;  // voltage applied to nichrome for second cut
const int PWM_DURATION = 2000;  // length of pwm in milliseconds

// pins
const int VOLTAGE_DIVIDER = A0;
const int NICHROME1 = A1;
const int NICHROME2 = A2;

// pressure calculation
const double SEALEVEL = 101325.0;


// Create barometer
Melon_MS5607 baro{};

void setup() {
  Serial.begin(9600);
  
  while ( !baro.begin(0x76) ) {
    Serial.println("Error connecting to barometer...");
    delay(500);
  }

  Serial.println("Barometer connected with calibration:");
  baro.printCalibData();
  analogReadResolution(10);
}

void loop() {
  switch(state) {
    case WAITING:
      waiting();
      break;
    case DEPLOYED:
      deployed();
      break;
    case PARTIAL_DISREEF:
      partialDisreef();
      break;
    case FULL_DISREEF:
      fullDisreef();
      break;
    case LANDED:
      landed();
      break;
  }
}

/*************************
 *      MAIN STATES      *
 ************************/

void waiting() {
  baro.getPressureBlocking();

  Serial.print("Pressure [Pa]: ");
  Serial.println(baro.getPressure());
  Serial.print("Altitude [m]: ");
  Serial.println(pressureToAltitude(baro.getPressure()));
  Serial.println(analogRead(VOLTAGE_DIVIDER));
  Serial.print("vbat: ");
  Serial.println(2.0 * (analogRead(VOLTAGE_DIVIDER) * 3.6) / 1023);
  Serial.println("---");
  delay(1000);
}

void deployed(){
  
}

void partialDisreef() {
  
}

void fullDisreef() {
  
}

void landed() {
  
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
  double vbat = 2.0 * (analogRead(VOLTAGE_DIVIDER) * 3.6) / 1023.0;
  return (targetVoltage / vbat) * 255.0;
}
 

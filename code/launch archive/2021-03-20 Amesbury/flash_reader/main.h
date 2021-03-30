#pragma once
#include <stdint.h>
#include "MovingAvg.h"
#include "Arduino.h"

int ARRAY_SIZE = 40; // 2 seconds
MovingAvg altitudeReadings(ARRAY_SIZE);  // Store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // Store differences between avgs calculated using ^
// Altitude calculation

float seaLevelFunc(float pressure, float altitude) { return pressure / pow(1.0 - (altitude / 44330.76), 5.25588); }
double seaLevel = seaLevelFunc(96549, 502.0); // ISH

// Variables used in loop()
const int DELAY = 50;  // milliseconds
unsigned long loopStart = 0;
unsigned long loopEnd = 0;
unsigned long lastBLE = 0;
unsigned long lastStateChangeTime = 0;
unsigned long lastLightTime = 0; // Last time that it was light
unsigned long lastDarkTime = 0; // Last time it was dark
uint8_t bleIdx = 0; // Index of data to send
// int32_t pressure;  // pascals
double altitude;  // meters
double previousAltitudeAvg;
double currentAltitudeAvg;
double delta;  // meters/second
double currentDeltaAvg;
// int light;
// Keeps track of if it's been dark for long enough for us to "be in the tube"
bool inTube = false;

// Photoresistor memes
const int LIGHT_THRESHOLD = 200; // Anything above this value is considered to be outside of the tube
const int DARK_TRIGGER_TIME = 20000; // Continuous time interval for which it much be dark for board to decide it's in the tube
const int LIGHT_TRIGGER_TIME = 2000; // Continuous time interval for which it must be light for board to decide it's been ejected

// VARIABLES WHICH MUST BE SET
// Requirements for state transitions
const double LIMIT_VELOCITY = -3.0;  // meters/second
const double ALTITUDE1 = 100;  // Disreefing altitudes, in meters (higher one first!!)
const double ALTITUDE2 = 67;
const int DISREEF1TIME = 45000; // Maximum delay after ejection is detected before the first line is cut.
const int DISREEF2TIME = 5000; // Maximum delay after the first line is cut before the second line is cut.

// // For writing data to flash
struct Data {
  uint8_t state;
  uint32_t timestamp;
  uint32_t pressure;
  float temperature;
  float accelX;
  float accelY;
  float accelZ;
  uint16_t battSense;
  uint16_t cutSense1;
  uint16_t cutSense2;
  uint16_t currentSense;
  uint16_t photoresistor;
};
//Data currentData;

enum states {
  WAITING,  // Anytime before parachute deployment
  DEPLOYED,
  PARTIAL_DISREEF,  // After first line is cut
  FULL_DISREEF,  // After second line is cut
  LANDED
};
int state = WAITING;
char* stateStrings[5] = { "WAITING","DEPLOYED","PARTIAL_DISREEF","FULL_DISREEF","LANDED" };

void printData(const Data& data) {
  Serial.printf("%u, %u, %u, %f, %f, %f, %f, %u, %u, %u, %u, %u\n", 
                data.state, data.timestamp, data.pressure, data.temperature,
                data.accelX, data.accelY, data.accelZ,
                data.battSense, data.cutSense1, data.cutSense2,
                data.currentSense, data.photoresistor);
}

#define VERBOSE true

// Convert pressure (in pascals) to altitude (in meters) using sea level pressure
double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / seaLevel, 1.0 / 5.25588));
}

void fakeSetup(int32_t pressure) {
//  Serial.println("Fake Setup");
  altitudeReadings.begin();
  altitudeReadings.reading(pressureToAltitude(pressure));
  altitudeAvgDeltas.begin();
  altitudeAvgDeltas.reading(0.0);

  if(VERBOSE) {
    Serial.print("Delay1: "); Serial.println(DISREEF1TIME);
    Serial.print("Delay2: "); Serial.println(DISREEF2TIME);
  }
}

void fakeLoop(uint32_t millis_, int32_t pressure, int light) {
//   while(millis < loopEnd + DELAY) {}
  
  loopStart = millis_;  // Used for timestamps in data log
  auto realStart = millis();
 
  altitude = pressureToAltitude(pressure);

  // Update moving averages
  previousAltitudeAvg = altitudeReadings.getAvg();
  currentAltitudeAvg = altitudeReadings.reading(altitude);  // Update and return new avg
  delta = (currentAltitudeAvg - previousAltitudeAvg) * (1000.0 / DELAY);
  currentDeltaAvg = altitudeAvgDeltas.reading(delta);  // Update and return new avg

  // Update flash
  // This calls:
  // updateData, which reads accelerometer stuff
  // GetPres/GetTemp, which just returns a local  number
  // updateFlash, which like actually erases a sector and writes tons of stuff
  // We know that we delay until millis() >= loopEnd + DELAY (50ms)
  // And that our average dT was 84ms
  // So the whole loop takes about 34ms
  // I'm gunna call it 10ms
//  delay(10); // We account for this down in the if statement

// UNCOMMENT ME FOR CSV PRINTS
  if(!VERBOSE) {
    Serial.printf("%u, %u, %f, %u, %f", state, millis_, currentAltitudeAvg, light, currentDeltaAvg);
    Serial.println();
  }

  // Photoresistor stuff
//   light = analogRead(PHOTO_PIN);
  //Keep track of whether it is dark or light and for how long
  if(light > LIGHT_THRESHOLD)
    lastLightTime = loopStart;
  if(light < LIGHT_THRESHOLD)
    lastDarkTime = loopStart;

  unsigned long now = millis();
  if(loopStart - lastLightTime > DARK_TRIGGER_TIME) {
    inTube = true;
    lastStateChangeTime = millis_ + 10 /* ms to write to flash */; // This extra delay is to account for the time it takes to write to flash
//    Serial.println("Now in tube!");
  }

//  Serial.printf("start %u real %u now %u dt %u last change %u ", loopStart, realStart, now, now - realStart, lastStateChangeTime);
//  Serial.print("delta "); Serial.print(loopStart - lastStateChangeTime); Serial.print(" test "); 
//  Serial.println(((unsigned long) 10) - ((unsigned long) 20));

  switch(state) {
    case WAITING:
      if ((currentAltitudeAvg > ALTITUDE1 && currentDeltaAvg < LIMIT_VELOCITY)
          || (inTube && loopStart - lastDarkTime > LIGHT_TRIGGER_TIME)) {
        Serial.printf("[DEPLOYED] Time since last change: %u\n", loopStart - (lastStateChangeTime));
        inTube = false;
        state = DEPLOYED;
        
        
        // InternalFS.remove(FILENAME);
        // writeStateChangeData(file, loopStart, state, 
        //                      pressure, altitude, currentAltitudeAvg, 
        //                      delta, currentDeltaAvg);
        
        if(VERBOSE) {
          Serial.print("Parachute deployed at time ");
          Serial.println(loopStart);
          Serial.printf("Altitude: %f\n", currentAltitudeAvg);
          Serial.printf("\nt: %u\ns: %u\np: %u\na: %f\nb: %f\nd: %f\ne: %f\nl: %u\n\n", 
            loopStart,
            state,
            pressure,
            altitude,
            currentAltitudeAvg,
            delta,
            currentDeltaAvg,
            light);
        }
        lastStateChangeTime = loopStart;
      }
      break;
    case DEPLOYED:
      if ((currentAltitudeAvg < ALTITUDE1)
          || (loopStart - (lastStateChangeTime) > DISREEF1TIME)) {
        Serial.print("Triggered by Altitude? "); Serial.println((currentAltitudeAvg < ALTITUDE1) ? "YES" : "NO");
        Serial.print("Triggered by Photores? "); Serial.println((loopStart - (lastStateChangeTime) > DISREEF1TIME) ? "YES" : "NO");
        Serial.printf("[PARTIAL_DISREEF] Time since last change: %u\n", loopStart - (lastStateChangeTime));
        // pwmExecute(NICHROME_PIN1, PWM_VOLTAGE1);
        state = PARTIAL_DISREEF;
        

        // writeStateChangeData(file, loopStart, state, 
        //                      pressure, altitude, currentAltitudeAvg, 
        //                      delta, currentDeltaAvg);

        if(VERBOSE) {
          Serial.print("First line cut at time ");
          Serial.println(loopStart);
          Serial.printf("Altitude: %f\n", currentAltitudeAvg);
          Serial.printf("\nt: %u\ns: %u\np: %u\na: %f\nb: %f\nd: %f\ne: %f\nl: %u\n\n", 
            loopStart,
            state,
            pressure,
            altitude,
            currentAltitudeAvg,
            delta,
            currentDeltaAvg,
            light);
        }
        lastStateChangeTime = loopStart;
      }
      break;
    case PARTIAL_DISREEF:
      if ((currentAltitudeAvg < ALTITUDE2)
          || (loopStart - (lastStateChangeTime) > DISREEF2TIME)) {
        Serial.print("Triggered by Altitude? "); Serial.println((currentAltitudeAvg < ALTITUDE2) ? "YES" : "NO");
        Serial.print("Triggered by Photores? "); Serial.println((loopStart - (lastStateChangeTime) > DISREEF2TIME) ? "YES" : "NO");
        Serial.printf("[FULL_DISREEF] Time since last change: %u\n", loopStart - (lastStateChangeTime));
        // pwmExecute(NICHROME_PIN2, PWM_VOLTAGE2);
        state = FULL_DISREEF;
        

        // writeStateChangeData(file, loopStart, state, 
        //                      pressure, altitude, currentAltitudeAvg, 
        //                      delta, currentDeltaAvg);
        if(VERBOSE) {
          Serial.print("Second line cut at time ");
          Serial.println(loopStart);
          Serial.printf("Altitude: %f\n", currentAltitudeAvg);
          Serial.printf("\nt: %u\ns: %u\np: %u\na: %f\nb: %f\nd: %f\ne: %f\nl: %u\n\n", 
            loopStart,
            state,
            pressure,
            altitude,
            currentAltitudeAvg,
            delta,
            currentDeltaAvg,
            light);
        }
        lastStateChangeTime = loopStart;
      }
      break;
    case FULL_DISREEF:
      if (abs(currentDeltaAvg) < 0.05) {
        state = LANDED;
        

        // writeStateChangeData(file, loopStart, state, 
        //                      pressure, altitude, currentAltitudeAvg, 
        //                      delta, currentDeltaAvg);

        // writeLandedData(file);
        if(VERBOSE) {
          Serial.print("Landed at time ");
          Serial.println(loopStart);
          Serial.printf("\nt: %u\ns: %u\np: %u\na: %f\nb: %f\nd: %f\ne: %f\nl: %u\n\n", 
            loopStart,
            state,
            pressure,
            altitude,
            currentAltitudeAvg,
            delta,
            currentDeltaAvg,
            light);
         }
        lastStateChangeTime = loopStart;
      }
      break;
    case LANDED:
      // Nothing happens after landing
      break;
    default:
      Serial.println("Something has gone horribly wrong if none of the states match.");
  }

//  currentData.state = state;
//  printData(currentData);
  
  loopEnd = millis_;
}

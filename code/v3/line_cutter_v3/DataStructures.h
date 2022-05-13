#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>  // Needed for fixed-with integers since this is not the main .ino file

// Represents the current state of the line cutter board
struct LineCutterState {
  uint8_t structType = 0;  // Data structure identification number
  bool isArmed;
  uint8_t state;
  uint32_t timestamp;
  uint32_t pressure;
  float altitude;
  float avgAltitude;
  float deltaAltitude;
  float avgDeltaAltitude;
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

// Holds the configuration of the line cutter board for a flight
struct LineCutterConfig {
  uint8_t structType = 1;  // Flight variable data structure identification number
  float limitVel;
  uint16_t altitude1;
  uint16_t altitude2;
  uint32_t disreefDelay1;
  uint32_t disreefDelay2;
  float pwmVoltage1;
  float pwmVoltage2;
  uint16_t pwmDuration;
  uint16_t lightThreshold;
  uint32_t lightTriggerTime;
  uint32_t seaLevelPressure;
};

#endif

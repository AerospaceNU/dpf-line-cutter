#include <SPI.h>
#include "S25FL.h"
#pragma once

struct Data {
  uint8_t structType = 0;  // data struct
  uint8_t state; // 1
  uint32_t timestamp; // 2
  uint32_t pressure; // 6
  float altitude; // 10
  float avgAltitude; // 14
  float deltaAltitude; // 18
  float avgDeltaAltitude; // 22
  float temperature; // 26
  float accelX; // 30
  float accelY; // 34
  float accelZ; // 38
  uint16_t battSense; // 42
  uint16_t cutSense1; // 44
  uint16_t cutSense2; // 46
  uint16_t currentSense; // 48
  uint16_t photoresistor; // 50 (ends at 52)
};

// For writing data to flash & flight variable log
struct FlightVariables {
  uint8_t structType = 1;  // flight variable struct
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
void flash_reader_setup(int readMode);

void readData(uint8_t* flashData, int readMode);

void print_flash_packet(int readMode);

#include <SPI.h>
#include "S25FL.h"
#include "flash_reader.h"

S25FL readModeFlash(8);  // Starts Flash class and initializes SPI
unsigned long flashLocation = 0x40000;
uint8_t metadata[64];
uint8_t sectorPortion[4096];
bool done = false;

Data currentData;

FlightVariables currentFlightVars;

void flash_reader_setup(int readMode) {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  Serial.println("Metadata: ");
  readModeFlash.read_start(0, metadata, 64);
  Serial.print("[");
  for (int i = 0; i < 64; i++) {
    if (i % 8 == 0 && i > 0) {
      Serial.println();
    }
    Serial.print(metadata[i], HEX);
    if (i < 63) {
      Serial.print(",\t");
    }
  }
  Serial.println("]");
  Serial.println();
  delay(10000);  // Give time to connect with CoolTerm or similar

  while (!done) {
    if (flashLocation % 4096 == 0) {
      readModeFlash.read_start(flashLocation, sectorPortion, 4096);
    }
    if (sectorPortion[flashLocation % 4096] == readMode) {
      readData(&sectorPortion[flashLocation % 4096], readMode);
      print_flash_packet(readMode);
    } else if (sectorPortion[flashLocation % 4096] == 0xff || flashLocation >= 0x1000000) {
      done = true;
    }
    flashLocation += 64;
  }
  delay(1);
}

void readData(uint8_t* flashData, int readMode) {
  if (readMode == 0) {
    currentData = *( Data* ) flashData;
  } else if (readMode == 1) {
    currentFlightVars = *( FlightVariables* ) flashData;
  } else {
    Serial.println("Invalid read mode");
  }
}

void print_flash_packet(int readMode) {
  if (readMode == 0) {
    Serial.printf("%u,%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u,%u,%li\n",
                  currentData.state, currentData.timestamp, currentData.pressure,
                  currentData.altitude, currentData.avgAltitude, currentData.deltaAltitude, currentData.avgDeltaAltitude,
                  currentData.temperature, currentData.accelX, currentData.accelY, currentData.accelZ,
                  currentData.battSense, currentData.cutSense1, currentData.cutSense2,
                  currentData.currentSense, currentData.photoresistor);
  } else if (readMode == 1) {
    Serial.printf("%f,%u,%u,%u,%u,%f,%f,%u,%u,%u,%u\n",
                  currentFlightVars.limitVel, currentFlightVars.altitude1, currentFlightVars.altitude2,
                  currentFlightVars.disreefDelay1, currentFlightVars.disreefDelay2,
                  currentFlightVars.pwmVoltage1, currentFlightVars.pwmVoltage2, currentFlightVars.pwmDuration,
                  currentFlightVars.lightThreshold, currentFlightVars.lightTriggerTime, currentFlightVars.seaLevelPressure);
  }
}

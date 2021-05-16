#include <SPI.h>
#include "S25FL.h"

int readMode = 0;  // set to 0 to read flight data entries, 1 to read flight variable entries
S25FL flash;  // Starts Flash class and initializes SPI
unsigned long flashLocation = 0x40000;
uint8_t metadata[64];
uint8_t sectorPortion[4096];
bool done = false;

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
Data currentData;

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
FlightVariables currentFlightVars;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  Serial.println("Metadata: ");
  flash.read_start(0, metadata, 64);
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
      flash.read_start(flashLocation, sectorPortion, 4096);
    }
    if (sectorPortion[flashLocation % 4096] == readMode) {
      readData(&sectorPortion[flashLocation % 4096]);
      print_flash_packet();
    } else if (sectorPortion[flashLocation % 4096] == 0xff || flashLocation >= 0x1000000) {
      done = true;
    }
    flashLocation += 64;
  }
  delay(1);
}

void loop() {
}

void readData(uint8_t* flashData) {
  if (readMode == 0) {
    currentData = *( Data* ) flashData;
  } else {
    currentFlightVars = *( FlightVariables* ) flashData;
  }
}

void print_flash_packet() {
  if (readMode == 0) {
    Serial.printf("%u,%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u,%u,%li\n",
                  currentData.state, currentData.timestamp, currentData.pressure,
                  currentData.altitude, currentData.avgAltitude, currentData.deltaAltitude, currentData.avgDeltaAltitude,
                  currentData.temperature, currentData.accelX, currentData.accelY, currentData.accelZ,
                  currentData.battSense, currentData.cutSense1, currentData.cutSense2,
                  currentData.currentSense, currentData.photoresistor);
  } else {
    Serial.printf("%f,%u,%u,%u,%u,%f,%f,%u,%u,%u,%u\n",
                  currentFlightVars.limitVel, currentFlightVars.altitude1, currentFlightVars.altitude2,
                  currentFlightVars.disreefDelay1, currentFlightVars.disreefDelay2,
                  currentFlightVars.pwmVoltage1, currentFlightVars.pwmVoltage2, currentFlightVars.pwmDuration,
                  currentFlightVars.lightThreshold, currentFlightVars.lightTriggerTime, currentFlightVars.seaLevelPressure);
  }
}

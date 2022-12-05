#ifndef LINE_CUTTER_H
#define LINE_CUTTER_H

#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"
#include "S25FL.h"
#include "icm20602/ICM20602.h"
#include "flash_reader.h"

/*****************************
          SETUP/LOOP
 ****************************/

void line_cutter_setup();


void line_cutter_loop();


/*****************************
            HELPERS
 ****************************/

int parse_command();

void startAdv(void);

void setFlashLocation();

// Get variables for this flight from InternalFS
void readFlightVariables();

// Average several readings to get "sea level" pressure
double calibrateSeaLevel(int samples);

// Convert pressure (in pascals) to altitude (in meters) using sea level pressure
double pressureToAltitude(int32_t pressure);
// PWM pin to heat nichrome and cut parachute line
void pwmStart();

// Calculate number to be used for PWM of nichrome pin
int pwmLevel(double targetVoltage);

void progressState();

void writeStateChangeData();

void updateDataStruct();

void sendSensorData();

void sendFlightVariables();

void updateFlash(uint8_t* data, int sizeOfData);

#endif

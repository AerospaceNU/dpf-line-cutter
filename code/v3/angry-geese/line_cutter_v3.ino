#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"
#include "S25FL.h"
#include "icm20602/ICM20602.h"
#include "altitude_kalman.h"

// For angry geese defs
#include "AngryGoose_pins.h"

enum states {
  WAITING,  // Before arming
  ARMED,  // Will be armed once above altitude specified in flight variables
  DEPLOYED,  // Apogee
  PARTIAL_DISREEF,  // After first line is cut
  FULL_DISREEF,  // After second line is cut
  LANDED
};
int state = WAITING;
char* stateStrings[6] = { "WAITING", "ARMED", "DEPLOYED", "PARTIAL_DISREEF", "FULL_DISREEF", "LANDED" };

// Barometer and moving averages
MS5xxx baro(&Wire);
int ARRAY_SIZE = 4;  // 2 seconds at 20Hz
MovingAvg altitudeReadings(ARRAY_SIZE);  // Store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // Store differences between avgs calculated using ^
// Altitude calculation
double seaLevel;

S25FL flash(FLASH_CS);  // Starts Flash class and initializes SPI
unsigned long flashLocation;  // Next location to write data to
unsigned long flashLocationLocation;  // Next location to write flash location to
uint8_t metadata[64];

// Variables used in loop()
const int DELAY = 1000 / 50;  // milliseconds
unsigned long loopStart = 0;
unsigned long lastStateChange = 0;
unsigned long cutStart1 = 0;
unsigned long cutStart2 = 0;
float previousAltitude;
int32_t pressure;  // pascals
double altitude;  // meters
bool armed;

AltitudeKalman kalman {DELAY / 1000.0};

IMU imu{}; // accelerometer
imu_out_t accelData;

// For writing data to flash & state transition log
struct Data {
  uint8_t structType = 0;  // data struct
  uint8_t state;
  uint32_t timestamp;
  uint32_t pressure;
  float altitude;
  float avgAltitude;
  float deltaAltitude;
  float temperature;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  uint16_t cutSense1;
  uint16_t cutSense2;
};
Data currentData;
const int DATA_SIZE = sizeof(Data);

// For writing data to flash & flight variable log
struct FlightVariables {
  uint8_t structType = 1; // Flight variable struct. Only altitude1, pwm vars, and seaLevel are used
  uint16_t altitude1 = 50;     // Used as arming altitude
  uint16_t altitude2 = 250; // Main deploy
  uint32_t seaLevelPressure;
};
FlightVariables currentFlightVars = {};
const int FLIGHT_VARIABLE_SIZE = sizeof(FlightVariables);

/*****************************
          SETUP/LOOP
 ****************************/

void setup() {
  Serial.begin(115200);

  Wire.begin();
  // Connect to accelerometer
  while (!imu.begin()) {
    Serial.println("Error connecting to accelerometer...");
  }
  // Connect to barometer
  while ( baro.connect() > 0 ) {
    Serial.println("Error connecting to barometer...");
    delay(500);
  }
  // Calibrate barometer and get initial reading
  baro.ReadProm();
  baro.Readout();

  // Initialize moving averages
  seaLevel = calibrateSeaLevel(50);
  Serial.print("Sea level pressure [Pa]: ");
  Serial.println(seaLevel);
  altitudeReadings.begin();
  altitudeReadings.reading(pressureToAltitude(baro.GetPres()));
  altitudeAvgDeltas.begin();
  altitudeAvgDeltas.reading(0.0);

  // Set up flash
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);

  setFlashLocation();
  readFlightVariables();
  updateFlash(( uint8_t* ) &currentFlightVars, FLIGHT_VARIABLE_SIZE);
}


void loop() {
  while (millis() < loopStart + DELAY) {}

  loopStart = millis();  // Used for timestamps in data log

  // Read pressure, calculate altitude
  baro.Readout();
  pressure = baro.GetPres();
  altitude = pressureToAltitude(pressure);

  // Update accelerometer
  accelData = imu.readout();

  // Update Kalman filter
  kalman.Correct(altitude, DEFAULT_KALMAN_GAIN);
  if(state < PARTIAL_DISREEF) {
    kalman.Predict(sqrtf(
      pow(accelData.accel_xout, 2) + 
      pow(accelData.accel_yout, 2) + 
      pow(accelData.accel_zout, 2)
      ));
  } else {
    kalman.Predict(0.0);
  }
  const auto xhat = kalman.GetXhat();

  // Update data struct, send to BLE central and flash
  updateDataStruct();
  updateFlash(( uint8_t* ) &currentData, DATA_SIZE);

  while (Serial.available()) {
    uint8_t ch;
    ch = (uint8_t) Serial.read();
    // '!' indicates start of a command. anything else is ignored
    if (ch == (uint8_t) '!') {
      parse_command();
    }
  }

  if (cutStart1 > 0 && loopStart - cutStart1 > PYRO_FIRE_DURATION) {
    analogWrite(PYRO_CUT1, 0);
    Serial.print("Ended PWM on pin ");
    Serial.println(PYRO_CUT1);
    cutStart1 = 0;
  }
  if (cutStart2 > 0 && loopStart - cutStart2 > PYRO_FIRE_DURATION) {
    analogWrite(PYRO_CUT2, 0);
    Serial.print("Ended PWM on pin ");
    Serial.println(PYRO_CUT2);
    cutStart2 = 0;
  }

  switch (state) {
    case WAITING:
      if (armed == true) {
        state = ARMED;
      }
      break;
    case ARMED:
      if (armed == false) {
        state = WAITING;
      }
      // Check if we're high enough
      if (xhat.estimatedAltitude > currentFlightVars.altitude1) {
        progressState();
      }
      break;
    case DEPLOYED:
      // If we started falling, deploy the drogue
      if (xhat.estimatedVelocity <= 0 && xhat.estimatedAltitude < previousAltitude) {
        pwmStart();
        cutStart1 = loopStart;
        progressState();
      }
      break;
    // 
    case PARTIAL_DISREEF:
      // If we're below the main altitude, deploy the main
      if (xhat.estimatedAltitude < currentFlightVars.altitude2) {
        pwmStart();
        cutStart2 = loopStart;
        progressState();
      }
      break;
    case FULL_DISREEF:
      // If we aren't really moving, we've landed
      if (abs(xhat.estimatedVelocity) < 0.05) {
        progressState();
      }
      break;
    case LANDED:
      // Nothing happens after landing
      break;
    default:
      Serial.println("Something has gone horribly wrong if none of the states match.");
  }
}


/*****************************
            HELPERS
 ****************************/

void parse_command() {
  uint8_t char_array[8];
  int current_num = 0;

  // read text after '!' which started command
  while ( Serial.available() && current_num < 8 )
  {
    char_array[current_num] = (uint8_t) Serial.read();
    current_num++;
  }

  String command = (char*) char_array;

  if (command.substring(0, 4).equals("help")) {
    Serial.print("Valid commands:\n");
    Serial.print("!arm\n");
    Serial.print("!disarm\n");
    Serial.print("!vars\n");
    Serial.print("!data\n");
  }
  else if (command.substring(0, 3).equals("arm")) {
    if (state == WAITING) {
      armed = true;
      Serial.print("Armed.\n");
    } else {
      Serial.print("Not in waiting state.\n");
    }
  }
  else if (command.substring(0, 6).equals("disarm")) {
    if (state == ARMED) {
      armed = false;
      Serial.print("Disarmed.\n");
    } else {
      Serial.print("Not in armed state.\n");
    }
  }
  else if (command.substring(0, 4).equals("vars")) {
    sendFlightVariables();
  }
  else if (command.substring(0, 4).equals("data")) {
    sendSensorData();
  }
  else {
    Serial.print("Not a valid command.\n");
    return;
  }
}

void setFlashLocation() {
  Serial.println("Setting flash location... ");
  flash.read_start(0, metadata, 64);
  flashLocationLocation = 1;

  uint8_t byte_from_flash;
  if (metadata[0] == 0) {  // If it wasn't bulk erased recently
    while (flashLocationLocation < 64 && metadata[flashLocationLocation] == 0) {
      flashLocationLocation++;
    }
    Serial.print("Flash is not empty, ");
    Serial.print(flashLocationLocation - 2);  // Subtract extra 1 because 1 sector is metadata
    Serial.println("/63 flight data sectors full.");

    flashLocation = (flashLocationLocation - 1) * 0x40000;  // Go to start of last non-empty sector
    // Linear search
    bool foundLocation = false;
    int i = 0;
    while (!foundLocation && i < 0x40000) {
//      if (i % 512 == 0) {  // Read large blocks at a time
//        flash.read_start(flashLocation + i, sectorPortion, 512);
//      }
      flash.read_start(flashLocation + i, &byte_from_flash, 1);
      foundLocation = (flashLocation == 0xff);  // Access the first byte of the struct to see if it's uninitilized
      i += 64;
    }
    flashLocation = flashLocation + i - 64;  // It overshoots by one position
  } else {
    Serial.println("Flash is empty, writing metadata.");
    uint8_t temp = 0;
    flash.write(0, &temp, 1); // Indicate that there is data in the metadata sector
    flashLocation = 0x40000;
  }
  Serial.print("Next flash write in sector ");
  Serial.print(flashLocation / 0x40000);
  Serial.print(" at position ");
  Serial.print(flashLocation % 0x40000);
  Serial.println(".");
}

// Get variables for this flight from InternalFS
void readFlightVariables() {
  // no-op
}

// Average several readings to get "sea level" pressure
double calibrateSeaLevel(int samples) {
  Serial.print("Reading current pressure... ");
  digitalWrite(CALIBRATION_LED, HIGH);
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    baro.Readout();
    sum += baro.GetPres();
    delay(100);
  }
  Serial.println("Done.");
  digitalWrite(CALIBRATION_LED, LOW);
  return sum / (double) samples;
  Serial.println("Done");
}

// Convert pressure (in pascals) to altitude (in meters) using sea level pressure
double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / seaLevel, 1.0 / 5.25588));
}

// PWM pin to heat nichrome and cut parachute line
void pwmStart() {
  Serial.print("Starting PWM on pin ");
  if (state == DEPLOYED) {
    Serial.print(PYRO_CUT1);
    analogWrite(PYRO_CUT1, PYRO_POWER);
  } else if (state == PARTIAL_DISREEF) {
    Serial.print(PYRO_CUT2);
    analogWrite(PYRO_CUT2, PYRO_POWER);
  }
  Serial.print(" with level ");
  Serial.println(PYRO_POWER);
}

void progressState() {
  Serial.print(state);
  Serial.print(" -> ");
  Serial.println(state + 1);
  Serial.println(loopStart);
  writeStateChangeData();
  lastStateChange = loopStart;
  state++;
}

void writeStateChangeData() {
  // no-op
}

void updateDataStruct() {
  currentData.state = state;
  currentData.timestamp = loopStart;
  currentData.pressure = pressure;
  currentData.altitude = altitude;
  const auto xhat = kalman.GetXhat();
  currentData.avgAltitude = xhat.estimatedAltitude;
  currentData.deltaAltitude = xhat.estimatedVelocity;
  currentData.temperature = baro.GetTemp();
  currentData.accelX = accelData.accel_xout;
  currentData.accelY = accelData.accel_yout;
  currentData.accelZ = accelData.accel_zout;
  currentData.cutSense1 = analogRead(PYRO_SENSE1);
  currentData.cutSense2 = analogRead(PYRO_SENSE2);
}

void sendSensorData() {
  Serial.printf("State [%i]\n", currentData.state);
  Serial.printf("Time [%i]\n", currentData.timestamp);
  Serial.printf("Press [%u Pa]\n", currentData.pressure);
  Serial.printf("Temp [%f C]\n", currentData.temperature);
  Serial.printf("Ax [%f]\n", currentData.accelX);
  Serial.printf("Ay [%f]\n", currentData.accelY);
  Serial.printf("Az [%f]\n", currentData.accelZ);
  Serial.printf("CutSens1 [%li]\n", currentData.cutSense1);
  Serial.printf("CutSens2 [%li]\n", currentData.cutSense2);
}

void sendFlightVariables() {
  Serial.printf("Cut1Alt [%u m]\n", currentFlightVars.altitude1);
  Serial.printf("Cut2Alt [%u m]\n", currentFlightVars.altitude2);
  Serial.printf("TimePWM [%u ms]\n", PYRO_FIRE_DURATION);
  Serial.printf("LevelPWm [%u/255]\n", PYRO_POWER);
  Serial.printf("SeaLevel [%u Pa]\n", currentFlightVars.seaLevelPressure);
}

void updateFlash(uint8_t* data, int sizeOfData) {
  if (flashLocation < 0x1000000) {
    // If flashLocation moves into a new sector, update meta location
    if (flashLocation % 0x40000 == 0) {
      uint8_t temp = 0;
      flash.write(flashLocationLocation, &temp, 1);
      flashLocationLocation++;
    }

    // Write things, increment location
    flash.write(flashLocation, data, sizeOfData);
    flashLocation += 64;
  }
}

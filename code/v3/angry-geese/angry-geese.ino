#include <Wire.h>
#include "MS5xxx.h"
#include "S25FL.h"
#include "ICM20602.h"
#include "altitude_kalman.h"

// For angry geese defs
#include "AngryGoose_pins.h"

//#define DEBUG 1

enum states {
  WAITING = 0,  // Before arming
  ARMED_PREFLIGHT,  // Will be armed once above altitude specified in flight variables
  BOOST_COAST,  // Apogee
  DROGUE_DESCENT,  // After first line is cut
  POST_MAIN,  // After second line is cut
  LANDED
};
int state = WAITING;
char* stateStrings[6] = { "WAITING", "ARMED", "BOOST_COAST", "DROGUE_DESCENT", "POST_MAIN", "LANDED" };

// Barometer and moving averages
MS5xxx baro(&Wire);
// Altitude calculation
double seaLevel;

S25FL flash(FLASH_CS);  // Starts Flash class and initializes SPI
unsigned long flashLocation;  // Next location to write data to
unsigned long flashLocationLocation;  // Next location to write flash location to
uint8_t metadata[64];

// Variables used in loop()
const int DELAY_PAD = 1000 / 10;  // milliseconds
const int DELAY_FLIGHT = 1000 / 50;  // milliseconds
unsigned long loopStart = 0;
unsigned long lastStateChange = 0;
unsigned long cutStart1 = 0;
unsigned long cutStart2 = 0;
float previousAltitude;
int32_t pressure;  // pascals
double altitude;  // meters
bool armed;

AltitudeKalman kalman {};

IMU imu{}; // accelerometer
imu_out_t accelData = {};

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
const uint8_t DATA_SIZE = sizeof(Data);

// For writing data to flash & flight variable log
struct FlightVariables {
  uint8_t structType = 1; // Flight variable struct.
  
  uint16_t altitude1 = 100;     // Used as arming altitude
  uint16_t altitude2 = 3; // Main deploy. UNUSED FOR BJORN!
  uint32_t seaLevelPressure;
};
FlightVariables currentFlightVars = {};
const int FLIGHT_VARIABLE_SIZE = sizeof(FlightVariables);

/*****************************
          SETUP/LOOP
 ****************************/

void setup() {
  Serial.begin(57600);

  Wire.begin();
  // Connect to accelerometer
  while (!imu.begin()) {
    Serial.println("Error connecting to accelerometer...");
    delay(500);
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
  seaLevel = calibrateSeaLevel(30);
  Serial.print("Sea level pressure [Pa]: ");
  Serial.println(seaLevel);

  // Set up flash
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
  SPI.setDataMode(SPI_MODE0);
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);

  setFlashLocation();
  readFlightVariables();
}

void loop() {
  #ifdef DEBUG
  print_flash_packet(currentData, 0);
  #endif
  
  //const auto delayTimeMs = (state < BOOST_COAST ? DELAY_PAD : DELAY_FLIGHT);
  const auto delayTimeMs = DELAY_FLIGHT;
  while (millis() < loopStart + delayTimeMs) {}
  float dt = millis() - loopStart;
  loopStart = millis();  // Used for timestamps in data log

  // Read pressure, calculate altitude
  baro.Readout();
  pressure = baro.GetPres();
  altitude = pressureToAltitude(pressure);

  // Update accelerometer
  accelData = imu.readout();

  // Update Kalman filter
  kalman.Correct(altitude, DEFAULT_KALMAN_GAIN);
  if(state < DROGUE_DESCENT) {
    kalman.Predict(sqrtf(
      pow(accelData.accel_xout, 2) + 
      pow(accelData.accel_yout, 2) + 
      pow(accelData.accel_zout, 2)
      ) * 9.81 - 9.81, // Gees to m/s
      dt / 1000.0);
  } else {
    kalman.Predict(0.0, dt / 1000.0);
  }
  const auto xhat = kalman.GetXhat();
  
  #ifdef DEBUG
  Serial.print("estimatedAltitude "); Serial.println(xhat.estimatedAltitude);
  Serial.print("estimatedVelocity "); Serial.println(xhat.estimatedVelocity);
  #endif

  // Update data struct, send to BLE central and flash
  updateDataStruct();

  updateFlash(( uint8_t* ) &currentData, DATA_SIZE);

  while (Serial.available()) {
    uint8_t ch;
    ch = (uint8_t) Serial.read();
    Serial.println(ch);
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
      state = ARMED_PREFLIGHT;
      break;
    case ARMED_PREFLIGHT:
      buzzer_play_periodic(4500, 30, 5000);
      // Check if we're high enough
      if (xhat.estimatedAltitude > currentFlightVars.altitude1) {
        progressState();
      }
      break;
    case BOOST_COAST:
      buzzer_play_periodic(4500, 30, 2000);
      // If we started falling, deploy the drogue
      if (xhat.estimatedVelocity <= 0 && xhat.estimatedAltitude < previousAltitude) {
        pwmStart();
        cutStart1 = loopStart;
        progressState();
      }
      break;
    // 
    case DROGUE_DESCENT:
      buzzer_play_periodic(4500, 30, 2000);
//      // If we're below the main altitude, deploy the main
//      if (xhat.estimatedAltitude < currentFlightVars.altitude2) {

      // If we've tried the main charge, try the backup too
      if(loopStart - lastStateChange > PYRO_FIRE_DURATION) {
        pwmStart();
        cutStart2 = loopStart;
        progressState();
      }
      break;
    case POST_MAIN:
      buzzer_play_periodic(4500, 30, 1000);
      // If we aren't really moving, we've landed
      if (abs(xhat.estimatedVelocity) < 0.05) {
        progressState();
      }
      break;
    case LANDED:
      buzzer_play_periodic(3000, 70, 1000);
      // Nothing happens after landing
      break;
    default:
      Serial.println("Something has gone horribly wrong if none of the states match.");
  }

  previousAltitude = xhat.estimatedAltitude;
}


/*****************************
            HELPERS
 ****************************/

void parse_command() {
//  uint8_t char_array[12];
//  int current_num = 0;

  // read text after '!' which started command
//  while ( Serial.available() && current_num < 12 )
//  {
//    char_array[current_num] = (uint8_t) Serial.read();
//    current_num++;
//  }
//
//  String command = (char*) char_array;
  String command = Serial.readStringUntil('\n');
  Serial.println(command);

  if (command.substring(0, 3).equals("arm")) {
    if (state == WAITING) {
      armed = true;
      Serial.print("Armed.\n");
    } else {
      Serial.print("Not in waiting state.\n");
    }
  }
  else if (command.substring(0, 6).equals("disarm")) {
    if (state == ARMED_PREFLIGHT) {
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
  else if (command.substring(0, 5).equals("erase")) {
    bulkErase();
  }
  else if (command.substring(0, 7).equals("offload")) {
    offloadData();
  }
  else {
    Serial.print("Valid commands:\n");
    Serial.print("!arm\n");
    Serial.print("!disarm\n");
    Serial.print("!vars\n");
    Serial.print("!data\n");
    Serial.print("!erase\n");
    Serial.print("!offload\n");
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
    uint32_t i = 0;
    while (!foundLocation && i < 0x40000) {
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
  return;
}

// Average several readings to get "sea level" pressure
double calibrateSeaLevel(int samples) {
  Serial.print("Reading current pressure... ");
  digitalWrite(CALIBRATION_LED, HIGH);
  int32_t sum = 0;
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
  if (state == BOOST_COAST) {
    Serial.print(PYRO_CUT1);
    analogWrite(PYRO_CUT1, PYRO_POWER);
  } else if (state == DROGUE_DESCENT) {
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
  currentData.gyroX = accelData.gyro_xout;
  currentData.gyroY = accelData.gyro_yout;
  currentData.gyroZ = accelData.gyro_zout;
  currentData.cutSense1 = analogRead(PYRO_SENSE1);
  currentData.cutSense2 = analogRead(PYRO_SENSE2);
}

void sendSensorData() {
  Serial.printf("State\t");    Serial.println(stateStrings[currentData.state]);
  Serial.printf("Time\t");     Serial.println(currentData.timestamp);
  Serial.printf("Press\t");    Serial.println(currentData.pressure);
  Serial.printf("CutSens1\t"); Serial.println(currentData.cutSense1);
  Serial.printf("CutSens2\t"); Serial.println(currentData.cutSense2);
  Serial.printf("Est Alt\t");     Serial.println(currentData.avgAltitude);
  Serial.printf("Est Vel\t");     Serial.println(currentData.deltaAltitude);
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

void bulkErase() {
  Serial.println("Erasing....");
  flash.erase_chip_start();
  bool completed = false;
  auto startTime = millis();
  while(true) {
    if (!completed) {
      completed = flash.is_erase_complete();
      Serial.println(".");
      if (completed) { 
        Serial.println("\nDone!");
        auto endTime = millis();
        Serial.print("Erase took ");
        Serial.print(endTime - startTime);
        Serial.println("ms");
      }
    } else {
      Serial.println("Reset me!");
    }
    delay(1000);
  }
}

void offloadData() {
  unsigned long flashReadLocation = 0x0;
  uint8_t metadata[64];
  
  uint8_t sectorPortion[64] = {};
  Data readDataFromFlash;

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
  
  delay(6000);  // Give time to connect with CoolTerm or similar
  Serial.println("Location,state,time,pressure,alt,avgAltitude,velocity,temp,ax,ay,az,wx,wy,wz,cutSense1,cutSense2");

  auto done = false;
  while (!done) {
    if (!(flashReadLocation >= 0x1000000)) {
      flash.read_start(flashReadLocation, sectorPortion, 64);
      readDataFromFlash = *( Data* ) &sectorPortion[0];
      if(readDataFromFlash.state >= 0 && readDataFromFlash.state < 255) 
        print_flash_packet(readDataFromFlash, flashReadLocation);
    } else {
      done = true;
    }
    flashReadLocation += 64;
  }
  delay(1);

  while(true) {
    Serial.println("Reset me!");
    delay(1000);
  }
}

void print_flash_packet(const Data& packet, unsigned long loc) {
  Serial.print(loc);
  Serial.print(", ");
  Serial.print(packet.state);
  Serial.print(", ");
  Serial.print(packet.timestamp);
  Serial.print(", ");
  Serial.print(packet.pressure);
  Serial.print(", ");
  Serial.print(packet.altitude);
  Serial.print(", ");
  Serial.print(packet.avgAltitude);
  Serial.print(", ");
  Serial.print(packet.deltaAltitude);
  Serial.print(", ");
  Serial.print(packet.temperature);
  Serial.print(", ");
  Serial.print(packet.accelX);
  Serial.print(", ");
  Serial.print(packet.accelY);
  Serial.print(", ");
  Serial.print(packet.accelZ);
  Serial.print(", ");
  Serial.print(packet.gyroX);
  Serial.print(", ");
  Serial.print(packet.gyroY);
  Serial.print(", ");
  Serial.print(packet.gyroZ);
  Serial.print(", ");
  Serial.print(packet.cutSense1);
  Serial.print(", ");
  Serial.print(packet.cutSense2);
  Serial.print("\n");
}

/**
 * The last time the tone was played.
 */
static uint32_t last_tone_time = 0;
void buzzer_play_periodic(unsigned int frequency, unsigned long duration, uint16_t period) {
    uint32_t now = millis();
    uint32_t elapsed = now - last_tone_time;

    if (elapsed >= period) {
        last_tone_time = now;

        tone(BUZZER, frequency, duration);
    }
}

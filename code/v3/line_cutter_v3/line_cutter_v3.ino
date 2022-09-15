#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"
#include "S25FL.h"
#include "icm20602/ICM20602.h"

char* boardName; // Get this from the board's ID file

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

// Pins
const int VOLTAGE_DIVIDER = A4;
const int NICHROME_PIN1 =   11;
const int NICHROME_PIN2 =   12;
const int PHOTO_PIN =       A3;
const int CALIBRATION_LED = PIN_LED2;
const int CUT_SENSE1 =      A5;
const int CUT_SENSE2 =      A0;
const int CURRENT_SENSE =   A1;
const int CHIP_SELECT_PIN = 8;
// We can only add ONE pin per HardwarePWM; adding more makes them randomly flash
HardwarePWM& hardwarePWM1 = HwPWM0;
HardwarePWM& hardwarePWM2 = HwPWM1;

// Barometer and moving averages
MS5xxx baro(&Wire);
int ARRAY_SIZE = 40;  // 2 seconds at 20Hz
MovingAvg altitudeReadings(ARRAY_SIZE);  // Store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // Store differences between avgs calculated using ^
// Altitude calculation
double seaLevel;

// For logging data in internal filesystem and flash
using namespace Adafruit_LittleFS_Namespace;
const char* STATE_FILE = "stateChangeLog.txt";
const char* ID_FILE = "ID.txt";
const char* FLIGHT_VARIABLE_FILE = "flightVariables.txt";
File file(InternalFS);
S25FL flash(CHIP_SELECT_PIN);  // Starts Flash class and initializes SPI
unsigned long flashLocation;  // Next location to write data to
unsigned long flashLocationLocation;  // Next location to write flash location to
uint8_t metadata[64];
uint8_t sectorPortion[4096];

// Variables used in loop()
const int DELAY = 50;  // milliseconds
unsigned long loopStart = 0;
unsigned long lastStateChange = 0;
unsigned long cutStart1 = 0;
unsigned long cutStart2 = 0;
bool armed = false;
int32_t pressure;  // pascals
double altitude;  // meters
double previousAltitudeAvg;
double currentAltitudeAvg;
double delta;  // meters/second
double previousDeltaAvg;
double currentDeltaAvg;
int light;

// bluetooth magic
// OTA DFU service
BLEDfu bledfu;
// Uart over BLE service
BLEUart bleuart;
// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);
// Packet buffer
extern uint8_t packetbuffer[];

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
Data currentData;
const int DATA_SIZE = sizeof(Data);

// For writing data to flash & flight variable log
struct FlightVariables {
  uint8_t structType = 1; // Flight variable struct. Only altitude1, pwm vars, and seaLevel are used
  float limitVel;
  uint16_t altitude1;     // Used as arming altitude
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
const int FLIGHT_VARIABLE_SIZE = sizeof(FlightVariables);

/*****************************
          SETUP/LOOP
 ****************************/

void setup() {
  // Manually add nichrome pins to HardwarePWMs and set them
  hardwarePWM1.setResolution(8);  // Write values in range [0, 255]
  hardwarePWM1.addPin(NICHROME_PIN1);
  hardwarePWM1.writePin(NICHROME_PIN1, 0);
  hardwarePWM2.setResolution(8);  // Write values in range [0, 255]
  hardwarePWM2.addPin(NICHROME_PIN2);
  hardwarePWM2.writePin(NICHROME_PIN2, 0);
  analogReadResolution(10);  // Read values in range [0, 1023]

  Serial.begin(115200);
//  while (!Serial) {
//    delay(10);
//  }

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

  // Initialize Internal File System
  if (InternalFS.begin()) {
    Serial.println("FS initialized.");
  } else {
    Serial.println("Could not start file system.");
  }

  // Get identifier info (board version and Bluetooth name)
  file.open(ID_FILE, FILE_O_READ);
  uint32_t readlen;
  char buffer[64] = { 0 };
  readlen = file.read(buffer, sizeof(buffer));
  buffer[readlen] = 0;
  file.close();
  boardName = buffer;
  Serial.print("Board: ");
  Serial.println(boardName);

  // Set up flash
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  // Set up BLE
  Bluefruit.begin(2, 0);
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName(boardName);
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();
  // Configure and start the BLE Uart service
  bleuart.begin();
  // Set up and start advertising
  startAdv();
  Serial.println("Started advertising.");

  setFlashLocation();
  readFlightVariables();
  updateFlash(( uint8_t* ) &currentFlightVars, FLIGHT_VARIABLE_SIZE);
}


void loop() {
  //Serial.print("In loop");
  //Serial.println(analogRead(VOLTAGE_DIVIDER) * 4.0 * 3.6 / 1023.0);
  while (millis() < loopStart + DELAY) {}

  loopStart = millis();  // Used for timestamps in data log

  // Read pressure, calculate altitude
  baro.Readout();
  pressure = baro.GetPres();
  // filter outlier readings
  if (abs(altitude - pressureToAltitude(pressure)) < 50 || state < ARMED) {  // Only accept new altitude during flight if it's within 50m
    altitude = pressureToAltitude(pressure);
  }

  // Update moving averages
  previousAltitudeAvg = altitudeReadings.getAvg();
  previousDeltaAvg = altitudeAvgDeltas.getAvg();
  currentAltitudeAvg = altitudeReadings.reading(altitude);  // Update and return new avg
  delta = (currentAltitudeAvg - previousAltitudeAvg) * (1000.0 / DELAY);
  currentDeltaAvg = altitudeAvgDeltas.reading(delta);  // Update and return new avg

  // Update photoresistor
  light = analogRead(PHOTO_PIN);

  // Update accelerometer
  accelData = imu.readout();

  // Update data struct, send to BLE central and flash
  updateDataStruct();
  updateFlash(( uint8_t* ) &currentData, DATA_SIZE);

  while (bleuart.available()) {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    // '!' indicates start of a command. anything else is ignored
    if (ch == (uint8_t) '!') {
      parse_command();
    }
  }

  if (cutStart1 > 0 && loopStart - cutStart1 > currentFlightVars.pwmDuration) {
    hardwarePWM1.writePin(NICHROME_PIN1, 0);
    Serial.print("Ended PWM on pin ");
    Serial.println(NICHROME_PIN1);
    cutStart1 = 0;
  }
  if (cutStart2 > 0 && loopStart - cutStart2 > currentFlightVars.pwmDuration) {
    hardwarePWM2.writePin(NICHROME_PIN2, 0);
    Serial.print("Ended PWM on pin ");
    Serial.println(NICHROME_PIN2);
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
      if (currentAltitudeAvg > currentFlightVars.altitude1) {
        InternalFS.remove(STATE_FILE);
        progressState();
      }
      break;
    case DEPLOYED:
      if (previousDeltaAvg >= 0 && currentDeltaAvg <= 0) {
        pwmStart();
        cutStart1 = loopStart;
        progressState();
      }
      break;
    case PARTIAL_DISREEF:
      if (loopStart - lastStateChange > currentFlightVars.pwmDuration) {
        pwmStart();
        cutStart2 = loopStart;
        progressState();
      }
      break;
    case FULL_DISREEF:
      if (abs(currentDeltaAvg) < 0.05) {
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
  while ( bleuart.available() && current_num < 8 )
  {
    char_array[current_num] = (uint8_t) bleuart.read();
    current_num++;
  }

  String command = (char*) char_array;

  if (command.substring(0, 4).equals("help")) {
    bleuart.print("Valid commands:\n");
    bleuart.print("!arm\n");
    bleuart.print("!disarm\n");
    bleuart.print("!vars\n");
    bleuart.print("!data\n");
  }
  else if (command.substring(0, 3).equals("arm")) {
    if (state == WAITING) {
      armed = true;
      bleuart.print("Armed.\n");
    } else {
      bleuart.print("Not in waiting state.\n");
    }
  }
  else if (command.substring(0, 6).equals("disarm")) {
    if (state == ARMED) {
      armed = false;
      bleuart.print("Disarmed.\n");
    } else {
      bleuart.print("Not in armed state.\n");
    }
  }
  else if (command.substring(0, 4).equals("vars")) {
    sendFlightVariables();
  }
  else if (command.substring(0, 4).equals("data")) {
    sendSensorData();
  }
  else if (command.substring(0, 4).equals("test")) {
    int level = pwmLevel(currentFlightVars.pwmVoltage2);
    hardwarePWM2.writePin(NICHROME_PIN2, level);
  }
  else {
    bleuart.print("Not a valid command.\n");
    return;
  }
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void setFlashLocation() {
  Serial.println("Setting flash location... ");
  flash.read_start(0, metadata, 64);
  flashLocationLocation = 1;
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
      if (i % 4096 == 0) {  // Read large blocks at a time
        flash.read_start(flashLocation + i, sectorPortion, 4096);
      }
      foundLocation = (sectorPortion[i % 4096] == 0xff);  // Access the first byte of the struct
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
  if (file.open(FLIGHT_VARIABLE_FILE, FILE_O_READ)) {
    uint8_t buff[64] = {0};
    file.read(buff, FLIGHT_VARIABLE_SIZE);
    currentFlightVars = *( FlightVariables* ) &buff;
    currentFlightVars.seaLevelPressure = seaLevel;
    Serial.println("Finished reading flight variables.");
  } else {
    Serial.println("Failed to read flight variables.");
  }
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
  int level;
  Serial.print("Starting PWM on pin ");
  if (state == DEPLOYED) {
    Serial.print(NICHROME_PIN1);
    level = pwmLevel(currentFlightVars.pwmVoltage1);
    hardwarePWM1.writePin(NICHROME_PIN1, level);
  } else if (state == PARTIAL_DISREEF) {
    Serial.print(NICHROME_PIN2);
    level = pwmLevel(currentFlightVars.pwmVoltage2);
    hardwarePWM2.writePin(NICHROME_PIN2, level);
  }
  Serial.print(" with level ");
  Serial.println(level);
}

// Calculate number to be used for PWM of nichrome pin
int pwmLevel(double targetVoltage) {
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // Analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  // Find proportion of vbat needed to apply target voltage, then make out of 255 for analog write
  return (targetVoltage / vbat) * 255;
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
  if ( file.open(STATE_FILE, FILE_O_WRITE) ) {
    uint8_t* buffer = ( uint8_t* ) &currentData;

    Serial.print("Writing data to file... ");
    file.write(state);
    file.write(" -> ");
    file.write(state + 1);
    file.write(buffer, DATA_SIZE);
    file.close();
    Serial.println("Done");
  } else {
    Serial.print("Failed to write ");
    Serial.println(STATE_FILE);
  }
}

void updateDataStruct() {
  currentData.state = state;
  currentData.timestamp = loopStart;
  currentData.pressure = pressure;
  currentData.altitude = altitude;
  currentData.avgAltitude = currentAltitudeAvg;
  currentData.deltaAltitude = delta;
  currentData.avgDeltaAltitude = currentDeltaAvg;
  currentData.temperature = baro.GetTemp();
  currentData.accelX = accelData.accel_xout;
  currentData.accelY = accelData.accel_yout;
  currentData.accelZ = accelData.accel_zout;
  currentData.battSense = analogRead(VOLTAGE_DIVIDER);
  currentData.cutSense1 = analogRead(CUT_SENSE1);
  currentData.cutSense2 = analogRead(CUT_SENSE2);
  currentData.currentSense = analogRead(CURRENT_SENSE);
  currentData.photoresistor = light;
}

void sendSensorData() {
  bleuart.printf("State [%i]\n", currentData.state);
  bleuart.printf("Time [%i]\n", currentData.timestamp);
  bleuart.printf("Press [%u Pa]\n", currentData.pressure);
  bleuart.printf("Temp [%f C]\n", currentData.temperature);
  bleuart.printf("Ax [%f]\n", currentData.accelX);
  bleuart.printf("Ay [%f]\n", currentData.accelY);
  bleuart.printf("Az [%f]\n", currentData.accelZ);
  bleuart.printf("Batt [%f]\n", currentData.battSense * 2.0 * 3.6 / 1023.0);
  bleuart.printf("CutSens1 [%li]\n", currentData.cutSense1);
  bleuart.printf("CutSens2 [%li]\n", currentData.cutSense2);
  bleuart.printf("CurrSens [%li]\n", currentData.currentSense);
  bleuart.printf("Photores [%li]\n", currentData.photoresistor);
}

void sendFlightVariables() {
  bleuart.printf("Cut1Alt [%u m]\n", currentFlightVars.altitude1);
  bleuart.printf("Cut2Alt [%u m]\n", currentFlightVars.altitude2);
  bleuart.printf("Cut1Time [%u ms]\n", currentFlightVars.disreefDelay1);
  bleuart.printf("Cut2Time [%u ms]\n", currentFlightVars.disreefDelay2);
  bleuart.printf("VoltPWM1 [%f V]\n", currentFlightVars.pwmVoltage1);
  bleuart.printf("VoltPWM2 [%f V]\n", currentFlightVars.pwmVoltage2);
  bleuart.printf("TimePWM [%u ms]\n", currentFlightVars.pwmDuration);
  bleuart.printf("LightThreshold [%u]\n", currentFlightVars.lightThreshold);
  bleuart.printf("LightTime [%u ms]\n", currentFlightVars.lightTriggerTime);
  bleuart.printf("SeaLevel [%u Pa]\n", currentFlightVars.seaLevelPressure);
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

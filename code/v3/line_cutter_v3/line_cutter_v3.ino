#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"
#include "S25FL.h"
#include "icm20602/ICM20602.h"
#include "DataStructures.h"
#include "Controller.h"

// Defines pin numbers for the connected hardware
#define VOLTAGE_DIVIDER  A4
#define NICHROME_PIN1    11
#define NICHROME_PIN2    12
#define PHOTO_PIN        A3
#define CALIBRATION_LED  PIN_LED2
#define CUT_SENSE1       A5
#define CUT_SENSE2       A0
#define CURRENT_SENSE    A1
#define CHIP_SELECT_PIN  8
#define CAPACITOR_PIN    -1  // Needs to be changed

// Other constants used in the program
#define ARRAY_SIZE       40  // 2 seconds at 20Hz
#define DELAY            50  // milliseconds
#define ID_FILE          "ID.txt"
#define FLIGHT_VAR_FILE  "flightVariables.txt"

// For logging data in internal filesystem and flash
using namespace Adafruit_LittleFS_Namespace;

// We can only add ONE pin per HardwarePWM; adding more makes them randomly flash
HardwarePWM& hardwarePWM1 = HwPWM0;
HardwarePWM& hardwarePWM2 = HwPWM1;

// Barometer and moving averages
MS5xxx baro(&Wire);
MovingAvg altitudeReadings(ARRAY_SIZE);  // Store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // Store differences between avgs calculated using ^

// Accelerometer
IMU imu; // accelerometer
imu_out_t accelData;

// Variables for interacting with the file system
File file(InternalFS);
S25FL flash(CHIP_SELECT_PIN);  // Initializes SPI
unsigned long flashLocation;  // Next location to write data to
unsigned long flashLocationLocation;  // Next location to write flash location to
uint8_t metadata[64];
uint8_t sectorPortion[4096];

// Variables used in loop()
// Persistant variables
bool armed = false;
bool charging = false;  // If the capacitor is currently charging
double seaLevel;  // Altitude calculation

// Variables computed every time
unsigned long loopStart;
int32_t pressure;  // pascals
double altitude;  // meters
double previousAltitudeAvg;
double currentAltitudeAvg;
double delta;  // meters/second
double currentDeltaAvg;

// Bluetooth setup
BLEDfu bledfu;  // OTA DFU service
BLEUart bleuart;  // Uart over BLE service

// The configuration and board state structs
LineCutterState currentState;
LineCutterConfig flightConfig;

const size_t STATE_SIZE = sizeof(currentState);
const size_t CONFIG_SIZE = sizeof(flightConfig);

// Gets the line cutter controller for the flight
Controller lineCutterController;

/*****************************
          SETUP/LOOP
 ****************************/

void setup() {
  // Begin serial communication
  Serial.begin(115200);

  // Connect to the accelerometer
  Wire.begin();
  while (!imu.begin()) {
    Serial.println("Error connecting to accelerometer...");
    delay(250);
  }
  Serial.println("Successfully connencted to accelerometer");
  
  // Connect to barometer
  while (baro.connect() > 0) {
    Serial.println("Error connecting to barometer...");
    delay(250);
  }
  Serial.println("Successfully connected to barometer");
  
  // Calibrate barometer and get initial reading
  baro.ReadProm();
  baro.Readout();
  // Calibrate sea level
  seaLevel = calibrateSeaLevel(50);
  Serial.print("Sea level pressure [Pa]: ");
  Serial.println(seaLevel);

  // Begin moving averages for barameter and accelerometer readings
  altitudeReadings.begin();
  altitudeReadings.reading(pressureToAltitude(baro.GetPres()));
  altitudeAvgDeltas.begin();
  altitudeAvgDeltas.reading(0.0);

  // Initialized the hardware PWMs for cutting the nichrome
  hardwarePWM1.setResolution(8);  // Write values in range [0, 255]
  hardwarePWM1.addPin(NICHROME_PIN1);
  hardwarePWM1.writePin(NICHROME_PIN1, 0);
  hardwarePWM2.setResolution(8);  // Write values in range [0, 255]
  hardwarePWM2.addPin(NICHROME_PIN2);
  hardwarePWM2.writePin(NICHROME_PIN2, 0);
  analogReadResolution(10);  // Read values in range [0, 1023]

  // Initialize Internal File System
  if (InternalFS.begin()) {
    Serial.println("FS initialized.");
  } else {
    Serial.println("Could not start file system.");
  }

  // Get the board's bluetooth name
  file.open(ID_FILE, FILE_O_READ);
  uint32_t readlen;
  char buffer[64] = {0};
  readlen = file.read(buffer, sizeof(buffer));
  buffer[readlen] = 0;
  file.close();
  
  char* boardName = buffer;
  Serial.print("Board: ");
  Serial.println(boardName);

  // Set up flash
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  // Set up bluetooth
  Bluefruit.begin(2, 0);
  Bluefruit.setTxPower(8);  // Supported values included in bluefruit.h
  Bluefruit.setName(boardName);
  
  // To be consistent, OTA DFU should be added first if it exists
  bledfu.begin();
  // Configure and start the BLE Uart service
  bleuart.begin();
  // Set up and start advertising
  startAdv();
  Serial.println("Started advertising.");

  // Sets the flash location, reads the configuration, and writes it to the flash
  setFlashLocation();
  readFlightVariables();
  updateFlash((uint8_t*) &flightConfig, CONFIG_SIZE);
}


void loop() {
  delay(DELAY);  // Pause to limit the speed the loop can run

  // Save the time the loop started running
  loopStart = millis();

  // Read pressure, calculate altitude
  baro.Readout();
  pressure = baro.GetPres();
  altitude = pressureToAltitude(pressure);

  // Update moving averages
  previousAltitudeAvg = altitudeReadings.getAvg();
  currentAltitudeAvg = altitudeReadings.reading(altitude);  // Update and return new avg
  delta = (currentAltitudeAvg - previousAltitudeAvg) * (1000.0 / DELAY);
  currentDeltaAvg = altitudeAvgDeltas.reading(delta);  // Update and return new avg

  // Update accelerometer
  accelData = imu.readout();

  // Checks and attempt to run any waiting bluetooth commands
  checkBluetoothCommand();

  // Update data struct and write it to the flash
  updateDataStruct();
  updateFlash((uint8_t*) &currentState, STATE_SIZE);

  // Only send the data to the state machine if the board is currently armed
  if (armed) {
    // Makes sure the capacitor is charging
    if (!charging) {
      digitalWrite(CAPACITOR_PIN, HIGH);
      charging = true;
    }
    
    // The board is armed. Pass the data to the state machine implemented by the user and check what actions
    // the board should perform
    switch (lineCutterController.getInstruction(flightConfig, currentState, loopStart)) {
      case NO_CHANGE:
        break;
      case BEGIN_CUT_1:
        startPWM(1);
        break;
      case BEGIN_CUT_2:
        startPWM(2);
        break;
      case END_CUT_1:
        hardwarePWM1.writePin(NICHROME_PIN1, 0);
        Serial.print("Ended PWM on pin ");
        Serial.println(NICHROME_PIN1);
        break;
      case END_CUT_2:
        hardwarePWM2.writePin(NICHROME_PIN2, 0);
        Serial.print("Ended PWM on pin ");
        Serial.println(NICHROME_PIN2);
        break;
      case DISARM:
        armed = false;
        Serial.println("Board disarmed by controller");
        break;
      default:
        Serial.println("Something has gone horribly wrong if none of the states match.");
    }
  
  // Make sure the capacitor is not charging since the board is disarmed
  } else if (charging) {
    digitalWrite(CAPACITOR_PIN, LOW);
    charging = false;
  }
}


/*****************************
            HELPERS
 ****************************/

void checkBluetoothCommand() {
  // Loops until all waiting characters have been read and processed
  while (bleuart.available()) {
    // Checks if the first character in the buffer is an '!' to signify the start of a command.
    // Anything else is ignored
    if ((uint8_t) bleuart.read() == (uint8_t) '!') {
      // If the first character in the buffer was a '!', attempts to read the command received by the board
      uint8_t char_array[6];  // Read up to 6 characters
      int current_num = 0;
    
      // Read text after the '!' that started the command. If an '!' symbol is next in the buffer, stop reading
      // because it signifies the start of a new command
      while (bleuart.available() && current_num < 6 && (uint8_t) bleuart.peek() != (uint8_t) '!') {
        char_array[current_num] = (uint8_t) bleuart.read();
        current_num++;
      }

      String command = (char*) char_array;

      // Confirm the length of the characters in the buffer matches the desired command. This ensures an invalid command, like
      // "!armies," does not inadvertently trigger an actual command, like "!arm"
      if (current_num == 4) {
        if (command.substring(0, 4).equalsIgnoreCase("help")) {
          bleuart.print("Valid commands:\n");
          bleuart.print("!arm\n");
          bleuart.print("!disarm\n");
          bleuart.print("!vars\n");
          bleuart.print("!data\n");
          continue;
        } else if (command.substring(0, 4).equalsIgnoreCase("vars")) {
          sendFlightVariables();
          continue;
        } else if (command.substring(0, 4).equalsIgnoreCase("data")) {
          sendSensorData();
          continue;
        }
      } else if (current_num == 3) {
        if (command.substring(0, 3).equalsIgnoreCase("arm")) {
          if (!armed) {
            armed = true;
            bleuart.print("Armed.\n");
          } else {
            bleuart.print("Board already armed.\n");
          }
          continue;
        }
      } else {
        if (command.substring(0, 6).equalsIgnoreCase("disarm")) {
          if (armed) {
            armed = false;
            bleuart.print("Disarmed.\n");
          } else {
            bleuart.print("Not in armed state.\n");
          }
          continue;
        }
      }

      // If the command was valid, the program will have continued to the next iteration by now because of the
      // the "continue" keyword. This part of the code will only run if a command was recieved and it is invalid
      bleuart.print("Not a valid command.\n");
    }
  }
}

void startAdv() {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  // Starts advertising
  // For recommended advertising intervals, visit https://developer.apple.com/library/content/qa/qa1931/_index.html
  Bluefruit.Advertising.restartOnDisconnect(true); // Enable auto advertising if disconnected
  Bluefruit.Advertising.setInterval(32, 244);  // In units of 0.625 ms; fast mode = 20 ms, slow mode = 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);  // Timeout for fast mode is 30 seconds
  Bluefruit.Advertising.start(0);  // Advertise forever, until the board is connected
}

// Finds the first unwritten location in the flash memory
void setFlashLocation() {
  Serial.println("Setting flash location... ");

  // Reads metadata already in the flash memory
  flash.read_start(0, metadata, 64);
  flashLocationLocation = 1;

  // If metadata[0] == 0, then the flash is not empty
  if (metadata[0] == 0) {  
    while (flashLocationLocation < 64 && metadata[flashLocationLocation] == 0) {
      flashLocationLocation++;
    }

    // Prints how many sectors are still completely unwritten
    Serial.print("Flash is not empty, ");
    Serial.print(flashLocationLocation - 2);  // Subtract extra 1 because 1 sector is metadata
    Serial.println("/63 flight data sectors full.");

    // Go to start of the last non-empty sector
    flashLocation = (flashLocationLocation - 1) * 0x40000;  
    
    // Performs a linear search to find the first unwritten position
    bool foundLocation = false;
    int i = 0;
    while (!foundLocation && i < 0x40000) {
      // Read large blocks at a time
      if (i % 4096 == 0) {
        flash.read_start(flashLocation + i, sectorPortion, 4096);
      }
      
      // Access the first byte of the struct
      foundLocation = (sectorPortion[i % 4096] == 0xff);
      i += 64;
    }

    // The first empty position has been found
    flashLocation = flashLocation + i - 64;  // Subtract 64 because it overshoots by one position

  // The flash is entirely empty
  } else {
    Serial.println("Flash is empty, writing metadata.");
    uint8_t temp = 0;
    flash.write(0, &temp, 1);  // Write to the metadata that data will be stored in the first sector
    flashLocation = 0x40000;
  }
  
  Serial.print("Next flash write in sector ");
  Serial.print(flashLocation / 0x40000);
  Serial.print(" at position ");
  Serial.println(flashLocation % 0x40000);
}

// Get the flight variables from the internal file system
void readFlightVariables() {
  // Opens the file
  if (file.open((char*) FLIGHT_VAR_FILE, FILE_O_READ)) {
    // Reads the data and converts it into a LineCutterConfig struct
    uint8_t buff[64] = {0};
    file.read(buff, CONFIG_SIZE);
    flightConfig = *(LineCutterConfig*) &buff;

    // Sets the sea level pressure that was calculated earlier in the program
    flightConfig.seaLevelPressure = seaLevel;
    
    Serial.println("Finished reading flight variables.");
  
  // Reading the flight variables was unsucessful
  } else {
    Serial.println("Failed to read flight variables.");
  }
}

// Computes the average "sea level" pressure
double calibrateSeaLevel(int sampleCount) {
  Serial.print("Reading current pressure... ");
  digitalWrite(CALIBRATION_LED, HIGH);
  
  // Creates a simple running sum of the last readings
  int sum = 0;
  for (int i = 0; i < sampleCount; i++) {
    baro.Readout();
    sum += baro.GetPres();
    delay(100);
  }

  // Returns the average of the readings
  Serial.println("Done.");
  digitalWrite(CALIBRATION_LED, LOW);
  return sum / (double) sampleCount;
}

// Convert pressure (in pascals) to altitude (in meters) using the sea level pressure
double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / seaLevel, 1.0 / 5.25588));
}

// Starts cutting one of the lines
void startPWM(uint8_t PWMNumber) {
  int level;  // The PMW level

  // Activates the hardware PWM
  if (PWMNumber == 1) {
    level = pwmLevel(flightConfig.pwmVoltage1);
    hardwarePWM1.writePin(NICHROME_PIN1, level);
  } else if (PWMNumber == 2) {
    level = pwmLevel(flightConfig.pwmVoltage2);
    hardwarePWM2.writePin(NICHROME_PIN2, level);
  } else {
    // If PWMNumber was invalid, print a warning and do nothing else.
    Serial.println("Something has gone terribly wrong and the line cutter attempted to cut a non-existant wire");
    return;
  }

  // Update the serial console
  Serial.print("Started line cutter on nichrome wire #");
  Serial.print(PWMNumber);
  Serial.print(" with level ");
  Serial.println(level);
}

// Calculate PWM level for cutting the nichrome
int pwmLevel(double targetVoltage) {
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // Analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  
  // Find proportion of vbat needed to apply target voltage, then make out of 255 for analog write
  return (targetVoltage / vbat) * 255;
}

// Updates the LineCutterState struct with the latest data
void updateDataStruct() {
  // Gets the current state of the line cutter from the controller only if the board is armed. Otherwise, note
  // the state as 255
  currentState.state = armed ? lineCutterController.getStateNum() : 255;
  currentState.isArmed = armed;
  currentState.timestamp = loopStart;
  currentState.pressure = pressure;
  currentState.altitude = altitude;
  currentState.avgAltitude = currentAltitudeAvg;
  currentState.deltaAltitude = delta;
  currentState.avgDeltaAltitude = currentDeltaAvg;
  currentState.temperature = baro.GetTemp();
  currentState.accelX = accelData.accel_xout;
  currentState.accelY = accelData.accel_yout;
  currentState.accelZ = accelData.accel_zout;
  currentState.battSense = analogRead(VOLTAGE_DIVIDER);
  currentState.cutSense1 = analogRead(CUT_SENSE1);
  currentState.cutSense2 = analogRead(CUT_SENSE2);
  currentState.currentSense = analogRead(CURRENT_SENSE);
  currentState.photoresistor = analogRead(PHOTO_PIN);
}

// Prints the current state of the line cutter to bluetooth
void sendSensorData() {
  bleuart.printf("State [%i]\n", currentState.state);
  bleuart.printf("Armed [%d]\n", armed); 
  bleuart.printf("Time [%i]\n", currentState.timestamp);
  bleuart.printf("Press [%u Pa]\n", currentState.pressure);
  bleuart.printf("Temp [%f C]\n", currentState.temperature);
  bleuart.printf("Ax [%f]\n", currentState.accelX);
  bleuart.printf("Ay [%f]\n", currentState.accelY);
  bleuart.printf("Az [%f]\n", currentState.accelZ);
  bleuart.printf("Batt [%f]\n", currentState.battSense * 2.0 * 3.6 / 1023.0);
  bleuart.printf("CutSens1 [%li]\n", currentState.cutSense1);
  bleuart.printf("CutSens2 [%li]\n", currentState.cutSense2);
  bleuart.printf("CurrSens [%li]\n", currentState.currentSense);
  bleuart.printf("Photores [%li]\n", currentState.photoresistor);
}

// Prints the flight variables to bluetooth
void sendFlightVariables() {
  bleuart.printf("Cut1Alt [%u m]\n", flightConfig.altitude1);
  bleuart.printf("Cut2Alt [%u m]\n", flightConfig.altitude2);
  bleuart.printf("Cut1Time [%u ms]\n", flightConfig.disreefDelay1);
  bleuart.printf("Cut2Time [%u ms]\n", flightConfig.disreefDelay2);
  bleuart.printf("VoltPWM1 [%f V]\n", flightConfig.pwmVoltage1);
  bleuart.printf("VoltPWM2 [%f V]\n", flightConfig.pwmVoltage2);
  bleuart.printf("TimePWM [%u ms]\n", flightConfig.pwmDuration);
  bleuart.printf("LightThreshold [%u]\n", flightConfig.lightThreshold);
  bleuart.printf("LightTime [%u ms]\n", flightConfig.lightTriggerTime);
  bleuart.printf("SeaLevel [%u Pa]\n", flightConfig.seaLevelPressure);
}

// Writes data to the flash storage
void updateFlash(uint8_t* data, int sizeOfData) {
  // Check if these is still room to write more data in the flash
  if (flashLocation < 0x1000000) {
    // If flashLocation moves into a new sector, update meta location
    if (flashLocation % 0x40000 == 0) {
      uint8_t temp = 0;
      flash.write(flashLocationLocation, &temp, 1);
      flashLocationLocation++;
    }

    // Write data and increment the location
    flash.write(flashLocation, data, sizeOfData);
    flashLocation += 64;
  }
}

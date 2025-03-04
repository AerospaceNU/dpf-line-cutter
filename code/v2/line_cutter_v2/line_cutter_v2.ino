#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"
#include "S25FL.h"
#include "icm20602/ICM20602.h"

int boardVersion; // Get these values from the board's ID file
bool logData;
char* boardName;

enum states {
  WAITING,  // Anytime before parachute deployment
  DEPLOYED,
  PARTIAL_DISREEF,  // After first line is cut
  FULL_DISREEF,  // After second line is cut
  LANDED
};
int state = WAITING;
char* stateStrings[5] = { "WAITING","DEPLOYED","PARTIAL_DISREEF","FULL_DISREEF","LANDED" };

// Pins. CHANGE FOR PERFBOARD VS PCB!
// PCB; A4 batt, 11 cut1, 12 cut2, A3 photo, PIN_LED2 for calibration
const int VOLTAGE_DIVIDER = A4;
const int NICHROME_PIN1 = 11;
const int NICHROME_PIN2 = 12;
const int PHOTO_PIN = A3;
const int CALIBRATION_LED = PIN_LED2;
// Perfboard; A0 batt, A1 cut1, A2 cut2, A4 photo, LED_BUILTIN for calibration
//const int VOLTAGE_DIVIDER = A0;
//const int NICHROME_PIN1 = A1;
//const int NICHROME_PIN2 = A2;
//const int PHOTO_PIN = A4;
//const int CALIBRATION_LED = LED_BUILTIN;

const int CUT_SENSE1 = A5;
const int CUT_SENSE2 = A0;
const int CURRENT_SENSE = A1;

// VARIABLES WHICH MUST BE SET
// Requirements for state transitions
const double LIMIT_VELOCITY = -3.0;  // meters/second
const double ALTITUDE1 = 243;  // Disreefing altitudes, in meters (higher one first!!)
const double ALTITUDE2 = 210;
const int DISREEF1TIME = 12000; // Maximum delay after ejection is detected before the first line is cut.
const int DISREEF2TIME = 5000; // Maximum delay after the first line is cut before the second line is cut.
// PWM settings
const double PWM_VOLTAGE1 = 2.0;  // Voltage applied to nichrome for line cuts
const double PWM_VOLTAGE2 = 2.0;
const int PWM_DURATION = 2500;  // Length of PWM, in milliseconds
// Photoresistor memes
const int LIGHT_THRESHOLD = 200; // Anything above this value is considered to be outside of the tube
const int DARK_TRIGGER_TIME = 20000; // Continuous time interval for which it much be dark for board to decide it's in the tube
const int LIGHT_TRIGGER_TIME = 2000; // Continuous time interval for which it must be light for board to decide it's been ejected

// Barometer and moving averages
MS5xxx baro(&Wire);
int ARRAY_SIZE = 40; // 2 seconds
MovingAvg altitudeReadings(ARRAY_SIZE);  // Store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // Store differences between avgs calculated using ^
// Altitude calculation
double seaLevel;

// For logging data in internal filesystem and flash
using namespace Adafruit_LittleFS_Namespace;
const char* FILENAME = "stateChangeLog.txt";
const char* ID_FILE = "ID.txt";
File file(InternalFS);
S25FL flash;  // Starts Flash class and initializes SPI
unsigned long flashLocation = 0; // Starting memory location to read and write to 

// Variables used in loop()
const int DELAY = 50;  // milliseconds
unsigned long loopStart = 0;
unsigned long loopEnd = 0;
unsigned long lastBLE = 0;
unsigned long lastStateChangeTime = 0;
unsigned long lastLightTime = 0; // Last time that it was light
unsigned long lastDarkTime = 0; // Last time it was dark
uint8_t bleIdx = 0; // Index of data to send
int32_t pressure;  // pascals
double altitude;  // meters
double previousAltitudeAvg;
double currentAltitudeAvg;
double delta;  // meters/second
double currentDeltaAvg;
int light;
// Keeps track of if it's been dark for long enough for us to "be in the tube"
bool inTube = false;

// Manually interact with HardwarePWM
// We can only add ONE pin per HardwarePWM; adding more makes them randomly flash
HardwarePWM& hpwm1 = HwPWM0;
HardwarePWM& hpwm2 = HwPWM1;

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

// For writing data to flash
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
Data currentData;

IMU imu{}; // accelerometer 

/*****************************
 *        SETUP/LOOP         *
 ****************************/
 
void setup() {
  // Manually add nichrome pins to HardwarePWMs and set them
  hpwm1.setResolution(8);  // Write values in range [0, 255]
  hpwm1.addPin(NICHROME_PIN1);
  hpwm1.writePin(NICHROME_PIN1, 0);
  hpwm2.setResolution(8);  // Write values in range [0, 255]
  hpwm2.addPin(NICHROME_PIN2);
  hpwm2.writePin(NICHROME_PIN2, 0);
  analogReadResolution(10);  // Read values in range [0, 1023]
  
  Serial.begin(115200);
  //while (!Serial) { delay(10); }

  // Connect to barometer
  while ( baro.connect() > 0 ) { // This calls Wire.begin() internally :scrunge:
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
  if(InternalFS.begin()) {
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
  boardVersion = buffer[0] - '0';
  logData = !(buffer[1] - '0');
  boardName = buffer + 2;
  Serial.print("Board: ");
  Serial.println(boardName);
  Serial.print("Recently landed? ");
  Serial.println(buffer[1] - '0');

  // Set up flash if we're logging data
  if (logData) {
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8); //Divide the clock by 8
    SPI.setDataMode(SPI_MODE0);
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);
  }

  if(boardVersion == 0) {
    while (!imu.begin()) {
      Serial.println("Could not connect to accelerometer!");
    }
  }
  
  // Set up bluetooth
  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName(boardName);
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();
  // Configure and start the BLE Uart service
  bleuart.begin();
  // Set up and start advertising
  startAdv();
  Serial.println("Started advertising.");

  lastLightTime = millis();
  lastStateChangeTime = millis();
}


void loop() {
  while(millis() < loopEnd + DELAY) {}
  
  loopStart = millis();  // Used for timestamps in data log
 
  // Read pressure, calculate altitude
  baro.Readout();
  pressure = baro.GetPres();
  altitude = pressureToAltitude(pressure);

  // Update moving averages
  previousAltitudeAvg = altitudeReadings.getAvg();
  currentAltitudeAvg = altitudeReadings.reading(altitude);  // Update and return new avg
  delta = (currentAltitudeAvg - previousAltitudeAvg) * (1000.0 / DELAY);
  currentDeltaAvg = altitudeAvgDeltas.reading(delta);  // Update and return new avg
  
  updateData(loopStart);
  if (logData && boardVersion == 0 && flashLocation < 0x8000000) {
    updateFlash();
  }

  // Photoresistor stuff
  light = analogRead(PHOTO_PIN);
  //Keep track of whether it is dark or light and for how long
  if(light > LIGHT_THRESHOLD)
    lastLightTime = loopStart;
  if(light < LIGHT_THRESHOLD)
    lastDarkTime = loopStart;

  if(loopStart - lastLightTime > DARK_TRIGGER_TIME) {
    inTube = true;
    lastStateChangeTime = millis();
  }

  // Send readings over BLEUart
  if (millis() - lastBLE > 400) {
    if(bleIdx == 0) bleuart.printf("State [%s]\n", stateStrings[currentData.state]);
    if(bleIdx == 1) bleuart.printf("Time [%i] ", currentData.timestamp);

    if(bleIdx == 2) bleuart.printf("Press [%u] ", currentData.pressure);
    if(bleIdx == 3) bleuart.printf("Temp [%f]\n", currentData.temperature);
    if(bleIdx == 4) bleuart.printf("Ax [%f] ", currentData.accelX);
    if(bleIdx == 5) bleuart.printf("Ay [%f] ", currentData.accelY);
    if(bleIdx == 6) bleuart.printf("Az [%f]\n", currentData.accelZ);

    if(bleIdx == 7) bleuart.printf("Batt [%f] ", currentData.battSense * 2.0 * 3.6 / 1023.0);
    if(bleIdx == 8) bleuart.printf("CutSens1 [%li] ", currentData.cutSense1);
    if(bleIdx == 9) bleuart.printf("CutSens2 [%li]\n", currentData.cutSense2);
    if(bleIdx == 10) bleuart.printf("CurrSens [%li] ", currentData.currentSense);
    if(bleIdx == 11) bleuart.printf("Photores [%li]\n", currentData.photoresistor);
    if(bleIdx == 11) bleuart.println("===================");

    bleIdx++;
    if(bleIdx >= 12) { bleIdx = 0; }
    lastBLE = millis();
  }

  switch(state) {
    case WAITING:
      if ((currentAltitudeAvg > ALTITUDE1 && currentDeltaAvg < LIMIT_VELOCITY)
          || (inTube && loopStart - lastDarkTime > LIGHT_TRIGGER_TIME)) {
        inTube = false;
        state = DEPLOYED;
        lastStateChangeTime = loopStart;
        
        InternalFS.remove(FILENAME);
        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);
        
        Serial.print("Parachute deployed at time ");
        Serial.println(loopStart);
      }
      break;
    case DEPLOYED:
      if ((currentAltitudeAvg < ALTITUDE1)
          || (loopStart - (lastStateChangeTime) > DISREEF1TIME)) {
        pwmExecute(NICHROME_PIN1, PWM_VOLTAGE1);
        state = PARTIAL_DISREEF;
        lastStateChangeTime = loopStart;

        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);
        
        Serial.print("First line cut at time ");
        Serial.println(loopStart);
      }
      break;
    case PARTIAL_DISREEF:
      if ((currentAltitudeAvg < ALTITUDE2)
          || (loopStart - (lastStateChangeTime) > DISREEF2TIME)) {
        pwmExecute(NICHROME_PIN2, PWM_VOLTAGE2);
        state = FULL_DISREEF;
        lastStateChangeTime = loopStart;

        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);
        
        Serial.print("Second line cut at time ");
        Serial.println(loopStart);
      }
      break;
    case FULL_DISREEF:
      if (abs(currentDeltaAvg) < 0.05) {
        state = LANDED;
        lastStateChangeTime = loopStart;

        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);

        writeLandedData(file);
        
        Serial.print("Landed at time ");
        Serial.println(loopStart);
      }
      break;
    case LANDED:
      // Nothing happens after landing
      break;
    default:
      Serial.println("Something has gone horribly wrong if none of the states match.");
  }
  
  loopEnd = millis();
}


/*****************************
 *          HELPERS          *
 ****************************/

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
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}


// Average several readings to get "sea level" pressure
double calibrateSeaLevel(int samples) {
  Serial.println("Reading current pressure...");
  digitalWrite(CALIBRATION_LED, HIGH);
  int sum = 0;
  for (int i=0; i<samples; i++) {
    baro.Readout();
    sum += baro.GetPres();
    delay(100);
  }
  Serial.println("Done.");
  digitalWrite(CALIBRATION_LED, LOW);
  return sum / (double) samples;
}


// Convert pressure (in pascals) to altitude (in meters) using sea level pressure
double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / seaLevel, 1.0 / 5.25588));
}


// PWM pin to heat nichrome and cut parachute line
void pwmExecute(int pin, double targetVoltage) {
  Serial.print("Starting PWM on pin ");
  Serial.println(pin);
  hardwareAnalogWrite(pin, pwmLevel(targetVoltage));
  delay(PWM_DURATION);
  hardwareAnalogWrite(pin, 0);
  Serial.println("Done.");
  return;
}

// Manually interact with HardwarePWM objects
void hardwareAnalogWrite(int pin, int level) {
  Serial.print("Pin "); Serial.print(pin); Serial.print(" level "); Serial.println(level);
  if(pin == NICHROME_PIN1) hpwm1.writePin(NICHROME_PIN1, level);
  if(pin == NICHROME_PIN2) hpwm2.writePin(NICHROME_PIN2, level);
}

// Calculate number to be used for PWM of nichrome pin
int pwmLevel(double targetVoltage) {
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // Analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  // Find proportion of vbat needed to apply target voltage, then make out of 255 for analog write
  return (targetVoltage / vbat) * 255;
}

void writeStateChangeData(File f, unsigned long timestamp, int state, 
                          int32_t pressure, double altitude, double avgAltitude, 
                          double delta, double avgDelta) {
  char* filename = "stateChangeLog.txt";
  if ( f.open(filename, FILE_O_WRITE) ) {
    Serial.println("Writing data to file.");
    uint8_t buff[4] = {};

    uint32_t iAltitude = double_to_uint32_bits(altitude);
    uint32_t iAvgAltitude = double_to_uint32_bits(avgAltitude);
    uint32_t iDelta = double_to_uint32_bits(delta);
    uint32_t iAvgDelta = double_to_uint32_bits(avgDelta);
    
    f.write("t", 1);
    f.write( u32_to_u8(timestamp, buff), 4 );
    f.write("s", 1);
    f.write( u32_to_u8(state, buff), 4 );
    f.write("p", 1);
    f.write( u32_to_u8(pressure, buff), 4 );
    f.write("a", 1);
    f.write( u32_to_u8(iAltitude, buff), 4 );
    f.write("b", 1);
    f.write( u32_to_u8(iAvgAltitude, buff), 4 );
    f.write("d", 1);
    f.write( u32_to_u8(iDelta, buff), 4 );
    f.write("e", 1);
    f.write( u32_to_u8(iAvgDelta, buff), 4 );
    f.close();
  } else {
    Serial.print("Failed to write ");
    Serial.println(filename);
  }
}

void writeLandedData(File f) {
  InternalFS.remove(ID_FILE);
  if ( f.open(ID_FILE, FILE_O_WRITE) ) {
    
    Serial.println("Writing data to file.");
    f.write('0' + boardVersion);
    f.write("1");
    f.write(boardName);
    uint8_t landingBuff[4];
    f.write(u32_to_u8(flashLocation, landingBuff), 4);  // So we can figure out when it landed
    f.close();
  } else {
    Serial.print("Failed to write ");
    Serial.println(ID_FILE);
  }
}

uint32_t double_to_uint32_bits(double d) {
  float f = (float) d;
  return * ( uint32_t * ) &f;
}

uint8_t* float_to_u8(float f, uint8_t buff[4]) {
  uint32_t u = * ( uint32_t * ) &f;
  return u32_to_u8(u, buff);
}

uint8_t* u32_to_u8(const uint32_t u32, uint8_t buff[4]) {
  buff[0] = (u32 & 0xff000000) >> 24;
  buff[1] = (u32 & 0x00ff0000) >> 16;
  buff[2] = (u32 & 0x0000ff00) >> 8;
  buff[3] = u32 & 0x000000ff;
  return buff;
}

uint8_t* u16_to_u8(const uint16_t u16, uint8_t buff[2]) {
  buff[0] = (u16 & 0x0000ff00) >> 8;
  buff[1] = u16 & 0x000000ff;
  return buff;
}

void updateData(unsigned long timestamp) {
  if(boardVersion == 0) {
    const auto t = imu.readout();
    currentData.accelX = t.accel_xout;
    currentData.accelY = t.accel_yout;
    currentData.accelZ = t.accel_zout;
    currentData.cutSense1 = analogRead(CUT_SENSE1);
    currentData.cutSense2 = analogRead(CUT_SENSE2);
    currentData.currentSense = analogRead(CURRENT_SENSE);
  }

  currentData.state = state;
  currentData.timestamp = timestamp;
  currentData.pressure = baro.GetPres();
  currentData.temperature = baro.GetTemp();
  currentData.battSense = analogRead(VOLTAGE_DIVIDER);
  currentData.photoresistor = analogRead(PHOTO_PIN);
}

void updateFlash() {

  if (flashLocation % 0x40000 == 0) {
    flash.erase_sector_start(flashLocation / 0x40000);
    while(flash.is_write_in_progress()) { delay(1); }
  }

  // Write things, increment location
  uint8_t buff2[2];
  uint8_t buff4[4];
  uint8_t emptyBuff[1] = {0};
  uint8_t state_ = state;
  flash.write(flashLocation, u32_to_u8(currentData.timestamp, buff4), 4);
  flash.write(flashLocation + 4, u32_to_u8(currentData.pressure, buff4), 4);
  flash.write(flashLocation + 8, float_to_u8(currentData.temperature, buff4), 4);
  flash.write(flashLocation + 12, float_to_u8(currentData.accelX, buff4), 4);
  flash.write(flashLocation + 16, float_to_u8(currentData.accelY, buff4), 4);
  flash.write(flashLocation + 20, float_to_u8(currentData.accelZ, buff4), 4);
  flash.write(flashLocation + 24, u16_to_u8(currentData.battSense, buff2), 2);
  flash.write(flashLocation + 26, u16_to_u8(currentData.cutSense1, buff2), 2);
  flash.write(flashLocation + 28, u16_to_u8(currentData.cutSense2, buff2), 2);
  flash.write(flashLocation + 30, u16_to_u8(currentData.currentSense, buff2), 2);
  flash.write(flashLocation + 32, u16_to_u8(currentData.photoresistor, buff2), 2);
  flash.write(flashLocation + 34, &state_, 1);
  flash.write(flashLocation + 35, emptyBuff, 1);

  flashLocation += 64;
}

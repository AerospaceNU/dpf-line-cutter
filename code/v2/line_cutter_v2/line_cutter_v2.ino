#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <SPI.h>
#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"
#include "S25FL.h"

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

// Pins
const int VOLTAGE_DIVIDER = A0;
const int NICHROME_PIN1 = A1;
const int NICHROME_PIN2 = A2;

// Requirements for state transitions
const double LIMIT_VELOCITY = -3.0;  // meters/second
const double ALTITUDE1 = 400;  // Disreefing altitudes, in meters (higher one first!!)
const double ALTITUDE2 = 100;

// PWM settings
const double PWM_VOLTAGE1 = 0.8;  // Voltage applied to nichrome for line cuts
const double PWM_VOLTAGE2 = 0.8;
const int PWM_DURATION = 2500;  // Length of PWM, in milliseconds

// Altitude calculation
double seaLevel;

// Barometer and moving averages
MS5xxx baro(&Wire);
int ARRAY_SIZE = 40; // 2 seconds
MovingAvg altitudeReadings(ARRAY_SIZE);  // Store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // Store differences between avgs calculated using ^

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
int32_t pressure;  // pascals
double altitude;  // meters
double previousAltitudeAvg;
double currentAltitudeAvg;
double delta;  // meters/second
double currentDeltaAvg;

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

// Manually interact with HardwarePWM
// We can only add ONE pin per HardwarePWM; adding more makes them randomly flash
HardwarePWM& hpwm1 = HwPWM0;
HardwarePWM& hpwm2 = HwPWM1;

// For writing data to flash
struct Data {
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
  while (!Serial) { delay(10); }

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

  if (logData && boardVersion == 0 && flashLocation < 0x8000000) {
    updateFlash(loopStart);
  }

  // Send readings over BLEUart
  if (millis() - lastBLE > 1000) {
    bleuart.write(String(pressure).c_str());
    bleuart.write(',');
    bleuart.write(String(altitude).c_str());
    bleuart.write('\n');
    lastBLE = millis();
  }

  switch(state) {
    case WAITING:
      if (currentAltitudeAvg > ALTITUDE1 && currentDeltaAvg < LIMIT_VELOCITY) {
        state = DEPLOYED;
        
        InternalFS.remove(FILENAME);
        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);
        
        Serial.print("Parachute deployed at time ");
        Serial.println(loopStart);
      }
      break;
    case DEPLOYED:
      if (currentAltitudeAvg < ALTITUDE1) {
        pwmExecute(NICHROME_PIN1, PWM_VOLTAGE1);
        state = PARTIAL_DISREEF;

        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);
        
        Serial.print("First line cut at time ");
        Serial.println(loopStart);
      }
      break;
    case PARTIAL_DISREEF:
      if (currentAltitudeAvg < ALTITUDE2) {
        pwmExecute(NICHROME_PIN2, PWM_VOLTAGE2);
        state = FULL_DISREEF;

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
  digitalWrite(PIN_LED2, HIGH);
  int sum = 0;
  for (int i=0; i<samples; i++) {
    baro.Readout();
    sum += baro.GetPres();
    delay(100);
  }
  Serial.println("Done.");
  digitalWrite(PIN_LED2, LOW);
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

void updateFlash(unsigned long timestamp) {
  currentData.timestamp = timestamp;
  currentData.pressure = baro.GetPres();
  currentData.temperature = baro.GetTemp();
  currentData.accelX = 0;
  currentData.accelY = 0;
  currentData.accelZ = 0;
  currentData.battSense = analogRead(A4);
  currentData.cutSense1 = analogRead(A5);
  currentData.cutSense2 = analogRead(A0);
  currentData.currentSense = analogRead(A1);
  currentData.photoresistor = analogRead(A3);

  if (flashLocation % 0x40000 == 0) {
    Serial.print("Erased sector at location ");
    Serial.println(flashLocation);
    flash.erase_sector_start(flashLocation / 0x40000);
    while(flash.is_write_in_progress()) { delay(1); }
  }

  // Write things, increment location
  Serial.print("Writing at location ");
  Serial.println(flashLocation);
  uint8_t buff2[2];
  uint8_t buff4[4];
  uint8_t emptyBuff[1] = {0};
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
  flash.write(flashLocation + 34, emptyBuff, 1);

  uint8_t readBuff[64] = {};
  flash.read_start(flashLocation, readBuff, 64);
  for (int i=0; i<64; i++) {
    Serial.print(readBuff[i], BIN);
    Serial.print(" ");
  }
  Serial.println();

  flashLocation += 64;
}

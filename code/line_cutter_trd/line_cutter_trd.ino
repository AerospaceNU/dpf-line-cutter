#include <Adafruit_LittleFS.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <Wire.h>
#include "MovingAvg.h"
#include "MS5xxx.h"


// states
int state = 0;
const int WAITING = 0;  // anytime before parachute deployment
const int DEPLOYED = 1;
const int PARTIAL_DISREEF = 2;  // after first line is cut
const int FULL_DISREEF = 3;  // after second line is cut
const int LANDED = 4;

// pins
const int VOLTAGE_DIVIDER = A0;
const int NICHROME_PIN1 = A1;
const int NICHROME_PIN2 = A2;

// requirements for state transitions
const double LIMIT_VELOCITY = -3;  // m/s
const double ALTITUDE1 = 7.0;  // disreefing altitudes, in meters
const double ALTITUDE2 = 3.5;

// PWM settings
const double PWM_VOLTAGE1 = 0.5;  // voltage applied to nichrome for line cuts
const double PWM_VOLTAGE2 = 0.5;
const int PWM_DURATION = 2000;  // length of pwm in milliseconds

// altitude calculation
double seaLevel;

// barometer and moving averages
MS5xxx baro(&Wire);
int ARRAY_SIZE = 40; // 2 seconds
MovingAvg altitudeReadings(ARRAY_SIZE);  // store recent altitude readings
MovingAvg altitudeAvgDeltas(ARRAY_SIZE);  // store differences between recent avgs calculated using ^

// for logging data in internal filesystem
using namespace Adafruit_LittleFS_Namespace;
const char* FILENAME = "stateChangeLog.txt";
File file(InternalFS);

// variables used in loop()
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


/*****************************
 *        SETUP/LOOP         *
 ****************************/
 
void setup() {
  // Turn off nichrome pins right away
  pinMode(NICHROME_PIN1, OUTPUT);
  analogWrite(NICHROME_PIN1, 0);
  pinMode(NICHROME_PIN2, OUTPUT);
  analogWrite(NICHROME_PIN2, 0);
  
  Serial.begin(115200);

  // connect to barometer
  while ( baro.connect() > 0 ) {
    Serial.println("Error connecting to barometer...");
    delay(500);
  }
//  Serial.println("Barometer connected with calibration:");
  baro.ReadProm();
//  baro.printCalibData();

  // initialize moving averages
  seaLevel = calibrateSeaLevel(50);  // average 20 readings to get "sea level" pressure
  Serial.print("Sea level pressure [Pa]: ");
  Serial.println(seaLevel);
  altitudeReadings.begin();
  altitudeReadings.reading(pressureToAltitude(baro.GetPres()));
  altitudeAvgDeltas.begin();
  altitudeAvgDeltas.reading(0.0);

  // analog pins should give a number from 0 to 1023 when read
  analogReadResolution(10);  // 10 bits

  // Initialize Internal File System
  InternalFS.begin();
  
  // set up bluetooth
  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName("Condor");
  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();
  // Configure and start the BLE Uart service
  bleuart.begin();
  // Set up and start advertising
  startAdv();
}


void loop() {
  while(millis() < loopEnd + DELAY) {}
  
  loopStart = millis();  // used for timestamps in data log
 
  // read pressure, calculate altitude
  baro.Readout();
  pressure = baro.GetPres();
  altitude = pressureToAltitude(pressure);

  // send readings over bleuart
  if (millis() - lastBLE > 1000) {
    bleuart.write(String(pressure).c_str());
    bleuart.write(',');
    bleuart.write(String(altitude).c_str());
    bleuart.write('\n');
    lastBLE = millis();
  }

  // update moving averages
  previousAltitudeAvg = altitudeReadings.getAvg();
  currentAltitudeAvg = altitudeReadings.reading(altitude);  // update and return new avg
  delta = (currentAltitudeAvg - previousAltitudeAvg) * (1000.0 / DELAY);
  currentDeltaAvg = altitudeAvgDeltas.reading(delta);  // update and return new avg

  // printData();
  
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
        printData();
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
        printData();
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
        printData();
      }
      break;
    case FULL_DISREEF:
      if (abs(currentDeltaAvg) < 0.05) {
        state = LANDED;

        writeStateChangeData(file, loopStart, state, 
                             pressure, altitude, currentAltitudeAvg, 
                             delta, currentDeltaAvg);
        
        Serial.print("Landed at time ");
        Serial.println(loopStart);
        printData();  
      }
      break;
    case LANDED:
      // this doesn't do anything right now.
      // if anyone has thoughts on whether the line cutter should do anything after landing, lmk.
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


// average several readings to get "sea level" pressure
// int -> double
double calibrateSeaLevel(int samples) {
  Serial.println("Reading current pressure...");
  digitalWrite(LED_BUILTIN, HIGH);
  int sum = 0;
  for (int i=0; i<samples; i++) {
    baro.Readout();
    sum += baro.GetPres();
    delay(100);
  }
  Serial.println("Done.");
  digitalWrite(LED_BUILTIN, LOW);
  return sum / (double) samples;
}


// convert pressure (in pascals) to altitude (in meters) using sea-level pressure
// int32_t -> double
double pressureToAltitude(int32_t pressure) {
  return 44330.76 * (1.0 - pow(pressure / seaLevel, 1.0 / 5.25588));
}


// PWM pin to heat nichrome and cut parachute line
// int, double -> _
void pwmExecute(int pin, double targetVoltage) {
  Serial.print("Starting PWM on pin ");
  Serial.println(pin);
  analogWrite(pin, pwmLevel(targetVoltage));
  delay(PWM_DURATION);
  analogWrite(pin, 0);
  Serial.println("Done.");
  return;
}


// calculate number to be used for analogWrite() of nichrome pin
// double -> int
int pwmLevel(double targetVoltage) {
  // get reading from voltage divider and multiply by 2
  int vbatAnalog = 2 * analogRead(VOLTAGE_DIVIDER);
  // analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  // to get current vbat  
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  // find proportion of vbat needed to apply target voltage, then make it out of 255 for analogWrite
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

uint32_t double_to_uint32_bits(double d) {
  float f = (float)d;
  uint32_t result = * ( uint32_t * ) &f;
  return result;
}

uint8_t* u32_to_u8(const uint32_t u32, uint8_t buff[4]) {
  buff[0] = (u32 & 0xff000000) >> 24;
  buff[1] = (u32 & 0x00ff0000) >> 16;
  buff[2] = (u32 & 0x0000ff00) >> 8;
  buff[3] = u32 & 0x000000ff;
  return buff;
}

void printData() {
  Serial.print("State: ");
  Serial.println(state);
  Serial.print("Pressure [Pa]: ");
  Serial.println(pressure);
  Serial.print("Altitude [m]: ");
  Serial.println(altitude);
  Serial.print("Averaged altitude [m]: ");
  Serial.println(currentAltitudeAvg);
  Serial.print("Change in altitude [m/s]: ");
  Serial.println(delta);
  Serial.print("Averaged change in altitude [m/s]: ");
  Serial.println(currentDeltaAvg);
  Serial.println("------------");
  return;
}

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

const char* FILENAME = "flightVariables.txt";
File file(InternalFS);

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
FlightVariables flightVars;
const int FLIGHT_VARIABLE_SIZE = 37;

void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  // Requirements for state transitions
  flightVars.limitVel         = -0.5;  // meters/second
  flightVars.altitude1        = 457;  // Disreefing altitudes, in meters (higher one first!!)
  flightVars.altitude2        = 366;
  flightVars.disreefDelay1    = 48000;  // Max delay after ejection before first disreef, in milliseconds
  flightVars.disreefDelay2    = 12500;  // Max delay after first disreef before second disreef, in milliseconds
  // PWM settings
  flightVars.pwmVoltage1      = 1.7;  // Voltage applied to nichrome for line cuts
  flightVars.pwmVoltage2      = 1.7;
  flightVars.pwmDuration      = 3000;  // Length of PWM, in milliseconds
  // Photoresistor variables
  flightVars.lightThreshold   = 400;  // Anything above this value is considered to be outside of tube
  flightVars.lightTriggerTime = 2000;  // Continuous time that it must be light for board to detect deployment, in milliseconds

  // Initialize Internal File System
  InternalFS.begin();
  InternalFS.remove(FILENAME);

  if ( file.open(FILENAME, FILE_O_WRITE) ) {
    uint8_t* contents = ( uint8_t* ) &flightVars;
    file.write(contents, FLIGHT_VARIABLE_SIZE);
    Serial.println("Done writing variables.");
  } else {
    Serial.println("Failed!");
  }
  if ( file.open(FILENAME, FILE_O_READ) ) {
    uint8_t buff[64] = {0};
    file.read(buff, FLIGHT_VARIABLE_SIZE);
    FlightVariables readback = *( FlightVariables* ) &buff;
    
    Serial.println(flightVars.limitVel);
    Serial.println(flightVars.altitude1);
    Serial.println(flightVars.altitude2);
    Serial.println(flightVars.disreefDelay1);
    Serial.println(flightVars.disreefDelay2);
    Serial.println(flightVars.pwmVoltage1);
    Serial.println(flightVars.pwmVoltage2);
    Serial.println(flightVars.pwmDuration);
    Serial.println(flightVars.lightThreshold);
    Serial.println(flightVars.lightTriggerTime);
  } else {
    Serial.println("Failed to read back data.");
  }
}

void loop()
{
}

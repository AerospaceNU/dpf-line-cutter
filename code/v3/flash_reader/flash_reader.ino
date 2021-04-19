#include <SPI.h>
#include "S25FL.h"

S25FL flash;  // Starts Flash class and initializes SPI
unsigned long flashLocation = 0x40000;
uint8_t buff[64] = {0};
bool done = false;

struct Data {
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

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  
//  Serial.println("Metadata: ");
//  flash.read_start(0, buff, 64);
//  Serial.print("[");
//  for (int i=0; i<64; i++) {
//    if (i % 8 == 0 && i > 0) { Serial.println(); }
//    Serial.print(buff[i], HEX);
//    if (i < 63) { Serial.print(",\t"); }
//  }
//  Serial.println("]");
//  Serial.println();
  delay(5000);  // Give time to connect with CoolTerm or similar
}

void loop() {
  flash.read_start(flashLocation, buff, 64);
  currentData = *( Data* ) &buff;
  flashLocation += 64;
  
  if (buff[0] == 0xff) { done = true; }
  if (!done) { print_flash_packet(); }
  delay(5);
}

void print_flash_packet() {
  Serial.printf("%u,%u,%u,%f,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u,%u,%u\n", 
                currentData.state, currentData.timestamp, currentData.pressure,
                currentData.altitude,currentData.avgAltitude,currentData.deltaAltitude,currentData.avgDeltaAltitude,
                currentData.temperature, currentData.accelX, currentData.accelY, currentData.accelZ,
                currentData.battSense, currentData.cutSense1, currentData.cutSense2,
                currentData.currentSense, currentData.photoresistor);
}

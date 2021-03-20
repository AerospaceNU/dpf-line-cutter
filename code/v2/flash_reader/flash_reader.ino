#include <SPI.h>
#include "S25FL.h"

S25FL flash;  // Starts Flash class and initializes SPI
unsigned long flashLocation = 0;
uint8_t buff[64] = {0};

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

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
}

void loop() {
  if (buff[34] == 0) {
    read_flash_packet();
    delay(20);
  }
}

void read_flash_packet() {
  flash.read_start(flashLocation, buff, 64);
  
  currentData.timestamp = u8_to_u32(&buff[0]);
  currentData.pressure = u8_to_u32(&buff[4]);
  currentData.temperature = u8_to_float(&buff[8]);
  currentData.accelX = u8_to_float(&buff[12]);
  currentData.accelY = u8_to_float(&buff[16]);
  currentData.accelZ = u8_to_float(&buff[20]);
  currentData.battSense = u8_to_u16(&buff[24]);
  currentData.cutSense1 = u8_to_u16(&buff[26]);
  currentData.cutSense2 = u8_to_u16(&buff[28]);
  currentData.currentSense = u8_to_u16(&buff[30]);
  currentData.photoresistor = u8_to_u16(&buff[32]);

  Serial.printf("%u, %u, %f, %f, %f, %f, %u, %u, %u, %u, %u\n", 
                currentData.timestamp, currentData.pressure, currentData.temperature,
                currentData.accelX, currentData.accelY, currentData.accelZ,
                currentData.battSense, currentData.cutSense1, currentData.cutSense2,
                currentData.currentSense, currentData.photoresistor);

  flashLocation += 64;
}

float u8_to_float(const uint8_t* bytes) {
  uint32_t u32 = u8_to_u32(bytes);
  float f = * ( float * ) &u32;
  return f;
}

uint32_t u8_to_u32(const uint8_t* bytes) {
  uint32_t u32 = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return u32;
}

uint32_t u8_to_u16(const uint8_t* bytes) {
  uint16_t u16 = (bytes[0] << 8) + bytes[1];
  return u16;
}

#include <Arduino.h>


#include "S25FL.h"

S25FL flash{};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial) {
    yield();
  }
  delay(1000);
  // while (Serial.available() < 1) {
    // Serial.println("hi");
    // delay(1000);
  // } 

  SPI2.begin();
  SPI2.setBitOrder(MSBFIRST);
  SPI2.setClockDivider(SPI_CLOCK_DIV8);//divide the clock by 8
  SPI2.setDataMode(SPI_MODE0);

  Serial.println("Starting flash");
  
  // Erase sector 0
  flash.init();
  flash.check_connected();
  while(!flash.is_write_completed()) { Serial.println("Write in process"); delay(100); }
  
  flash.erase_sector_start(0);
  while(!flash.is_write_completed()) { Serial.println("Write in process2"); delay(100); }
  Serial.println("Erased");

  // // Write a thing
  uint8_t b[32] = {};
  memset(b, 0b00111100, 32);
  flash.write_start(80, 32, b);
  while(!flash.is_write_completed());
  Serial.println("Wrote");

  // And try to read it out
  static uint8_t buf[64] = {};
  Serial.println("Starting read");
  flash.read_start(0, 64, buf);
  for(int i = 0; i < 64; i++) {
    Serial.print(buf[i], BIN); Serial.print(" ");
  }
  Serial.println();
  flash.read_start(64, 64, buf);
  for(int i = 0; i < 64; i++) {
    Serial.print(buf[i], BIN); Serial.print(" ");
  }
  Serial.println();
  flash.read_start(128, 64, buf);
  for(int i = 0; i < 64; i++) {
    Serial.print(buf[i], BIN); Serial.print(" ");
  }
  Serial.println();
}

void loop() {
}

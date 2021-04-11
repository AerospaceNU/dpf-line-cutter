#include "S25FL.h"

const int CHIP_SELECT_PIN = 8;
S25FL flash(CHIP_SELECT_PIN);  // Starts Flash class and initializes SPI
bool completed = false;

void setup() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  Serial.print("Erasing chip... ");
  flash.erase_chip_start();
}

void loop() {
  if (!completed) {
    completed = flash.is_erase_complete();
    if (completed) { Serial.println("Done"); }
  }
  delay(10);
}

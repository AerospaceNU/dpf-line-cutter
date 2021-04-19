#include "S25FL.h"

const int CHIP_SELECT_PIN = 8;
S25FL flash(CHIP_SELECT_PIN);  // Starts Flash class
bool completed = false;

unsigned long startTime;
unsigned long endTime;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  Serial.print("Erasing chip... ");
  flash.erase_chip_start();
  startTime = millis();
}

void loop() {
  if (!completed) {
    completed = flash.is_erase_complete();
    if (completed) { 
      Serial.println("Done");
      endTime = millis();
      Serial.print("Erase took ");
      Serial.print(endTime-startTime);
      Serial.println("ms");
    }
  }
  delay(10);
}

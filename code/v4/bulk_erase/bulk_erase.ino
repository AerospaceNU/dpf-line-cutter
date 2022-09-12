#include "S25FL.h"
SPIClass SPI2(NRF_SPIM3,  PIN_SPI_MISO,  PIN_QSPI_SCK,  PIN_SPI_MOSI); //alternate SPI object to use QSPI clock
const int CHIP_SELECT_PIN = 8;
S25FL flash(CHIP_SELECT_PIN);  // Starts Flash class
bool completed = false;

unsigned long startTime;
unsigned long endTime;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  SPI2.begin();
  SPI2.setBitOrder(MSBFIRST);
  SPI2.setClockDivider(SPI_CLOCK_DIV8);  // Divide the clock by 8
  SPI2.setDataMode(SPI_MODE0);
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

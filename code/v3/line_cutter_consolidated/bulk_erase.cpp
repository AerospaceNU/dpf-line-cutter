#include "bulk_erase.h"
#include "S25FL.h"

const int CHIP_SELECT_PIN = 8;
S25FL eraseModeFlash(CHIP_SELECT_PIN);  // Starts Flash class
bool completed = false;

unsigned long startTime;
unsigned long endTime;

void bulk_erase_setup() {
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);

  Serial.print("Erasing chip... ");
  eraseModeFlash.erase_chip_start();
  startTime = millis();
}

void bulk_erase_loop() {
  while (!completed) {
    completed = eraseModeFlash.is_erase_complete();
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

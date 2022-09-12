#include "LIS2DE12.h"
#include <Arduino.h>
#include <Wire.h>

LIS2DE12 accel{};

void setup() {
  Serial.begin(9600);

  Serial.print("Hello from ");
  Serial.println(Read8(0x0F), BIN);

  if(!accel.Connected()) {
    Serial.println("Error connecting!");
  }
  Serial.println("Hello2");

  accel.SetDataRate(LIS2DE12::DataRate::RATE_50HZ);
  Serial.println("Hello3");
  accel.SetFullscale(LIS2DE12::AccelFullscale::SCALE_4G);
  Serial.println("Hello4");
}

void loop() {
  Serial.print(millis() / 1000.0);
  Serial.print(" X: "); Serial.print(accel.GetAccelX());
  Serial.print(" Y: "); Serial.print(accel.GetAccelY());
  Serial.print(" Z: "); Serial.println(accel.GetAccelZ());
  delay(100);
}

uint8_t Read8(uint8_t reg) {
  Wire.beginTransmission(0x19); // or 0x19, 0x76
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(0x19, (uint8_t)1);
  return Wire.read();
}

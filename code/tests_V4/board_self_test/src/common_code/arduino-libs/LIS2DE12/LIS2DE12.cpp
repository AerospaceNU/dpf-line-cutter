#include "LIS2DE12.h"
#include "Wire.h"
#include <Arduino.h>

bool LIS2DE12::Connected() {
  return Read8(0x0F) == 0b00110011;
}

bool LIS2DE12::SetFullscale(AccelFullscale scale) {
  // Get the old value
  uint8_t reg = Read8(CTRL_REG1 + 3);
  // Mask out the old fullscale value
  reg = reg & 0b11001111;
  // Set bits 1 and 2 to the desired fullscale
  reg = reg | scale << 4;
  // and write to I2c
  Write8(CTRL_REG1 + 3, reg);

  this->scale = scale;

  Serial.print("Fullscale: "); Serial.println(Read8(CTRL_REG1 + 3), BIN);
  return true;
}

LIS2DE12::AccelFullscale LIS2DE12::GetFullscale() {
  return this->scale;
}

bool LIS2DE12::SetDataRate(DataRate rate) {
  // Get the old value
  uint8_t reg = Read8(CTRL_REG1);
  // Mask out the old fullscale value
  reg &= 0b00011111;
  // Set the highest 3 bits to the desired fullscale
  reg |= rate << 5;

  // and write to I2c
  Write8(CTRL_REG1, reg);

  this->rate = rate;

  Serial.print("Data Rate: "); Serial.println(Read8(CTRL_REG1 + 3), BIN);
  return true;
}

LIS2DE12::DataRate LIS2DE12::GetDataRate() {
  return this->rate;
}

int8_t LIS2DE12::GetAccelX_raw() {
  return (int8_t) Read8(OUT_X_H);
}

int8_t LIS2DE12::GetAccelY_raw() {
  return (int8_t) Read8(OUT_X_H + 2);
}

int8_t LIS2DE12::GetAccelZ_raw() {
  return (int8_t) Read8(OUT_X_H + 4);
}

float LIS2DE12::GetAccelX() {
  // digits * mg/digit * gees/mg
  return GetAccelX_raw() * GetSenstivityMgPerDigit() / 1000.0;
}

float LIS2DE12::GetAccelY() {
  return GetAccelY_raw() * GetSenstivityMgPerDigit() / 1000.0;
}

float LIS2DE12::GetAccelZ() {
  return GetAccelZ_raw() * GetSenstivityMgPerDigit() / 1000.0;
}

float LIS2DE12::GetSenstivityMgPerDigit() {
  switch(this->scale) {
    case SCALE_2G:
      return 15.6;
    case SCALE_4G:
      return 31.2;
    case SCALE_8G:
      return 62.5;
    case SCALE_16G:
      return 187.5;
    default:
      return 1;
  }
}

uint8_t LIS2DE12::Write8(uint8_t reg, uint8_t byte) {
  Wire.beginTransmission(this->address);
  Wire.write(reg);
  Wire.write(byte);
  return Wire.endTransmission();
}

uint8_t LIS2DE12::Read8(uint8_t reg) {
  Wire.beginTransmission(this->address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(this->address, 1, true);
  return Wire.read();
}

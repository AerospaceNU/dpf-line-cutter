#pragma once

#include <Wire.h>
#include <stdint.h>

class LIS2DE12 {
 public:
  LIS2DE12(uint8_t address = 0x19) {
    this->address = address;
    Wire.begin();
  }

  bool Connected();

  enum AccelFullscale : uint8_t {
    SCALE_2G = 0b00,
    SCALE_4G = 0b01,
    SCALE_8G = 0b10,
    SCALE_16G = 0b11
  };

  enum DataRate : uint8_t {
    RATE_POWERDOWN = 0,
    RATE_1HZ = 1,
    RATE_10HZ = 2,
    RATE_25HZ = 3,
    RATE_50HZ = 4,
    RATE_100HZ = 5,
    RATE_200HZ = 6,
    RATE_400HZ = 7,
  };

  bool SetFullscale(AccelFullscale scale);
  AccelFullscale GetFullscale();

  bool SetDataRate(DataRate scale);
  DataRate GetDataRate();

  float GetAccelX();
  float GetAccelY();
  float GetAccelZ();

 private:
  uint8_t Write8(uint8_t reg, uint8_t byte);
  uint8_t Read8(uint8_t reg);

  int8_t GetAccelX_raw();
  int8_t GetAccelY_raw();
  int8_t GetAccelZ_raw();

  float GetSenstivityMgPerDigit();

  uint8_t address;

  AccelFullscale scale = SCALE_16G;
  DataRate rate = RATE_POWERDOWN;

  static constexpr uint8_t CTRL_REG1 = 0x20;
  static constexpr uint8_t OUT_X_H = 0x29;
};

#pragma once

#include <Wire.h>
#include "i2cUtil.h"
#include <Arduino.h>

using namespace i2c;

typedef struct
{
  float accel_xout;
  float accel_yout;
  float accel_zout;
  int16_t temp_out;
  int16_t gyro_xout;
  int16_t gyro_yout;
  int16_t gyro_zout;
} imu_out_t;
#define ACCEL_OUT_SIZE 14 // No longer sizeOf sinze imu_out_t holds the output stuff, not just raw int16s

// Registers
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B

static constexpr uint8_t REG_WHO_AM_I = 0x75;
static constexpr uint8_t WHOAMI = 0x12;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
  return (msb << 8u) | lsb;
}

// An ICM-20602 accelerometer
class IMU
{
public:
  static constexpr const uint8_t address = 0x69;

  /** Enumerated value corresponds with ACCEL_FS_SEL in the ACCEL_CONFIG
    * register. Values listed are the full +/- G range. */
  enum icm20602_accel_g : uint8_t {
    ICM20602_ACCEL_RANGE_2G = 0,
    ICM20602_ACCEL_RANGE_4G = 1,
    ICM20602_ACCEL_RANGE_8G = 2,
    ICM20602_ACCEL_RANGE_16G = 3,
  };
  
  // The fullscale range of the accelerometer
  icm20602_accel_g accel_g = ICM20602_ACCEL_RANGE_16G;

  IMU() {}

  bool begin()
  {
    // Reset the device
    //Serial.println("resetting");
    Write8(PWR_MGMT_1, Bit7, address);
    delay(2); // Wait 2ms

    // Disable sleep, use the best clock source
    Write8(PWR_MGMT_1, 0x00 | Bit0, address); 
    if(!isConnected()) return false;

    // +- 16 Gs default
    setFullscale(ICM20602_ACCEL_RANGE_16G);
    
    return true;
  }

  void setFullscale(icm20602_accel_g range) {
    //Serial.print("Setting full to "); Serial.println(range << 3, BIN);
    Write8(0x1C, 0x00 | range << 3, address);
    accel_g = range;
  }

  uint8_t isConnected() {
    return Read8(REG_WHO_AM_I, address) == WHOAMI;
  }

  void printReg(uint8_t reg) {
    Serial.print(reg, HEX); Serial.print(": "); Serial.println(Read8(reg, address), DEC);
  }

  imu_out_t readout()
  {
    // Allocate a buffer 
    static uint8_t buff[ACCEL_OUT_SIZE];
    size_t read = ReadBytes<ACCEL_OUT_SIZE>(ACCEL_XOUT_H, buff, address);

    float sensitivity = _get_accel_sensitivity(accel_g);
    //Serial.print("sensitivity"); Serial.println(sensitivity);

    return {
      ((float) combine(buff[0], buff[1])) / sensitivity,
      ((float) combine(buff[2], buff[3])) / sensitivity,
      ((float) combine(buff[4], buff[5])) / sensitivity,
      ((float) combine(buff[6], buff[7]) / 326.8) + 25.0,
      combine(buff[8], buff[9]),
      combine(buff[10], buff[11]),
      combine(buff[12], buff[13]),
    };
  }


 private:
  float _get_accel_sensitivity(enum icm20602_accel_g scale) {
    float f = 0.0;
  
    switch (scale) {
    case (ICM20602_ACCEL_RANGE_2G):
      f = 16384.0;
      break;
    case (ICM20602_ACCEL_RANGE_4G):
      f = 8192.0;
      break;
    case (ICM20602_ACCEL_RANGE_8G):
      f = 4096.0;
      break;
    case (ICM20602_ACCEL_RANGE_16G):
      f = 2048.0;
      break;
    }
  
    return f;
  }
 
};

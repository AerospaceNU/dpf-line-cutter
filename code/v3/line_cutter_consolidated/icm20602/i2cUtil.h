#include <Wire.h>

namespace i2c {

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

template<int size>
size_t ReadBytes(uint8_t reg, uint8_t buff[size], uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, size);
  return Wire.readBytes(buff, size);
}

inline uint8_t Read8(uint8_t reg, uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  return Wire.read();
}

inline uint8_t Write8(uint8_t reg, uint8_t byte_, uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(byte_);
  return Wire.endTransmission();
}

} // namespace i2c

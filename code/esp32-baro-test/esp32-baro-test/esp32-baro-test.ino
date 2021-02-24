/*
    MS5xxx.h - Library for accessing MS5xxx sensors via I2C
    Copyright (c) 2012 Roman Schmitz

    This file is part of arduino-ms5xxx.

    arduino-ms5xxx is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    arduino-ms5xxx is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with arduino-ms5xxx.  If not, see <http://www.gnu.org/licenses/>.

*/

//#include <Wire.h>
//
//#define PROM0 0xA0
//#define ADDRESS 0x76
//#define RESET 0x1E
//
//void setup() {
//  Wire.begin();
//  Serial.begin(115200);
//
//  // Reset the barometer
//  Wire.beginTransmission(ADDRESS);
//  Wire.write(RESET);
//  Wire.endTransmission();
//
//  // Do we need this delay?
//  delay(10);
//
//  // Next ask for the first 2 prom devices
//  Wire.beginTransmission(ADDRESS);
//  Wire.write(0xA2);
//  Wire.endTransmission();
//  Wire.requestFrom(ADDRESS, 2);
//  uint8_t higher_8 = Wire.read();
//  uint8_t lower_8 = Wire.read();
//  uint16_t value = (higher_8 << 8) | lower_8; // Read the first 8 bits; shift them left 8; OR with the next 8 bits
//  Serial.print("Higher 8: "); Serial.print(higher_8, BIN); Serial.print("Lower 8: "); Serial.print(lower_8, BIN); Serial.print("Full: "); Serial.println(value, BIN); 
//
//  Wire.beginTransmission(ADDRESS);
//  Wire.write(0xA4);
//  Wire.endTransmission();
//  Wire.requestFrom(ADDRESS, 2);
//  higher_8 = Wire.read();
//  lower_8 = Wire.read();
//  value = (higher_8 << 8) | lower_8; // Read the first 8 bits; shift them left 8; OR with the next 8 bits
//  Serial.print("Higher 8: "); Serial.print(higher_8, BIN); Serial.print("Lower 8: "); Serial.print(lower_8, BIN); Serial.print("Full: "); Serial.println(value, BIN); 
//}
//
//void loop() {}

#include <Wire.h>
//#include "BluetoothSerial.h"

#include "Melon2.h"
#include "nonmelon.h"

// Change this to chagne the library used
//#define MELON

#ifdef MELON
Melon_MS5607 baro;
#else
MS5xxx baro(&Wire);
#endif

//BluetoothSerial SerialBT;

#ifdef MELON
int32_t maxPressure;
#else
double maxPressure;
#endif

void setup() {
  Serial.begin(115200);
//  SerialBT.begin("ESP32test"); //Bluetooth device name

#ifdef MELON
  Serial.println("Starting on MELON");
  baro.begin(0x76);
  baro.printCalibData();

#else
  Serial.println("Starting on MS5xxx");
  baro.connect();

  baro.ReadProm();
  baro.Readout();
  
  baro.printCalibData();

#endif
}

void loop() {
#ifdef MELON
  baro.getPressureBlocking();

  maxPressure = max(baro.getPressure(), maxPressure);
//  SerialBT.print(baro.getTemperature() / 100.0); SerialBT.print(", "); SerialBT.print(baro.getPressure()); SerialBT.print(", "); SerialBT.println(maxPressure);
  Serial.println(baro.getTemperature() / 100.0);
#else
//  baro.ReadProm();
  baro.Readout();

  maxPressure = max(baro.GetPres(), maxPressure);
//  SerialBT.print(baro.GetTemp() / 100.0); SerialBT.print(", "); SerialBT.print(baro.GetPres()); SerialBT.print(", "); SerialBT.println(maxPressure);

  Serial.println(baro.GetTemp() / 100.0);
  Serial.println(baro.GetPres());
#endif
  delay(1000);
}

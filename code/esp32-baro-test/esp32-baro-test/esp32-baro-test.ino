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

#include <Wire.h>
#include "BluetoothSerial.h"
#include "Melon.h"

Melon_MS5607 baro;
BluetoothSerial SerialBT;

int32_t maxPressure;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name

  baro.begin(0x76);
}

void loop() {
  baro.getPressureBlocking();

  maxPressure = max(baro.getPressure(), maxPressure);
  SerialBT.print(baro.getTemperature() / 100.0); SerialBT.print(", "); SerialBT.print(baro.getPressure()); SerialBT.print(", "); SerialBT.println(maxPressure);
  delay(100);
}

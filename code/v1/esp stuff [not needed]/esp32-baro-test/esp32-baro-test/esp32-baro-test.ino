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

#include <bluefruit.h>
BLEUart bleuart;
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];


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

  analogReadResolution(10);  // 10 bits

  Bluefruit.begin();
  Bluefruit.setTxPower(8);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  // To be consistent OTA DFU should be added first if it exists
//  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
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
  bleuart.print("temp "); bleuart.print(baro.GetTemp() / 100.0);


  int vbatAnalog = 2 * analogRead(A0);
  // analog reading is out of 1023, so divide and then multiply by 3.6 (reference voltage)
  // to get current vbat  
  double vbat =  3.6 * (vbatAnalog / 1023.0);
  bleuart.print(" vbat "); bleuart.println(vbat);

  Serial.println(vbatAnalog);
  
  
#endif
  delay(1000);
}

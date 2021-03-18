# Signals

The following signals are connected to the following pins on the nRF module, which correlate to the following Arduino pins on the [Adafruit core](https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/variants/feather_nrf52840_express/variant.cpp). Analog pins are listed along with their digital equivilants.

| Description | Physical pin(s) | Arduino pin(s) | Uses | Driver |
| ----- | ----------- | ------------ | --------- | ------ |
| I2c SCL/SDA | P0.11/P0.12 | 22/23 | [Barometer](https://www.digikey.com/en/products/detail/te-connectivity-measurement-specialties/MS560702BA03-50/4700921), [Accelerometer](https://www.digikey.com/en/products/detail/tdk-invensense/ICM-20602/5872870) | Modified MS5xxx, Need to write one |
| SPI MISO/MOSI/SCK/CS0 | P0.15, P0.13, P0.14, P0.16 | 25, 25, 26, 8 | Flash memory | [S25FLx](https://github.com/BleepLabs/S25FLx) |
| SWDIO/SWDCLK | SWDIO/SWDCLK | | Used to program the chip (initially), debugging | |
| BATT_SENSE | P0.02 | A4 (18) | Battery voltage divider | Battery voltage = 2 * (analogRead(A4) / analogReadResolution) |
| CUT_SENSE1/2 | P0.03/0.04 | A5/A0 (19/14)  | Nichrome continuity | Should read about half of Vbatt when connected |
| V_CURR_SENSE | P0.05 | A1 (15) | [Current sense IC](https://www.digikey.com/en/products/detail/allegro-microsystems/ACS711KEXLT-15AB-T/3868192) | The output rises by 90mV / Amp, centered around VCC/2. We should figure out how to read Vcc |
| PHOTO- | P0.28 | A3 (17) | Output of the photoresistor voltage divider |
| CUT1/2 | P0.06/0.08 | 11, 12 | MOSFETs for nichrome |
| RESET | P0.18 | Not Usable | Reset button |
| UART RXD/TXD | P0.24/P0.25 | 1, 0 | UART (Not used currently) |
| USER LED | P1.10 | 4 (PIN_LED2) | Lefmost LED ("blue" LED on feathers). By default blinks for Bluetooth connection |
| RGB LED | P0.17, P0.19, P0.20 | 29, 27, 28 | Red, Green, Blue, respectively |

# Things to check before plugging a board in

- Resistance between ground and:
  - 3v3 (header)
  - VBUS (from the USB, header)
  - VBAT
  - SCL/SDA (should be open -- short will cause sketch to hang)

- Resistance between 3v3 and:
  - SCL/SDA (4.7k pullup expected)

# Places to watch out for

- Accelerometer -- Can't see shorts
- nRF -- can't see solder joints, and reflowing after oven will likely kill it

# Board names

| Board Number | Name | Known Issues |
| ----- | ----------- | ------------ |
| 1 | 0Strawberry | |
| 2 | 0Cherry | |
| 3 | 0Did you ever hear the trajedy of Darth Plagueis the Wise? | |
| 4 | 0Apple | |
 

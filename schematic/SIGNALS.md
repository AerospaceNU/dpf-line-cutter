# Signals

The following signals are connected to the following pins on the nRF module:

- I2c SCL/SDA: P0.11/P0.12
  - Used for barometer, accelerometer
- SPI MISO/MOSI/SCK/CS0: P0.15, P0.13, P0.14, P0.16
  - Used for flash memory
- SWDIO/SWDCLK
  - SWD pins needed to program the chip (initially)
- BATT_SENSE: P0.02 (battery voltage divider)
- CUT_SENSE1/2: P0.03/0.04 (nichrome continuity)
- V_CURR_SENSE: P0.05 (current sense IC)
- PHOTO-: P0.28 (photoresistor)
- CUT1/2: P0.06/0.08 (MOSFETs for nichrome)
- RESET: P0.18 (Reset button)
- UART RXD/TXD: P0.24/P0.25
- USER LED: P1.10 ("blue" LED on feathers)
- RGB LED: P0.17, P0.19, P0.20

# Things to check before plugging a board in

- Resistance between ground and:
  - 3v3 (header)
  - VBUS (from the USB, header)
  - VBAT
  - SCL/SDA (should be open -- short will cause sketch to hang)

- Resistance between 3v3 and:
  - SCL/SDA (4.7k pullup expected)
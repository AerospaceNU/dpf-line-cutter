# Programming NRF52 chips with OpenOCD

## With ST-Link

St-link V2s can be had for as little as $5, and work on Windows or Linux. A Windows build of OpenOCD, along with config file for the nrf52, is in `code/libraries and drivers/openocd_windows.zip`.

## With a Pi 3ish

Diagrams taken from [here](https://iosoft.blog/2019/01/28/raspberry-pi-openocd/) -- the Medium article went poof but we have the [Internet Archive](https://web.archive.org/web/20191204234928/https://medium.com/@ly.lee/coding-nrf52-with-rust-and-apache-mynewt-on-visual-studio-code-9521bcba6004) for that.

Wire the nRF52 to a Pi 2+ with the following:

```
GND -> GND
SWDIO/TMS -> Physical pin 18 (BCM 24)
SWCLK/TCK -> Physical pin 22 (BCM 25)
nRESET -> Phsyical pin 12
```

Note that the 3v3 rail on the Pi gets sad when you plug a whole Feather in. I'd suggest powering it externally.

Flash a Pi with Raspbian. You'll likely want one with a desktop environment so you can register with Resnet.

Install OpenOCD with `sudo apt update && sudo apt install openocd`

Create a file `bcm-nrf.cfg` with the following:

```
# Use RPi GPIO pins
interface bcm2835gpio

# Base address of I/O port
bcm2835gpio_peripheral_base 0x3F000000

# Clock scaling
bcm2835gpio_speed_coeffs 146203 36

# SWD                swclk swdio
# Header pin numbers 22    18    (counting from 1 on the top left)
bcm2835gpio_swd_nums 25    24

transport select swd

source [find target/nrf52.cfg]

# gdb_breakpoint_override hard
```

Run `sudo openocd -f bcm-nrf.cfg`. This should produce something that looks like this:

```
Open On-Chip Debugger 0.11.0-rc2+dev-00001-g5c17ce508-dirty (2021-01-27-17:38)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
DEPRECATED! use 'adapter driver' not 'interface'
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : BCM2835 GPIO JTAG/SWD bitbang driver
Info : clock speed 1001 kHz
Info : SWD DPIDR 0x2ba01477
Info : nrf52.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server for nrf52.cpu on 3333
Info : Listening on port 3333 for gdb connections
```

If you see something that starts with `Error:`, check your connections and try again.

## Flashing Bootloader

Connect with `telnet localhost 4444`. You may need to `sudo apt install telnet` (Linux) or enable the telnet clilent (Windows) first.

We will use the Adafruit bootloader from [here](https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/bootloader). A copy is in `code/libraries and drivers/nrf52840_express.hex`. Transfer it to the Pi and run the following command in OpenOCD.

```
reset init
halt
nrf5 mass_erase
program <full/path/to/bootloader.hex> verify
reset
```

Note that you must call `reset` for the board to enumerate. To erase all:

```
nrf52.dap apreg 1 0x04 0x01
nrf52.dap apreg 1 0x04 // This should be 0x01
```

Now the nRF52 can be connected over USB and programmed with the Arduino IDE.

<!-- pi@raspberrypi3:~ $ sudo openocd -f interface/raspberrypi2-native.cfg -c "transport select swd" -f target/nrf52.cfg -d2 -c init -c "reset init" -c halt -c "nrf5 mass_erase" -c "program espruino.hex verify" -c reset -c exit -->

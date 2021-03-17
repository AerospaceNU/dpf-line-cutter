# requires adafruit-circuitpython-ble
import time

from adafruit_ble import BLEConnection, BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

ble = BLERadio()
csv = open("out.txt", "w")
while True:
    # Iterate over every bluetooth connection, finding only UART ones
    start = time.time()
    while ble.connected and any(
        UARTService in connection for connection in ble.connections
    ):
        for connection in ble.connections:
            if UARTService not in connection:
                continue
            uart: UARTService = connection[UARTService]
            # If we have data waiting, decode it and write it to a file
            if uart.in_waiting > 0:
                line = uart.readline().decode()
                # print(line, end="")
                now = time.time() - start
                csv.write("{},{}".format(now, line))
                csv.flush()
        time.sleep(0.1)

    # This code is run initially. I think it connects to the first device with BLEUart?
    print("disconnected, scanning")
    for advertisement in ble.start_scan(ProvideServicesAdvertisement, timeout=1):
        if UARTService not in advertisement.services:
            continue
        ble.connect(advertisement)
        print("connected")
        break
    ble.stop_scan()
import serial
import csv
import time

ser = serial.Serial("COM3", baudrate = 9600, timeout = None)

data = []

file = open('Baro1.csv', 'r', newline='\n')
reader: csv.Dialect = csv.reader(file)
for idx, line in enumerate(reader):
    if idx % 20 == 0:
        line = list(map(lambda element : float(element), line))
        data.append(line)

for line in data:
    ser.write('{},{}\n'.format(line[1], line[2]))
    time.sleep(1.0 / 50.0)

ser.close()
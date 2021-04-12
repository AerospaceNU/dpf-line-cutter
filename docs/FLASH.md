# Flash procedure
The line cutter has 16MiB of flash memory divided into 64 sectors of 256kiB each. The first sector is reserved for metadata, including flight variables and a record of which sectors have been written to.

## Writing metadata
Since there are 64 sectors, the first 64 bytes are used to keep track of which sectors are not empty. The rest of the first sector is used for other metadata such as flight information. When a line cutter is turned on, it will use the first 64 bytes to determine the last non-empty sector. Then, it will conduct a linear search of this sector to find the first location with no data, and pick up logging data at that location. When the current sector fills up, the next byte of location metadata is updated. The main advantages of this system are that there are no sector erases (which take ~500ms, or 10 missed data points), and that if a line cutter restarts it is fairly simple to find the first empty flash location.

The line cutter currently logs 64B of data at 20 Hz. At this rate, about 3.5 hours of data can be recorded. After the flash is full, no more data is logged. Leading up to a launch, the flash should be bulk erased before the main program is uploaded, after which the line cutter should be connected to power as little as possible to make sure it doesn't run out of memory.

## Flash contents
| Address (start -> end) | Contents |
| -----------------------| -------- |
| 0000000h -> 000003Fh | Whether the corresponding sector is empty. |
| 0000040h -> 003FFFFh | Additional metadata. |
| 0040000h -> 0FFFFFFh | Flight data. |

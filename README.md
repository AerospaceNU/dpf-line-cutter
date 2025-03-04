# Line Cutter: For Cutting Lines
The line cutter is a system for deploying a single parachute as both drogue and main. Reefing lines are used to constrict the parachute to two intermediate sizes, and these lines are threaded through line cutters. At the set altitudes (or after a time delay), nichrome wires burn through the reefing lines.

## Organization
* `code`: Line cutter code and flight data analyses.
  * `launch archive`: Raw flight data and summaries of successes/failures from past line cutter flights. Subfolders should be named with ISO date and launch site.
  * `libraries and drivers`: Should be obsolete soon (drivers etc. will be published as Arduino libraries).
    * `openocd_windows.zip`: Windows build of openOCD with nrf52940 bootloader and cheap st-link config packaged.
  * `tests`: Code that is useful for testing various line cutter features across hardware versions.
  * `v1`: Code for v1 perfboard line cutters.
  * `v2`: Code for mixed perfboard and PCB line cutters.
  * `v3`: Code for current PCB line cutters.
    * `bulk_erase`: Erases all data on flash chip.
    * `flash_reader`: Reads all data on flash chip.
    * `flight_variable_upload`: Sets parameters for flight.
    * `identifier`: Names PCB for Bluetooth advertising.
    * `InternalFS_format`: Erases files on NRF52840 and formats filesystem.
    * `line_cutter_v3`: Main program that runs on flights.
    * `memory_reader`: Reads state transition log.
* `docs`: More detailed documentation, see below for list.
* `schematic`: ECAD files for the line cutter hardware. Contains schematics for all line cutter hardware. Folder structure is described in the [readme](schematic/README.md)

## Documentation
Code documentation applies to the latest version (v1/v2 are not compatible with current line cutters).
* [`CHECKLIST`](docs/CHECKLIST.md): Pre-flight checklist for line cutter launches.
* [`FLASH`](docs/FLASH.md): Procedure for logging data to flash on line cutters.
* [`SIGNALS`](docs/SIGNALS.md): Description of current board.
* [`STATES`](docs/STATES.md): Description of state machine.
* [`nrf52_openocd`](docs/nrf52_openocd.md): Instructions for programming NRF52 chips with OpenOCD.

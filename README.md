# Line Cutter: For Cutting Lines
The line cutter is a system for deploying a single parachute as both drogue and main. Reefing lines are used to constrict the parachute to two intermediate sizes, and these lines are threaded through line cutters. At the set altitudes (or after a time delay), nichrome wires burn through the reefing lines.

## Organization
* `code`: Line cutter code and flight data analyses.
  * `launch archive`: Raw flight data and summaries of successes/failures from past line cutter flights. Subfolders should be named with ISO date and launch site.
  * `libraries and drivers`: Should be obsolete soon (drivers etc. will be published as Arduino libraries).
  * `tests`: Code that is useful for testing various line cutter features across hardware versions.
  * `v1`: Code for v1 perfboard line cutters.
  * `v2`: Code for mixed perfboard and PCB line cutters.
  * `v3`: Code for current PCB line cutters.
* `docs`: More detailed documentation, see below for list.
* `schematic`:

## Documentation
Code documentation applies to the latest version (v1/v2 are not compatible with current line cutters).
* `FLASH`: Procedure for logging data to flash on line cutters.
* `SIGNALS`: Description of current board.
* `STATES`: Description of state machine.
* `nrf52_openocd`: Instructions for programming NRF52 chips with OpenOCD.

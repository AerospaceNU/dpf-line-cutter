# Line Cutter: For Cutting Lines
Code and schematics for the line cutter.

## Organization
* `code`: Line cutter code and flight data analyses.
  * `launch archive`: Raw flight data and summaries of successes/failures from past line cutter flights. Subfolders should be named with ISO date and launch site.
  * `libraries and drivers`: Should be obsolete soon (drivers etc. will be published as Arduino libraries).
  * `tests`: Code that is useful for testing various line cutter features across hardware versions.
  * `v1`: Code for v1 perfboard line cutters.
  * `v2`: Code for mixed perfboard and PCB line cutters.
  * `v3`: Code for current PCB line cutters.
* `schematic`
* `docs`: More detailed documentation, see below for list.

## Documentation
Code documentation applies to the latest version (v1/v2 are not compatible with current line cutters).
* `nrf52_openocd`
* `SIGNALS`

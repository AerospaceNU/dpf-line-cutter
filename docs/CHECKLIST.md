This is the pre-flight checklist that should be followed for the marman clamp line cutter launch. It mostly covers software-side preparations, unlike the normal line cutter checklist which also goes through the physical setup necessary for a line cutter launch. Read the checklist once before carrying out any of it to ensure you have an overall picture of the process.

5 days before launch:
- [ ] Make a local copy of the `dpf-line-cutter` repository, then switch to the `marmot` branch. Use `git log` to check the commit history; the most recent ones should be by trholdridge if you're on the right branch
- [ ] Check that you can upload code to all of the line cutters that will be used (the [flash reader](https://gitlab.com/aeronu/dollar-per-foot/dpf-line-cutter/-/tree/marmot/code/v3/flash_reader) is a good test)
- [ ] Download the Bluefruit Connect app for confirming data and flight variables day-of over bluetooth

1 day before launch:
- [ ] Charge all batteries (and spares)
- [ ] Erase the flash memory on the PCBs using the [bulk erase](https://gitlab.com/aeronu/dollar-per-foot/dpf-line-cutter/-/tree/marmot/code/v3/bulk_erase) code (note: it will take about 30 seconds and then print a confirmation message)
- [ ] Decide on and upload [flight variables](https://gitlab.com/aeronu/dollar-per-foot/dpf-line-cutter/-/tree/marmot/code/v3/flight_variable_upload). The comments explain what each one is used for in this version of the code
- [ ] Upload the [flight code](https://gitlab.com/aeronu/dollar-per-foot/dpf-line-cutter/-/tree/marmot/code/v3/line_cutter_v3), then quickly open the serial monitor to see confirmation messages. Disconnect the line cutter shortly after to avoid using up the flash memory (it can store about 2.5 hours of data)

On launch day:
- [ ] Plug in the batteries once you have a reasonable expectation that the flash memory won't run out (< 2.5 hours before launch)
- [ ] While the line cutters are still accessible, connect to them with Bluefruit Connect; use `!vars` and `!data` over UART to check for expected values from both line cutters

On the rail:
- [ ] Use `!data` for final confirmation that the state is `WAITING` on both line cutters
- [ ] Launch!

After landing:
- [ ] Take photos of both line cutters and visually confirm that both PWMs have occurred
- [ ] Unplug from batteries when convenient to cut down on the amount of irrelevant data at the end of the flash memory
- [ ] Once you're back at the lab, follow the [flash offload](https://gitlab.com/aeronu/dollar-per-foot/dpf-line-cutter/-/blob/marmot/docs/OFFLOAD.md) instructions to read and graph flight data
- [ ] Report any potential issues with or damage to the individual line cutters

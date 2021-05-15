This is the pre-flight checklist that should be followed for every line cutter launch. Detailed instructions for setting up the line cutters are elsewhere; read those first. Consider any time limits mentioned to be a lower bound---completing items *earlier* than the listed time is encouraged, *on-time* is acceptable, and *after* will result in a rushed (read: risky) process. Keep in mind that although the checklist only covers line cutter prep, it implicitly requires that the rest of your rocket is also prepared in a timely manner.

14 days before launch:
- [ ] Ensure that you have all materials needed for setup (if you need to ship more stuff, it's important to know this early)

6 days before launch:
- [ ] Sew outer cases and loops onto your parachute
- [ ] Make a local copy of the `dpf-line-cutter` repository

3 days before launch:
- [ ] Assemble the cases (including nichrome)
- [ ] Decide on reefing amounts and reef the parachute
- [ ] Ejection test with the assembled cases

1 day before launch:
- [ ] Charge all batteries (and spares)
- [ ] Put your PCBs in the cases
- [ ] Erase the flash memory on the PCBs
- [ ] Decide on and upload flight variables
- [ ] Upload flight code

On launch day:
- [ ] Plug in the batteries once you have a reasonable expectation that the PCBs won't run out of memory (< 3 hours before launch)
- [ ] Immediately use `!vars` and `!data` to check for expected values from both line cutters
- [ ] Put (preferably black) etape over LEDs
- [ ] Burrito your parachute and put in your rocket, but don't screw things together yet
- [ ] Use `!data` to check that the photoresistors are reading below ~200 on both line cutters
- [ ] Finish assembling rocket

On the rail:
- [ ] Use `!data` for final confirmation that photoresistors are reading below ~200 and state is `WAITING` on both line cutters
- [ ] Arm both line cutters with `!arm`
- [ ] Use `!data` to confirm that both line cutters are in the `ARMED` state
- [ ] *Optional: if the rocket needs to be taken off the rail for any reason, you should `!disarm` both line cutters and confirm using `!data`*
- [ ] Launch!

After landing:
- [ ] Take photos of both line cutters when you recover your rocket
- [ ] Visually confirm that both lines have disreefed
- [ ] Once you're back at the lab, follow the flash offload instructions to get flight data
- [ ] Send data to the line cutter team
- [ ] Report any potential issues with or damage to the individual line cutters
- [ ] Disassemble and return components

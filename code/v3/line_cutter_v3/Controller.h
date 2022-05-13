/*
 * A header for the Controller class
 * All controllers for the line cutter must be instances of, at a minimum, the Controller class.
 * Individual Controllers are welcome to add additional functionality, but doing so is not required.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "DataStructures.h"
#include <Arduino.h>

// These instructions are the possible actions that a Controller can instruct the line cutter to take.
// The line cutter does not verify that an Instruction it receives is valid. Controllers are responsible for
// only returning valid Instructions given the state of the line cutter
enum Instruction {
  NO_CHANGE,  // Instructs the line cutter to take no action
  BEGIN_CUT_1,
  BEGIN_CUT_2,
  END_CUT_1,
  END_CUT_2,
  DISARM  // Disarms the line cutter. The line cutter must be rearmed over bluetooth before any further actions can happen
};

// An abstract base for the Controller class. All line cutter controllers must extend the Controller class or implement the methods below
class Controller {
  public:
    Controller();  // The default constructor

    // getInstruction is called periodically by the line cutter. It recieves the current state of the line cutter and returns an
    // Instruction that the board will perform. This method is only called while the line cutter is armed
    virtual Instruction getInstruction(LineCutterConfig flightVars, LineCutterState state, unsigned long loopStart);
    virtual uint8_t getStateNum();  // Gets the current state of the Controller. This identifier is saved in the flash memory

};
#endif

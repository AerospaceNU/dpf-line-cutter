// A basic controller for the line cutter

#include "Controller.h"
#include <Arduino.h>  // It is necessary to explicitly include Arduino.h because this is not the main .ino file

// Defines the internal states of this Controller
enum State {
  ARMED,  // Rocket has yet to launch or is still ascending
  DEPLOYED,  // Parachute has deployed but no lines have been cut
  PARTIAL_DISREEF,  // After first line is cut
  FULL_DISREEF  // After second line is cut
};

// Declares variables that describe the state of the controller
unsigned lastDark;  // The time when the board was last in the dark. Used to check for parachute deployment
unsigned long lastStateChange;
unsigned long startCut1;
bool finishedCut1;
unsigned long startCut2;
bool finishedCut2;
State controllerState;

// Pre-declaration of the checkEndPWM function
Instruction checkEndPWM(unsigned long loopStart, LineCutterConfig flightConfig);

// Initializes variables when a new Controller is created
Controller::Controller() {
  lastDark = 0xffffffff;  // 0xffffffff is the maximum value for an unsigned long
  startCut1 = 0xffffffff;
  startCut2 = 0xffffffff;
  controllerState = ARMED;
  lastStateChange = millis();
  finishedCut1 = false;
  finishedCut2 = false;
}

// A method called by the line cutter to find out what actions it should take
Instruction Controller::getInstruction(LineCutterConfig flightConfig, LineCutterState state, unsigned long loopStart) {
  // The potential actions depend on the current state of the controller
  switch (controllerState) {
    // No lines have been cut yet and the parachute is still closed. The rocket is either on the launch rail
    // or ascending
    case ARMED:
      // If it line cutter board is still dark, it must be inside the rocket. Note the current time as the last time the
      // board was dark
      if (state.photoresistor < flightConfig.lightThreshold) {
        lastDark = loopStart;

      // If it has been at least flightConfig.lightTriggerTime since the board was inside the rocket, the parachute must have
      // deployed. Advance the state to note the deployment
      } else if (loopStart - lastDark >= flightConfig.lightTriggerTime) {
        controllerState = DEPLOYED;
        lastStateChange = loopStart;
      }

      // The line cutter will never have to take an action while the rocket is in the ARMED State
      return NO_CHANGE;

    // The parachute has deployed but none of the lines have been cut yet
    case DEPLOYED:
      // Checks if it is time to start cutting the first line
      if (state.avgAltitude < flightConfig.altitude1 || loopStart - lastStateChange > flightConfig.disreefDelay1) {
        // Note that the first line will be cut
        lastStateChange = loopStart;
        controllerState = PARTIAL_DISREEF;

        // Instruct the line cutter to start the first cut
        return BEGIN_CUT_1;  
      }

      // The line cutter does not need to take any action because it is not time to cut the first line
      return NO_CHANGE;
   
    // The first line has started being cut. It may still be getting cut, or the cutting may have stopped
    case PARTIAL_DISREEF:
      // Checks if it is time to start cutting the second line
      if (state.avgAltitude < flightConfig.altitude2 || loopStart - lastStateChange > flightConfig.disreefDelay2) {
        // Note that the second line will be cut
        lastStateChange = loopStart;
        controllerState = FULL_DISREEF;
        
        // Instruct the line cutter to start cutting the second cut
        return BEGIN_CUT_2;  
      }

      // If it is not time to start cutting the second line, check if it is time to stop cutting the first line
      return checkEndPWM(loopStart, flightConfig);

    // Both lines have started getting cut, and at least one line is still getting cut. Check if it is time
    // to stop a line from getting cut. If no lines are getting cut, disarm the board because the flight
    // is over
    case FULL_DISREEF:
      // Disarm the board if both lines are cut
      if (finishedCut1 && finishedCut2) {
        return DISARM;
      }

      // Checks whether it it is time to end either of the cuts
      return checkEndPWM(loopStart, flightConfig);
    
    // If none of the states match, something has gone terribly wrong
    default:
      return NO_CHANGE;
  }
}

// Returns a number corresponding to the current state of the controller to be saved in the flash memory
uint8_t Controller::getStateNum() {
  return (uint8_t) controllerState;
}

// Checks if it time to stop cutting the lines
Instruction checkEndPWM(unsigned long loopStart, LineCutterConfig flightConfig) {
  // It is time to stop cutting the first line
  if (!finishedCut1 && loopStart - startCut1 > flightConfig.pwmDuration) {
    finishedCut1 = true;
    return END_CUT_1;

  // It is time to stop cutting the second line
  } else if (!finishedCut2 && loopStart - startCut2 > flightConfig.pwmDuration) {
    finishedCut2 = true;
    return END_CUT_2;

  // Any lines currently being cut should continue to get cut
  } else {
    return NO_CHANGE;
  }
}

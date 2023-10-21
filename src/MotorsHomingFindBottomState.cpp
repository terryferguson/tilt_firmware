#include "MotorsHomingFindBottomState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

void MotorsHomingFindBottomState::enter() {
  controller->resetSoftMovement();
  controller->resetCurrentInformation();
  controller->clearPositionChange();

  hasTransition = false;

  Serial.println("-----------------------------------------------------------");
  Serial.println("|              Entering Homing Find Bottom State          |");
  Serial.println("-----------------------------------------------------------");
}

/**
 * Updates the MotorsHomingFindBottomState.
 *
 * @throws ErrorType description of error
 */
void MotorsHomingFindBottomState::update() {

  // Check if the motors have stopped
  if (controller->motorsStopped()) {
    // Print message to serial monitor
    Serial.println("Bottom Found.");

    // Set transition flag to true
    hasTransition = true;

    // Set next state type to MOTORS_HOMING_BOTTOM_FOUND_STATE
    nextStateType = MOTORS_HOMING_BOTTOM_FOUND_STATE;
  }

  // Update the motors
  controller->updateMotors();
}

void MotorsHomingFindBottomState::leave() {
  controller->resetSoftMovement();
  controller->resetCurrentInformation();
  controller->resetMotorCurrentAlarms();
  controller->clearPositionChange();

  hasTransition = false;

  Serial.println("-----------------------------------------------------------");
  Serial.println("|               Leaving Homing Find Bottom State          |");
  Serial.println("-----------------------------------------------------------");
}

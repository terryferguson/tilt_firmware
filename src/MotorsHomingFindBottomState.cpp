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

void MotorsHomingFindBottomState::update() {

  if (controller->motorsStopped()) {
    Serial.println("Bottom Found.");

    hasTransition = true;
    nextStateType = MOTORS_HOMING_BOTTOM_FOUND_STATE;
  }

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

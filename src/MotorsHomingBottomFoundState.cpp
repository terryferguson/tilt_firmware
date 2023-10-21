#include "MotorsHomingBottomFoundState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

void MotorsHomingBottomFoundState::enter() {
  controller->resetSoftMovement();
  controller->resetCurrentInformation();

  hasTransition = false;

  Serial.println("-----------------------------------------------------------");
  Serial.println("|              Entering Homing Bottom Found State         |");
  Serial.println("-----------------------------------------------------------");

  controller->zero();
  controller->speed = MIN_MOTOR_TRAVEL_SPEED;
  controller->setPos(ALARM_REVERSE_AMOUNT);
  controller->systemDirection = controller->requestedDirection =
      Direction::EXTEND;
  controller->extend();
}

void MotorsHomingBottomFoundState::update() {
  hasTransition = true;
  nextStateType = MOTORS_STARTING_STATE;
  // Check if the set position has been reached
  controller->checkIfSetPositionReached();
  controller->handlePid();
  controller->updateMotors();
}

void MotorsHomingBottomFoundState::leave() {
  hasTransition = false;
  controller->resetSoftMovement();
  controller->resetCurrentInformation();
  controller->resetMotorCurrentAlarms();
  controller->zero();
  controller->clearHoming();
  controller->speed = DEFAULT_MOTOR_SPEED;

  Serial.println("-----------------------------------------------------------");
  Serial.println("|               Leaving Homing Bottom Found State         |");
  Serial.println("-----------------------------------------------------------");
}
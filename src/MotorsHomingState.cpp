#include "MotorsHomingState.hpp"
#include "MotorController.hpp"
#include "defs.hpp"
#include <Arduino.h>

void MotorsHomingState::enter() {
  pid_on = true;
  hasTransition = false;
  controller->resetSoftMovement();
  controller->resetCurrentInformation();
  controller->resetMotorCurrentAlarms();

  controller->speed = MIN_MOTOR_TRAVEL_SPEED;
  controller->hardSetSpeed();
  controller->setBottomCurrentLimit(CURRENT_LIMIT);
  controller->setHoming();
  currentAlarmStart = micros();
  controller->retract();

  Serial.println("-----------------------------------------------------------");
  Serial.println("|                  Entering Start Homing State            |");
  Serial.println("-----------------------------------------------------------");
}

void MotorsHomingState::update() {
  const auto currentTime = micros();
  const auto elapsedTime = currentTime - currentAlarmStart;

  if (elapsedTime > HOMING_CURRENT_ALARM_DELAY) {
    hasTransition = true;
    nextStateType = MOTORS_HOMING_FIND_BOTTOM_STATE;
  }

  controller->updateMotors();
}

void MotorsHomingState::leave() {
  controller->setBottomCurrentLimit(HOMING_CURRENT_LIMIT);
  hasTransition = false;
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                   Leaving Start Homing State            |");
  Serial.println("-----------------------------------------------------------");
}
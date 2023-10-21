// clang-format off
#include "MotorsStoppedState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the stopped state.
 *
 * @return void
 */
void MotorsStoppedState::enter() {
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                    Entering Stopped State               |");
  Serial.println("-----------------------------------------------------------");

  hasTransition = false;

  if (nullptr != controller) {
    // Reset current information
    controller->resetCurrentInformation();

    // Reset the current alarms
    controller->resetMotorCurrentAlarms();

    // Reset soft movement
    controller->resetSoftMovement();

    // Clear timestamps for position changes
    controller->clearPositionChange();

    // Set directions to stop
    controller->requestedDirection = Direction::STOP;
    controller->systemDirection = Direction::STOP;

    // Reset speeds
    controller->speed = 0;
    controller->targetSpeed = -1;

    // Disable motors
    controller->disableMotors();
  } else {
    Serial.println("Motors Stopped State - No controller");
  }
}

/**
 * @brief Update the motor control system in stopped state
 *
 * @return void
 */
void MotorsStoppedState::update() { controller->updateMotors(); }

/**
 * @brief Handle leaving the stopped state
 *
 * @return void
 */
void MotorsStoppedState::leave() {
  hasTransition = false;
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                     Leaving Stopped State               |");
  Serial.println("-----------------------------------------------------------");
}
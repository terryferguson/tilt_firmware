#include "MotorsStoppedState.hpp"
#include "MotorController.hpp"
#include "debugging.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the stopped state.
 *
 * @return void
 */
void MotorsStoppedState::enter() {
  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                    Entering Stopped State               |");
    Serial.println(
        "-----------------------------------------------------------");
  }

  hasTransition = false;

  if (nullptr != controller) {
    // Reset current information
    controller->resetCurrentInformation();
    controller->alarmTriggered = false;

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
    controller->intermediateSpeed = -1;

    // Reset desired position
    controller->resetDesiredPosition();

    // We're not moving, so there's no move start time. Set to the invalid
    // sentinel value
    controller->moveStart = -1;

    // Disable motors
    controller->disableMotors();

    // Save the motor positions and system state
    systemState.SaveMotorPostions();

    controller->report();
  } else {
    DebugPrintln("Motors Stopped State - No controller");
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
  controller->resetCurrentInformation();
  controller->alarmTriggered = false;

  // Reset the current alarms
  controller->resetMotorCurrentAlarms();

  // Reset soft movement
  controller->resetSoftMovement();

  // Clear timestamps for position changes
  controller->clearPositionChange();

  // We're not moving, so there's no move start time.
  controller->moveStart = 0UL;

  // Reset speeds
  controller->speed = 0;
  controller->targetSpeed = -1;
  controller->intermediateSpeed = -1;

  // Save the motor positions and system state
  systemState.SaveMotorPostions();

  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                     Leaving Stopped State               |");
    Serial.println(
        "-----------------------------------------------------------");
  }
}
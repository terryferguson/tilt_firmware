#include "MotorsMovingState.hpp"
#include "MotorController.hpp"
#include "defs.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the moving state.
 *
 * @return void
 */
void MotorsMovingState::enter() {
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                      Entering Moving State              |");
  Serial.println("-----------------------------------------------------------");
  hasTransition = false;
}

/**
 * @brief Update the motor control system in the moving state.
 *
 * This function updates the motor control system by performing various checks
 * and actions. It checks if the motors are close to the end of the range and
 * sets the speed accordingly. It also checks if the current time has exceeded
 * the current alarm delay and handles the current alarm if triggered.
 * Finally, it checks if the set position has been reached, handles the PID
 * control, and updates the motors.
 *
 * @return void
 */
void MotorsMovingState::update() {
  // Check if a valid controller object exists
  if (nullptr != controller) {
    // Store the relevant variables from the controller object
    const auto speed = controller->speed;
    const auto targetSpeed = controller->targetSpeed;
    const auto currentTime = micros();
    const auto moveStart = controller->moveStart;
    const auto currentAlarmSet = controller->currentAlarmSet;

    if (controller->motorsStopped()) {
      hasTransition = true;
      nextStateType = MOTORS_STOPPED_STATE;
    } else {
      // Check if the motors are close to the end of the range of movement and
      // are not in a soft-stop, and aren't ramping up/down, then set the speed
      // to end of range speed
      if (controller->motorsCloseToEndOfRange() &&
          speed != MOTOR_END_OF_RANGE_SPEED && targetSpeed < 0) {
        hasTransition = true;
        nextStateType = MOTORS_END_OF_RANGE_STATE;
      }

      // Check if the current time has exceeded the current alarm delay
      if ((currentTime - moveStart) >= CURRENT_ALARM_DELAY) {
        // Check if the current alarm is not set, and set the current limit
        if (!currentAlarmSet) {
          controller->setCurrentLimit();
        } else {
          // Check if the current alarm has been triggered, and handle the alarm
          if (controller->currentAlarmTriggered()) {
            controller->handleCurrentAlarm();
          }
        }
      } else {
        controller->handleCurrentUpdate();
      }

      // Check if the set position has been reached
      controller->checkIfSetPositionReached();

      controller->updateLeadingAndLaggingIndicies();

      // Handle the PID control
      controller->handlePid();

      // Update the motors
      controller->updateMotors();
    }
  } else {
    // Print error message if no controller object exists
    Serial.println("MotorsMovingState - No controller");
  }
}

/**
 * @brief Handle leaving the moving state
 *
 * @return void
 */
void MotorsMovingState::leave() {
  hasTransition = false;
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                      Leaving Moving State               |");
  Serial.println("-----------------------------------------------------------");
}
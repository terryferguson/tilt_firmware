#include "MotorsMovingState.hpp"
#include "MotorController.hpp"
#include "debugging.hpp"
#include "defs.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the moving state.
 *
 * @return void
 */
void MotorsMovingState::enter() {
  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                      Entering Moving State              |");
    Serial.println(
        "-----------------------------------------------------------");
  }

  hasTransition = false;
  enteredStateTime = micros();
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

    if (controller->motorsStopped()) {
      DebugPrintln("MotorsMovingState - Motors stopped");
      hasTransition = true;
      nextStateType = MOTORS_STOPPED_STATE;

    } else {
      // Check if the motors are close to the end of the range of movement and
      // are not in a soft-stop, and aren't ramping up/down, then set the
      // speed to end of range speed
      if (controller->motorsCloseToEndOfRange() &&
          speed != MOTOR_END_OF_RANGE_SPEED && targetSpeed < 0) {
        hasTransition = true;
        nextStateType = MOTORS_END_OF_RANGE_STATE;
      }

      controller->handleCurrentUpdate();

      // Check if the set position has been reached
      controller->checkIfSetPositionReached();

      // Check if we are close to the set position
      if (controller->motorsNearDesiredPosition()) {
        hasTransition = true;
        nextStateType = MOTORS_STOPPING_STATE;
      }

      controller->updateLeadingAndLaggingIndicies();

      // Handle the PID control
      controller->handlePid();

      // Update the motors
      controller->updateMotors();
    }
  } else {
    // Print error message if no controller object exists
    DebugPrintln("MotorsMovingState - No controller");
  }
}

/**
 * @brief Handle leaving the moving state
 *
 * @return void
 */
void MotorsMovingState::leave() {
  hasTransition = false;
  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                      Leaving Moving State               |");
    Serial.printf("|                Elapsed Time: %6d ms                  |\n",
                  elapsedTime() / 1000);
    Serial.println(
        "-----------------------------------------------------------");
  }
}
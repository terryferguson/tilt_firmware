// clang-format off
#include "MotorsStartingState.hpp"
#include "MotorController.hpp"
#include "defs.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the moving state.
 *
 * @return void
 */
void MotorsStartingState::enter() {
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                    Entering Starting State              |");
  Serial.println("-----------------------------------------------------------");

  hasTransition = false;

  if (nullptr != controller) {
    // Reset PID controller
    controller->resetPid();

    // Reset soft movement information
    controller->resetSoftMovement();

    // Reset current calibration information
    controller->resetCurrentInformation();

    // Reset the current alarms for the motors
    controller->resetMotorCurrentAlarms();

    // Set travel speed to the minimum
    controller->speed = MIN_MOTOR_TRAVEL_SPEED;

    // Save the start time of the motors
    controller->moveStart = micros();

    enteredStateTime = micros();
    hasTransition = false;
  } else {
    Serial.println("MotorsStoppingState - No controller");
  }
}

/**
 * @brief Update the motor control system in starting state
 *
 * This function updates the motor control system when the motors are in the
 * starting state. It checks if the controller is not null, and then performs
 * various actions related to motor control. If the controller is null, it
 * prints an error message.
 *
 * @return void
 */
void MotorsStartingState::update() {
  // Check if the controller is not null
  if (nullptr != controller) {
    const auto speed = controller->speed;
    const auto targetSpeed = controller->targetSpeed;

    // We stay here for only one system cycle, so we immediately queue
    // a transition to the next state
    hasTransition = true;

    // If the motors are halted, then stop
    if (controller->motorsStopped()) {
      Serial.printf("MotorsStartingState - Motors are halted. Stopping.\n");
      hasTransition = true;
      nextStateType = MOTORS_STOPPED_STATE;
    } else {
      // If the motors are close to the end of the range of movement and are not
      // in a soft-stop, and aren't ramping up/down, then set the speed to end
      // of range speed
      if (controller->motorsCloseToEndOfRange() &&
          speed != MOTOR_END_OF_RANGE_SPEED && targetSpeed < 0) {
        nextStateType = MOTORS_END_OF_RANGE_STATE;
      } else {
        nextStateType = MOTORS_SOFT_MOVEMENT_STATE;
      }

      // Check if the set position has been reached
      controller->checkIfSetPositionReached();

      // Sample the currents
      controller->sampleCurrents();

      controller->updateLeadingAndLaggingIndicies();

      // Handle the PID control
      controller->handlePid();

      // Update the motors
      controller->updateMotors();
    }
  } else {
    // Print an error message if the controller is null
    Serial.println("MotorsStartingState - No controller");
  }
}

/**
 * @brief Handle leaving the starting state
 *
 * @return void
 */
void MotorsStartingState::leave() {
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                     Leaving Starting State              |");
  Serial.printf("|                 Elapsed Time: %6d ms                 |\n",
                elapsedTime() / 1000);
  Serial.println("-----------------------------------------------------------");
  hasTransition = false;
  // State transition may cause sufficient lag to cause system to falsely think
  // motors have stopped, so reset timestamps
  controller->clearPositionChange();
  enteredStateTime = 0;
}
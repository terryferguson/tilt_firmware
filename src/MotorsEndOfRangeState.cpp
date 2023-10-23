#include "MotorsEndOfRangeState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

/**
 * @brief Handles entering the end of range state for the motors.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorsEndOfRangeState::enter() {
  // Store the relevant variables from the controller object
  const auto speed = controller->speed;
  const auto targetSpeed = controller->targetSpeed;

  Serial.println("-----------------------------------------------------------");
  Serial.println("|                 Entering End of Range State             |");
  Serial.println("-----------------------------------------------------------");

  enteredStateTime = micros();

  hasTransition = false;

  // Check if the motors are close to the end of the range of movement and
  // are not in a soft-stop, and aren't ramping up/down, then set the speed
  // to end of range speed
  if (nullptr != controller) {
    if (speed != MOTOR_END_OF_RANGE_SPEED) {
      controller->setSpeed(MOTOR_END_OF_RANGE_SPEED, 80);
    } else {
      Serial.println("MotorsEndOfRangeState - No controller");
    }
  }
}

/**
 * @brief Updates the motors in the end of range state.
 *
 * @throws None
 */
void MotorsEndOfRangeState::update() {
  // Check if the controller object exists
  if (nullptr != controller) {
    // Store the relevant variables from the controller object
    const auto speed = controller->speed;
    const auto targetSpeed = controller->targetSpeed;
    const auto currentTime = micros();
    const auto moveStart = controller->moveStart;
    const auto currentAlarmSet = controller->currentAlarmSet;

    // Check if the motors have stopped
    if (controller->motorsStopped()) {
      // Set the flag for state transition
      hasTransition = true;
      // Set the next state type
      nextStateType = MOTORS_STOPPED_STATE;
    } else {
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
      }

      // Update the PWM
      updatePWM();

      // Check if the set position has been reached
      controller->checkIfSetPositionReached();

      // Handle the PID control
      controller->handlePid();

      // Update the motors
      controller->updateMotors();
    }
  } else {
    // Print error message if no controller object exists
    Serial.println("MotorsEndOfRangeState - No controller");
  }
}

/**
 * @brief Updates the pulse-width modulation (PWM) for the MotorsEndOfRangeState
 * controller.
 *
 * @return void
 */
void MotorsEndOfRangeState::updatePWM() {
  // Get the current time in microseconds
  const auto currentTime = micros();

  // Get the target speed for the motors
  const auto targetSpeed = controller->targetSpeed;

  // Calculate the difference between the current speed and the target speed
  const auto speedDelta = abs(controller->speed - targetSpeed);

  // Get the time when the movement started
  const auto moveStart = controller->moveStart;

  // Calculate the time difference between the current time and the movement
  // start time
  const auto moveTimeDelta = currentTime - moveStart;

  // Get the time when the last PWM update occurred
  const auto lastPWMUpdate = controller->lastPWMUpdate;

  // Calculate the time difference between the current time and the last PWM
  // update time
  const auto updateTimeDelta = currentTime - lastPWMUpdate;

  // Get the soft movement time for the controller
  const auto softMovingTime = controller->softMovingTime;

  // Get the PWM update amount for the controller
  const auto pwmUpdateAmount = controller->pwmUpdateAmount;

  // Check if the target speed is positive
  if (targetSpeed > 0) {
    // Check if the movement is still ongoing
    const auto stillUpdating = moveTimeDelta < softMovingTime;

    // Calculate the true PWM update amount based on the actual elapsed time
    const auto truePWMUpdateAmount =
        pwmUpdateAmount * (static_cast<float>(updateTimeDelta) /
                           SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);

    // Check if the speed difference is large enough for the update
    const bool speedDeltaEnough = speedDelta > abs(truePWMUpdateAmount);

    // Check if the movement is still ongoing
    if (stillUpdating) {
      // Check if the speed difference is large enough for the update
      if (speedDeltaEnough) {
        // Calculate the new speed by adding the true PWM update amount to the
        // current speed
        const auto newSpeed = controller->speed + truePWMUpdateAmount;

        // Update the controller's speed with the new speed
        controller->speed = floorf(newSpeed);
      }
    } else {
      // Set the controller's speed to the target speed
      controller->speed = targetSpeed;
    }
  }
}

/**
 * @brief Handles leaving the of the end of range state for the motors.
 *
 * @return void
 *
 * This function sets the hasTransition variable to false and prints a message
 * to the Serial monitor.
 *
 * @throws None
 */
void MotorsEndOfRangeState::leave() {
  hasTransition = false;
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                 Leaving End of Range State              |");
  Serial.printf("|                    Elapsed Time: %6d ms                  |\n",
                elapsedTime() / 1000);
  Serial.println("-----------------------------------------------------------");
  enteredStateTime = 0;
}

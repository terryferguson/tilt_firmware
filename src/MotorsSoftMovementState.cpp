#include "MotorsSoftMovementState.hpp"
#include "MotorController.hpp"
#include "SystemState.hpp"
#include "debugging.hpp"
#include "defs.hpp"
#include <Arduino.h>

extern SystemState systemState;

/**
 * @brief Handle the system entering the soft movement state.
 *
 * @return void
 */
void MotorsSoftMovementState::enter() {
  // Print separator lines to indicate the start of the soft movement state
  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                  Entering Soft Movement State           |");
    Serial.println(
        "-----------------------------------------------------------");
  }

  // Reset the transition flag to false
  hasTransition = false;

  enteredStateTime = micros();

  // Check if the controller is initialized
  if (nullptr != controller) {
    const auto direction = controller->systemDirection;
    if (Direction::EXTEND == direction) {
      controller->setPidParams(EXTEND_RAMP_KP, 0, 0);
    } else if (Direction::RETRACT == direction) {
      controller->setPidParams(RETRACT_RAMP_KP, 0, 0);
    }

    // Clear the position change in the controller
    controller->clearPositionChange();

    // Set the travel speed to the default motor speed with the soft movement
    // time limit
    controller->setSpeed(systemState.systemSpeed, SOFT_MOVEMENT_TIME_MS);

    // Reset the transition flag to false
    hasTransition = false;

    // Update the motors' positions
    controller->updateMotors();
  } else {
    // Print error message if the controller is not initialized
    DebugPrintln("MotorsSoftMovementState - No controller");
  }
}

/**
 * @brief Update the motor control system in soft movement state
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorsSoftMovementState::update() {
  // Check if the controller is not null
  if (nullptr != controller) {
    // Get the current speed and target speed from the controller
    const auto speed = controller->speed;
    const auto targetSpeed = controller->targetSpeed;

    controller->updateMotors();

    // Check if the motors are stopped
    if (controller->motorsStopped()) {
      DebugPrintln("MotorsSoftMovementState - Motors stopped");
      // Set the transition flag and the next state type
      hasTransition = true;
      nextStateType = MOTORS_STOPPED_STATE;
    } else {
      // Check if the motors are close to the end of the range and the speed is
      // not the end of range speed
      if (controller->motorsCloseToEndOfRange() &&
          speed != MOTOR_END_OF_RANGE_SPEED && targetSpeed < 0) {
        // Set the transition flag and the next state type
        hasTransition = true;
        nextStateType = MOTORS_END_OF_RANGE_STATE;
      }

      // Update the pulse width modulation (PWM)
      updatePWM();

      controller->updateLeadingAndLaggingIndicies();

      // Handle the proportional-integral-derivative (PID) control
      controller->handlePid();

      // Sample the currents
      controller->sampleCurrents();

      // Handle the current update
      controller->handleCurrentUpdate();

      // Check if the set position has been reached
      controller->checkIfSetPositionReached();

      // Update the motors
      controller->updateMotors();
    }
  } else {
    // Print an error message if the controller is null
    DebugPrintln("MotorsSoftMovementState - No controller");
  }
}

/**
 * Updates the PWM of the MotorsSoftMovementState.
 *
 * This function calculates the necessary updates for the PWM based on the
 * current state of the controller and the target speed. It checks if the target
 * speed is greater than -1 and if the movement is still updating. If so, it
 * calculates the true PWM update amount based on the elapsed time and updates
 * the controller's speed accordingly. If the movement is no longer updating, it
 * sets the necessary flags and variables for transitioning to the
 * MOTORS_MOVING_STATE.
 *
 * @return void
 */
void MotorsSoftMovementState::updatePWM() {
  // Get the current time
  const auto currentTime = micros();

  // Get the target speed from the controller
  const auto targetSpeed = controller->targetSpeed;

  // Calculate the absolute difference between the current speed and the target
  // speed
  const auto speedDelta = abs(controller->speed - targetSpeed);

  // Get the move start time from the controller
  const auto moveStart = controller->moveStart;

  // Calculate the time difference between the current time and the move start
  // time
  const auto moveTimeDelta = currentTime - moveStart;

  // Get the last PWM update time from the controller
  const auto lastPWMUpdate = controller->lastPWMUpdate;

  // Calculate the time difference between the current time and the last PWM
  // update time
  const auto updateTimeDelta = currentTime - lastPWMUpdate;

  // Get the soft moving time from the controller
  const auto softMovingTime = controller->softMovingTime;

  // Get the PWM update amount from the controller
  const auto pwmUpdateAmount = controller->pwmUpdateAmount;

  // Check if the target speed is greater than -1
  if (targetSpeed > -1) {
    // Check if the move time is less than the soft moving time
    const auto stillUpdating = moveTimeDelta < controller->softMovingTime;

    // Calculate the true PWM update amount based on the actual elapsed time
    const auto truePWMUpdateAmount =
        pwmUpdateAmount * (static_cast<float>(updateTimeDelta) /
                           SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);

    // Check if the speed difference is greater than the true PWM update amount
    const bool speedDeltaEnough = speedDelta > abs(truePWMUpdateAmount);

    // Check if the move is still updating
    if (stillUpdating) {
      // Check if the speed difference is enough to apply the true PWM update
      // amount
      if (speedDeltaEnough) {
        // Calculate the new speed
        const auto newSpeed = controller->speed + truePWMUpdateAmount;

        // Update the speed with the new value
        controller->speed = floorf(newSpeed);
      }
    } else {
      // Set the speed to the target speed
      controller->speed = targetSpeed;

      // Set the has transition flag to true
      hasTransition = true;

      // Set the next state type to MOTORS_MOVING_STATE
      nextStateType = MOTORS_MOVING_STATE;

      // Reset target speed
      controller->targetSpeed = -1;
    }
  }
}

void MotorsSoftMovementState::leave() {
  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                  Leaving Soft Movement State            |");
    Serial.printf("|                 Elapsed Time: %6d ms                 |\n",
                  elapsedTime() / 1000);
    Serial.println(
        "-----------------------------------------------------------");
  }

  controller->resetSoftMovement();
  hasTransition = false;
  // State transition may cause sufficient lag to cause system to falsely think
  // motors have stopped, so reset timestamps
  controller->clearPositionChange();

  enteredStateTime = 0;
}
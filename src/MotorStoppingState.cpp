// clang-format off
#include "MotorsStoppingState.hpp"
#include "MotorController.hpp"
#include "defs.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the moving state.
 *
 * @return void
 */
void MotorsStoppingState::enter() {
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                    Entering Stopping State              |");
  Serial.println("-----------------------------------------------------------");

  hasTransition = false;

  if (nullptr != controller) {
    // Reset PID controller
    controller->resetPid();

    // Save the start time of the motors
    controller->moveStart = micros();

    // Start the ramp down
    controller->setSpeed(0, SOFT_STOP_TIME_MS);
  } else {
    Serial.println("MotorsStoppingState - No controller");
  }
}

/**
 * @brief Update the motor control system in stopping state
 *
 * This function updates the motor control system when the motors are in the
 * stopping state. It checks if the controller is not null, and then performs
 * various actions related to motor control. If the controller is null, it
 * prints an error message.
 *
 * @return void
 */
void MotorsStoppingState::update() {
  // Check if the controller is not null
  if (nullptr != controller) {
    const auto speed = controller->speed;
    const auto targetSpeed = controller->targetSpeed;
    const auto currentAlarmSet = controller->currentAlarmSet;

    if (controller->motorsStopped()) {
      hasTransition = true;
      nextStateType = MOTORS_STOPPED_STATE;
    } else {
      // If the motors are close to the end of the range of movement and are not
      // in a soft-stop, and aren't ramping up/down, then set the speed to end
      // of range speed
      if (controller->motorsCloseToEndOfRange() &&
          speed != MOTOR_END_OF_RANGE_SPEED && targetSpeed < 0) {
        hasTransition = true;
        nextStateType = MOTORS_END_OF_RANGE_STATE;
      }
    }

    updatePWM();

    // Check if the set position has been reached
    controller->checkIfSetPositionReached();

    // Sample the currents
    controller->sampleCurrents();

    // Handle the PID control
    controller->handlePid();

    // Update the motors
    controller->updateMotors();

  } else {
    // Print an error message if the controller is null
    Serial.println("MotorsStoppingState - No controller");
  }
}

/**
 * @brief Updates the PWM for the MotorsStoppingState.
 *
 * This function calculates the necessary updates for the PWM based on the current
 * state of the controller and the target speed. It checks if the target speed is
 * greater than -1 and if the movement is still updating. If so, it calculates the
 * true PWM update amount based on the elapsed time and updates the controller's
 * speed accordingly. If the movement is no longer updating, it sets the necessary
 * flags and variables for transitioning to the MOTORS_STOPPED_STATE and halts the
 * controller immediately.
 *
 * @throws None
 */
void MotorsStoppingState::updatePWM() {
  // Get the current time
  const auto currentTime = micros();
  
  // Get the target speed
  const auto targetSpeed = controller->targetSpeed;
  
  // Calculate the difference between the current speed and target speed
  const auto speedDelta = abs(controller->speed - targetSpeed);
  
  // Get the time when the movement started
  const auto moveStart = controller->moveStart;
  
  // Calculate the time difference between the current time and move start time
  const auto moveTimeDelta = currentTime - moveStart;
  
  // Get the time when the last PWM update occurred
  const auto lastPWMUpdate = controller->lastPWMUpdate;
  
  // Calculate the time difference between the current time and last PWM update time
  const auto updateTimeDelta = currentTime - lastPWMUpdate;
  
  // Get the soft moving time
  const auto softMovingTime = controller->softMovingTime;
  
  // Get the PWM update amount
  const auto pwmUpdateAmount = controller->pwmUpdateAmount;

  // Check if the target speed is greater than -1
  if (targetSpeed > -1) {
    // Check if the movement is still updating
    const auto stillUpdating = moveTimeDelta < softMovingTime;

    // Get the true PWM update amount based on the actual elapsed time
    const auto truePWMUpdateAmount =
        pwmUpdateAmount * (static_cast<float>(updateTimeDelta) /
                           SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);
    
    // Check if the speed difference is large enough for the update
    const bool speedDeltaEnough = speedDelta > abs(truePWMUpdateAmount);

    // Check if the movement is still updating
    if (stillUpdating) {
      // Check if the speed difference is large enough for the update
      if (speedDeltaEnough) {
        // Calculate the new speed and update the controller's speed
        const auto newSpeed = controller->speed + truePWMUpdateAmount;
        controller->speed = floorf(newSpeed);
      }
    } else {
      // Set the hasTransition flag to true
      hasTransition = true;
      
      // Set the nextStateType to MOTORS_STOPPED_STATE
      nextStateType = MOTORS_STOPPED_STATE;
      
      // Set the controller's speed to 0
      controller->speed = 0;

      // Reset target speed
      controller->targetSpeed = -1;
      
      // Call the immediateHalt() function of the controller
      controller->immediateHalt();
    }
  }
}

/**
 * @brief Handle leaving the stopping state
 *
 * @return void
 */
void MotorsStoppingState::leave() {
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                     Leaving Stopping State              |");
  Serial.println("-----------------------------------------------------------");
  hasTransition = false;
  controller->resetSoftMovement();
  controller->resetCurrentInformation();
  controller->resetMotorCurrentAlarms();
}
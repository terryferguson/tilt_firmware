#include "MotorsHomingState.hpp"
#include "MotorController.hpp"
#include "defs.hpp"
#include <Arduino.h>

/**
 * Enter the MotorsHomingState.
 *
 * This function enables the PID controller, resets the transition flag,
 * resets the soft movement data, resets the current information, resets
 * the motor current alarms, sets the motor speed to the minimum travel
 * speed, applies the motor speed, sets the bottom current limit, sets
 * the homing mode, records the current time, retracts the controller,
 * and prints a message indicating the start of the homing state.
 *
 * @throws None
 */
void MotorsHomingState::enter() {
  // Enable the PID controller
  pid_on = true;

  // Reset the transition flag
  hasTransition = false;

  // Reset the soft movement data
  controller->resetSoftMovement();

  // Reset the current information
  controller->resetCurrentInformation();

  // Reset the motor current alarms
  controller->resetMotorCurrentAlarms();

  // Set the motor speed to the minimum travel speed
  controller->speed = MIN_MOTOR_TRAVEL_SPEED;

  // Apply the motor speed
  controller->hardSetSpeed();

  // Set the bottom current limit
  controller->setBottomCurrentLimit(CURRENT_LIMIT);

  // Set the homing mode
  controller->setHoming();

  // Record the current time
  currentAlarmStart = micros();

  // Retract the controller
  controller->retract();

  // Print a message indicating the start of the homing state
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                  Entering Start Homing State            |");
  Serial.println("-----------------------------------------------------------");
}

void MotorsHomingState::update() {
  /**
   * Updates the MotorsHomingState.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  // Get the current time in microseconds
  const auto currentTime = micros();

  // Calculate the elapsed time since the start of the current alarm
  const auto elapsedTime = currentTime - currentAlarmStart;

  // Check if the elapsed time is greater than the delay for the current alarm
  if (elapsedTime > HOMING_CURRENT_ALARM_DELAY) {
    // Set the hasTransition flag to true
    hasTransition = true;
    // Set the nextStateType to MOTORS_HOMING_FIND_BOTTOM_STATE
    nextStateType = MOTORS_HOMING_FIND_BOTTOM_STATE;
  }

  // Update the motors using the controller
  controller->updateMotors();
}

/**
 * This function sets the bottom current limit for MotorsHomingState and
 * resets the transition state flag. It also prints a message indicating that
 * we are leaving the start homing state.
 *
 * @throws ErrorType description of error
 */
void MotorsHomingState::leave() {
  // Set the bottom current limit for the controller
  controller->setBottomCurrentLimit(HOMING_CURRENT_LIMIT);

  // Reset the transition state flag
  hasTransition = false;

  // Print a message indicating that the function is being executed
  Serial.println("-----------------------------------------------------------");
  Serial.println("|                   Leaving Start Homing State            |");
  Serial.println("-----------------------------------------------------------");
}

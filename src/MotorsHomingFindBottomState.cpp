#include "MotorsHomingFindBottomState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

/**
 * @brief Handle the system entering the find bottom homing state.
 *
 * @return void
 */
void MotorsHomingFindBottomState::enter() {
  controller->resetSoftMovement();
  controller->resetCurrentInformation();
  controller->clearPositionChange();

  hasTransition = false;

  enteredStateTime = micros();

  Serial.println("-----------------------------------------------------------");
  Serial.println("|              Entering Homing Find Bottom State          |");
  Serial.println("-----------------------------------------------------------");
}

/**
 * @brief Update the motor control system in the find bottom homing state.
 *
 * @return void
 */
void MotorsHomingFindBottomState::update() {

  // Check if the motors have stopped
  if (controller->motorsStopped()) {
    // Print message to serial monitor
    Serial.println("Bottom Found.");

    // Set transition flag to true
    hasTransition = true;

    // Set next state type to MOTORS_HOMING_BOTTOM_FOUND_STATE
    nextStateType = MOTORS_HOMING_BOTTOM_FOUND_STATE;
  }

  // Update the motors
  controller->updateMotors();
}

/**
 * @brief Handle leaving the find bottom homing state
 *
 * @return void
 */
void MotorsHomingFindBottomState::leave() {
  // Reset the soft movement of the controller
  controller->resetSoftMovement();

  // Reset the current information of the controller
  controller->resetCurrentInformation();

  // Reset the motor current alarms of the controller
  controller->resetMotorCurrentAlarms();

  // Clear the position change of the controller
  controller->clearPositionChange();

  // Set the hasTransition flag to false
  hasTransition = false;

  // Print a message indicating that the function is leaving the Homing Find
  // Bottom State
  Serial.println("-----------------------------------------------------------");
  Serial.println("|               Leaving Homing Find Bottom State          |");
  Serial.printf("|                 Elapsed Time: %6d ms                 |\n",
                elapsedTime() / 1000);
  Serial.println("-----------------------------------------------------------");
  enteredStateTime = 0;
}

#include "MotorsHomingBottomFoundState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

void MotorsHomingBottomFoundState::enter() {
  // Reset the soft movement and current information in the controller
  controller->resetSoftMovement();
  controller->resetCurrentInformation();

  // Set the transition flag to false
  hasTransition = false;

  enteredStateTime  = micros();

  // Print the message indicating that we are entering the Homing Bottom Found
  // State
  Serial.println("-----------------------------------------------------------");
  Serial.println("|              Entering Homing Bottom Found State         |");
  Serial.println("-----------------------------------------------------------");

  // Zero the controller position
  controller->zero();

  // Set the motor speed to the minimum travel speed
  controller->speed = MIN_MOTOR_TRAVEL_SPEED;

  // Set the position to the reverse amount for the alarm
  controller->setPos(ALARM_REVERSE_AMOUNT);

  // Set the system direction and requested direction to EXTEND
  controller->systemDirection = controller->requestedDirection =
      Direction::EXTEND;

  // Extend the controller
  controller->extend();
}

/**
 * Updates the MotorsHomingBottomFoundState.
 *
 * This function is responsible for updating the MotorsHomingBottomFoundState by performing the following steps:
 * 1. Checks if the set position has been reached.
 * 2. Checks if the current position is within 3 units of the reverse alarm position.
 * 3. Sets the transition flag to true if the condition is met.
 * 4. Sets the next state type to MOTORS_STOPPED_STATE if the condition is met.
 * 5. Updates the PID controller.
 * 6. Updates the motors.
 *
 * @throws None
 */
void MotorsHomingBottomFoundState::update() {
  // Check if the set position has been reached
  controller->checkIfSetPositionReached();

  // Check if the current position is within 3 units of the reverse alarm position
  if (abs(controller->getPos() - ALARM_REVERSE_AMOUNT) <= 3) {
    // Set the transition flag to true
    hasTransition = true;

    // Set the next state type to MOTORS_STOPPED_STATE
    nextStateType = MOTORS_STOPPED_STATE;
  }

  // Update the PID controller
  controller->handlePid();

  // Update the motors
  controller->updateMotors();
}

/**
 * Leave the MotorsHomingBottomFoundState.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorsHomingBottomFoundState::leave() {
  // Set hasTransition flag to false
  hasTransition = false;

  // Reset soft movement on the controller
  controller->resetSoftMovement();

  // Reset current information on the controller
  controller->resetCurrentInformation();

  // Reset motor current alarms on the controller
  controller->resetMotorCurrentAlarms();

  // Set the motor position to zero
  controller->zero();

  // Clear the homing state on the controller
  controller->clearHoming();

  // Set the motor speed to the default value
  controller->speed = DEFAULT_MOTOR_SPEED;

  Serial.println("-----------------------------------------------------------");
  Serial.println("|               Leaving Homing Bottom Found State         |");
  Serial.printf("|                Elapsed Time: %6d ms                 |\n",
                elapsedTime() / 1000);
  Serial.println("-----------------------------------------------------------");
  enteredStateTime = 0;
}

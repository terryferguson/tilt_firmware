#include "MotorsHomingBottomFoundState.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

void MotorsHomingBottomFoundState::enter() {
  // Reset the soft movement and current information in the controller
  controller->resetSoftMovement();
  controller->resetCurrentInformation();

  // Set the transition flag to false
  hasTransition = false;

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

void MotorsHomingBottomFoundState::update() {
  // Check if the set position has been reached
  controller->checkIfSetPositionReached();
  if (abs(controller->getPos() - ALARM_REVERSE_AMOUNT) <= 3) {
    // Set the transition flag to true
    hasTransition = true;
    nextStateType = MOTORS_STOPPED_STATE;
  }
  controller->handlePid();
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
  Serial.println("-----------------------------------------------------------");
}

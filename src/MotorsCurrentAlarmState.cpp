#include "MotorsCurrentAlarmState.hpp"
#include "MotorController.hpp"
#include "SystemState.hpp"
#include "debugging.hpp"
#include <Arduino.h>

void MotorsCurrentAlarmState::enter() {
  // Set the transition flag to false
  hasTransition = false;

  enteredStateTime = micros();

  // Print the message indicating that we are entering the Homing Bottom Found
  // State
  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                  Entering Current Alarm State           |");
    Serial.println(
        "-----------------------------------------------------------");
  }

  if (nullptr == controller) {
    return;
  }

  // Set the motor speed to the minimum travel speed
  controller->speed = MIN_MOTOR_TRAVEL_SPEED;

  // Reset the soft movement and current information in the controller
  controller->resetSoftMovement();
  controller->resetCurrentInformation();

  systemState.EnableAlarm();
}

/**
 * Updates the state of the current MotorsCurrentAlarmState.
 *
 * This function checks if the motors are stopped. If they are stopped, it
 * prints a message and sets the hasTransition flag to true, and nextStateType
 * to MOTORS_STOPPED_STATE. If the motors are not stopped, it checks if the set
 * position has been reached. Then it checks if the current position is within
 * 20 units of the reverse alarm position. If it is, it sets hasTransition to
 * true and nextStateType to MOTORS_STOPPING_STATE. After that, it updates the
 * PID controller and the motors.
 */
void MotorsCurrentAlarmState::update() {
  if (nullptr == controller) {
    return;
  }

  if (controller->motorsStopped()) {
    DebugPrintln("MotorsCurrentAlarmState - Motors stopped");
    hasTransition = true;
    nextStateType = MOTORS_STOPPED_STATE;

  } else {
    // Check if the set position has been reached
    controller->checkIfSetPositionReached();

    // Check if we are close to the set position
    if (controller->motorsNearDesiredPosition(20)) {
      /*
      hasTransition = true;
      nextStateType = MOTORS_STOPPING_STATE;
      */
      controller->immediateHalt();
    }

    // Update the PID controller
    controller->handlePid();

    // Update the motors
    controller->updateMotors();
  }
}

/**
 * Leave the MotorsCurrentAlarmState.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorsCurrentAlarmState::leave() {
  // Set hasTransition flag to false
  hasTransition = false;

  if (nullptr != controller) {
    // Reset soft movement on the controller
    controller->resetSoftMovement();

    // Reset current information on the controller
    controller->resetCurrentInformation();

    // Reset motor current alarms on the controller
    controller->resetMotorCurrentAlarms();

    // Set the motor speed to the default value
    controller->setSpeed(systemState.systemSpeed);

    systemState.DisableAlarm();

    systemState.SaveMotorPostions();
  }

  if (systemState.debugEnabled) {
    Serial.println(
        "-----------------------------------------------------------");
    Serial.println(
        "|                Leaving Current Alarm State             |");
    Serial.printf("|                Elapsed Time: %6d ms                 |\n",
                  elapsedTime() / 1000);
    Serial.println(
        "-----------------------------------------------------------");
  }
  enteredStateTime = 0UL;
}

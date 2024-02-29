#include "MotorController.hpp"
#include "SystemState.hpp"
#include "debugging.hpp"

extern SystemState systemState;

MotorController::MotorController(const int pwmFrequency = PWM_FREQUENCY,
                                 const int pwmResolution = PWM_RESOLUTION_BITS,
                                 const int defaultSpeed = DEFAULT_MOTOR_SPEED)
    : pwmFrequency(pwmFrequency), pwmResolution(pwmResolution),
      defaultSpeed(defaultSpeed), currentAlarmSet(false) {
  char buf[256];
  snprintf(
      buf, 255,
      "Controller Params: Frequency: %d - Resolution: %d - Duty Cycle: %d\n",
      pwmFrequency, pwmResolution, defaultSpeed);
  DebugPrintln(buf);
  speed = targetSpeed = 0;
  systemDirection = Direction::STOP;
  ALL_MOTORS(motors[motor].speed = 0;)
  ALL_MOTORS(motors[motor].currentAlarmLimit = CURRENT_LIMIT;)
}

/**
 * @brief Initialize the motors, position storage, current sensors, and PID
 * controllers.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::initialize() {
  // Initialize the leader motor
  motors[0] = Motor("Leader",                   // Motor name
                    MotorPin::MOTOR1_RPWM_PIN,  // Right PWM pin
                    MotorPin::MOTOR1_LPWM_PIN,  // Left PWM pin
                    MotorPin::MOTOR1_R_EN_PIN,  // Right enable pin
                    MotorPin::MOTOR1_L_EN_PIN,  // Left enable pin
                    MotorPin::MOTOR1_HALL1_PIN, // Hall sensor 1 pin
                    MotorPin::MOTOR1_HALL2_PIN, // Hall sensor 2 pin
                    LEADER_CURRENT_SENSE_PIN,   // Current sense pin
                    motorPulseTotals[0],        // Motor pulse total
                    PWM_FREQUENCY,              // PWM frequency
                    defaultSpeed,               // Default speed
                    pwmResolution,              // PWM resolution
                    MOTOR1_LIMIT,               // Motor bottom limit
                    minCurrent,    // Motor current limit for bottom finding
                    CURRENT_LIMIT, // Alarm current in mA
                    LEADER_BUFFER  // Buffer for retraction stop in hall pulses
  );

  // Initialize the follower motor
  motors[1] = Motor("Follower",                 // Motor name
                    MotorPin::MOTOR2_RPWM_PIN,  // Right PWM pin
                    MotorPin::MOTOR2_LPWM_PIN,  // Left PWM pin
                    MotorPin::MOTOR2_R_EN_PIN,  // Right enable pin
                    MotorPin::MOTOR2_L_EN_PIN,  // Left enable pin
                    MotorPin::MOTOR2_HALL1_PIN, // Hall sensor 1 pin
                    MotorPin::MOTOR2_HALL2_PIN, // Hall sensor 2 pin
                    FOLLOWER_CURRENT_SENSE_PIN, // Current sense pin
                    motorPulseTotals[1],        // Motor pulse total
                    PWM_FREQUENCY,              // PWM frequency
                    defaultSpeed,               // Default speed
                    pwmResolution,              // PWM resolution
                    MOTOR2_LIMIT,               // Motor bottom limit
                    minCurrent,     // Motor current limit for bottom finding
                    CURRENT_LIMIT,  // Alarm current in mA
                    FOLLOWER_BUFFER // Buffer for retraction stop in hall pulse
  );

  // Begin position storage
  positionStorage.begin("evox-tilt", false);

  // Load the stored positions
  loadPositions();

  // Initialize the motors
  initializeMotors();

  // Get the current of the leader motor
  leaderCurrent = motors[LEADER].getCurrent();

  // Get the current of the follower motor
  followerCurrent = motors[FOLLOWER].getCurrent();

  // Set parameters for PID controller to defaults
  pidController.setParams(DEFAULT_KP, MAX_SPEED);

  // Print system initialization message
  DebugPrintln("System initialized.");
}

/**
 * @brief Extends the motorized system.
 *
 * This function enables PID control, resets soft movement,
 * sets the speed to the default speed, resets the out of range
 * flag for all motors, sends the extend command to all motors,
 * and sets the system direction and requested direction to extend.
 */
void MotorController::extend() {
  ALL_MOTORS_COMMAND(extend); // Send extend command to all motors
  systemDirection = requestedDirection = Direction::EXTEND;
}

/**
 * @brief Retracts the motorized system.
 *
 * This function is used to tell the motorized system to retract.
 * It sets the PID flag to true, resets the soft movement,
 * sets the speed to the default speed, sets the outOfRange flag
 * to false for all motors, sets the command to retract for all motors,
 * sets the system direction and requested direction to retract.
 */
void MotorController::retract() {
  ALL_MOTORS_COMMAND(retract); // Send retract command to all motors
  systemDirection = requestedDirection = Direction::RETRACT;
}

void MotorController::clearOutOfRange() {
  ALL_MOTORS(motors[motor].outOfRange = false;)
}

/**
 * Halts the system immediately.
 *
 * This function immediately halts the system by setting the speed, target
 * speed, system direction, and requested direction to 0. It also resets the
 * soft movement and disables the motors. Additionally, it resets the current
 * update interval, current information, motor current alarms, and position
 * change.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::immediateHalt() {
  speed = targetSpeed = 0;
  ALL_MOTORS(motors[motor].speed = 0;)
  systemDirection = Direction::STOP;
  requestedDirection = Direction::STOP;
  resetSoftMovement();
  currentUpdateInterval = CURRENT_UPDATE_INTERVAL;

  resetCurrentInformation();
  resetMotorCurrentAlarms();
  clearPositionChange();

  ALL_MOTORS_COMMAND(disable)
  ALL_MOTORS_COMMAND(update)
  moveStart = 0UL;
}

/**
 * Set the outOfRange flag to false and homing range to true for all motors.
 *
 * @throws ErrorType description of error
 */
void MotorController::setHoming() {
  ALL_MOTORS(motors[motor].outOfRange = false;)
  ALL_MOTORS(motors[motor].homing = true;)
}

void MotorController::hardSetSpeed() {
  ALL_MOTORS(motors[motor].speed = speed;)
}

void MotorController::clearHoming() {
  ALL_MOTORS(motors[motor].homing = false;)
}

/**
 * @brief Set the speed of the motor.
 *
 * @param newSpeed The new speed value.
 */
void MotorController::setSpeed(const int newSpeed, const int softMovementTime) {

  targetSpeed = newSpeed;

  // Reset the soft start and last PWM update times
  softStart = lastPWMUpdate = micros();
  const int softMovementTimeMicros = softMovementTime * MICROS_IN_MS;
  softMovingTime = softMovementTimeMicros;
  const int softMovementUpdateSteps =
      (softMovementTimeMicros / SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);
  // Calculate the amount to update the PWM duty cycle per step
  pwmUpdateAmount = (float)abs(targetSpeed - speed) / softMovementUpdateSteps;

  // If the new speed is lower, make the pwmUpdateAmount negative
  if (targetSpeed < speed) {
    pwmUpdateAmount = -pwmUpdateAmount;
  }

  // Print debug information if debugEnabled is true
  if (systemState.debugEnabled) {
    Serial.printf("MotorController\n"
                  "------------\n"
                  "setSpeed(%d)\n"
                  "speed: %3d\n"
                  "target speed: %3d\n"
                  "pwmUpdateAmount: %3.6f\n\n",
                  newSpeed, speed, targetSpeed, pwmUpdateAmount);
  }
}

/**
 * Clears the position change variables for the leader and follower.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::clearPositionChange() {
  leaderLastPos = 0;          // Set the leader's last position to 0.
  followerLastPos = 0;        // Set the follower's last position 0
  leaderLastPosChange = -1;   // Set the leader's last position change to a
                              // negative sentinel value.
  followerLastPosChange = -1; // Set the follower's last position change to a
                              // negative sentinel value.
}

/**
 * @brief Zeroes out the position count of all motors.
 *
 * @return void
 *
 * @throws None
 */
void MotorController::zero() {
  ALL_MOTORS_COMMAND(zero)
  clearPositionChange();
}

/**
 * @brief Reports the current state of the motor controller to the serial
 * console.
 *
 * @return void
 *
 * @throws None
 */
void MotorController::report() {
  ALL_MOTORS_COMMAND(update)
  if (!systemState.debugEnabled) {
    return;
  }

  DebugPrintln("MotorController");
  DebugPrintln("--------------------------------");
  DebugPrint("Speed: ");
  DebugPrintln(speed);
  DebugPrint("Target Speed: ");
  DebugPrintln(targetSpeed);
  DebugPrint("Desired Pos: ");
  DebugPrintln(desiredPos);

  pidController.report();
  Serial.printf("\n\n\n");
  displayCurrents();
}

void MotorController::savePosition(const int slot, const int position_value) {
  // Make sure the slot index is within valid range and if no position was
  // manually specified, then use the current position of the leader motor
  const int positionToSave =
      position_value > -1 ? position_value : motors[LEADER].pos;

  if (slot > 0 && slot < NUM_POSITION_SLOTS + 1) {
    // Store the position value in the savedPositions array.
    savedPositions[slot - 1] = positionToSave;

    // Store the position value in the positionStorage.
    positionStorage.putInt(save_position_slot_names[slot - 1], positionToSave);
    DebugPrint("Saving position: ");
    DebugPrintln(positionToSave);
  }
}

void MotorController::saveConfiguration(const int slot,
                                        const int position_value) {
  // Make sure the slot index is within valid range and if no position was
  // manually specified, then use the current position of the leader motor
  const int positionToSave =
      position_value > -1 ? position_value : motors[LEADER].pos;

  if (slot > 0 && slot < NUM_POSITION_SLOTS + 1) {
    // Store the position value in the savedPositions array.
    savedConfigurations[slot - 1] = positionToSave;

    // Store the position value in the positionStorage.
    positionStorage.putInt(save_configuration_slot_names[slot - 1],
                           positionToSave);
    DebugPrint("Saving configuration: ");
    DebugPrintln(positionToSave);
  }
}

/**
 * Set the desired position for the motors and move them accordingly.
 *
 * @param newPos The new desired position for the motors.
 */
void MotorController::setPos(const int newPos) {
  desiredPos = constrain(newPos, 0, motors[LEADER].totalPulseCount);
  DebugPrint("Going to pos: ");
  DebugPrintln(desiredPos);
}

/**
 * @brief Updates the current readings of the leader and follower motors
 * based on the elapsed time.
 *
 * @param elapsedTime the elapsed time in microseconds
 *
 * @return void
 *
 * @throws None
 */
void MotorController::updateCurrentReadings(const int elapsedTime) {
  // Store the last readings of the leader and follower currents
  const double lastLeaderCurrent = leaderCurrent;
  const double lastFollowerCurrent = followerCurrent;

  const int lCurrent = motors[LEADER].getCurrent();
  const int fCurrent = motors[FOLLOWER].getCurrent();

  // Get the filtered current readings of the leader and follower motors
  leaderCurrent = leaderCurrentFilter(lCurrent);
  followerCurrent = followerCurrentFilter(fCurrent);
  /*
      // Calculate the time factor
      const double timeFactor = 1000000.0 / elapsedTime;

      // Calculate the velocities of the leader and follower currents
      leaderCurrentVelocity = static_cast<int>(
          (leaderCurrent - lastLeaderCurrent) * timeFactor + 0.5);
      followerCurrentVelocity = static_cast<int>(
          (followerCurrent - lastFollowerCurrent) * timeFactor + 0.5);
  */
}

bool MotorController::motorsStopped() {
  if (leaderLastPosChange == 0UL || followerLastPosChange == 0UL) {
    leaderLastPosChange = followerLastPosChange = micros();
    return false; // We don't have a time that this was last changed
  }

  // Get the current timestamp
  const unsigned long timestamp = micros();

  // Read the positions of all motors
  ALL_MOTORS_COMMAND(readPos)

  // Get the positions of the leader and follower motors
  const int leaderPos = motors[LEADER].pos;
  const int followerPos = motors[FOLLOWER].pos;

  // Check if the leader motor position has changed
  const bool leaderPositionChanged = leaderPos != leaderLastPos;
  // Check if the follower motor position has changed
  const bool followerPositionChanged = followerPos != followerLastPos;

  // Update the last position change timestamp for the leader motor
  leaderLastPosChange = leaderPositionChanged ? timestamp : leaderLastPosChange;
  ;
  // Update the last position change timestamp for the follower motor
  followerLastPosChange =
      followerPositionChanged ? timestamp : followerLastPosChange;

  // Check if the leader motor is stopped
  const bool isLeaderStopped =
      leaderPositionChanged
          ? false
          : (leaderLastPosChange > 0UL) &&
                (timestamp - leaderLastPosChange) > MAX_TIME_SINCE_CHANGE;

  // Check if the follower motor is stopped
  const bool isFollowerStopped =
      followerPositionChanged
          ? false
          : (followerLastPosChange > 0UL) &&
                (timestamp - followerLastPosChange) > MAX_TIME_SINCE_CHANGE;

  // Update the last positions of the leader and follower motors
  leaderLastPos = leaderPos;
  followerLastPos = followerPos;

  const auto stopped = isLeaderStopped && isFollowerStopped;

  // Return true if both motors are stopped
  return stopped;
}

/**
 * @brief Checks if the current alarm is triggered.
 *
 * @return true if the current alarm is triggered, false otherwise
 */
bool MotorController::currentAlarmTriggered() {
  return currentAlarmSet &&
             (leaderCurrent > motors[LEADER].currentAlarmLimit) ||
         (followerCurrent > motors[FOLLOWER].currentAlarmLimit);
}

/**
 * Check if the motors are desynchronized.
 *
 * @return true if the motors are desynchronized, false otherwise
 */
bool MotorController::motorsDesynced(void) const {
  return abs(motors[LEADER].pos - motors[FOLLOWER].pos) > DESYNC_TOLERANCE;
}

/**
 * @brief Check if the motors are close to the end of their range.
 *
 * @return True if either motor is close to the end of its range, false
 * otherwise.
 */
bool MotorController::motorsCloseToEndOfRange() {
  // Get the normalized position of the leader motor
  double leaderPos = motors[LEADER].getNormalizedPos();

  // Get the normalized position of the follower motor
  double followerPos = motors[FOLLOWER].getNormalizedPos();

  // Check if the leader motor is close to the end of its range
  bool leaderCloseToEnd =
      (leaderPos < 0.1 && motors[LEADER].dir == Direction::RETRACT) ||
      (leaderPos > 0.85 && motors[LEADER].dir == Direction::EXTEND);

  // Check if the follower motor is close to the end of its range
  bool followerCloseToEnd =
      (followerPos < 0.1 && motors[FOLLOWER].dir == Direction::RETRACT) ||
      (followerPos > 0.85 && motors[FOLLOWER].dir == Direction::EXTEND);

  // Return true if either motor is close to the end of its range
  return leaderCloseToEnd && followerCloseToEnd;
}

/**
 * @brief Disable all motors, reset speed and direction variables,
 * and turn off PID control. Print debug message if debugEnabled is true.
 */
void MotorController::handleCurrentAlarm() {
  // Print debug message if debugEnabled is true

  displayCurrents();
  DebugPrintln("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  DebugPrintln("!!!!!!!!!!!!!!!!!!!!!!!!ALARM!!!!!!!!!!!!!!!!!!");
  DebugPrintln("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  alarmTriggered = true;
}

/**
 * @brief Update the leading and lagging indices based on the system
 * direction.
 *
 * @return void
 */
void MotorController::updateLeadingAndLaggingIndicies() {
  // Check if the system direction is to extend
  if (Direction::EXTEND == systemDirection) {
    // Update the lagging index based on the normalized positions of the
    // motors
    laggingIndex = (motors[LEADER].getNormalizedPos() <
                    motors[FOLLOWER].getNormalizedPos())
                       ? MotorRoles::LEADER
                       : MotorRoles::FOLLOWER;
    // Update the leading index based on the normalized positions of the
    // motors
    leadingIndex = (motors[LEADER].getNormalizedPos() >=
                    motors[FOLLOWER].getNormalizedPos())
                       ? MotorRoles::LEADER
                       : MotorRoles::FOLLOWER;
  }
  // Check if the system direction is to retract
  else if (Direction::RETRACT == systemDirection) {
    // Update the lagging index based on the normalized positions of the
    // motors
    laggingIndex = (motors[LEADER].getNormalizedPos() >
                    motors[FOLLOWER].getNormalizedPos())
                       ? MotorRoles::LEADER
                       : MotorRoles::FOLLOWER;
    // Update the leading index based on the normalized positions of the
    // motors
    leadingIndex = (motors[LEADER].getNormalizedPos() <=
                    motors[FOLLOWER].getNormalizedPos())
                       ? MotorRoles::LEADER
                       : MotorRoles::FOLLOWER;
  }
}

/**
 * @brief Displays the current values of the leader and follower motor
 * currents and velocities.
 *
 * @return void
 */
void MotorController::displayCurrents() {
  if (systemState.debugEnabled) {
    DebugPrint("Leader Motor Current: ");
    Serial.println(leaderCurrent);
    DebugPrint("Follower Motor Current: ");
    Serial.println(followerCurrent);
  }
}

/**
 * Sets the current limit for the leader and follower motors based on their
 * average current. Prints the current alarms set to the
 * serial monitor.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::setCurrentLimit() {
  if (samples > 0) {
    const int leaderAverageCurrent = leaderCurrentSum / samples;
    const int followerAverageCurrent = followerCurrentSum / samples;

    const int leaderCurrentLimit =
        static_cast<int>(leaderAverageCurrent * CURRENT_INCREASE_MULTIPLIER);

    const int followerCurrentLimit =
        static_cast<int>(followerAverageCurrent * CURRENT_INCREASE_MULTIPLIER);
    if (systemState.debugEnabled) {
      Serial.print("Current alarms set to: Leader => ");
      Serial.print(leaderCurrentLimit);
      Serial.print(", Follower => ");
      Serial.println(followerCurrentLimit);
    }

    motors[LEADER].currentAlarmLimit = leaderCurrentLimit;
    motors[FOLLOWER].currentAlarmLimit = followerCurrentLimit;

    currentAlarmSet = true;
  }
}

/**
 * @brief Check if a desired position is set and clear it if we have reached it.
 *
 * @throws None
 */
void MotorController::checkIfSetPositionReached() {
  if (desiredPos != -1) {
    if (abs(desiredPos - motors[LEADER].pos) < SET_POSITION_BUFFER) {
      desiredPos = -1;
      immediateHalt();
    }
  }
}

/**
 * Handles the current update.
 *
 * @return void
 */
void MotorController::handleCurrentUpdate() {
  // Get current time in microseconds
  const unsigned long currentTime = micros();
  // Calculate the time since the last current update
  const unsigned long currentUpdateDelta = currentTime - lastCurrentUpdate;
  moveTimeDelta = currentTime - moveStart;

  // Check if it's time to update the current readings
  if (currentUpdateDelta >= currentUpdateInterval) {
    // Check if the motors are not stopped
    if (!motorsStopped()) {
      updateCurrentReadings(currentUpdateDelta);
      // Update the last current update time
      lastCurrentUpdate = currentTime;

      // Display the current readings if debug is enabled
      // displayCurrents();

      if (!currentAlarmSet) {
        sampleCurrents();
      }

      if (moveStart > 0UL && moveTimeDelta > CURRENT_ALARM_DELAY) {
        if (!currentAlarmSet) {
          setCurrentLimit();
        } else {
          if (currentAlarmTriggered()) {
            handleCurrentAlarm();
          }
        }
      }
    }
  }
}

/**
 * Reset the soft movement.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::resetSoftMovement() {
  pwmUpdateAmount = 0;
  lastPWMUpdate = 0UL;
  softStart = 0UL;
  targetSpeed = -1;
  currentUpdateInterval = CURRENT_UPDATE_INTERVAL;
  pidController.setParams(DEFAULT_KP);
  pidController.reset();
  followerLastPosChange = 0UL;
  leaderLastPosChange = 0UL;
}

/**
 * @brief Reset the current information for the system.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::resetCurrentInformation() {
  currentAlarmSet = false;
  maxCurrent = -1;
  samples = 0;
  leaderCurrentSum = 0;
  followerCurrentSum = 0;
  resetMotorCurrentAlarms();
  leaderCurrentFilter.reset();
  followerCurrentFilter.reset();
  lastCurrentUpdate = 0UL;
}

/**
 * @brief Reset the current alarms for the motors.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::resetMotorCurrentAlarms() {
  motors[LEADER].currentAlarmLimit = CURRENT_LIMIT;
  motors[FOLLOWER].currentAlarmLimit = CURRENT_LIMIT;
}

/**
 * @brief Disable the motors.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::disableMotors() {
  immediateHalt();
  DebugPrintln("Motors disabled.");
}

/**
 * @brief Update the motors.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void MotorController::updateMotors() { ALL_MOTORS_COMMAND(update) }

void MotorController::resetPid() {
  // Set parameters for PID controller to defaults
  pidController.setParams(DEFAULT_KP, MAX_SPEED);
}

void MotorController::setPidParams(const int Kp, const float Ki,
                                   const float Kd) {
  pidController.setParams(Kp, Ki, Kd);
}

/**
 * Set the bottom current limit for both the leader and follower motors.
 *
 * @param currentLimit the current limit to set
 *
 * @throws ErrorType if an error occurs while setting the current limit
 */
void MotorController::setBottomCurrentLimit(const int currentLimit) {
  motors[LEADER].bottomCurrentLimit = currentLimit;
  motors[FOLLOWER].bottomCurrentLimit = currentLimit;
}

Direction MotorController::getRestoreDirection() const {
  const int curPos = getPos();

  // Don't bother moving if we are already within tolerance
  if (abs(curPos - desiredPos) < DESIRED_POSITION_BUFFER / 3) {
    return Direction::STOP;
  }

  // Return the direction we need to go
  return curPos < desiredPos ? Direction::EXTEND : Direction::RETRACT;
}

bool MotorController::motorsNearDesiredPosition(const int maxDelta) const {
  return desiredPos > -1 && (abs(motors[LEADER].pos - desiredPos) < maxDelta &&
                             abs(motors[FOLLOWER].pos - desiredPos) < maxDelta);
}

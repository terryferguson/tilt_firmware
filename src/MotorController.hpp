/*! \file MotorController.hpp */

#ifndef _MOTOR_CONTROLLER_HPP_
#define _MOTOR_CONTROLLER_HPP_

#include <Preferences.h>
#include <math.h>

#include "EMA.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "PinMacros.hpp"
#include "defs.hpp"

#define ALL_MOTORS(operation)                                                  \
  for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++) {                     \
    operation                                                                  \
  }

#define ALL_MOTORS_COMMAND(command) ALL_MOTORS(motors[motor].command();)

#define RESTORE_POSITION(slot) motor_controller.setPos(savedPositions[slot]);

#define SERIAL_SAVE_POSITION(slot)                                             \
  if (Serial.available() > 0) {                                                \
    int new_pos = Serial.parseInt();                                           \
    motor_controller.savePosition(slot, new_pos);                              \
  }

/** @class MotorController
 *
 *  @brief This is the controller of the motors
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorController {
private:
  typedef void (MotorController::*Handler)();
  /// @enum MotorRoles
  /// @brief The roles of this motor for this PID control
  enum MotorRoles {
    LEADER,  /** The leader motor that is speed-matched */
    FOLLOWER /** The follower motor that is speed-matched to the leader motor */
  };

  const int motorPulseTotals[NUMBER_OF_MOTORS] = {LEADER_MAX_PULSES,
                                                  FOLLOWER_MAX_PULSES};

  /// @brief Hall sensor pulse totals for the motor travel limit feature
  // const int motorPulseTotals[2] = {2055, 2050};

  /** @brief The frequency of the PWM signal in hertz */
  int pwmFrequency = PWM_FREQUENCY;

  /** @brief The PWM bitdepth resolution */
  int pwmResolution = PWM_RESOLUTION_BITS;

  /** @brief The requested motor position for the setPos method */
  int desiredPos = -1;

  /** @brief On flash storage for positions motor positions */
  Preferences positionStorage;

  int maxCurrent = -1;

  /** @brief The last position of the follower motor */
  int followerLastPos = 0;

  /** @brief The last position of the leader motor */
  int leaderLastPos = 0;

  /** @brief The time of the last position change of the leader motor */
  int leaderLastPosChange = -1;

  /** @brief The time of the last position change of the follower motor */
  int followerLastPosChange = -1;

  /** @brief The maximum time a motor can stay at the same position before it's
   * considered stopped */
  const int MAX_TIME_SINCE_CHANGE = 500000;

  /**
   * @brief Loads the positions from position storage.
   *
   * @throws None
   */
  void loadPositions() {
    for (int slot = 0; slot < NUM_POSITION_SLOTS; slot++) {
      savedPositions[slot] =
          positionStorage.getInt(save_position_slot_names[slot]);
    }
  }

  /**
   * @brief Initializes the motors and sets them to halt immediately.
   *
   * @throws None
   */
  void initializeMotors() {
    resetSoftMovement();
    ALL_MOTORS_COMMAND(initialize)
    immediateHalt();
  }

  double Kp, Ki, Kd;
  double prevError;
  double integral;
  double maxIntegral = 1000.0; // limit for integral wind-up
  int intermediateSpeed = -1;

  double control(double setpoint, double actualPosition) {
    double error = setpoint - actualPosition;

    integral += error;
    // Prevent integral wind-up
    if (integral > maxIntegral)
      integral = maxIntegral;
    else if (integral < -maxIntegral)
      integral = -maxIntegral;

    double derivative = error - prevError;

    double output = Kp * error + Ki * integral + Kd * derivative;
    prevError = error;
    return output;
  }

  void calculatePid() {
    updateLeadingAndLaggingIndicies();
    // Speed to set the faster motor to as calculated by the PID algorithm
    const int adjustedSpeed = pidController.adjustSpeed(
        motors[leadingIndex], motors[laggingIndex], speed, deltaT);

    motors[leadingIndex].speed = adjustedSpeed;
    motors[laggingIndex].speed = speed;
  }

  void ignorePid() {
    motors[leadingIndex].speed = speed;
    motors[laggingIndex].speed = speed;
  }

  Handler pidFuncts[2] = {&MotorController::ignorePid,
                          &MotorController::calculatePid};

  int endOfRangeSpeeds[2] = {MOTOR_END_OF_RANGE_SPEED, MIN_MOTOR_TRAVEL_SPEED};
  int travelSpeeds[2] = {DEFAULT_MOTOR_SPEED, 0};
  int endOfrangeTimes[2] = {SOFT_MOVEMENT_TIME_MS, SOFT_STOP_TIME_MS};

public:
  float deltaT = 0.0f;

  /** @brief The time in microseconds since a motor movement started */
  int moveStart = -1;

  /** @brief The target speed of soft movement */
  int targetSpeed = -1;

  /** @brief The proprotional gain for the PID controller  */
  int K_p = DEFAULT_KP;

  /** @brief The PID controller for the motor synchonization */
  PIDController pidController;

  /** @brief The default speed to operate the
      motors at on startup */
  int defaultSpeed = DEFAULT_MOTOR_SPEED;

  /** @brief Current target speed */
  int speed = 0;

  /** @brief Indicates whether the current alarm is set */
  bool currentAlarmSet = false;

  /** @brief Leader motor current */
  int leaderCurrent = 0;

  /** @brief Follower motor current */
  int followerCurrent = 0;

  /** @brief Last leader current reading */
  int lastLeaderCurrent = 0;

  /** @brief Last follower current reading */
  int lastFollowerCurrent = 0;

  /** @brief Leader current velocity */
  int leaderCurrentVelocity = 0;

  /** @brief Follower current velocity */
  int followerCurrentVelocity = 0;

  /** @brief Minimum current to enable alarm system for motors  */
  int minCurrent = 300;

  /** @brief Current delay for overcurrent alarm system */
  int currentAlarmDelay = 250000;

  /** @brief Current velocity limit for alarm system for motors to enable  */
  int alarmCurrentVelocity = 10000;

  /** @brief The motors controlled by this motor controller instance */
  Motor motors[NUMBER_OF_MOTORS];

  /// @brief The requested system level direction
  Direction requestedDirection = Direction::STOP;

  /** @brief The current system level direction indicator */
  Direction systemDirection = Direction::STOP;

  /// @brief The index in the motors array of the motor that is behind as
  /// indicated by the hall sensor
  int laggingIndex = 0;

  /// @brief The index in the motors array of the motor that is farther along as
  /// indicated by the hall sensor
  int leadingIndex = 0;

  /** @brief The timestamp since soft start of movement */
  long softStart = -1;

  /** @brief The last PWM update interval in microseconds */
  int lastPWMUpdate = -1;

  /** @brief The amount to change the PWM duty cycle on soft start */
  float pwmUpdateAmount = -1.0f;

  /** @brief The last time a debug serial print was sent */
  long lastPrintTime = -1;

  /** @brief Interval of time to pass between current updates microseconds  */
  long currentUpdateInterval = CURRENT_UPDATE_INTERVAL;

  /** @brief Time in microseconds since the last current update */
  long lastCurrentUpdate = -1;

  long softMovingTime = -1;

  /** @brief Input filter for leader current readings */
  EMA<2> leaderCurrentFilter;

  /** @brief Input filter for follower current readings */
  EMA<2> followerCurrentFilter;

  /** @brief Number of current samples */
  int_fast32_t samples = 0;

  /** @brief Current difference sum */
  int_fast32_t currentDifferenceSum = 0;

  /** @brief Leader current sum */
  int_fast32_t leaderCurrentSum = 0;

  /** @brief Follower current sum */
  int_fast32_t followerCurrentSum = 0;

  /** @brief Time since this movement began in microseconds */
  long moveTimeDelta = 0;

  /** @brief Time since last PWM update on soft start in microseconds */
  long updateTimeDelta = 0;

  /**
   * @brief This is the class that controls the motors
   *
   * @param PWM_FREQUENCY the frequency of the PWM signal
   * @param PWM_RESOLUTION_BITS the resolution of the PWM signal in bits
   * @param DEFAULT_MOTOR_SPEED the default speed of the motor
   *
   * @return none
   *
   * @throws none
   */
  MotorController(const int pwmFrequency = PWM_FREQUENCY,
                  const int pwmResolution = PWM_RESOLUTION_BITS,
                  const int defaultSpeed = DEFAULT_MOTOR_SPEED)
      : pwmFrequency(pwmFrequency), pwmResolution(pwmResolution),
        defaultSpeed(defaultSpeed), currentAlarmSet(false) {
    char buf[256];
    snprintf(
        buf, 256,
        "Controller Params: Frequency: %d - Resolution: %d - Duty Cycle: %d\n",
        pwmFrequency, pwmResolution, defaultSpeed);
    Serial.println(buf);
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
  void initialize() {
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
                      LEADER_BUFFER // Buffer for retraction stop in hall pulses
    );

    // Initialize the follower motor
    motors[1] =
        Motor("Follower",                 // Motor name
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
              minCurrent, // Motor current limit for bottom finding
              CURRENT_LIMIT - CURRENT_OFFSET, // Alarm current in mA
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
    Serial.println("System initialized.");
  }

  /**
   * @brief Extends the motorized system.
   *
   * This function enables PID control, resets soft movement,
   * sets the speed to the default speed, resets the out of range
   * flag for all motors, sends the extend command to all motors,
   * and sets the system direction and requested direction to extend.
   */
  void extend() {
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
  void retract() {
    ALL_MOTORS_COMMAND(retract); // Send extend command to all motors
    systemDirection = requestedDirection = Direction::RETRACT;
  }

  void clearOutOfRange() {
    ALL_MOTORS(motors[motor].outOfRange =
                   false;) ///< Set the outOfRange flag to false for all motors
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
  void immediateHalt() {
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
  }

  /**
   * Set the outOfRange flag to false and homing range to true for all motors.
   *
   * @throws ErrorType description of error
   */
  void setHoming() {
    ALL_MOTORS(motors[motor].outOfRange =
                   false;) ///< Set the outOfRange flag to false for all motors
    ALL_MOTORS(motors[motor].homing = true;)
  }

  void hardSetSpeed() { ALL_MOTORS(motors[motor].speed = speed;) }

  void clearHoming() { ALL_MOTORS(motors[motor].homing = false;) }

  /**
   * @brief Set the speed of the motor.
   *
   * @param newSpeed The new speed value.
   */
  void setSpeed(const int newSpeed,
                const int softMovementTime = SOFT_MOVEMENT_TIME_MS) {

    targetSpeed = newSpeed;

    // Update the target speed
    Serial.printf("SetSpeed(%d)\n", newSpeed);
    Serial.printf("Speed: %d\n", speed);
    Serial.printf("Target speed: %d\n", targetSpeed);

    // Reset the soft start and last PWM update times
    softStart = lastPWMUpdate = micros();
    const int softMovementTimeMicros = softMovementTime * MICROS_IN_MS;
    softMovingTime = softMovementTimeMicros;
    const int softMovementUpdateSteps =
        (softMovementTimeMicros / SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);
    // Calculate the amount to update the PWM duty cycle per step
    pwmUpdateAmount =
        ceil((float)abs(targetSpeed - speed) / softMovementUpdateSteps);

    // If the new speed is lower, make the pwmUpdateAmount negative
    if (targetSpeed < speed) {
      pwmUpdateAmount = -pwmUpdateAmount;
    }

    // Print debug information if debugEnabled is true
    if (debugEnabled) {
      Serial.printf("MotorController\n"
                    "------------\n"
                    "setSpeed(%d)\n"
                    "speed: %3d\n"
                    "target speed: %3d\n"
                    "pwmUpdateAmount: %3.6f\n\n",
                    newSpeed, speed, targetSpeed, pwmUpdateAmount);
    }

    if (newSpeed < speed) {
      // Reducing speed - use intermediate setpoint
      intermediateSpeed = speed - (speed - newSpeed) / 3;
    }

    if (requestedDirection != Direction::RETRACT) {
      pidController.setParams(RETRACT_RAMP_KP);
    } else if (requestedDirection == Direction::EXTEND) {
      pidController.setParams(EXTEND_RAMP_KP);
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
  void clearPositionChange() {
    leaderLastPos =
        -9999; // Set the leader's last position to a negative sentinel value.
    followerLastPos =
        -9999; // Set the follower's last position to a negative sentinel value.
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
  void zero() {
    ALL_MOTORS_COMMAND(zero)
    leaderLastPos = 0;
    followerLastPos = 0;
    leaderLastPosChange = -1;
    followerLastPosChange = -1;
  }

  /**
   * @brief Reports the current state of the motor controller to the serial
   * console.
   *
   * @return void
   *
   * @throws None
   */
  void report() {
    ALL_MOTORS_COMMAND(update)
    Serial.printf("MotorController\n--------------------\nSpeed: %d\nTarget "
                  "Speed: %d\n\n",
                  speed, targetSpeed);
    pidController.report();
    Serial.printf("\n\n\n");
    displayCurrents();
    Serial.printf("Current Alarm Status: %s\n\n",
                  currentAlarmSet ? "true" : "false");
    ALL_MOTORS_COMMAND(displayInfo)
  }

  /**
   * @brief Saves the position value for a given slot.
   *
   * @param slot The slot index.
   * @param position_value The position value to save.
   */
  void savePosition(const int slot, const int position_value = -1) {
    // Make sure the slot index is within valid range and if no position was
    // manually specified, then use the current position of the leader motor
    const int positionToSave =
        position_value > -1 ? position_value : motors[LEADER].pos;

    if (slot > 0 && slot < NUM_POSITION_SLOTS) {
      // Store the position value in the savedPositions array.
      savedPositions[slot - 1] = positionToSave;

      // Store the position value in the positionStorage.
      positionStorage.putInt(save_position_slot_names[slot], positionToSave);
    }
  }

  /**
   * Set the desired position for the motors and move them accordingly.
   *
   * @param newPos The new desired position for the motors.
   */
  void setPos(const int newPos) {
    desiredPos = constrain(newPos, 0, motors[LEADER].totalPulseCount);
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
  void updateCurrentReadings(const int elapsedTime) {
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

  /**
   * @brief Check if the system is stopped.
   *
   * @return true if the system is stopped, false otherwise
   */
  bool isStopped() const { return systemDirection == Direction::STOP; }

  /**
   * Check if both motors are stopped.
   *
   * @return True if both motors are stopped, false otherwise.
   */
  bool motorsStopped() {
    // Get the current timestamp
    const int timestamp = micros();

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
    leaderLastPosChange =
        leaderPositionChanged ? timestamp : leaderLastPosChange;
    // Update the last position change timestamp for the follower motor
    followerLastPosChange =
        followerPositionChanged ? timestamp : followerLastPosChange;

    // Check if the leader motor is stopped
    const bool isLeaderStopped =
        leaderPositionChanged
            ? false
            : (leaderLastPosChange > 0) &&
                  (timestamp - leaderLastPosChange) > MAX_TIME_SINCE_CHANGE;

    // Check if the follower motor is stopped
    const bool isFollowerStopped =
        followerPositionChanged
            ? false
            : (followerLastPosChange > 0) &&
                  (timestamp - followerLastPosChange) > MAX_TIME_SINCE_CHANGE;

    // Update the last positions of the leader and follower motors
    leaderLastPos = leaderPos;
    followerLastPos = followerPos;

    // Return true if both motors are stopped
    return isLeaderStopped && isFollowerStopped;
  }

  /**
   * @brief Checks if the current alarm is triggered.
   *
   * @return true if the current alarm is triggered, false otherwise
   */
  bool currentAlarmTriggered() {
    return currentAlarmSet &&
               (leaderCurrent > motors[LEADER].currentAlarmLimit) ||
           (followerCurrent > motors[FOLLOWER].currentAlarmLimit);
  }

  /**
   * Check if the motors are desynchronized.
   *
   * @return true if the motors are desynchronized, false otherwise
   */
  bool motorsDesynced(void) const {
    return abs(motors[LEADER].pos - motors[FOLLOWER].pos) > DESYNC_TOLERANCE;
  }

  void handlePid() { (this->*pidFuncts[static_cast<int>(pid_on)])(); }

  /**
   * @brief Check if the motors are close to the end of their range.
   *
   * @return True if either motor is close to the end of its range, false
   * otherwise.
   */
  bool motorsCloseToEndOfRange() {
    // Get the normalized position of the leader motor
    double leaderPos = motors[LEADER].getNormalizedPos();

    // Get the normalized position of the follower motor
    double followerPos = motors[FOLLOWER].getNormalizedPos();

    // Check if the leader motor is close to the end of its range
    // Check if the leader motor is close to the end of its range
    bool leaderCloseToEnd =
        (leaderPos < 0.1 && motors[LEADER].dir == Direction::RETRACT) ||
        (leaderPos > 0.9 && motors[LEADER].dir == Direction::EXTEND);

    // Check if the follower motor is close to the end of its range
    bool followerCloseToEnd =
        (followerPos < 0.15 && motors[FOLLOWER].dir == Direction::RETRACT) ||
        (followerPos > 0.85 && motors[FOLLOWER].dir == Direction::EXTEND);

    // Return true if either motor is close to the end of its range
    return leaderCloseToEnd && followerCloseToEnd;
  }

  /**
   * @brief Disable all motors, reset speed and direction variables,
   * and turn off PID control. Print debug message if debugEnabled is true.
   */
  void handleCurrentAlarm() {
    // Print debug message if debugEnabled is true

    displayCurrents();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!ALARM!!!!!!!!!!!!!!!!!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    immediateHalt();
    systemDirection = requestedDirection = Direction::STOP;
    resetSoftMovement();
    resetCurrentInformation();

    report();
  }

  /**
   * @brief Update the leading and lagging indices based on the system
   * direction.
   *
   * @return void
   */
  void updateLeadingAndLaggingIndicies() {
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
  void displayCurrents() {
    Serial.printf("Leader Motor Current: %d\n", leaderCurrent);
    // Serial.printf("Leader current velocity: %d\n", leaderCurrentVelocity);
    Serial.printf("Follower Motor Current: %d\n", followerCurrent);
    // Serial.printf("Follower current velocity: %d\n",
    // followerCurrentVelocity);
  }

  /**
   * Calculate the larger current between leader and follower,
   * update the maxCurrent if necessary, and update the
   * currentDifferenceSum, currentSum, and samples.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void sampleCurrents() {
    const int largerCurrent = std::max(leaderCurrent, followerCurrent);
    if (largerCurrent > maxCurrent) {
      maxCurrent = largerCurrent;
    }

    const int currentDifference = abs(leaderCurrent - followerCurrent);
    currentDifferenceSum += currentDifference;
    leaderCurrentSum += leaderCurrent;
    followerCurrentSum += followerCurrent;
    samples++;
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
  void setCurrentLimit() {
    if (samples > 0) {
      const int leaderAverageCurrent = leaderCurrentSum / samples;
      const int followerAverageCurrent = followerCurrentSum / samples;

      const int leaderCurrentLimit =
          static_cast<int>(leaderAverageCurrent * CURRENT_INCREASE_MULTIPLIER);

      const int followerCurrentLimit = static_cast<int>(
          followerAverageCurrent * CURRENT_INCREASE_MULTIPLIER);
      Serial.printf("Current alarms set to: Leader => %d, Follower => %d\n",
                    leaderCurrentLimit, followerCurrentLimit);

      motors[LEADER].currentAlarmLimit = leaderCurrentLimit;
      motors[FOLLOWER].currentAlarmLimit = followerCurrentLimit;

      currentAlarmSet = true;
    }
  }

  /**
   * Updates the soft movement of the system.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void updateSoftMovement() {
    const long currentTime = micros();

    if (targetSpeed >= 0) {
      // Distance from the current speed to the target speed
      const int speedDelta = abs(speed - targetSpeed);

      // The time since soft movement started
      moveTimeDelta = currentTime - softStart;

      // Calculate the time since the last PWM update
      const long updateTimeDelta = currentTime - lastPWMUpdate;

      if (updateTimeDelta >= SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS) {
        const bool timeToUpdate = moveTimeDelta < softMovingTime;
        // Get the true PWM update amount based on the actual elapsed time
        const float updateAmount =
            pwmUpdateAmount * (static_cast<float>(updateTimeDelta) /
                               SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);
        const bool speedDeltaEnough = speedDelta >= abs(updateAmount);

        if (timeToUpdate && speedDeltaEnough) {

          const float newSpeed = static_cast<float>(speed + updateAmount);
          speed = static_cast<int>(floorf(newSpeed));
          // Serial.printf("Speed <== %d\n", speed);
          lastPWMUpdate = micros();
        } else {
          // Set the speed to the target speed and reset the soft movement if
          // time expired or there is less than one full update step until we
          // reach the target speed.
          speed = targetSpeed;
          resetSoftMovement();

          if (requestedDirection == Direction::STOP) {
            systemDirection = Direction::STOP;

            immediateHalt();
            report();
          }
        }
      }
      if (intermediateSpeed >= 0) {
        // Transition to intermediate speed
        double output = control(intermediateSpeed,
                                speed); // Assuming speed is the current speed
        if (abs(output - intermediateSpeed) < 1) { // Or a small threshold
          intermediateSpeed = -1;
        }
      } else {
        // Regular target speed tracking
        control(targetSpeed, speed);
      }
    }
  }

  /**
   * Check if a desired position is set and clear it if we have reached it.
   *
   * @throws None
   */
  void checkIfSetPositionReached() {
    if (desiredPos != -1) {
      if (abs(desiredPos - motors[LEADER].pos) < SET_POSITION_BUFFER) {
        if (debugEnabled) {
          Serial.printf("Desired Pos: %d - REACHED\n", desiredPos);
        }
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
  void handleCurrentUpdate() {
    // Get current time in microseconds
    const long currentTime = micros();
    // Calculate the time since the last current update
    const long currentUpdateDelta = currentTime - lastCurrentUpdate;
    const int moveTimeDelta = currentTime - moveStart;

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

        if (moveTimeDelta > CURRENT_ALARM_DELAY) {
          if (!currentAlarmSet) {
            setCurrentLimit();
          }

          if (currentAlarmTriggered()) {
            handleCurrentAlarm();
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
  void resetSoftMovement() {
    pwmUpdateAmount = 0;
    lastPWMUpdate = -1;
    softStart = -1;
    targetSpeed = -1;
    currentUpdateInterval = CURRENT_UPDATE_INTERVAL;
    pidController.setParams(DEFAULT_KP);
    followerLastPosChange = -1;
    leaderLastPosChange = -1;
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
  void resetCurrentInformation() {
    currentAlarmSet = false;
    maxCurrent = -1;
    samples = 0;
    currentDifferenceSum = 0;
    leaderCurrentSum = 0;
    followerCurrentSum = 0;
    resetMotorCurrentAlarms();
    leaderCurrentFilter.reset();
    followerCurrentFilter.reset();
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
  void disableMotors() {
    immediateHalt();
    Serial.println("Motors disabled.");
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
  void updateMotors() { ALL_MOTORS_COMMAND(update) }

  /**
   * @brief Reset the PID controller.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void resetPid() {
    // Set parameters for PID controller to defaults
    pidController.setParams(DEFAULT_KP, MAX_SPEED);
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
  void resetMotorCurrentAlarms() {
    motors[LEADER].currentAlarmLimit = CURRENT_LIMIT;
    motors[FOLLOWER].currentAlarmLimit = CURRENT_LIMIT;
  }

  /**
   * Set the bottom current limit for both the leader and follower motors.
   *
   * @param currentLimit the current limit to set
   *
   * @throws ErrorType if an error occurs while setting the current limit
   */
  void setBottomCurrentLimit(const int currentLimit) {
    motors[LEADER].bottomCurrentLimit = currentLimit;
    motors[FOLLOWER].bottomCurrentLimit = currentLimit;
  }

  int getPos() const { return motors[LEADER].pos; }
};

#endif // _MOTOR_CONTROLLER_HPP_

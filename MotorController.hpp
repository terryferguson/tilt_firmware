/*! \file MotorController.hpp */

#ifndef _MOTOR_CONTROLLER_HPP_
#define _MOTOR_CONTROLLER_HPP_

#include <Preferences.h>
#include <math.h>

#include "Motor.hpp"
#include "PIDController.hpp"
#include "PinMacros.hpp"
#include "defs.hpp"

#define NUMBER_OF_MOTORS 2

#define ALL_MOTORS(operation)                                                  \
  for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++) {                     \
    operation                                                                  \
  }

#define ALL_MOTORS_COMMAND(command) ALL_MOTORS(motors[motor].command();)

#define RESET_SOFT_MOVEMENT                                                    \
  pwmUpdateAmount = 0;                                                         \
  lastPWMUpdate = 0UL;                                                         \
  softStart = 0UL;                                                             \
  targetSpeed = 0UL;                                                           \
  currentUpdateInterval = CURRENT_UPDATE_INTERVAL;                             \
  pidController.setParams(DEFAULT_KP);

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
  /// @enum MotorRoles
  /// @brief The roles of this motor for this PID control
  enum MotorRoles {
    LEADER,  /** The leader motor that is speed-matched */
    FOLLOWER /** The follower motor that is speed-matched to the leader motor */
  };

  const int motorPulseTotals[2] = {2845, 2845};

  /** @brief Indicates whether system is homing */
  bool homing = false;

  /// @brief Hall sensor pulse totals for the motor travel limit feature
  // const int motorPulseTotals[2] = {2055, 2050};

  /// @brief The index in the motors array of the motor that is behind as
  /// indicated by the hall sensor
  int laggingIndex = 0;

  /// @brief The index in the motors array of the motor that is farther along as
  /// indicated by the hall sensor
  int leadingIndex = 0;

  /** @brief The timestamp since soft start of movement */
  unsigned long softStart = 0UL;

  /** @brief The last PWM update interval in microseconds */
  unsigned long lastPWMUpdate = 0UL;

  /** @brief The target speed of soft movement */
  int targetSpeed = -1;

  /** @brief The amount to change the PWM duty cycle on soft start */
  float pwmUpdateAmount = -1.0f;

  /** @brief The last time a debug serial print was sent */
  unsigned long lastPrintTime = 0UL;

  /** @brief Interval of time to pass between current updates microseconds  */
  unsigned long currentUpdateInterval = CURRENT_UPDATE_INTERVAL;

  /** @brief The time in microseconds since a motor movement started */
  unsigned long moveStart = 0UL;

  /** @brief Time in microseconds since the last current update */
  unsigned long lastCurrentUpdate = 0UL;

  /// @brief The requested system level direction
  Direction requestedDirection = Direction::STOP;

  /** @brief The frequency of the PWM signal in hertz */
  int pwmFrequency = PWM_FREQUENCY;

  /** @brief The PWM bitdepth resolution */
  int pwmResolution = PWM_RESOLUTION_BITS;

  /** @brief The requested motor position for the setPos method */
  int desiredPos = -1;

  /** @brief On flash storage for positions motor positions */
  Preferences positionStorage;

  bool stopping = false;

  /**
   * Halts the system immediately.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void immediateHalt() {
    speed = targetSpeed = 0;
    systemDirection = Direction::STOP;
    requestedDirection = Direction::STOP;
    RESET_SOFT_MOVEMENT

    ALL_MOTORS_COMMAND(disable);
    currentUpdateInterval = CURRENT_UPDATE_INTERVAL;
  }

  /// @brief Load the position preferences slots from ROM into memory
  void loadPositions() {
    for (int slot = 0; slot < NUM_POSITION_SLOTS; slot++) {
      savedPositions[slot] =
          positionStorage.getInt(save_position_slot_names[slot]);
    }
  }

  /// @brief Initialize the motors and then set them to halt immediately
  void initializeMotors() {
    RESET_SOFT_MOVEMENT
    ALL_MOTORS_COMMAND(initialize)
    immediateHalt();
  }

public:
  /** @brief The proprotional gain for the PID controller  */
  int K_p = DEFAULT_KP;

  /** @brief The PID controller for the motor synchonization */
  PIDController pidController;

  /** @brief The default speed to operate the
      motors at on startup */
  int defaultSpeed = DEFAULT_MOTOR_SPEED;

  /** @brief Current target speed */
  int speed = 0;

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
  int minCurrent = 900;

  /** @brief Current delay for overcurrent alarm system */
  int currentAlarmDelay = 250000;

  /** @brief Current velocity limit for alarm system for motors to enable  */
  int alarmCurrentVelocity = 10000;

  /** @brief The motors controlled by this motor controller instance */
  Motor motors[NUMBER_OF_MOTORS];

  /** @brief The current system level direction indicator */
  Direction systemDirection = Direction::STOP;

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
        defaultSpeed(defaultSpeed) {
    char buf[256];
    snprintf(
        buf, 256,
        "Controller Params: Frequency: %d - Resolution: %d - Duty Cycle: %d\n",
        pwmFrequency, pwmResolution, defaultSpeed);
    Serial.println(buf);
    speed = targetSpeed = 0;
    systemDirection = Direction::STOP;
    ALL_MOTORS(motors[motor].speed = 0;)
    ALL_MOTORS(motors[motor].currentAlarmLimit = 1000;)
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
                      minCurrent, // Motor current limit for bottom finding
                      CURRENT_ALARM_AMOUNT, // Alarm current in mA
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
              CURRENT_ALARM_AMOUNT - CURRENT_OFFSET, // Alarm current in mA
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
    pidController.setParams(DEFAULT_KP, (1 << PWM_RESOLUTION_BITS) - 1);

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
    pid_on = true; // Enable PID control
    stopping = false;
    pidController.reset(); // Reset the time parameters for PID
    RESET_SOFT_MOVEMENT;   // Reset soft movement
    if (speed != DEFAULT_MOTOR_SPEED) {
      setSpeed(DEFAULT_MOTOR_SPEED);
    }
    ALL_MOTORS(motors[motor].outOfRange =
                   false;)      // Reset out of range flag for all motors
    ALL_MOTORS_COMMAND(extend); // Send extend command to all motors
    systemDirection = Direction::EXTEND;    // Set system direction to extend
    requestedDirection = Direction::EXTEND; // Set requested direction to extend
    moveStart = micros();
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
    pid_on = true; ///< Set the PID flag to true
    stopping = false;
    pidController.reset(); // Reset the time parameters for PID
    RESET_SOFT_MOVEMENT;   ///< Reset the soft movement
    if (speed != DEFAULT_MOTOR_SPEED) {
      setSpeed(DEFAULT_MOTOR_SPEED);
    }
    ALL_MOTORS(motors[motor].outOfRange =
                   false;) ///< Set the outOfRange flag to false for all motors
    ALL_MOTORS_COMMAND(retract) ///< Set the command to retract for all motors
    systemDirection =
        Direction::RETRACT; ///< Set the system direction to retract
    requestedDirection =
        Direction::RETRACT; ///< Set the requested direction to retract
    moveStart = micros();
  }

  /**
   * @brief Stops the motorized system.
   *
   * This function stops the motorized system by resetting the soft movement and
   * setting the speed to 0.
   */
  void stop() {
    // Reset the soft movement
    stopping = true;
    pidController.reset(); // Reset the time parameters for PID
    RESET_SOFT_MOVEMENT;
    setSpeed(0);
    pidController.setParams(STOP_KP);

    // Update the requested direction to STOP
    requestedDirection = Direction::STOP;
    moveStart = micros();
  }

  /**
   * @brief Function to control the homing.
   *
   * This function retracts, checks leader and follower out of range status,
   * and updates the motor status until both motors are out of range.
   * It then zeros out the positions of the motors and resets the soft movement
   * status
   */
  void home() {
    // Print a message indicating that the home placeholder has been called
    Serial.println("Home placeholder called.");

    // Retract the motors
    pid_on = true;       ///< Set the PID flag to true
    RESET_SOFT_MOVEMENT; ///< Reset the soft movement
    speed = 100;
    targetSpeed = -1;
    ALL_MOTORS(motors[motor].outOfRange =
                   false;) ///< Set the outOfRange flag to false for all motors
    ALL_MOTORS_COMMAND(retract) ///< Set the command to retract for all motors
    systemDirection =
        Direction::RETRACT; ///< Set the system direction to retract
    requestedDirection =
        Direction::RETRACT; ///< Set the requested direction to retract

    long lastTimestamp = micros();
    long lastPrint = lastTimestamp + 1000;
    int currentAlarmStart = micros();
    const int currentAlarmDelay = 500000L;
    motors[LEADER].bottomCurrentLimit = minCurrent = 1600;
    motors[FOLLOWER].bottomCurrentLimit = minCurrent = 1600;
    bool followerBottomHit = false;
    bool leaderBottomHit = false;
    pid_on = false;
    ALL_MOTORS(motors[motor].speed = 100;)
    homing = true;
    ALL_MOTORS(motors[motor].homing = true;)

    // Loop until both leader and follower motors are out of range
    for (;;) {
      const long timestamp = micros();
      const int currentDeltaTime = timestamp - currentAlarmStart;
      // Check if it's time to update the motor current limits
      if (currentDeltaTime > currentAlarmDelay) {
        motors[LEADER].bottomCurrentLimit = 300;
        motors[FOLLOWER].bottomCurrentLimit = 300;
      }

      if (!leaderBottomHit) {
        leaderBottomHit = motors[LEADER].isStopped();
      }

      if (!followerBottomHit) {
        followerBottomHit = motors[FOLLOWER].isStopped();
      }

      Serial.printf("Leader bottom hit: %d\n", leaderBottomHit);
      Serial.printf("Follower bottom hit: %d\n", followerBottomHit);
      //  Check if both leader and follower motors are out of range
      if (leaderBottomHit && followerBottomHit) {
        Serial.println("Bottom hit.");
        pid_on = true;

        // Zero the leader and follower motors
        ALL_MOTORS_COMMAND(zero)

        // Disable all motors, reset soft movement, and set the direction to
        // STOP
        ALL_MOTORS_COMMAND(disable)
        RESET_SOFT_MOVEMENT systemDirection = requestedDirection =
            Direction::STOP;

        // Calculate the time delta and update the motor status
        const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
        update(deltaT);

        // Exit the loop
        break;
      }

      // Calculate the time delta and update the motor status
      const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
      lastTimestamp = timestamp;
      update(deltaT);

      if (timestamp - lastPrint > 250000) {
        report();
      }
    }

    Serial.printf("Backing up %d pulses\n", ALARM_REVERSE_AMOUNT);
    lastTimestamp = micros();
    // Extend slightly
    pid_on = true;       // Enable PID control
    RESET_SOFT_MOVEMENT; // Reset soft movement
    speed = 100;
    targetSpeed = -1;
    ALL_MOTORS(motors[motor].outOfRange =
                   false;) // Reset out of range flag for all motors
    ALL_MOTORS(motors[motor].speed =
                   100;)        // Reset out of range flag for all motors
    ALL_MOTORS_COMMAND(extend); // Send extend command to all motors
    systemDirection = Direction::EXTEND;    // Set system direction to extend
    requestedDirection = Direction::EXTEND; // Set requested direction to extend

    for (;;) {
      const long timestamp = micros();

      if (motors[LEADER].pos >= ALARM_REVERSE_AMOUNT ||
          motors[FOLLOWER].pos >= FOLLOWER_ALARM_REVERSE_AMOUNT) {
        // Zero the leader and follower motors
        ALL_MOTORS_COMMAND(zero)
        ALL_MOTORS(motors[motor].homing = false;)
        homing = false;

        // Disable all motors, reset soft movement, and set the direction to
        // STOP
        Serial.println("Stopping");
        RESET_SOFT_MOVEMENT;

        // Set the speed to 0
        speed = 0;
        targetSpeed = -1;
        ALL_MOTORS_COMMAND(disable)
        systemDirection = requestedDirection = Direction::STOP;

        // Calculate the time delta and update the motor status
        const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
        pidController.reset(); // Reset the time parameters for PID
        update(deltaT);
        break;
      }
      // Calculate the time delta and update the motor status
      const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
      update(deltaT);
    }
  }

  /**
   * @brief Set the speed of the motor.
   *
   * @param newSpeed The new speed value.
   */
  void setSpeed(int newSpeed) {

    targetSpeed = newSpeed;

    // Update the target speed
    Serial.printf("SetSpeed(%d)\n", newSpeed);
    Serial.printf("Speed: %d\n", speed);
    Serial.printf("Target speed: %d\n", targetSpeed);

    // Reset the soft start and last PWM update times
    softStart = lastPWMUpdate = micros();

    // Calculate the amount to update the PWM duty cycle per step
    pwmUpdateAmount =
        ceil((float)abs(targetSpeed - speed) / SOFT_MOVEMENT_UPDATE_STEPS);

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

    if (requestedDirection != Direction::RETRACT) {
      pidController.setParams(RETRACT_RAMP_KP);
    } else if (requestedDirection == Direction::EXTEND) {
      pidController.setParams(EXTEND_RAMP_KP);
    }
  }

  void setCurrentAlarmValue(const int alarmValue) {}

  /**
   * @brief Zeroes out the position count of all motors.
   *
   * @return void
   *
   * @throws None
   */
  void zero() { ALL_MOTORS_COMMAND(zero) }

  /**
   * @brief Reports the current state of the motor controller to the serial
   * console.
   *
   * @return void
   *
   * @throws None
   */
  void report() {
    ALL_MOTORS_COMMAND(readPos)
    Serial.printf("MotorController\n--------------------\nSpeed: %d\nTarget "
                  "Speed: %d\npwmUpdateAmount: %f\n",
                  speed, targetSpeed, pwmUpdateAmount);
    pidController.report();
    Serial.print("Leading motor: ");
    Serial.println(motor_roles[leadingIndex]);
    Serial.print("Lagging motor: ");
    Serial.println(motor_roles[laggingIndex]);
    Serial.printf("\n\n\n");
    printCurrent();

    ALL_MOTORS_COMMAND(displayInfo)
  }

  /**
   * @brief Prints the current values of the leader and follower motors.
   *
   * @return void
   */
  void printCurrent() {
    // Check if the motors are stopped
    if (!motorsStopped()) {
      // Get the current values of the leader and follower motors
      leaderCurrent = motors[LEADER].getCurrent();
      followerCurrent = motors[FOLLOWER].getCurrent();

      // Print the current values
      Serial.printf("Leader Current: %d\n", leaderCurrent);
      Serial.printf("Follower Current: %d\n", followerCurrent);
    }
  }

  /**
   * @brief Saves the position value for a given slot.
   *
   * @param slot The slot index.
   * @param position_value The position value to save.
   */
  void savePosition(const int slot, const int position_value) {
    // Make sure the slot index is within valid range and the position value is
    // non-negative.
    if (slot > 0 && slot < NUM_POSITION_SLOTS && position_value > -1) {
      // Set the position value for all motors.
      ALL_MOTORS(motors[motor].setPos(position_value);)

      // Store the position value in the savedPositions array.
      savedPositions[slot - 1] = position_value;

      // Store the position value in the positionStorage.
      positionStorage.putInt(save_position_slot_names[slot], position_value);
    }
  }

  /**
   * Set the desired position for the motors and move them accordingly.
   *
   * @param newPos The new desired position for the motors.
   */
  void setPos(const int newPos) {
    desiredPos = constrain(newPos, 0, motors[LEADER].totalPulseCount);

    // Check if the current position is less than the desired position
    if (motors[LEADER].pos < desiredPos) {
      extend();
    }
    // Check if the current position is greater than the desired position
    else if (motors[LEADER].pos > desiredPos) {
      retract();
    }
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

    // Get the current readings of the leader and follower motors
    leaderCurrent = motors[LEADER].getCurrent();
    followerCurrent = motors[FOLLOWER].getCurrent();

    // Calculate the time factor
    const double timeFactor = 1000000.0 / elapsedTime;

    // Calculate the velocities of the leader and follower currents
    leaderCurrentVelocity = static_cast<int>(
        (leaderCurrent - lastLeaderCurrent) * timeFactor + 0.5);
    followerCurrentVelocity = static_cast<int>(
        (followerCurrent - lastFollowerCurrent) * timeFactor + 0.5);
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
  bool motorsStopped() const {
    // Check if the leader motor is stopped
    bool isLeaderStopped = motors[LEADER].isStopped();

    // Check if the follower motor is stopped
    bool isFollowerStopped = motors[FOLLOWER].isStopped();

    // Return true if both motors are stopped
    return isLeaderStopped && isFollowerStopped;
  }

  /**
   * @brief Checks if the current alarm is triggered.
   *
   * @return true if the current alarm is triggered, false otherwise
   */
  bool currentAlarmTriggered() const {
    return motors[LEADER].isCurrentAlarm() || motors[FOLLOWER].isCurrentAlarm();
  }

  /**
   * @brief Check if the motors are close to the end of their range.
   *
   * @return True if either motor is close to the end of its range, false
   * otherwise.
   */
  bool motorsCloseToEndOfRange() {
    // Read the current position of all motors
    ALL_MOTORS_COMMAND(readPos)

    // Get the normalized position of the leader motor
    double leaderPos = motors[LEADER].getNormalizedPos();

    // Get the normalized position of the follower motor
    double followerPos = motors[FOLLOWER].getNormalizedPos();

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
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!ALARM!!!!!!!!!!!!!!!!!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    /*
    immediateHalt();
    systemDirection = requestedDirection = Direction::STOP;
    RESET_SOFT_MOVEMENT
    */
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
    Serial.printf("Leader current velocity: %d\n", leaderCurrentVelocity);
    Serial.printf("Follower Motor Current: %d\n", followerCurrent);
    Serial.printf("Follower current velocity: %d\n", followerCurrentVelocity);
  }

  /**
   * @brief Updates the state of the motor system.
   *
   * @param deltaT the time interval since the last update (default: 0.0f)
   *
   * @throws None
   */
  void update(const float deltaT = 0.0f) {
    // Get current time in microseconds
    const long currentTime = micros();
    // Calculate the time since the last current update
    const long currentUpdateDelta = currentTime - lastCurrentUpdate;

    if (Direction::STOP == systemDirection || motorsStopped()) {
      for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        motors[motor].disable();
        motors[motor].update();
      }
      return;
    }

    if (motorsCloseToEndOfRange() && !stopping &&
        speed != MOTOR_END_OF_RANGE_SPEED && targetSpeed < 0) {
      setSpeed(MOTOR_END_OF_RANGE_SPEED);
    }

    // Check if it's time to update the current readings
    if (currentUpdateDelta >= currentUpdateInterval) {
      // Check if the motors are not stopped
      if (!motorsStopped()) {
        updateCurrentReadings(currentUpdateDelta);
        // Update the last current update time
        lastCurrentUpdate = currentTime;

        // Display the current readings if debug is enabled
        displayCurrents();

        if ((currentTime - moveStart) > CURRENT_ALARM_DELAY &&
            currentAlarmTriggered()) {
          handleCurrentAlarm();
        }
      }
    }

    if (desiredPos != -1) {
      if (abs(desiredPos - motors[LEADER].pos) < 10) {
        if (systemState.debugEnabled) {
          Serial.printf("Desired Pos: %d - REACHED\n", desiredPos);
        }
        desiredPos = -1;
        stop();
      }
    }

    // Check if the soft movement system has a target speed
    if (targetSpeed >= 0) {
      // Distance from the current speed to the target speed
      const int speedDelta = abs(speed - targetSpeed);

      // The time since soft movement started
      const long moveTimeDelta = currentTime - softStart;

      // Calculate the time since the last PWM update
      const long updateTimeDelta = currentTime - lastPWMUpdate;

      if (updateTimeDelta >= SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS) {
        const bool timeToUpdate = moveTimeDelta < SOFT_MOVEMENT_MICROS;

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
          RESET_SOFT_MOVEMENT

          if (requestedDirection == Direction::STOP) {
            systemDirection = Direction::STOP;

            immediateHalt();
            stopping = false;
            report();
          }
        }
      }
    }

    if (pid_on) {
      updateLeadingAndLaggingIndicies();
      // Speed to set the faster motor to as calculated by the PID algorithm
      const int adjustedSpeed = pidController.adjustSpeed(
          motors[leadingIndex], motors[laggingIndex], speed, deltaT);

      motors[leadingIndex].speed = adjustedSpeed;
      motors[laggingIndex].speed = speed;
    } else {
      motors[leadingIndex].speed = speed;
      motors[laggingIndex].speed = speed;
    }

    ALL_MOTORS_COMMAND(update)
  }
};

#endif // _MOTOR_CONTROLLER_HPP_

/*! \file MotorController.hpp */

#ifndef _MOTOR_CONTROLLER_HPP_
#define _MOTOR_CONTROLLER_HPP_

#include <Preferences.h>
#include <math.h>

#include "CurrentSettings.hpp"
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
  lastPWMUpdate = -1;                                                          \
  softStart = -1;                                                              \
  targetSpeed = -1;                                                            \
  eIntegral = 0.0f;                                                            \
  currentUpdateInterval = 500000;

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

  const int motorPulseTotals[2] = {2800, 2800};

  /// @brief Hall sensor pulse totals for the motor travel limit feature
  // const int motorPulseTotals[2] = {2055, 2050};

  /// @brief The index in the motors array of the motor that is behind as
  /// indicated by the hall sensor
  int laggingIndex = 0;

  /// @brief The index in the motors array of the motor that is farther along as
  /// indicated by the hall sensor
  int leadingIndex = 0;

  /** The timestamp since soft start of movement */
  int softStart = -1;

  /** The last PWM update interval in microseconds */
  int lastPWMUpdate = -1;

  /** The target speed of soft movement */
  int targetSpeed = -1;

  /** The amount to change the PWM duty cycle on soft start */
  float pwmUpdateAmount = -1.0f;

  /** The last time a debug serial print was sent */
  int lastPrintTime = -1;

  /** Interval of time to pass between current updates microseconds  */
  int currentUpdateInterval = 500000;

  int lastCurrentUpdate = -1;

  // PIDController pidController;

  /// @brief The requested system level direction
  Direction requestedDirection = Direction::STOP;

  /** The frequency of the PWM signal in hertz */
  int pwmFrequency = PWM_FREQUENCY;

  /** The PWM bitdepth resolution */
  int pwmResolution = PWM_RESOLUTION_BITS;

  int desiredPos = -1;

  Preferences positionStorage; //

  void immediateHalt() {
    speed = targetSpeed = 0;
    systemDirection = Direction::STOP;
    requestedDirection = Direction::STOP;
    RESET_SOFT_MOVEMENT

    ALL_MOTORS_COMMAND(disable);
    currentUpdateInterval = 500000;
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
  /** The proprotional gain for the PID controller  */
  int K_p = 50000;

  /** The intagral gain for the PID controller  */
  float K_i = 0.1f;

  /** The integral error coefficient for the PID controller  */
  float eIntegral = 0.0f;

  /** The default speed to operate the
      motors at on startup */
  int defaultSpeed = DEFAULT_MOTOR_SPEED;

  /** Current target speed */
  int speed = 0;

  /** Leader motor current */
  int leaderCurrent = 0;

  /** Follower motor current */
  int followerCurrent = 0;

  /** Last leader current reading */
  int lastLeaderCurrent = 0;

  /** Last follower current reading */
  int lastFollowerCurrent = 0;

  /** Leader current velocity */
  int leaderCurrentVelocity = 0;

  /*  Follower current velocity */
  int followerCurrentVelocity = 0;

  /** Minimum current for alarm system for motors to enable  */
  int minCurrent = 850;

  /** Current delay for overcurrent alarm system */
  int currentAlarmDelay = 250000;

  /** Current velocity limit for alarm system for motors to enable  */
  int alarmCurrentVelocity = 10000;

  /** Maxim1um current increase limit for
            motor cutoff */
  int currentIncreaseTolerance = DEFAULT_CURRENT_INCREASE_LIMIT;

  /// @brief The motors controlled by this motor controller instance
  Motor motors[NUMBER_OF_MOTORS];

  /// @brief The current system level direction indicator
  Direction systemDirection = Direction::STOP;

  /// @brief This class controls the motors connected to the microcontoller
  /// @param pwmFrequency The frequency of the PWM signal to the motors
  /// @param pwmResolution The bitdepth resolution of the PWM signal to the
  /// motors
  /// @param defaultSpeed The default speed of the motors that the control
  /// program starts them off with
  MotorController(
      const int pwmFrequency = PWM_FREQUENCY,
      const int pwmResolution = PWM_RESOLUTION_BITS,
      const int defaultSpeed = DEFAULT_MOTOR_SPEED,
      const int currentIncreaseLimit = DEFAULT_CURRENT_INCREASE_LIMIT)
      : pwmFrequency(pwmFrequency), pwmResolution(pwmResolution),
        defaultSpeed(defaultSpeed),
        currentIncreaseTolerance(currentIncreaseLimit) {
    char buf[256];
    sprintf(
        buf,
        "Controller Params: Frequency: %d - Resolution: %d - Duty Cycle: %d\n",
        pwmFrequency, pwmResolution, defaultSpeed);
    Serial.println(buf);
    speed = targetSpeed = 0;
    Direction systemDirection = Direction::STOP;
    ALL_MOTORS(motors[motor].speed = 0;)
  }

  /// @brief Load the stored position preferences into RAM and initialize the
  /// motors
  void initialize() {
    // Read in saved positions
    // Open in read-write mode
    motors[0] =
        Motor("Leader", MotorPin::MOTOR1_RPWM_PIN, MotorPin::MOTOR1_LPWM_PIN,
              MotorPin::MOTOR1_R_EN_PIN, MotorPin::MOTOR1_L_EN_PIN,
              MotorPin::MOTOR1_HALL1_PIN, MotorPin::MOTOR1_HALL2_PIN,
              LEADER_CURRENT_SENSE_PIN, motorPulseTotals[0], PWM_FREQUENCY,
              defaultSpeed, pwmResolution, MOTOR1_LIMIT, MOTOR1_TLIMIT);

    motors[1] =
        Motor("Follower", MotorPin::MOTOR2_RPWM_PIN, MotorPin::MOTOR2_LPWM_PIN,
              MotorPin::MOTOR2_R_EN_PIN, MotorPin::MOTOR2_L_EN_PIN,
              MotorPin::MOTOR2_HALL1_PIN, MotorPin::MOTOR2_HALL2_PIN,
              FOLLOWER_CURRENT_SENSE_PIN, motorPulseTotals[1], PWM_FREQUENCY,
              defaultSpeed, pwmResolution, MOTOR2_LIMIT, MOTOR2_TLIMIT);

    positionStorage.begin("evox-tilt", false);
    loadPositions();
    initializeMotors();

    leaderCurrent = motors[LEADER].getCurrent();
    followerCurrent = motors[FOLLOWER].getCurrent();

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
    pid_on = true;          // Enable PID control
    RESET_SOFT_MOVEMENT;    // Reset soft movement
    setSpeed(defaultSpeed); // Set the speed to the default speed
    ALL_MOTORS(motors[motor].outOfRange =
                   false;)      // Reset out of range flag for all motors
    ALL_MOTORS_COMMAND(extend); // Send extend command to all motors
    systemDirection = Direction::EXTEND;    // Set system direction to extend
    requestedDirection = Direction::EXTEND; // Set requested direction to extend
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
    pid_on = true;          ///< Set the PID flag to true
    RESET_SOFT_MOVEMENT;    ///< Reset the soft movement
    setSpeed(defaultSpeed); ///< Set the speed to the default speed
    ALL_MOTORS(motors[motor].outOfRange =
                   false;) ///< Set the outOfRange flag to false for all motors
    ALL_MOTORS_COMMAND(retract) ///< Set the command to retract for all motors
    systemDirection =
        Direction::RETRACT; ///< Set the system direction to retract
    requestedDirection =
        Direction::RETRACT; ///< Set the requested direction to retract
  }

  /**
   * @brief Stops the motorized system.
   *
   * This function stops the motorized system by resetting soft movement and
   * setting the speed to 0.
   */
  void stop() {
    RESET_SOFT_MOVEMENT;
    setSpeed(0);
    requestedDirection = Direction::STOP;
  }

  /// @brief Home the lift columns
  ///
  /// This function retracts all the motors and then waits until both the leader
  /// and follower motors are out of range. Once both motors are out of range,
  /// the function zeros the leader and follower motors and exits the loop.
  ///
  /// @note This function assumes that the `motors` array is already defined and
  ///       contains the leader and follower motor objects.
  void home() {
    ALL_MOTORS_COMMAND(retract); ///< Retract all motors

    for (;;) {
      if (motors[LEADER].outOfRange && motors[FOLLOWER].outOfRange) {
        motors[LEADER].zero();   ///< Zero the leader motor
        motors[FOLLOWER].zero(); ///< Zero the follower motor
        break;
      }
      update(); ///< Update the motor status
    }
  }

  /**
   * Set the speed of the motor.
   *
   * @param newSpeed The new speed value.
   */
  void setSpeed(int newSpeed) {
    // Ensure that the new speed is at least 75
    if (newSpeed < 75) {
      newSpeed = 75;
    }

    // Update the target speed
    targetSpeed = newSpeed;

    // Reset the soft start and last PWM update times
    softStart = lastPWMUpdate = micros();

    // If the speed is less than the minimum, set to the minimum
    if (speed < 75) {
      speed = 75;
    }

    // Calculate the amount to update the PWM duty cycle per step
    pwmUpdateAmount =
        ceil((float)abs(targetSpeed - speed) / SOFT_MOVEMENT_UPDATE_STEPS);

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
  }

  /// @brief Get the system speed
  /// @return The average speed of the system
  int getSpeed() const {
    // Return the average
    return (motors[0].speed + motors[1].speed) / 2;
  }

  /// @brief Indicates whether the motor counts are unequal
  /// @return True if the motor counts are different, false otherwise
  bool countsAreUnequal(void) const {
    bool areUnequal = true;
    ALL_MOTORS(areUnequal &= motors[motor].pos == motors[motor].lastPos;)
    return areUnequal;
  }

  /// @brief Report debugging information to the serial console
  void report() {
    ALL_MOTORS_COMMAND(readPos)
    Serial.printf(
        "MotorController\n--------------------\nSpeed: %d\nTarget "
        "Speed: %d\nK_p: %d\nK_i: %f\neIntegral:%f\npwmUpdateAmont: %f \n",
        speed, targetSpeed, K_p, K_i, eIntegral, pwmUpdateAmount);
    Serial.print("Leading motor: ");
    Serial.println(motor_roles[leadingIndex]);
    Serial.print("Lagging motor: ");
    Serial.println(motor_roles[laggingIndex]);
    Serial.printf("\n\n\n");
    printCurrent();

    ALL_MOTORS_COMMAND(displayInfo)
  }

  /*
  void pidReport(const float deltaT) const {
    int leaderPos = motors[0].pos;
    int followerPos = motors[1].pos;
    int followerSpeed = motors[1].speed;
    Direction dir = motors[1].dir;

    pidController.report(leaderPos, followerPos, deltaT, followerSpeed, dir);
  }

  */

  /**
   * Prints the current values of the leader and follower motors.
   * Only prints the values if the motors are not stopped.
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

  /// @brief Save a position to the preferences slot
  /// @param slot The selected slot to save the position information to
  /// @param position_value The position value in hall sensor pulses to save to
  /// the selected slot
  void savePosition(const int slot, const int position_value) {
    if (slot > 0 && slot < NUM_POSITION_SLOTS && position_value > -1) {
      ALL_MOTORS(motors[motor].setPos(position_value);)
      savedPositions[slot - 1] = position_value;
      positionStorage.putInt(save_position_slot_names[slot], position_value);
    }
  }

  /// @brief Move the motors to the given position
  /// @param newPos The new target position for the motors in hall sensor pulses
  void setPos(const int newPos) {
    desiredPos = newPos;

    if (motors[LEADER].pos < desiredPos) {
      extend();
    } else if (motors[LEADER].pos > desiredPos) {
      retract();
    }
  }

  // Update the current readings of the motors
  // Parameters:
  // - elapsedTime: the elapsed time in microseconds
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

  /// @brief Check whether the system is in a STOP state
  /// @return True if the system is in a STOP state, else false
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

  /// @brief Perform one update interval for the motor system
  /// @param deltaT The amount of time that has passed since the last update

  /**
   * Checks if the current alarm is triggered.
   *
   * @return true if the current alarm is triggered, false otherwise
   */
  bool currentAlarmTriggered() {

    // Initialize variable to store alarm status
    bool currentAlarm = false;

    // Check if leader motor current is above minimum and system direction is
    // retract
    if ((leaderCurrent > minCurrent) && systemDirection == Direction::RETRACT) {
      // Set leader motor outOfRange flag to true
      motors[LEADER].outOfRange = true;
      // Set current alarm to true
      currentAlarm = true;
    }

    // Check if follower motor current is above minimum and system direction is
    // retract
    if ((followerCurrent > minCurrent) &&
        systemDirection == Direction::RETRACT) {
      // Set follower motor outOfRange flag to true
      motors[FOLLOWER].outOfRange = true;
      // Set current alarm to true
      currentAlarm = true;
    }

    // Return the current alarm status
    return currentAlarm;
  }

  /**
   * Check if the motors are close to the end of their range.
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
    bool leaderCloseToEnd =
        (leaderPos < 0.1 && motors[LEADER].dir == Direction::RETRACT) ||
        (leaderPos > 0.9 && motors[LEADER].dir == Direction::EXTEND);

    // Check if the follower motor is close to the end of its range
    bool followerCloseToEnd =
        (followerPos < 0.1 && motors[FOLLOWER].dir == Direction::RETRACT) ||
        (followerPos > 0.9 && motors[FOLLOWER].dir == Direction::EXTEND);

    // Return true if either motor is close to the end of its range
    return leaderCloseToEnd || followerCloseToEnd;
  }

  /**
   * Disable all motors, reset speed and direction variables,
   * and turn off PID control. Print debug message if debugEnabled is true.
   */
  void handleCurrentAlarm() {
    ALL_MOTORS_COMMAND(disable);
    speed = targetSpeed = 0;
    systemDirection = requestedDirection = Direction::STOP;
    pid_on = false;

    if (debugEnabled) {
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!ALARM!!!!!!!!!!!!!!!!!!");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
  }

  void updateLeadingAndLaggingIndicies() {
    if (Direction::EXTEND == systemDirection) {
      if (motors[LEADER].getNormalizedPos() <
          motors[FOLLOWER].getNormalizedPos()) {
        laggingIndex = MotorRoles::LEADER;
        leadingIndex = MotorRoles::FOLLOWER;
      } else {
        laggingIndex = MotorRoles::FOLLOWER;
        leadingIndex = MotorRoles::LEADER;
      }
    } else if (Direction::RETRACT == systemDirection) {
      if (motors[LEADER].getNormalizedPos() >
          motors[FOLLOWER].getNormalizedPos()) {
        laggingIndex = MotorRoles::LEADER;
        leadingIndex = MotorRoles::FOLLOWER;
      } else {
        laggingIndex = MotorRoles::FOLLOWER;
        leadingIndex = MotorRoles::LEADER;
      }
    }
  }

  /**
   * Updates the state of the motor system.
   *
   * @param deltaT the time interval since the last update (default: 0.0f)
   *
   * @throws None
   */
  void update(const float deltaT = 0.0f) {
    if (motorsCloseToEndOfRange()) {
      currentUpdateInterval = 10000;
    }

    const int currentTime = micros();
    const int currentUpdateDelta = currentTime - lastCurrentUpdate;

    if (currentUpdateDelta >= currentUpdateInterval) {
      updateCurrentReadings(currentUpdateDelta);
      lastCurrentUpdate = currentTime;
      if (!motorsStopped()) {
        Serial.printf("Leader Motor Current: %d\n", leaderCurrent);
        Serial.printf("Leader current velocity: %d\n", leaderCurrentVelocity);
        Serial.printf("Follower Motor Current: %d\n", followerCurrent);
        Serial.printf("Follower current velocity: %d\n",
                      followerCurrentVelocity);
      }
    }

    if (Direction::STOP == systemDirection) {
      for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        motors[motor].disable();
        motors[motor].update();
      }
      return;
    }

    const int speedDelta = abs(speed - targetSpeed);
    const int moveTimeDelta = currentTime - softStart;
    const int updateTimeDelta = currentTime - lastPWMUpdate;

    if (desiredPos != -1) {
      if (abs(desiredPos - motors[LEADER].pos) < 20) {
        if (debugEnabled) {
          Serial.printf("Desired Pos: %d - REACHED\n", desiredPos);
        }
        desiredPos = -1;
        stop();
      }
    }

    if (targetSpeed >= 0) {
      if (updateTimeDelta < SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS) {
        return;
      }

      if (speedDelta >= abs(pwmUpdateAmount) &&
          moveTimeDelta < SOFT_MOVEMENT_MICROS) {
        const float newSpeed = (float)speed + pwmUpdateAmount;
        speed = (int)floorf(newSpeed);
        lastPWMUpdate = micros();

        const double timeSinceSoftStart =
            (double)(micros() - softStart) / (double)MICROS_IN_MS;

        if (false) {
          Serial.printf("Soft Movement PWM Update - "
                        "speed: %d - "
                        "target speed: %d - "
                        "time since soft start: %f ms - "
                        "pwmUpdateAmount: %f\n",
                        speed, targetSpeed, timeSinceSoftStart,
                        pwmUpdateAmount);
        }
      } else {
        speed = targetSpeed;
        RESET_SOFT_MOVEMENT

        if (requestedDirection == Direction::STOP) {
          systemDirection = Direction::STOP;

          if (false) {
            Serial.println("System Direction: STOP");
            const double timeSinceSoftStart =
                (double)(micros() - softStart) / (double)MICROS_IN_MS;

            Serial.printf("Soft Movement PWM Update - "
                          "speed: %d - "
                          "target speed: %d - "
                          "time since soft start: %f ms - "
                          "pwmUpdateAmount: %f\n",
                          speed, targetSpeed, timeSinceSoftStart,
                          pwmUpdateAmount);
          }

          immediateHalt();
        }
      }
    }

    if (pid_on) {
      const float error = abs(motors[laggingIndex].getNormalizedPos() -
                              motors[leadingIndex].getNormalizedPos());

      eIntegral += error * deltaT;

      const int adjustedSpeed = speed - int((error * K_p));

      motors[leadingIndex].speed =
          constrain(adjustedSpeed, 0, 2 << pwmResolution);
      motors[laggingIndex].speed = speed;
    }

    if (currentAlarmTriggered()) {
      handleCurrentAlarm();
    }

    for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
      motors[motor].update();
    }
  }
};

#endif // _MOTOR_CONTROLLER_HPP_

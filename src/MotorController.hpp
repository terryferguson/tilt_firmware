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
  unsigned long leaderLastPosChange = 0UL;

  /** @brief The time of the last position change of the follower motor */
  unsigned long followerLastPosChange = 0UL;

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
      savedConfigurations[slot] =
          positionStorage.getInt(save_configuration_slot_names[slot]);
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
  unsigned long endOfrangeTimes[2] = {SOFT_MOVEMENT_TIME_MS, SOFT_STOP_TIME_MS};

public:
  /// @enum MotorRoles
  /// @brief The roles of this motor for this PID control
  enum MotorRoles {
    LEADER,  /** The leader motor that is speed-matched */
    FOLLOWER /** The follower motor that is speed-matched to the leader motor */
  };

  double deltaT = 0.0;

  /** @brief The time in microseconds since a motor movement started */
  unsigned long moveStart = 0UL;

  /** @brief The target speed of soft movement */
  int targetSpeed = -1;

  int intermediateSpeed = -1;

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
  unsigned long currentAlarmDelay = 250000UL;

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
  unsigned long softStart = 0UL;

  /** @brief The last PWM update interval in microseconds */
  unsigned long lastPWMUpdate = 0UL;

  /** @brief The amount to change the PWM duty cycle on soft start */
  float pwmUpdateAmount = -1.0f;

  /** @brief The last time a debug serial print was sent */
  unsigned long lastPrintTime = 0UL;

  /** @brief Interval of time to pass between current updates microseconds  */
  unsigned long currentUpdateInterval = CURRENT_UPDATE_INTERVAL;

  /** @brief Time in microseconds since the last current update */
  unsigned long lastCurrentUpdate = 0UL;

  unsigned long softMovingTime = 0UL;

  /** @brief Input filter for leader current readings */
  EMA<2> leaderCurrentFilter;

  /** @brief Input filter for follower current readings */
  EMA<2> followerCurrentFilter;

  /** @brief Number of current samples */
  int_fast32_t samples = 0;

  /** @brief Leader current sum */
  int_fast32_t leaderCurrentSum = 0;

  /** @brief Follower current sum */
  int_fast32_t followerCurrentSum = 0;

  /** @brief Time since this movement began in microseconds */
  unsigned long moveTimeDelta = 0UL;

  /** @brief Time since last PWM update on soft start in microseconds */
  unsigned long updateTimeDelta = 0UL;

  /** @brief Whether the current alarm has been triggered */
  bool alarmTriggered = false;

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
  MotorController(const int pwmFrequency, const int pwmResolution,
                  const int defaultSpeed);

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
  void initialize();

  /**
   * @brief Extends the motorized system.
   *
   * This function enables PID control, resets soft movement,
   * sets the speed to the default speed, resets the out of range
   * flag for all motors, sends the extend command to all motors,
   * and sets the system direction and requested direction to extend.
   */
  void extend();

  /**
   * @brief Retracts the motorized system.
   *
   * This function is used to tell the motorized system to retract.
   * It sets the PID flag to true, resets the soft movement,
   * sets the speed to the default speed, sets the outOfRange flag
   * to false for all motors, sets the command to retract for all motors,
   * sets the system direction and requested direction to retract.
   */
  void retract();

  void clearOutOfRange();

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
  void immediateHalt();

  /**
   * Set the outOfRange flag to false and homing range to true for all motors.
   *
   * @throws ErrorType description of error
   */
  void setHoming();

  void hardSetSpeed();

  void clearHoming();

  /**
   * @brief Set the speed of the motor.
   *
   * @param newSpeed The new speed value.
   */
  void setSpeed(const int newSpeed,
                const int softMovementTime = SOFT_MOVEMENT_TIME_MS);

  /**
   * Clears the position change variables for the leader and follower.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void clearPositionChange();

  /**
   * @brief Zeroes out the position count of all motors.
   *
   * @return void
   *
   * @throws None
   */
  void zero();

  /**
   * @brief Reports the current state of the motor controller to the serial
   * console.
   *
   * @return void
   *
   * @throws None
   */
  void report();

  /**
   * @brief Saves the position value for a given slot.
   *
   * @param slot The slot index.
   * @param position_value The position value to save.
   */
  void savePosition(const int slot, const int position_value = -1);

  void saveConfiguration(const int slot, const int position_value = -1);

  /**
   * Set the desired position for the motors and move them accordingly.
   *
   * @param newPos The new desired position for the motors.
   */
  void setPos(const int newPos);

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
  void updateCurrentReadings(const int elapsedTime);

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
  bool motorsStopped();

  /**
   * @brief Checks if the current alarm is triggered.
   *
   * @return true if the current alarm is triggered, false otherwise
   */
  bool currentAlarmTriggered();

  /**
   * Check if the motors are desynchronized.
   *
   * @return true if the motors are desynchronized, false otherwise
   */
  bool motorsDesynced(void) const;

  void handlePid() { (this->*pidFuncts[static_cast<int>(pid_on)])(); }

  /**
   * @brief Check if the motors are close to the end of their range.
   *
   * @return True if either motor is close to the end of its range, false
   * otherwise.
   */
  bool motorsCloseToEndOfRange();

  /**
   * @brief Disable all motors, reset speed and direction variables,
   * and turn off PID control. Print debug message if debugEnabled is true.
   */
  void handleCurrentAlarm();

  /**
   * @brief Update the leading and lagging indices based on the system
   * direction.
   *
   * @return void
   */
  void updateLeadingAndLaggingIndicies();

  /**
   * @brief Displays the current values of the leader and follower motor
   * currents and velocities.
   *
   * @return void
   */
  void displayCurrents();

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
  void setCurrentLimit();

  /**
   * @brief Check if a desired position is set and clear it if we have reached
   * it.
   *
   * @throws None
   */
  void checkIfSetPositionReached();

  /**
   * Handles the current update.
   *
   * @return void
   */
  void handleCurrentUpdate();

  /**
   * Reset the soft movement.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void resetSoftMovement();

  /**
   * @brief Reset the current information for the system.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void resetCurrentInformation();

  /**
   * @brief Reset the current alarms for the motors.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void resetMotorCurrentAlarms();

  /**
   * @brief Disable the motors.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void disableMotors();

  /**
   * @brief Update the motors.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void updateMotors();

  /**
   * @brief Reset the PID controller.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void resetPid();

  void setPidParams(const int Kp = DEFAULT_KP, const float Ki = DEFAULT_KI,
                    const float Kd = DEFAULT_KD);

  /**
   * Set the bottom current limit for both the leader and follower motors.
   *
   * @param currentLimit the current limit to set
   *
   * @throws ErrorType if an error occurs while setting the current limit
   */
  void setBottomCurrentLimit(const int currentLimit);

  int getPos() const { return motors[LEADER].pos; }

  Direction getRestoreDirection() const;

  bool
  motorsNearDesiredPosition(const int maxDelta = DESIRED_POSITION_BUFFER) const;

  void resetDesiredPosition() { desiredPos = -1; }

  int getDesiredPosition() const { return desiredPos; }
};

#endif // _MOTOR_CONTROLLER_HPP_

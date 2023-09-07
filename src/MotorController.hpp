/*! \file MotorController.hpp */

#ifndef _MOTOR_CONTROLLER_HPP_
#define _MOTOR_CONTROLLER_HPP_

#include <math.h>
#include <Preferences.h>

#include "CurrentSettings.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "PinMacros.hpp"
#include "defs.hpp"

#define NUMBER_OF_MOTORS 2

#define ALL_MOTORS(operation)                            \
  for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++) \
  {                                                      \
    operation                                            \
  }

#define ALL_MOTORS_COMMAND(command) ALL_MOTORS(motors[motor].command();)

#define RESET_SOFT_MOVEMENT \
  pwmUpdateAmount = 0;      \
  lastPWMUpdate = -1;       \
  softStart = -1;           \
  targetSpeed = -1;         \
  eIntegral = 0.0f;

#define RESTORE_POSITION(slot) motor_controller.setPos(savedPositions[slot]);

#define SERIAL_SAVE_POSITION(slot)                \
  if (Serial.available() > 0)                     \
  {                                               \
    int new_pos = Serial.parseInt();              \
    motor_controller.savePosition(slot, new_pos); \
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
class MotorController
{
private:
  /// @enum MotorRoles
  /// @brief The roles of this motor for this PID control
  enum MotorRoles
  {
    LEADER,  /** The leader motor that is speed-matched */
    FOLLOWER /** The follower motor that is speed-matched to the leader motor */
  };

  const int motorPulseTotals[2] = {2600, 2600};

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

  /** Interval of time to pass between debug serial prints in microseconds  */
  const int printDelta = 500000;

  // PIDController pidController;

  /// @brief The requested system level direction
  Direction requestedDirection = Direction::STOP;

  /** The frequency of the PWM signal in hertz */
  int pwmFrequency = PWM_FREQUENCY;

  /** The PWM bitdepth resolution */
  int pwmResolution = PWM_RESOLUTION_BITS;

  int initialCurrentReadings[NUMBER_OF_MOTORS] = {0, 0};

  int desiredPos = -1;

  Preferences positionStorage; //

  void immediateHalt()
  {
    speed = targetSpeed = 0;
    systemDirection = Direction::STOP;
    requestedDirection = Direction::STOP;
    RESET_SOFT_MOVEMENT

    ALL_MOTORS(motors[motor].speed = 0;)
    ALL_MOTORS(motors[motor].dir = Direction::STOP;)
  }

  /// @brief Load the position preferences slots from ROM into memory
  void loadPositions()
  {
    for (int slot = 0; slot < NUM_POSITION_SLOTS; slot++)
    {
      savedPositions[slot] =
          positionStorage.getInt(save_position_slot_names[slot]);
    }
  }

  /// @brief Initialize the motors and then set them to halt immediately
  void initializeMotors()
  {
    RESET_SOFT_MOVEMENT
    ALL_MOTORS_COMMAND(initialize)
    immediateHalt();
  }

public:
  /** The proprotional gain for the PID controller  */
  int K_p = 10000;

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

  /** Minimum current for alarm system for motors to enable  */
  int minCurrent = 1000;

  /** Current velocity limit for alarm system for motors to enable  */
  int alarmCurrentVelocity = 200;

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
  MotorController(const int pwmFrequency = PWM_FREQUENCY,
                  const int pwmResolution = PWM_RESOLUTION_BITS,
                  const int defaultSpeed = DEFAULT_MOTOR_SPEED,
                  const int currentIncreaseLimit = DEFAULT_CURRENT_INCREASE_LIMIT)
      : pwmFrequency(pwmFrequency), pwmResolution(pwmResolution),
        defaultSpeed(defaultSpeed),
        currentIncreaseTolerance(currentIncreaseLimit)
  {
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
  void initialize()
  {
    // Read in saved positions
    // Open in read-write mode
    motors[0] =
        Motor("Leader", MotorPin::MOTOR1_RPWM_PIN, MotorPin::MOTOR1_LPWM_PIN,
              MotorPin::MOTOR1_R_EN_PIN, MotorPin::MOTOR1_L_EN_PIN,
              MotorPin::MOTOR1_HALL1_PIN, MotorPin::MOTOR1_HALL2_PIN,
              LEADER_CURRENT_SENSE_PIN,
              motorPulseTotals[0], PWM_FREQUENCY, defaultSpeed, pwmResolution);

    motors[1] =
        Motor("Follower", MotorPin::MOTOR2_RPWM_PIN, MotorPin::MOTOR2_LPWM_PIN,
              MotorPin::MOTOR2_R_EN_PIN, MotorPin::MOTOR2_L_EN_PIN,
              MotorPin::MOTOR2_HALL1_PIN, MotorPin::MOTOR2_HALL2_PIN,
              FOLLOWER_CURRENT_SENSE_PIN,
              motorPulseTotals[1], PWM_FREQUENCY, defaultSpeed, pwmResolution);

    positionStorage.begin("evox-tilt", false);
    loadPositions();
    initializeMotors();
    Serial.println("System initialized.");

    ALL_MOTORS(initialCurrentReadings[motor] = motors[motor].getCurrent();)
  }

  /// @brief Tell the motorized system to extend
  void extend()
  {
    // SET_TO_ANALOG_PIN_FUNC(SPEED_POT_PIN, this->setSpeed, 0, 2 <<
    // PWM_RESOLUTION_BITS - 1);
    setSpeed(defaultSpeed);
    ALL_MOTORS_COMMAND(extend)
    systemDirection = Direction::EXTEND;
    requestedDirection = Direction::EXTEND;
  }

  /// @brief Tell the motorized system to retract
  void retract()
  {
    // SET_TO_ANALOG_PIN_FUNC(SPEED_POT_PIN, this->setSpeed, 0, 2 <<
    // PWM_RESOLUTION_BITS - 1);
    setSpeed(defaultSpeed);
    ALL_MOTORS_COMMAND(retract)
    systemDirection = Direction::RETRACT;
    requestedDirection = Direction::RETRACT;
  }

  /// @brief Tell the motorized system to stop
  void stop()
  {
    RESET_SOFT_MOVEMENT

    setSpeed(0);
    requestedDirection = Direction::STOP;
  }

  /// @brief Home the linear actuator to recalibrate the position sensor
  /// void home() { ALL_MOTORS_COMMAND(home) }

  /// @brief Clear all position information
  void zero() { ALL_MOTORS_COMMAND(zero) }

  /// @brief Smoothly change to the newly requested speed
  /// @param newSpeed The new speed to target
  void setSpeed(int newSpeed)
  {
    targetSpeed = newSpeed;
    softStart = lastPWMUpdate = micros();

    // Calculate the difference between the current speed and the requested
    // speed and divide that difference by the number of update steps to get
    // the PWM duty cycle increase/decrease per step.
    //
    // This will usually have a fractional part, so we make it a float value. We
    // handle the rounding and conversion to an integer in the update method.
    pwmUpdateAmount =
        ceil((float)abs(targetSpeed - speed) / SOFT_MOVEMENT_UPDATE_STEPS);

    // If the new speed is lower, make it negative, as we add the
    // pwmUpdateAmount to the speed
    if (targetSpeed < speed)
    {
      pwmUpdateAmount = -pwmUpdateAmount;
    }

    if (debugEnabled)
    {
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
  int getSpeed() const
  {
    // Return the average
    return (motors[0].speed + motors[1].speed) / 2;
  }

  /// @brief Indicates whether the motor counts are unequal
  /// @return True if the motor counts are different, false otherwise
  bool countsAreUnequal(void) const
  {
    bool areUnequal = true;
    ALL_MOTORS(areUnequal &= motors[motor].pos == motors[motor].lastPos;)
    return areUnequal;
  }

  /// @brief Report debugging information to the serial console
  void report()
  {
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

  void printCurrent()
  {
    leaderCurrent = motors[LEADER].getCurrent();
    followerCurrent = motors[FOLLOWER].getCurrent();

    Serial.printf("Leader Current: %d\n", leaderCurrent);
    Serial.printf("Follower Current: %d\n", followerCurrent);
  }

  /// @brief Save a position to the preferences slot
  /// @param slot The selected slot to save the position information to
  /// @param position_value The position value in hall sensor pulses to save to
  /// the selected slot
  void savePosition(const int slot, const int position_value)
  {
    if (slot > 0 && slot < NUM_POSITION_SLOTS && position_value > -1)
    {
      ALL_MOTORS(motors[motor].setPos(position_value);)
      savedPositions[slot - 1] = position_value;
      positionStorage.putInt(save_position_slot_names[slot], position_value);
    }
  }

  /// @brief Move the motors to the given position
  /// @param newPos The new target position for the motors in hall sensor pulses
  void setPos(const int newPos)
  {
    desiredPos = newPos;

    if (motors[LEADER].pos < desiredPos)
    {
      extend();
    }
    else if (motors[LEADER].pos > desiredPos)
    {
      retract();
    }
  }

  void doPid(const float deltaT = 0.0f)
  {
    const float error = abs(motors[laggingIndex].getNormalizedPos() -
                            motors[leadingIndex].getNormalizedPos());

    eIntegral += error * deltaT;

    const int adjustedSpeed = speed - int((error * K_p));

    motors[leadingIndex].speed = constrain(adjustedSpeed, 0, 2 << pwmResolution);
    motors[laggingIndex].speed = speed;
  }

  /// @brief Check whether the system is in a STOP state
  /// @return True if the system is in a STOP state, else false
  bool isStopped() const { return systemDirection == Direction::STOP; }

  /// @brief Perform one update interval for the motor system
  /// @param deltaT The amount of time that has passed since the last update

  /**
   * Checks if the current alarm is triggered.
   *
   * @return true if the current alarm is triggered, false otherwise
   */
  bool currentAlarmTriggered() const
  {
    if ((leaderCurrent > minCurrent || followerCurrent > minCurrent) && systemDirection != Direction::STOP)
    {
      return true;
    }
    return false;
  }

  void update(const float deltaT = 0.0f)
  {
    if (Direction::STOP == systemDirection)
    {
      for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++)
      {
        motors[motor].disable();
        motors[motor].update();
      }
      return;
    }

    if (Direction::EXTEND == systemDirection)
    {
      if (motors[LEADER].getNormalizedPos() <
          motors[FOLLOWER].getNormalizedPos())
      {
        laggingIndex = MotorRoles::LEADER;
        leadingIndex = MotorRoles::FOLLOWER;
      }
      else
      {
        laggingIndex = MotorRoles::FOLLOWER;
        leadingIndex = MotorRoles::LEADER;
      }
    }
    else if (Direction::RETRACT == systemDirection)
    {
      if (motors[LEADER].getNormalizedPos() >
          motors[FOLLOWER].getNormalizedPos())
      {
        laggingIndex = MotorRoles::LEADER;
        leadingIndex = MotorRoles::FOLLOWER;
      }
      else
      {
        laggingIndex = MotorRoles::FOLLOWER;
        leadingIndex = MotorRoles::LEADER;
      }
    }

    if (!pid_on)
    {
      motors[leadingIndex].speed = 75;
      motors[laggingIndex].speed = 75;
      ALL_MOTORS_COMMAND(update);
      return;
    }

    const int speedDelta = abs(speed - targetSpeed);
    const int currentTime = micros();
    const int moveTimeDelta = currentTime - softStart;
    const int updateTimeDelta = currentTime - lastPWMUpdate;

    if (desiredPos != -1)
    {
      if (abs(desiredPos - motors[LEADER].pos) < 20)
      {
        if (debugEnabled)
        {
          Serial.printf("Desired Pos: %d - REACHED\n", desiredPos);
        }
        desiredPos = -1;
        stop();
      }
    }

    if (targetSpeed >= 0)
    {
      if (updateTimeDelta < SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS)
      {
        return;
      }

      if (speedDelta >= abs(pwmUpdateAmount) &&
          moveTimeDelta < SOFT_MOVEMENT_MICROS)
      {
        const float newSpeed = (float)speed + pwmUpdateAmount;
        speed = (int)floorf(newSpeed);
        lastPWMUpdate = micros();

        const double timeSinceSoftStart =
            (double)(micros() - softStart) / (double)MICROS_IN_MS;

        if (false)
        {
          Serial.printf("Soft Movement PWM Update - "
                        "speed: %d - "
                        "target speed: %d - "
                        "time since soft start: %f ms - "
                        "pwmUpdateAmount: %f\n",
                        speed, targetSpeed, timeSinceSoftStart, pwmUpdateAmount);
        }
      }
      else
      {
        speed = targetSpeed;
        RESET_SOFT_MOVEMENT

        if (requestedDirection == Direction::STOP)
        {
          systemDirection = Direction::STOP;

          if (false)
          {
            Serial.println("System Direction: STOP");
            const double timeSinceSoftStart =
                (double)(micros() - softStart) / (double)MICROS_IN_MS;

            Serial.printf("Soft Movement PWM Update - "
                          "speed: %d - "
                          "target speed: %d - "
                          "time since soft start: %f ms - "
                          "pwmUpdateAmount: %f\n",
                          speed, targetSpeed, timeSinceSoftStart, pwmUpdateAmount);
          }

          immediateHalt();
        }
      }
    }

    if (pid_on)
    {
      doPid(deltaT);
    }

    if (currentAlarmTriggered())
    {
      stop();
      Serial.printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      Serial.printf("!!!!!!!!!!!!!!!!!!!!!!!!ALARM!!!!!!!!!!!!!!!!!!\n");
      Serial.printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    }

    for (int motor = 0; motor < NUMBER_OF_MOTORS; motor++)
    {
      if (motors[motor].pos < motors[motor].totalPulseCount && motors[motor].dir == Direction::EXTEND || motors[motor].pos > 0 && motors[motor].dir == Direction::RETRACT)
      {
        motors[motor].update();
      }
      else
      {
        motors[motor].disable();
        motors[motor].update();
      }
    }
  }
};

#endif // _MOTOR_CONTROLLER_HPP_

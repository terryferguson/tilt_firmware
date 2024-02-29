/*! \file Motor.hpp */

#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include "ControlPins.hpp"
#include "CurrentSense.hpp"
#include "PinMacros.hpp"
#include "defs.hpp"
#include <ESP32Encoder.h>
#include <cstring>
#include <driver/adc.h>

#define MOTOR1_LIMIT 32
#define MOTOR2_LIMIT 33

#define MOVE_TO_POS(setpoint, min_delta, buffer)                               \
  if (abs(pos - setpoint) > min_delta) {                                       \
    if (pos < setpoint) {                                                      \
      desiredPos = setpoint - buffer;                                          \
    } else if (pos > newPos) {                                                 \
      desiredPos = setpoint + buffer;                                          \
    }                                                                          \
  }

/** @class Motor
 *
 *  @brief This class represents the motor controlled by the microcontroller
 *
 * @author Terry Paul Ferguson
 * @version 0.1
 */
class Motor {
private:
  char id[16];          /** The name of this motor for debug prints */
  int pwmRChannel = -1; /** The right PWM channel - Extension */
  int pwmLChannel = -1; /** The left PWM channel - Retraction */
  MotorPin rPWM_Pin = MotorPin::UNASSIGNED;   /** The right PWM pin */
  MotorPin lPWM_Pin = MotorPin::UNASSIGNED;   /** The left PWM pin */
  MotorPin r_EN_Pin = MotorPin::UNASSIGNED;   /** The right PWM enable pin */
  MotorPin l_EN_Pin = MotorPin::UNASSIGNED;   /** The left PWM enable pin */
  MotorPin hall_1_Pin = MotorPin::UNASSIGNED; /** The pin for hall sensor 1 */
  MotorPin hall_2_Pin = MotorPin::UNASSIGNED; /** The pin for hall sensor 2 */
  MotorPin l_is_pin =
      MotorPin::UNASSIGNED; /** The pin for left PWM current sensor */
  MotorPin r_is_pin =
      MotorPin::UNASSIGNED; /** The pin for right PWM current sensor */
  adc1_channel_t currentSensePin = ADC1_CHANNEL_0; /** The current sense pin */
  int frequency = PWM_FREQUENCY; /** The frequency of the PWM signal in hertz */
  int pwmResolution = 8;         /** The PWM bitdepth resolution */
  int desiredPos =
      -1; /** The desired position of the motor (-1 if none exists) */

  ESP32Encoder distanceSensor; /** The motor position encoder (quadrature signal
                                  from 2 hall sensors) */

  CurrentSense currentSense; /** The current sensor */

public:
  /** The current position of the motor based on hall sensor pulses */
  int pos = 0;
  /** The last position of the motor based on hall sensor pulses */
  int lastPos = 0;
  /** The current speed of the motor. The duty cycle of the PWM
                      signal is speed/(2^pwmResolution - 1) */
  int speed = DEFAULT_MOTOR_SPEED;
  /** The total number of pulses from full retraction
                              to full extension */
  int totalPulseCount = 0;

  int stopBuffer = 0;

  int bottomLimitPin = -1;
  int bottomCurrentLimit = -1;
  int currentAlarmLimit = -1;

  bool outOfRange = false;

  Direction dir = Direction::STOP; /** The direction of the motor rotation */
  bool homing = false;

  Motor(); // end default constructor

  /// @brief The constructor for the motor controlled by the microcontroller
  /// @param name The name of this motor for debug prints
  /// @param rpwm The right PWM signal pin
  /// @param lpwm The left PWM signal pin
  /// @param r_en The right PWM enable pin
  /// @param l_en The left PWM enable pin
  /// @param hall_1 The pin for hall sensor 1
  /// @param hall_2  The pin for hall sensor 2
  /// @param currentSensePin The pin for current sensor
  /// @param totalPulses The total number of pulses from full retraction to full
  /// extension
  /// @param freq The frequency of the PWM signal
  /// @param defSpeed The default motor speed
  /// @param pwmRes The PWM bitdepth resolution
  Motor(const char *name, const MotorPin rpwm, const MotorPin lpwm,
        const MotorPin r_en, const MotorPin l_en, const MotorPin hall_1,
        const MotorPin hall_2, const adc1_channel_t currentSensePin,
        const int totalPulses, const int freq = PWM_FREQUENCY,
        const int defSpeed = DEFAULT_MOTOR_SPEED,
        const int pwmRes = PWM_RESOLUTION_BITS, const int bottomLimitPin = -1,
        const int currentLimit = -1, const int alarmCurrentLimit = -1,
        const int stopBuffer = 0);

  /// @brief Initialize motor
  void initialize();

  /**
   * Drives the motor in the specified direction at the specified speed.
   *
   * @param motorDirection the direction in which to drive the motor, either
   * "EXTEND" or "RETRACT"
   * @param specifiedSpeed the speed at which to drive the motor, defaults to 0
   * if not specified
   *
   * @throws None
   */
  void drive(const Direction motorDirection, const int specifiedSpeed = 0);

  /// @brief Tell the motor to rotate in the direction of extension
  void extend();

  /// @brief Tell the motor to rotate in the direction of retraction
  void retract();

  /**
   * @brief Disables the motor.
   * If the motor is already stopped, this function does nothing.
   * Otherwise, it stops the motor, sets the speed to 0, and logs a message.
   */
  void disable();

  /** @brief Reset the distance sensor count and position variables for the
   * motor to zero
   */
  void zero() {
    distanceSensor.clearCount();
    lastPos = pos = 0;
  }

  /**
   * @brief Checks if the current position has reached or exceeded the current
   * limit and if the direction is set to retract.
   *
   * @return true if the current position has reached or exceeded the current
   * limit and the direction is set to retract, false otherwise.
   */
  bool hitBottom() const;

  /**
   * @brief Check if the top has been reached.
   *
   * @return true if the top has been reached, false otherwise
   */
  bool topReached() const { return pos >= totalPulseCount; }

  /**
   * @brief Update the state of the motor.
   *
   * @param newSpeed the new speed value to set (default: MAX_SPEED + 1)
   *
   * @throws None
   */
  void update(const int newSpeed = (MAX_SPEED + 1));

  /// @brief Read the position of the motor
  void readPos() { pos = distanceSensor.getCount(); }

  /**
   * @brief Get a normalized indicaton of the position of this motor based on
   * its total range
   *
   * @return The fraction that represents how much of total extension we are
   * currently at as a float value. If the total pulse count is 0, it returns
   * 0.0f.
   *
   * @throws None
   */
  float getNormalizedPos() {
    pos = distanceSensor.getCount();

    if (totalPulseCount == 0)
      return 0.0f;
    return static_cast<float>(pos) / static_cast<float>(totalPulseCount);
  }

  /**
   * @brief Displays information about the motor.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void displayInfo();

  /**
   *@brief Retrieve the current value as milliamps
   *
   * @return The current value as milliamps
   */
  int getCurrent() const { return currentSense.getCurrent(); }

  /**
   * @brief Checks if the current motor is in a stopped state.
   *
   * @return true if the motor is in a stopped state, false otherwise.
   */
  bool isStopped() const { return dir == Direction::STOP; }

  /**
   * @brief Set the speed to the specified value.
   *
   * @param newSpeed The new speed value.
   */
  void setSpeed(int newSpeed) {
    // Constrain the new speed value between 0 and the maximum value allowed by
    // the PWM resolution.
    speed = constrain(newSpeed, 0, MAX_SPEED);
  }

  void setPos(int newPos);

  ~Motor() {}
}; // end class Motor

#endif // _MOTOR_HPP_

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
#define MOTOR1_TLIMIT 34
#define MOTOR2_TLIMIT 35

#define READ_POSITION_ENCODER() this->pos = distanceSensor.getCount();
#define MOVE_TO_POS(setpoint, min_delta, buffer)                               \
  if (abs(pos - setpoint) > min_delta) {                                       \
    if (pos < setpoint) {                                                      \
      desiredPos = setpoint - buffer;                                          \
    } else if (pos > newPos) {                                                 \
      desiredPos = setpoint + buffer;                                          \
    }                                                                          \
  }

int currentPWMChannel = 0;

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

  Motor() {} // end default constructor

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
        const int defSpeed = 70, const int pwmRes = 8,
        const int bottomLimitPin = -1, const int currentLimit = -1,
        const int alarmCurrentLimit = -1, const int stopBuffer = 0)
      : rPWM_Pin(rpwm), lPWM_Pin(lpwm), r_EN_Pin(r_en), l_EN_Pin(l_en),
        hall_1_Pin(hall_1), hall_2_Pin(hall_2),
        currentSensePin(currentSensePin), totalPulseCount(totalPulses),
        frequency(freq), speed(defSpeed), pwmResolution(pwmRes),
        bottomLimitPin(bottomLimitPin), bottomCurrentLimit(currentLimit),
        currentAlarmLimit(alarmCurrentLimit), stopBuffer(stopBuffer),
        outOfRange(false) {
    /// Copy name of linear actuator into ID field
    strncpy(id, name, sizeof(id) - 1);
    id[sizeof(id) - 1] = '\0';
  } // end constructor

  /// @brief Initialize motor
  void initialize() {
    // At least two channels are needed for the linear actuator motor
    if (currentPWMChannel > -1 && currentPWMChannel < 14) {
      pwmRChannel = currentPWMChannel++;
      pwmLChannel = currentPWMChannel++;
    }

    ledcSetup(pwmRChannel, frequency, pwmResolution);
    ledcSetup(pwmLChannel, frequency, pwmResolution);

    motorAttachPin(rPWM_Pin, pwmRChannel);
    Serial.printf("Attaching pin %d to RPWM Channel %d\n", rPWM_Pin,
                  pwmRChannel);
    motorAttachPin(lPWM_Pin, pwmLChannel);
    Serial.printf("Attaching pin %d to LPWM Channel %d\n\n", lPWM_Pin,
                  pwmLChannel);

    pinMode(bottomLimitPin, INPUT_PULLUP);

    motorPinMode(r_EN_Pin, OUTPUT);
    motorPinMode(l_EN_Pin, OUTPUT);

    motorPinWrite(r_EN_Pin, HIGH);
    motorPinWrite(l_EN_Pin, HIGH);

    ledcWrite(pwmRChannel, 0);
    ledcWrite(pwmLChannel, 0);

    distanceSensor.attachSingleEdge(static_cast<int>(hall_1_Pin),
                                    static_cast<int>(hall_2_Pin));
    distanceSensor.clearCount();
    READ_POSITION_ENCODER()

    currentSense.initialize(currentSensePin);

    Serial.printf("Motor: %s\n"
                  "-------------------\n"
                  "Frequency:    %5d\n"
                  "Resolution:   %5d\n"
                  "Speed:        %5d\n"
                  "Position:     %5d\n"
                  "R_EN Pin:     %5d\n"
                  "L_EN Pin:     %5d\n"
                  "RPWM Pin:     %5d\n"
                  "LPWM Pin:     %5d\n"
                  "Hall 1 Pin:   %5d\n"
                  "Hall 2 Pin:   %5d\n"
                  "Max Position: %5d\n\n"
                  "Stop Buffer: %5d pulses\n\n"
                  "Alarm Current: %5d mA\n\n",
                  id, frequency, pwmResolution, speed, pos, r_EN_Pin, l_EN_Pin,
                  rPWM_Pin, lPWM_Pin, hall_1_Pin, hall_2_Pin, totalPulseCount,
                  stopBuffer, currentAlarmLimit);
    Serial.printf("RPWM Channel %d - LPWM Channel: %d\n\n", pwmRChannel,
                  pwmLChannel);
  }

  /**
   * @brief Drives the motor in the specified direction at the specified speed.
   *
   * @param motorDirection the direction in which the motor should be driven
   * @param specifiedSpeed the specified speed at which the motor should be
   * driven (default: 0)
   */
  void drive(const Direction motorDirection, const int specifiedSpeed = 0) {
    // Set the drive speed based on the specified speed or the default speed
    const int driveSpeed = specifiedSpeed > 0 ? specifiedSpeed : speed;

    switch (motorDirection) {
    case Direction::EXTEND:
      // Drive the motor in the extend direction
      motorPinWrite(r_EN_Pin, HIGH);
      motorPinWrite(l_EN_Pin, HIGH);
      ledcWrite(pwmRChannel, driveSpeed);
      ledcWrite(pwmLChannel, 0);
      break;

    case Direction::STOP:
      // Stop the motor
      motorPinWrite(r_EN_Pin, LOW);
      motorPinWrite(l_EN_Pin, LOW);
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, 0);
      break;

    case Direction::RETRACT:
      // Drive the motor in the retract direction
      motorPinWrite(r_EN_Pin, HIGH);
      motorPinWrite(l_EN_Pin, HIGH);
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, driveSpeed);
      break;

    default:
      break;
    }

    // Update the last position variable
    lastPos = pos;
  }

  /// @brief Tell the motor to rotate in the direction of extension
  void extend() {
    // Works as a toggle
    dir = (dir != Direction::EXTEND) ? Direction::EXTEND : Direction::STOP;
  }

  /// @brief Tell the motor to rotate in the direction of retraction
  void retract() {
    // Works as a toggle
    dir = (dir != Direction::RETRACT) ? Direction::RETRACT : Direction::STOP;
  }

  /**
   * @brief Disables the motor.
   * If the motor is already stopped, this function does nothing.
   * Otherwise, it stops the motor, sets the speed to 0, and logs a message.
   */
  void disable() {
    // Works as a toggle
    if (dir != Direction::STOP) {
      dir = Direction::STOP;
      motorPinWrite(r_EN_Pin, LOW);
      motorPinWrite(l_EN_Pin, LOW);
      speed = 0;
      Serial.printf("Disabled motor: %s\n", id);
    }
  }

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
  bool hitBottom() const {
    const int current = getCurrent();
    Serial.printf("Current %d <=> Bottom current limit: %d\n", current,
                  bottomCurrentLimit);
    return (current >= bottomCurrentLimit) && (dir == Direction::RETRACT);
  }

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
  void update(const int newSpeed = (MAX_SPEED + 1)) {
    READ_POSITION_ENCODER()

    bool goingPastBottom = false;
    // Check if the motor is going past the bottom limit
    if (!homing) {
      goingPastBottom = (pos < (stopBuffer + 5)) && dir == Direction::RETRACT;
    } else {
      speed = speed <= 100 ? speed : 100;
      goingPastBottom = hitBottom();
      Serial.printf("Motor current: %d\n", getCurrent());
    }

    // Check if the motor is going past the top limit
    const bool goingPastTop = topReached() && dir == Direction::EXTEND;

    // Check whether the motor is out of range
    outOfRange = goingPastTop || goingPastBottom;

    // If the motor is out of range or in the STOP direction, stop it
    if (outOfRange || dir == Direction::STOP) {
      if (!homing && (dir == Direction::STOP) && pos < 0) {
        zero();
      }
      dir = Direction::STOP;
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, 0);
      motorPinWrite(r_EN_Pin, LOW);
      motorPinWrite(l_EN_Pin, LOW);
      return;
    }

    // Set the motor speed based on the input
    if (newSpeed > MAX_SPEED || newSpeed < 0) {
      drive(dir, this->speed);
    } else {
      drive(dir, newSpeed);
    }
  }

  /// @brief Set a new target position for this motor
  /// @param newPos The new target position to move the motor to
  void setPos(const int newPos) {
    READ_POSITION_ENCODER()
    MOVE_TO_POS(newPos, 3, 20)
  }

  /// @brief Read the position of the motor
  void readPos() { READ_POSITION_ENCODER() }

  /// @brief Get a normalized indicaton of the position of this motor based on
  /// its total range
  /// @return A fraction that represents how much of total extension we are
  /// currently at
  float getNormalizedPos() {
    READ_POSITION_ENCODER()

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
  void displayInfo() {
    Serial.printf("Motor %s - Direction: %s, pos: %d\n", id,
                  directions[static_cast<int>(dir)], pos);
    Serial.printf("Motor %s - Speed: %d, desired pos: %d\n", id, speed,
                  desiredPos);
    Serial.printf("Motor %s - Max hall position: %d \n\n", id, totalPulseCount);
  }

  /// @return The current used by the motor
  int getCurrent() const { return currentSense.getCurrent(); }

  /**
   * @brief Checks if the current motor is in a stopped state.
   *
   * @return true if the motor is in a stopped state, false otherwise.
   */
  bool isStopped() const { return dir == Direction::STOP; }

  /**
   * @brief Checks if the current value exceeds the current alarm limit.
   *
   * @return true if the current value exceeds the current alarm limit, false
   * otherwise.
   */
  bool isCurrentAlarm() const {
    return currentAlarmLimit > 0 && getCurrent() >= currentAlarmLimit;
  }

  /**
   * @brief Set the speed to the specified value.
   *
   * @param newSpeed The new speed value.
   */
  void setSpeed(int newSpeed) {
    // Constrain the new speed value between 0 and the maximum value allowed by
    // the PWM resolution.
    speed = constrain(newSpeed, 0, (2 << PWM_RESOLUTION_BITS) - 1);
  }
}; // end class Motor

#endif // _MOTOR_HPP_

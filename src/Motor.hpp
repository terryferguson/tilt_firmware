/*! \file Motor.hpp */

#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include <driver/adc.h>
#include "PinMacros.hpp"
#include "defs.hpp"
#include "ControlPins.hpp"
#include "CurrentSense.hpp"
#include <ESP32Encoder.h>
#include <cstring>

#define MOTOR1_LIMIT 32
#define MOTOR2_LIMIT 33

#define READ_POSITION_ENCODER() this->pos = distanceSensor.getCount();
#define MOVE_TO_POS(setpoint, min_delta, buffer) \
  if (abs(pos - setpoint) > min_delta)           \
  {                                              \
    if (pos < setpoint)                          \
    {                                            \
      desiredPos = setpoint - buffer;            \
    }                                            \
    else if (pos > newPos)                       \
    {                                            \
      desiredPos = setpoint + buffer;            \
    }                                            \
  }

int currentPWMChannel = 0;

/** @class Motor
 *
 *  @brief This class represents the motor controlled by the microcontroller
 *
 * @author Terry Paul Ferguson
 * @version 0.1
 */
class Motor
{
private:
  char id[16];                                /** The name of this motor for debug prints */
  int pwmRChannel = -1;                       /** The right PWM channel - Extension */
  int pwmLChannel = -1;                       /** The left PWM channel - Retraction */
  MotorPin rPWM_Pin = MotorPin::UNASSIGNED;   /** The right PWM pin */
  MotorPin lPWM_Pin = MotorPin::UNASSIGNED;   /** The left PWM pin */
  MotorPin r_EN_Pin = MotorPin::UNASSIGNED;   /** The right PWM enable pin */
  MotorPin l_EN_Pin = MotorPin::UNASSIGNED;   /** The left PWM enable pin */
  MotorPin hall_1_Pin = MotorPin::UNASSIGNED; /** The pin for hall sensor 1 */
  MotorPin hall_2_Pin = MotorPin::UNASSIGNED; /** The pin for hall sensor 2 */
  MotorPin l_is_pin =
      MotorPin::UNASSIGNED; /** The pin for left PWM current sensor */
  MotorPin r_is_pin =
      MotorPin::UNASSIGNED;                        /** The pin for right PWM current sensor */
  adc1_channel_t currentSensePin = ADC1_CHANNEL_0; /** The current sense pin */
  int frequency = PWM_FREQUENCY;                   /** The frequency of the PWM signal in hertz */
  int pwmResolution = 8;                           /** The PWM bitdepth resolution */
  int desiredPos =
      -1;                  /** The desired position of the motor (-1 if none exists) */

  ESP32Encoder distanceSensor; /** The motor position encoder (quadrature signal
                                  from 2 hall sensors) */

  CurrentSense currentSense; /** The current sensor */

public:
  int pos =
      0; /** The current position of the motor based on hall sensor pulses */
  int lastPos =
      0;           /** The last position of the motor based on hall sensor pulses */
  int speed = 255; /** The current speed of the motor. The duty cycle of the PWM
                      signal is speed/(2^pwmResolution - 1) */
  int maxPulses = -1;
  int totalPulseCount = 0; /** The total number of pulses from full retraction
                              to full extension */

  int bottomLimitPin = -1;
  int topLimitPin = -1;

  bool outOfRange = false;

  Direction dir = Direction::STOP; /** The direction of the motor rotation */

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
        const int defSpeed = 70, const int pwmRes = 8, const int bottomLimitPin = -1)
      : rPWM_Pin(rpwm), lPWM_Pin(lpwm), r_EN_Pin(r_en), l_EN_Pin(l_en),
        hall_1_Pin(hall_1), hall_2_Pin(hall_2), currentSensePin(currentSensePin), totalPulseCount(totalPulses), frequency(freq),
        speed(defSpeed), pwmResolution(pwmRes), bottomLimitPin(bottomLimitPin), outOfRange(false)
  {
    /// Copy name of linear actuator into ID field
    strncpy(id, name, sizeof(id) - 1);
    id[sizeof(id) - 1] = '\0';
  } // end constructor

  /// @brief Initialize motor
  void initialize()
  {
    // At least two channels are needed for the linear actuator motor
    if (currentPWMChannel > -1 && currentPWMChannel < 14)
    {
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

    int val = digitalRead(bottomLimitPin);

    if (debugEnabled)
    {
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
                    "Bottom Reached: %5d\n\n",
                    id, frequency, pwmResolution, speed, pos, r_EN_Pin, l_EN_Pin, rPWM_Pin,
                    lPWM_Pin, hall_1_Pin, hall_2_Pin, totalPulseCount, val);
      Serial.printf("RPWM Channel %d - LPWM Channel: %d\n\n", pwmRChannel,
                    pwmLChannel);
    }
  }

  /**
   * Drives the motor in the specified direction at the specified speed.
   *
   * @param motorDirection the direction in which the motor should be driven
   * @param specifiedSpeed the specified speed at which the motor should be
   * driven (default: 0)
   *
   */
  void drive(const Direction motorDirection, const int specifiedSpeed = 0)
  {
    const int reachedBottom = digitalRead(bottomLimitPin) == LOW;

    if ((reachedBottom && dir == Direction::RETRACT) || (((pos >= totalPulseCount) && (dir == Direction::EXTEND))))
    {
      Serial.printf("Motor out of range!\n");
      dir = Direction::STOP;
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, 0);
      motorPinWrite(r_EN_Pin, LOW);
      motorPinWrite(l_EN_Pin, LOW);
      speed = 0;
      lastPos = pos;
      READ_POSITION_ENCODER()
      return;
    }
    const int driveSpeed = specifiedSpeed > 0 ? specifiedSpeed : speed;

    motorPinWrite(r_EN_Pin, HIGH);
    motorPinWrite(l_EN_Pin, HIGH);

    switch (motorDirection)
    {
    case Direction::EXTEND:
      ledcWrite(pwmRChannel, driveSpeed);
      ledcWrite(pwmLChannel, 0);
      break;
    case Direction::STOP:
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, 0);
      motorPinWrite(r_EN_Pin, LOW);
      motorPinWrite(l_EN_Pin, LOW);
      break;
    case Direction::RETRACT:
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, driveSpeed);
      break;
    default:
      break;
    } // end direction handler

    lastPos = pos;
    READ_POSITION_ENCODER()
  } // end drive

  /// @brief Tell the motor to rotate in the direction of extension
  void extend()
  {
    // Works as a toggle
    dir = (dir != Direction::EXTEND) ? Direction::EXTEND : Direction::STOP;
  }

  /// @brief Tell the motor to rotate in the direction of retraction
  void retract()
  {
    // Works as a toggle
    dir = (dir != Direction::RETRACT) ? Direction::RETRACT : Direction::STOP;
  }

  /// @brief Tell the motor to stop
  void disable()
  {
    // Works as a toggle
    dir = Direction::STOP;
  }

  /// @brief Zero out position information for this motor
  void zero()
  {
    distanceSensor.clearCount();
    lastPos = pos = 0;
  }

  /// @brief Update the position information for this motor and move it
  void update(const int newSpeed = (MAX_SPEED + 1))
  {
    // const bool bottomReached = digitalRead(bottomLimitPin) == HIGH;

    outOfRange =  ((pos >= totalPulseCount) && (dir == Direction::EXTEND));

    // Serial.printf("Out of Range: %d", outOfRange);

    if (outOfRange)
    {
      Serial.printf("Disabled due to range\n");
      dir = Direction::STOP;
      ledcWrite(pwmRChannel, 0);
      ledcWrite(pwmLChannel, 0);
      motorPinWrite(r_EN_Pin, LOW);
      motorPinWrite(l_EN_Pin, LOW);
      speed = 0;
      lastPos = pos;
      READ_POSITION_ENCODER()
      return;
    }
    if (newSpeed > MAX_SPEED || newSpeed < 0)
    {
      drive(dir, this->speed);
    }
    else
    {
      drive(dir, newSpeed);
    }
  }

  /// @brief Set a new target position for this motor
  /// @param newPos The new target position to move the motor to
  void setPos(const int newPos)
  {
    READ_POSITION_ENCODER()
    MOVE_TO_POS(newPos, 15, 40)
  }

  /// @brief Read rotary encoder value into position variable
  void readPos() { READ_POSITION_ENCODER() }

  /// @brief Get a normalized indicaton of the position of this motor based on
  /// its total range
  /// @return A fraction that represents how much of total extension we are
  /// currently at
  float getNormalizedPos() const
  {
    if (totalPulseCount == 0)
      return 0.0f;
    return static_cast<float>(pos) / static_cast<float>(totalPulseCount);
  }

  /**
   * Displays information about the motor.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void displayInfo()
  {
    Serial.printf("Motor %s - Direction: %s, pos: %d\n", id, directions[static_cast<int>(dir)],
                  pos);
    Serial.printf("Motor %s - Speed: %d, desired pos: %d\n", id, speed,
                  desiredPos);
    Serial.printf("Motor %s - Max hall position: %d \n\n", id, totalPulseCount);
  }

  /// @return The current used by the motor
  int getCurrent() const
  {
    return currentSense.getCurrent();
  }

  void setSpeed(int newSpeed) { speed = newSpeed; }
}; // end class Motor

#endif // _MOTOR_HPP_

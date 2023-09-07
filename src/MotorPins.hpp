/*! \file MotorPins.hpp */

#ifndef _MOTOR_PINS_HPP_
#define _MOTOR_PINS_HPP_

#include <cstdint>

/** @enum MotorPins
 * 
 * @class MotorPins
 * 
 * @brief Pin number definitions for the motor
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 * 
 * This has the pin numbering to  wire to the microcontroller
 */

enum class MotorPin : std::uint8_t {
  /** NULL pin for unassigned */
  UNASSIGNED = 0,

  /**  Motor RPWM Pin for extension square wave */
  MOTOR1_RPWM_PIN = 25,

  /**  Motor LPWM Pin for extension square wave */
  MOTOR1_LPWM_PIN = 19,

  /**  Enable pin for RPWM channel (extension) */
  MOTOR1_R_EN_PIN = 26,

  /** Enable pin for LPWM channel (retraction) */
  MOTOR1_L_EN_PIN = 18,

  /** Hall 1 sensor pin */
  MOTOR1_HALL1_PIN = 22,

  /** Hall 2 sensor pin */
  MOTOR1_HALL2_PIN = 23,

  /** Left motor channel current sense pin  */
  MOTOR1_LIS_PIN = 31,

  /** Right motor channel current sense pin  */
  MOTOR1_RIS_PIN = 32,

  /** Motor RPWM Pin for extension square wave */
  MOTOR2_RPWM_PIN = 5,

  /** Motor LPWM Pin for retraction square wave */
  MOTOR2_LPWM_PIN = 17,

  /** Enable pin for RPWM channel (extension) */
  MOTOR2_R_EN_PIN = 16,

  /** Enable pin for LPWM channel (retraction) */
  MOTOR2_L_EN_PIN = 15,

  /** Hall 1 sensor pin */
  MOTOR2_HALL1_PIN = 14,

  /** Hall 2 sensor pin */
  MOTOR2_HALL2_PIN = 13,

  /** Left motor channel current sense pin  */
  MOTOR2_LIS_PIN = 16,

   /** Right motor channel current sense pin  */
  MOTOR2_RIS_PIN = 11,
};

#endif // _MOTOR_PINS_HPP_

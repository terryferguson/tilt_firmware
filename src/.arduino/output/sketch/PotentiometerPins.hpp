#line 1 "/home/terry/Projects/motor_control_firmware/PotentiometerPins.hpp"
/*! \file PotentiometerPins.hpp */

#ifndef _POTENTIOMTER_PINS_HPP_
#define _POTENTIOMTER_PINS_HPP_

#include <cstdint>

/** @enum MotorPins
 * 
 * @class MotorPins
 * 
 * @brief Pin number definitions for the potentiometer controlled parameters
 * 
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 * 
 * @version 0.1
 */
enum class PotentiometerPins : std::uint8_t {
  /** Speed potentiometer pin */
  SPEED_POT_PIN = 35,

  /** PID gain potentiometer pin */
  KP_POT_PIN = 32,
};

#endif // _POTENTIOMTER_PINS_HPP_

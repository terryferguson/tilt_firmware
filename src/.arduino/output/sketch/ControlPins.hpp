#line 1 "/home/terry/Projects/motor_control_firmware/ControlPins.hpp"
/*! \file ControlPins.hpp */

#ifndef _CONTROL_PINS_HPP_
#define _CONTROL_PINS_HPP_

#include <cstdint>
#include <driver/adc.h>

/** @enum ControlPin
 * 
 * @class ControlPin
 * 
 * @brief Pin number definitions for the controlled parameters
 * 
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 * 
 * @version 0.1
 */
enum class ControlPin : std::uint8_t {
  /** NULL pin for unassigned */
  UNASSIGNED = 255,
  
  /**  Left current sense pin */
  LEFT_CURRENT_SENSE_PIN = ADC1_CHANNEL_3, /** ADC1 channel 5 is GPIO33 */

  /**  Right current sense pin */
  RIGHT_CURRENT_SENSE_PIN = ADC1_CHANNEL_0, /** ADC1 channel 0 is GPIO36 */
};

#endif // _CONTROL_PINS_HPP_

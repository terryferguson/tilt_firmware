/*! \file PinMacros.hpp */
#ifndef _PIN_MACROS_HPP_
#define _PIN_MACROS_HPP_

#include "defs.hpp"

/// @brief Map 12-bit ADC value to a value within defined range
/// @param x 12-bit ADC value
/// @param in_min Minimum input value
/// @param in_max  Maximum input value
/// @param out_min Minimum output value
/// @param out_max Maximum output value
/// @return Mapped output value for input value
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define FSET_TO_ANALOG_PIN(pin, var_to_set, range_min, range_max)              \
  var_to_set = fmap(analogRead(pin), 0, MAX_ADC_VALUE, range_min, range_max)

#define SET_TO_ANALOG_PIN(pin, range_min, range_max)                           \
  map(analogRead(pin), 0, MAX_ADC_VALUE, range_min, range_max)

#define SET_TO_ANALOG_PIN_FUNC(pin, func, range_min, range_max)                \
  func(map(analogRead(pin), 0, MAX_ADC_VALUE, range_min, range_max))

#define motorPinWrite(pin, value)                                              \
  digitalWrite(static_cast<std::uint8_t>(pin), value)

#define motorPinWrite(pin, value)                                              \
  digitalWrite(static_cast<std::uint8_t>(pin), value)

#define motorPinMode(pin, value) pinMode(static_cast<std::uint8_t>(pin), value)

#define motorAttachPin(pin, channel)                                           \
  ledcAttachPin(static_cast<std::uint8_t>(pin), channel)

#define motorAnalogRead(pin) analogRead(static_cast<std::uint8_t>(pin))

#endif // _PIN_MACROS_HPP_

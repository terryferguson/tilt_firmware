/*! \file PinMacros.hpp */
#ifndef _PIN_MACROS_HPP_
#define _PIN_MACROS_HPP_
#pragma once

#define motorPinWrite(pin, value)                                              \
  digitalWrite(static_cast<std::uint8_t>(pin), value)

#define motorPinWrite(pin, value)                                              \
  digitalWrite(static_cast<std::uint8_t>(pin), value)

#define motorPinMode(pin, value) pinMode(static_cast<std::uint8_t>(pin), value)

#define motorAttachPin(pin, channel)                                           \
  ledcAttachPin(static_cast<std::uint8_t>(pin), channel)

#define motorAnalogRead(pin) analogRead(static_cast<std::uint8_t>(pin))

#endif // _PIN_MACROS_HPP_

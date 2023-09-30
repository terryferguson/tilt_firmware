/*! \file ControlPins.hpp */

#ifndef _CONTROL_PINS_HPP_
#define _CONTROL_PINS_HPP_

#include <cstdint>
#include <driver/adc.h>

/// @brief Leader current sense ADC channel pin
#define LEADER_CURRENT_SENSE_PIN ADC1_CHANNEL_0

/// @brief Follower current sense ADC channel pin
#define FOLLOWER_CURRENT_SENSE_PIN ADC1_CHANNEL_3

#endif // _CONTROL_PINS_HPP_

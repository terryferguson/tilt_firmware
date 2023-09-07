/*! \file CurrentSettings.hpp */

#ifndef _CURRENT_SETTINGS_HPP_
#define _CURRENT_SETTINGS_HPP_

#define ADC_BITS 12

#define ADC_MAX (2 << ADC_BITS)

#define LOGICAL_LEVEL_VOLTAGE 3.3f

#define DEFAULT_CURRENT_INCREASE_LIMIT                                         \
  ((int)((0.07 * LOGICAL_LEVEL_VOLTAGE) * ADC_MAX))
  
#define CURRENT_INCREASE_LIMIT_MAX                                             \
  ((int)((0.15 * LOGICAL_LEVEL_VOLTAGE) * ADC_MAX))

/// @brief Indicate whether the increase in current on the driver current sense pin has exceeded the threshold
/// @param currentSensePin The current sense pin input from the motor driver
/// @param baseValue The base value of the current of the motor (min load) 
/// @param threshold The threshold value to use
/// @return true if the increase exceeds threshold value, else false
bool currentIncreseExceedsThreshold(const int currentSensePin, const int baseValue,
                             const int threshold) {
  int currentValue = analogRead(currentSensePin);

  return (currentValue - baseValue) >= threshold;
}

#endif // _CURRENT_SETTINGS_HPP_

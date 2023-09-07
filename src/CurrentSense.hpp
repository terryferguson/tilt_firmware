/*! \file CurrentSense.hpp */

#ifndef _CURRENT_SENSE_HPP_
#define _CURRENT_SENSE_HPP_

#include <stdint.h>
#include <driver/adc.h>
#include <cmath>

#include "defs.hpp"
#include "ControlPins.hpp"


/** @class CurrentSense
 *
 *  @brief This implements the current sense functionality for the motors
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */

class CurrentSense
{
private:
  int_fast32_t CALIBRATE_ITERATIONS_SHIFT = 12;
  int_fast32_t SAMPLE_CURRENT_ITERATIONS_SHIFT = 7;

  int32_t MV_PER_AMP = static_cast<int32_t>(185 * 1.132);
  /// @brief The number of millivolts per detected ampere of current

  int32_t ACS_OFFSET = 1885;

  /// @brief The voltage offset of the ACS. It should be Vin / 2. Values
  /// above this point indicate postive current flow. Values below indicate
  // negitive current flow.

  adc1_channel_t currentSensePin;
  double logicVoltage =  3.3;
  int32_t maxAdcValue = 4096;

public:
  CurrentSense(const adc1_channel_t pCurrentSensePin = ADC1_CHANNEL_0,
               const double pLogicVoltage = 3.3,
               const int32_t pMaxAdcValue = MAX_ADC_VALUE)
      : currentSensePin(pCurrentSensePin), logicVoltage(pLogicVoltage), maxAdcValue(pMaxAdcValue)
  {
  }

  void initialize(const adc1_channel_t pCurrentSensePin = ADC1_CHANNEL_0)
  {
    currentSensePin = pCurrentSensePin;
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(currentSensePin, ADC_ATTEN_DB_11);

    Serial.print("Pin: ");
    Serial.println(static_cast<uint8_t>(currentSensePin));
    Serial.print("Logic Voltage: ");
    Serial.println(logicVoltage);
    Serial.print("Max ADC Value: ");
    Serial.println(maxAdcValue);
    Serial.print("mV per A: ");
    Serial.println(MV_PER_AMP);

    const int iterations = 1 << CALIBRATE_ITERATIONS_SHIFT;

    int32_t adcSum = 0;
    double currentSum = 0;

    for (int32_t i = 0; i < iterations; i++)
    {
      const int adcValue = adc1_get_raw(currentSensePin);
      adcSum += adcValue;
    }

    ACS_OFFSET = adcSum >> CALIBRATE_ITERATIONS_SHIFT;

    Serial.printf("ACS Offset: %d\n", ACS_OFFSET);
  }

  /**
   * Calculates the average current.
   *
   *
   * @return the average current of the sampling
   *
   * @throws None
   */
  int getCurrent() const
  {
    const int32_t iterations = 1 << SAMPLE_CURRENT_ITERATIONS_SHIFT;

    int32_t currentSum = 0;
    for (int i = 0; i < iterations; i++)
    {
      const int adcOffset = adc1_get_raw(currentSensePin) - ACS_OFFSET;
      // const int adcOffset = analogRead(static_cast<uint8_t>(currentSensePin)) - ACS_OFFSET;
      const double voltageDelta = (adcOffset * (logicVoltage / maxAdcValue));
      const int current = static_cast<int>(voltageDelta * 1000000.0 / MV_PER_AMP);
      currentSum += current;
    }

    const double averageCurrent = std::abs(static_cast<double>(currentSum >> SAMPLE_CURRENT_ITERATIONS_SHIFT));

    return static_cast<int>(averageCurrent);
  } // end method getCurrent
};  // end class CurrentSense

#endif // _CURRENT_SENSE_HPP_
#pragma once
#include <cstdint>     // uint_fast16_t
#include <limits>      // std::numeric_limits
#include <type_traits> // std::make_unsigned_t, make_signed_t, is_unsigned

/** @class EMA
 *
 *  @brief Calculates a fast exponential moving average
 *
 * @tparam  K
 *          The amount of bits to shift by. This determines the location
 *          of the pole in the EMA transfer function, and therefore the
 *          cut-off frequency.
 *          The higher this number, the more filtering takes place.
 *          The pole location is @f$ 1 - 2^{-K} @f$.
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
template <uint8_t K, class uint_t = uint_fast32_t> class EMA {
public:
  /// Update the filter with the given input and return the filtered output.
  uint_t operator()(uint_t input) {
    state += input;
    uint_t output = (state + half) >> K;
    state -= output;
    return output;
  }

  /// Fixed point representation of one half, used for rounding.
  constexpr static uint_t half = 1 << (K - 1);

  void reset() { state = 0; }

private:
  uint_t state = 0;
};

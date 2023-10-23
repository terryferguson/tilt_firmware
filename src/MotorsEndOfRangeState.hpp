/*! \file MotorsEndOfRangeState.hpp */

#ifndef _MOTORS_END_OF_RANGE_STATE_HPP_
#define _MOTORS_END_OF_RANGE_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsEndOfRangeState
 *
 * @brief Handles the the end of range state for motor control system
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsEndOfRangeState : public ControllerState {
public:
  MotorsEndOfRangeState() {}

  MotorsEndOfRangeState(MotorController *pMotorController) {
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the the end of range  state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in the end of range  state
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the end of range state
   *
   * @return void
   */
  void leave();

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char* getName() const { return "MotorsEndOfRangeState"; }

  void updatePWM();

  ~MotorsEndOfRangeState(){};
}; // end class State

#endif // _MOTORS_END_OF_RANGE_STATE_HPP_

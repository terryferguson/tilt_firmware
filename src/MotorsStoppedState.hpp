/*! \file MotorsStoppedState.hpp */

#ifndef _MOTORS_STOPPED_STATE_HPP_
#define _MOTORS_STOPPED_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsStoppedState
 *
 * @brief Handles the stopped state for motor control system
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsStoppedState : public ControllerState {
public:
  MotorsStoppedState() {}

  MotorsStoppedState(MotorController *pMotorController) {
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the stopped state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in stopped state
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the stopped state
   *
   * @return void
   */
  void leave();

  ~MotorsStoppedState(){};
}; // end class State

#endif // _MOTORS_STOPPED_STATE_HPP_

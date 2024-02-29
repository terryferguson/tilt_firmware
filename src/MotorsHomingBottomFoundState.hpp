/*! \file MotorsHomingBottomFoundState.hpp */

#ifndef _MOTORS_HOMING_BOTTOM_FOUND_STATE_HPP_
#define _MOTORS_HOMING_BOTTOM_FOUND_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsHomingBottomFoundState
 *
 * @brief Represents the state of the homing sequence after the bottom of the
 * channel has been found.
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsHomingBottomFoundState : public ControllerState {
private:
  bool motorsStopped;

public:
  MotorsHomingBottomFoundState() { type = MOTORS_HOMING_BOTTOM_FOUND_STATE; }
  MotorsHomingBottomFoundState(MotorController *pMotorController) {
    type = MOTORS_HOMING_BOTTOM_FOUND_STATE;
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the bottom found homing state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in the bottom found homing state.
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the bottom found homing state
   *
   * @return void
   */
  void leave();

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char *getName() const { return "MotorsHomingBottomFoundState"; }

}; // end class MotorsHomingFindBottomState

#endif // _MOTORS_HOMING_BOTTOM_FOUND_STATE_HPP_
/*! \file MotorsHomingFindBottomState.hpp */

#ifndef _MOTORS_HOMING_FIND_BOTTOM_STATE_HPP_
#define _MOTORS_HOMING_FIND_BOTTOM_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsHomingState
 *
 * @brief Represents the state of the homing sequence wherein the system is
 * locating the channel bottom.
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsHomingFindBottomState : public ControllerState {
private:
  static constexpr long MAX_TIME_SINCE_CHANGE = 1500000L;

public:
  MotorsHomingFindBottomState() { type = MOTORS_HOMING_FIND_BOTTOM_STATE; }
  MotorsHomingFindBottomState(MotorController *pMotorController) {
    type = MOTORS_HOMING_FIND_BOTTOM_STATE;
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the find bottom homing state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in the find bottom homing state.
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the find bottom homing state
   *
   * @return void
   */
  void leave();

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char *getName() const { return "MotorsHomingFindBottomState"; }

}; // end class MotorsHomingFindBottomState

#endif // _MOTORS_HOMING_FIND_BOTTOM_STATE_HPP_
#ifndef _MOTORS_HOMING_FIND_BOTTOM_STATE_HPP_
#define _MOTORS_HOMING_FIND_BOTTOM_STATE_HPP_

#include "ControllerState.hpp"

class MotorsHomingFindBottomState : public ControllerState {
private:
  static constexpr long MAX_TIME_SINCE_CHANGE = 1500000L;

public:
  MotorsHomingFindBottomState() {}
  MotorsHomingFindBottomState(MotorController *pMotorController) {
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

}; // end class MotorsHomingFindBottomState

#endif // _MOTORS_HOMING_FIND_BOTTOM_STATE_HPP_
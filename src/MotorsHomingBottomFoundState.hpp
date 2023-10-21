#ifndef _MOTORS_HOMING_BOTTOM_FOUND_STATE_HPP_
#define _MOTORS_HOMING_BOTTOM_FOUND_STATE_HPP_

#include "ControllerState.hpp"

class MotorsHomingBottomFoundState : public ControllerState {
private:
  bool motorsStopped;

public:
  MotorsHomingBottomFoundState() {}
  MotorsHomingBottomFoundState(MotorController *pMotorController) {
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

}; // end class MotorsHomingFindBottomState

#endif // _MOTORS_HOMING_BOTTOM_FOUND_STATE_HPP_
#ifndef _MOTORS_HOMING_STATE_HPP_
#define _MOTORS_HOMING_STATE_HPP_

#include "ControllerState.hpp"

class MotorsHomingState : public ControllerState {
private:
  static constexpr int HOMING_CURRENT_LIMIT = 300;
  static constexpr int HOMING_CURRENT_ALARM_DELAY = 1000000;
  long currentAlarmStart = 0;

public:
  MotorsHomingState() {}
  MotorsHomingState(MotorController *pMotorController) {
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the homing state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in the homing state.
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the homing state
   *
   * @return void
   */
  void leave();

}; // end class MotorsHomingState

#endif // _MOTORS_HOMING_STATE_HPP_
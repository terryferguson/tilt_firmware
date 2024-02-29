/*! \file MotorsCurrentAlarmState.hpp */

#ifndef _MOTORS_CURRENT_ALARM_STATE_HPP_
#define _MOTORS_CURRENT_ALARM_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsCurrentAlarmState
 *
 * @brief Represents of having a current alarm
 * channel has been found.
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsCurrentAlarmState : public ControllerState {
private:
  bool motorsStopped;

public:
  MotorsCurrentAlarmState() { type = MOTORS_CURRENT_ALARM_STATE; }
  MotorsCurrentAlarmState(MotorController *pMotorController) {
    type = MOTORS_CURRENT_ALARM_STATE;
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
  const char *getName() const { return "MotorsCurrentAlarmState"; }

}; // end class MotorsCurrentAlarmState

#endif // _MOTORS_CURRENT_ALARM_STATE_HPP_
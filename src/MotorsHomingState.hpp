/*! \file MotorsHomingState.hpp */

#ifndef _MOTORS_HOMING_STATE_HPP_
#define _MOTORS_HOMING_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsHomingState
 *
 * @brief Represents the initiation of the homing sequence
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
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
 * @brief Enter the MotorsHomingState.
 *
 * This function enables the PID controller, resets the transition flag,
 * resets the soft movement data, resets the current information, resets
 * the motor current alarms, sets the motor speed to the minimum travel
 * speed, applies the motor speed, sets the bottom current limit, sets
 * the homing mode, records the current time, retracts the controller,
 * and prints a message indicating the start of the homing state.
 *
 * @throws None
 * 
 * @returns void
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

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char* getName() const { return "MotorsHomingState"; }

}; // end class MotorsHomingState

#endif // _MOTORS_HOMING_STATE_HPP_
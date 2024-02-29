/*! \file MotorsStartingState.hpp */

#ifndef _MOTORS_STARTING_STATE_HPP_
#define _MOTORS_STARTING_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsStartingState
 *
 * @brief Handles the starting state for motor control system
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsStartingState : public ControllerState {
public:
  MotorsStartingState() { type = MOTORS_STARTING_STATE; }
  MotorsStartingState(MotorController *pMotorController) {
    type = MOTORS_STARTING_STATE;
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the moving state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in starting state
   *
   * This function updates the motor control system when the motors are in the
   * starting state. It checks if the controller is not null, and then performs
   * various actions related to motor control. If the controller is null, it
   * prints an error message.
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the starting state
   *
   * @return void
   */
  void leave();

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char *getName() const { return "MotorsStartingState"; }

  ~MotorsStartingState() {}

}; // end class MotorsStartingState

#endif // _MOTORS_STARTING_STATE_HPP_

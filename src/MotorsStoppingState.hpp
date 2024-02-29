/*! \file MotorsStoppingState.hpp */

#ifndef _MOTORS_STOPPING_STATE_HPP_
#define _MOTORS_STOPPING_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsStoppingState
 *
 * @brief Handles the stopping state for motor control system
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsStoppingState : public ControllerState {
public:
  MotorsStoppingState() { type = MOTORS_STOPPING_STATE; }
  MotorsStoppingState(MotorController *pMotorController) {
    type = MOTORS_STOPPING_STATE;
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the stopping state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in stopping state
   *
   * This function updates the motor control system when the motors are in the
   * stopping state. It checks if the controller is not null, and then performs
   * various actions related to motor control. If the controller is null, it
   * prints an error message.
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the stopping state
   *
   * @return void
   */
  void leave();

  /**
   * @brief Updates the PWM for the MotorsStoppingState.
   *
   * This function calculates the necessary updates for the PWM based on the
   * current state of the controller and the target speed. It checks if the
   * target speed is greater than -1 and if the movement is still updating. If
   * so, it calculates the true PWM update amount based on the elapsed time and
   * updates the controller's speed accordingly. If the movement is no longer
   * updating, it sets the necessary flags and variables for transitioning to
   * the MOTORS_STOPPED_STATE and halts the controller immediately.
   *
   * @throws None
   */
  void updatePWM();

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char *getName() const { return "MotorsStoppingState"; }

  ~MotorsStoppingState() {}

}; // end class MotorsStoppingState

#endif // _MOTORS_STOPPING_STATE_HPP_

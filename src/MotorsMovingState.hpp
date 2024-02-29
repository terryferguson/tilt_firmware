/*! \file MotorsMovingState.hpp */

#ifndef _MOTORS_MOVING_STATE_HPP_
#define _MOTORS_MOVING_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsMovingState
 *
 * @brief Handles the movement of the motors
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsMovingState : public ControllerState {
public:
  MotorsMovingState() { type = MOTORS_MOVING_STATE; }
  MotorsMovingState(MotorController *pMotorController) {
    type = MOTORS_MOVING_STATE;
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the moving state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in the moving state.
   *
   * This function updates the motor control system by performing various checks
   * and actions. It checks if the motors are close to the end of the range and
   * sets the speed accordingly. It also checks if the current time has exceeded
   * the current alarm delay and handles the current alarm if triggered.
   * Finally, it checks if the set position has been reached, handles the PID
   * control, and updates the motors.
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the moving state
   *
   * @return void
   */
  void leave();

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  const char *getName() const { return "MotorsMovingState"; }

  ~MotorsMovingState(){};
}; // end class MotorsMovingState

#endif // _MOTORS_MOVING_STATE_HPP_

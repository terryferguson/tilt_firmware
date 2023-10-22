/*! \file MotorsSoftMovementState.hpp */
#ifndef _MOTORS_SOFT_MOVEMENT_STATE_HPP_
#define _MOTORS_SOFT_MOVEMENT_STATE_HPP_

#include "ControllerState.hpp"

/** @class MotorsSoftMovementState
 *
 * @brief Represents the motors making a soft movement
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class MotorsSoftMovementState : public ControllerState {
public:
  MotorsSoftMovementState() {}

  MotorsSoftMovementState(MotorController *pMotorController) {
    ControllerState::setController(pMotorController);
  }

  /**
   * @brief Handle the system entering the soft movement state.
   *
   * @return void
   */
  void enter();

  /**
   * @brief Update the motor control system in soft movement state
   *
   * @return void
   */
  void update();

  /**
   * @brief Handle leaving the soft movement state
   *
   * @return void
   */
  void leave();

  void updatePWM();

  ~MotorsSoftMovementState() {}
}; // end class MotorsSoftMovementState

#endif // _MOTORS_SOFT_MOVEMENT_STATE_HPP_

/*! \file ControllerState.hpp */
/*! \file ControllerState.hpp */

#pragma once
#ifndef _CONTROLLER_STATE_HPP_
#define _CONTROLLER_STATE_HPP_

#include <Arduino.h>

class MotorController;

enum MotorControllerState {
  MOTORS_STOPPED_STATE,
  MOTORS_STARTING_STATE,
  MOTORS_SOFT_MOVEMENT_STATE,
  MOTORS_STOPPING_STATE,
  MOTORS_MOVING_STATE,
  MOTORS_END_OF_RANGE_STATE,
  MOTORS_START_HOMING_STATE,
  MOTORS_HOMING_FIND_BOTTOM_STATE,
  MOTORS_HOMING_BOTTOM_FOUND_STATE,
};

/** @class ControllerState
 *
 * @brief Base class for controller state
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class ControllerState {
protected:
  MotorController *controller;
  long enteredStateTime = 0L;

public:
  ControllerState(MotorController *pMotorController = nullptr) {
    setController(pMotorController);
    enteredStateTime = 0L;
  }

  MotorControllerState type;
  MotorControllerState nextStateType;
  bool hasTransition = false;

  /**
   * Sets the MotorController for the object.
   *
   * @param pMotorController The MotorController to set.
   *
   * @throws None
   */
  void setController(MotorController *pMotorController) {
    controller = pMotorController;
  }

  virtual void enter() = 0;

  virtual void update() = 0;

  virtual void leave() = 0;

  /**
   * @brief Get the name of this state
   *
   * @return name of the state
   */
  virtual const char* getName() const = 0;

  long elapsedTime() const { return micros() - enteredStateTime; }

  bool operator==(const ControllerState &rhs) const { return type == rhs.type; }

  bool operator!=(const ControllerState &rhs) const { return type != rhs.type; }

  virtual ~ControllerState() = default;
}; // end class ControllerState

#endif //_CONTROLLER_STATE_HPP_
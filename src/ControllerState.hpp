
/*! \file ControllerState.hpp */
/*! \file ControllerState.hpp */

#pragma once
#ifndef _CONTROLLER_STATE_HPP_
#define _CONTROLLER_STATE_HPP_

#include "SystemState.hpp"
#include <Arduino.h>

extern SystemState systemState;

class MotorController;

enum MotorControllerState {
  MOTORS_STOPPED_STATE,             // 1
  MOTORS_STARTING_STATE,            // 2
  MOTORS_SOFT_MOVEMENT_STATE,       // 3
  MOTORS_STOPPING_STATE,            // 4
  MOTORS_MOVING_STATE,              // 5
  MOTORS_END_OF_RANGE_STATE,        // 6
  MOTORS_START_HOMING_STATE,        // 7
  MOTORS_HOMING_FIND_BOTTOM_STATE,  // 8
  MOTORS_HOMING_BOTTOM_FOUND_STATE, // 9
  MOTORS_CURRENT_ALARM_STATE,       // 10
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
  unsigned long enteredStateTime = 0L;

public:
  ControllerState(MotorController *pMotorController = nullptr) {
    setController(pMotorController);
    enteredStateTime = 0UL;
    nextStateType = MOTORS_STOPPED_STATE;
    type = MOTORS_STOPPED_STATE;
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
  virtual const char *getName() const = 0;

  unsigned long elapsedTime() const { return (micros() - enteredStateTime); }

  bool operator==(const ControllerState &rhs) const { return type == rhs.type; }

  bool operator!=(const ControllerState &rhs) const { return type != rhs.type; }

  virtual ~ControllerState() = default;
}; // end class ControllerState

#endif //_CONTROLLER_STATE_HPP_
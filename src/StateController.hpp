#ifndef _STATE_CONTROLLER_
#define _STATE_CONTROLLER_

#include "Direction.hpp"
#include "MotorsEndOfRangeState.hpp"
#include "MotorsHomingBottomFoundState.hpp"
#include "MotorsHomingFindBottomState.hpp"
#include "MotorsHomingState.hpp"
#include "MotorsMovingState.hpp"
#include "MotorsSoftMovementState.hpp"
#include "MotorsStartingState.hpp"
#include "MotorsStoppedState.hpp"
#include "MotorsStoppingState.hpp"

constexpr int NUMBER_OF_MOTOR_STATES = 9;

class StateController {
  MotorsStartingState motorsStartingState;
  MotorsSoftMovementState motorsSoftMovementState;
  MotorsStoppedState motorsStoppedState;
  MotorsStoppingState motorsStoppingState;
  MotorsMovingState motorsMovingState;
  MotorsEndOfRangeState motorsEndOfRangeState;

  MotorsHomingState motorsHomingState;
  MotorsHomingFindBottomState motorsHomingFindBottomState;
  MotorsHomingBottomFoundState motorsHomingBottomFoundState;

  ControllerState *motorsStateMap[NUMBER_OF_MOTOR_STATES];
  ControllerState *currentState = nullptr;

  MotorController *motorController;

  int lastTimestamp = -1;

  /**
   * Initializes the state map for the motors.
   *
   * This function assigns the starting and stopped states of the motors to the
   * corresponding keys in the motorsStateMap.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void initializeStateMap();

public:
  StateController();
  StateController(MotorController *pMotorController);

  /**
   * Sets the MotorController for the state objects
   *
   * @param pMotorController the MotorController object to set
   *
   * @throws None
   */
  void setController(MotorController *pMotorController);

  /**
   * Sets the state of the controller to a new state.
   *
   * @param newState the new state to set the controller to
   *
   * @throws None
   */
  void setState(MotorControllerState newState);

  /**
   * Updates the current state of the motor controller system.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void update();

  /**
   * Indicates whether there is a pending state transition
   *
   * @param None
   *
   * @return True if there is a pending state transition; false otherwise
   *
   * @throws None
   */
  bool hasTransition() const;

  /**
   * Handles the starting transition of state of the Extension/Retraction event
   *
   * @param direction the motor system direction requested for the state
   * transition
   *
   * @return None
   *
   * @throws None
   */
  void OnStarting(const Direction &direction);

  /**
   * Handles the transition of state from starting to moving of the
   * Extension/Retraction event
   *
   * @return None
   *
   * @throws None
   */
  void OnMoving();

  /**
   * Handles the transition of to a stopping state for a
   * Extension/Retraction event. This state is entered into before
   * the system actually stops.
   *
   * @return None
   *
   * @throws None
   */
  void OnStopping();

  /**
   * Handles the transition from stopping state to stopped state.
   *
   * @return None
   *
   * @throws None
   */
  void OnStopped();

  /**
   * Handles end of range speed change
   *
   * @return None
   *
   * @throws None
   */
  void OnEndOfRange();

  void OnHome();

  MotorControllerState getState(void) const { return currentState->type; }
}; // end class StateController

#endif // _STATE_CONTROLLER_
#include "StateController.hpp"
#include "MotorController.hpp"
#include <Arduino.h>

StateController::StateController() {
  Serial.println("State Controller created");
}

StateController::StateController(MotorController *pMotorController) {
  setController(pMotorController);
}

/**
 * Sets the MotorController for the state objects
 *
 * @param pMotorController the MotorController object to set
 *
 * @throws None
 */
void StateController::setController(MotorController *pMotorController) {
  Serial.println("State Controller - setController()");
  motorController = pMotorController;
  motorsStartingState.setController(pMotorController);
  motorsSoftMovementState.setController(pMotorController);
  motorsStoppedState.setController(pMotorController);
  motorsStoppingState.setController(pMotorController);
  motorsMovingState.setController(pMotorController);
  motorsEndOfRangeState.setController(pMotorController);
  motorsHomingState.setController(pMotorController);
  motorsHomingFindBottomState.setController(pMotorController);
  motorsHomingBottomFoundState.setController(pMotorController);
  initializeStateMap();
  setState(MOTORS_STOPPED_STATE);
}

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
void StateController::initializeStateMap() {
  motorsStateMap[MOTORS_STARTING_STATE] = &motorsStartingState;
  motorsStateMap[MOTORS_SOFT_MOVEMENT_STATE] = &motorsSoftMovementState;
  motorsStateMap[MOTORS_STOPPED_STATE] = &motorsStoppedState;
  motorsStateMap[MOTORS_STOPPING_STATE] = &motorsStoppingState;
  motorsStateMap[MOTORS_MOVING_STATE] = &motorsMovingState;
  motorsStateMap[MOTORS_END_OF_RANGE_STATE] = &motorsEndOfRangeState;
  motorsStateMap[MOTORS_START_HOMING_STATE] = &motorsHomingState;
  motorsStateMap[MOTORS_HOMING_FIND_BOTTOM_STATE] =
      &motorsHomingFindBottomState;
  motorsStateMap[MOTORS_HOMING_BOTTOM_FOUND_STATE] =
      &motorsHomingBottomFoundState;
}

/**
 * Sets the state of the controller to a new state.
 *
 * @param newState the new state to set the controller to
 *
 * @throws None
 */
void StateController::setState(MotorControllerState newState) {
  if (currentState != motorsStateMap[newState]) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    currentState = motorsStateMap[newState];
    currentState->enter();
    currentState->update();
  }
}

inline bool StateController::hasTransition() const {
  return currentState->hasTransition;
}

/**
 * Handles the starting transition of state of the extension/Retraction event
 *
 * @param direction the motor system direction requested for the state
 * transition
 *
 * @return None
 *
 * @throws None
 */
void StateController::OnStarting(const Direction &direction) {
  if ((currentState != &motorsStartingState) &&
      (motorController->systemDirection != direction)) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    motorController->systemDirection = direction;
    if (direction == Direction::EXTEND) {
      motorController->extend();
    } else {
      motorController->retract();
    }
    currentState = &motorsStartingState;
    currentState->enter();
    currentState->update();
  }
}

/**
 * Handles the transition of state from starting to moving of the
 * Extension/Retraction event
 *
 * @return None
 *
 * @throws None
 */
void StateController::OnMoving() {
  if ((currentState == &motorsStartingState)) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    currentState = &motorsMovingState;
    currentState->enter();
    currentState->update();
  }
}

/**
 * Handles the transition of to a stopping state for a
 * Extension/Retraction event. This state is entered into before
 * the system actually stops.
 *
 * @return None
 *
 * @throws None
 */
void StateController::OnStopping() {
  if ((currentState != &motorsStoppingState &&
       currentState != &motorsStoppedState)) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    currentState = &motorsStoppingState;
    currentState->enter();
    currentState->update();
  }
}

/**
 * Handles the transition from stopping state to stopped state.
 *
 * @return None
 *
 * @throws None
 */
void StateController::OnStopped() {
  if ((currentState != &motorsStoppedState)) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    currentState = &motorsStoppedState;
    currentState->enter();
    currentState->update();
  }
}

/**
 * Handles end of range speed change
 *
 * @return None
 *
 * @throws None
 */
void StateController::OnEndOfRange() {
  if ((currentState != &motorsEndOfRangeState &&
       currentState != &motorsStoppedState)) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    currentState = &motorsEndOfRangeState;
    currentState->enter();
    currentState->update();
  }
}

/**
 * Sets the current state of the StateController to the motorsHomingState
 * and updates it. If the current state is not motorsHomingState,
 * motorsHomingFindBottomState, or motorsHomingBottomFoundState, the
 * currentState is changed to motorsHomingState and its enter() and update()
 * methods are called. If the currentState is not null, its leave() method is
 * called before changing the currentState.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void StateController::OnHome() {
  if ((currentState != &motorsHomingState &&
       currentState != &motorsHomingFindBottomState &&
       currentState != &motorsHomingBottomFoundState)) {
    if (currentState != nullptr) {
      currentState->leave();
    }

    currentState = &motorsHomingState;
    currentState->enter();
    currentState->update();
  }
}

/**
 * Updates the current state of the motor controller system.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void StateController::update() {
  if (currentState != nullptr && nullptr != motorController) {
    const long timestamp = micros();
    const int moveStart = motorController->moveStart;
    const int moveTimeDelta = timestamp - moveStart;
    const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);

    motorController->deltaT = deltaT;
    if (hasTransition()) {
      setState(currentState->nextStateType);
    }

    currentState->update();

    lastTimestamp = timestamp;
  }
}

/*! \file PidController.hpp */

#ifndef _PID_CONTROLLER_HPP__
#define _PID_CONTROLLER_HPP__

#include "Motor.hpp"
#include "debugging.hpp"
#include "defs.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <stdio.h>

/** @class PIDController
 *
 *  @brief This is the PID controller for motor synchronization
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */

class PIDController {
private:
  float errorIntegral = 0.0f;       /// The  error integral value
  float previousMeasurement = 0.0f; /// The previous follower motor measurement
  float positionDifference = 0.0f;
  float maxSpeedAdjustmentRate = 900.0f;
  float filteredDerivative = 0.0f;
  const float alpha = 0.5;        // Filter constant
  int K_p;                        /// the controller path proportional gain
  float K_i = DEFAULT_KI;         /// the controller's integral component
  float K_d = DEFAULT_KD;         /// the controller's differential component
  int uMax;                       /// Maximum magnitude of control signal
  float derivative = 0.0f;        /// The derivative term value
  float limMinInteg, limMaxInteg; /// Anti Windup Integrator Limits
  float lastError = 0.0f;         /// The last error value
  float followerMaxAccel =
      5.0f; /// New variable for follower's max acceleration
  int followerMaxSpeed = MAX_SPEED; /// New variable for follower's max speed
  float filteredSpeed = 0.0f;       /// Declare filteredSpeed here

  const float MAX_ACCELERATION_INCREASE = 0.01f;
  const float MAX_ACCELERATION_LIMIT = 5.0f;
  const float SPEED_ALPHA = 0.5f;      // The filtering constant for speed.
  const float SETPOINT_WEIGHT = 1.15f; // or 0.8f as per your requirement
  const int POSITION_DELTA_SPEED_SCALER_EXTEND = 23000;
  const int POSITION_DELTA_SPEED_SCALER_RETRACT = 23000;

  const int MAX_POSITION_SCALE_VALUE = 32;

public:
  /**
   * @brief This is the PID controller for motor synchronization
   *
   * @param kp The proportional gain
   * @param ki The integral gain
   * @param kd The derivative gain
   * @param tau The controller's low-pass filter time constant
   * @param uMax Max control signal (speed) for the motors
   */
  PIDController(const int kp = DEFAULT_KP, const float ki = DEFAULT_KI,
                const float kd = DEFAULT_KD, const int uMax = MAX_SPEED);

  /**
   * Sets the parameters for the controller.
   *
   * @param kpIn the proportional gain parameter
   * @param kiIn the integral gain parameter
   * @param kdIn the derivative gain parameter
   * @param uMaxIn the maximum control signal value (default: MAX_SPEED)
   *
   * @throws None
   */
  void setParams(const int kpIn, const float kiIn = DEFAULT_KI,
                 const float kdIn = DEFAULT_KD, const int uMaxIn = MAX_SPEED);

  /**
   * @brief Compute the adjusted speed based on the current value, target value,
   *        and speed.
   *
   * @param leader The leader motor
   * @param follower The follower motor
   * @param speed The reference to the speed variable
   * @param deltaT The time difference between the current and previous
   * iteration
   * @return The adjusted speed
   */

  int adjustSpeed(Motor &leader, Motor &follower, const int speed,
                  const float deltaT = 0.0f);

  void setFollowerMaxAccel(float newMaxAccel) {
    followerMaxAccel = newMaxAccel;
  }

  void setFollowerMaxSpeed(int newMaxSpeed) { followerMaxSpeed = newMaxSpeed; }

  /**
   * @brief Reports the PID parameters.
   *
   * @return void
   */
  void report() const;

  /**
   * @brief Reset the PID Controller
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void reset() {
    lastError = 0.0f;
    errorIntegral = 0.0f;
  }
};

#endif // _PID_CONTROLLER_HPP__

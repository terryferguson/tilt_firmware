/*! \file PidController.hpp */

#ifndef _PID_CONTROLLER_HPP__
#define _PID_CONTROLLER_HPP__

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
  const float alpha = 0.5; // Filter constant
  int K_p;                 /// the controller path proportional gain
  float K_i = DEFAULT_KI;  /// the controller's integral component
  float K_d = DEFAULT_KD;  /// the controller's differential component
  float tau = 1.0f;        /// the controller's low-pass filter time constant
  int uMax;                /// Maximum magnitude of control signal
  float derivative = 0.0f; /// The derivative term value
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
  const int POSITION_DELTA_SPEED_SCALER_EXTEND = 85000;
  const int POSITION_DELTA_SPEED_SCALER_RETRACT = 85000;

  const int MAX_POSITION_SCALE_VALUE = 55;

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
                const float kd = DEFAULT_KD, const float tau = 1.0f,
                const int uMax = MAX_SPEED)
      : K_p(kp), K_i(ki), K_d(kd), tau(tau), uMax(uMax),
        followerMaxSpeed(0.9 * uMax), // Initialize followerMaxSpeed
        limMinInteg(
            -std::numeric_limits<float>::infinity()), // Or another appropriate
                                                      // value
        limMaxInteg(
            std::numeric_limits<float>::infinity()), // Or another appropriate
                                                     // value
        errorIntegral(limMinInteg), // Now it is safe to use limMinInteg
        filteredSpeed(0.0f) {

    if (debugEnabled) {
      Serial.println("PID Controller Initialized");
      Serial.printf("PID Parameters:\n");
      Serial.printf("K_p: %d\n", K_p);
      Serial.printf("K_i: %d\n", K_i);
      Serial.printf("K_d: %d\n", K_d);
      Serial.printf("Last Error: %f\n", lastError);
      Serial.printf("Error integral: %f\n", errorIntegral);
      Serial.printf("Max Speed: %d\n", uMax);
      Serial.println("---------------------------");
    }
  }

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
                 const float kdIn = DEFAULT_KD, const int uMaxIn = MAX_SPEED) {
    K_p = kpIn;
    K_i = kiIn;
    K_d = kdIn;
    uMax = uMaxIn;

    if (fabs(kiIn) < 1e-6) { // Checks if Ki is almost zero
      K_i = 0.0f;
      limMinInteg =
          -std::numeric_limits<float>::infinity(); // Changed to negative
                                                   // infinity
      limMaxInteg = std::numeric_limits<float>::infinity(); // Changed to
                                                            // positive infinity
    } else {
      K_i = kiIn;
      limMinInteg = -uMax / K_i; // Recalculated the minimum limit
      limMaxInteg = uMax / K_i;
    }
    errorIntegral = constrain(errorIntegral, limMinInteg, limMaxInteg);
  }

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
                  const float deltaT = 0.0f) {

    const float leadingMotorPosition = leader.getNormalizedPos();
    const float laggingMotorPosition = follower.getNormalizedPos();
    positionDifference = (laggingMotorPosition - leadingMotorPosition);
    const float error = SETPOINT_WEIGHT * positionDifference;
    const float derivativeTerm = K_d * filteredDerivative;
    const float integralTerm = K_i * errorIntegral;
    maxSpeedAdjustmentRate +=
        MAX_ACCELERATION_INCREASE * 1000; // scaling up the increment
    maxSpeedAdjustmentRate = std::min(
        maxSpeedAdjustmentRate, followerMaxAccel * 1000); // scale up the limit

    // Update filtered speed before it's used in PID calculations
    filteredSpeed = SPEED_ALPHA * filteredSpeed + (1 - SPEED_ALPHA) * speed;

    // Calculate the derivative and then use it to calculate the filtered
    // derivative
    if (deltaT > 0.0f) { // Avoid division by zero
      derivative = (error - lastError) / deltaT;
      filteredDerivative =
          alpha * filteredDerivative + (1 - alpha) * derivative;
    } else {
      derivative = 0.0f;
    }

    const int proportionalTerm = static_cast<int>(round(error * K_p * 3));

    errorIntegral += error * deltaT;
    errorIntegral =
        constrain(errorIntegral, limMinInteg, limMaxInteg); // Anti-Windup

    const int scalarValue = leader.dir == Direction::EXTEND
                                ? POSITION_DELTA_SPEED_SCALER_EXTEND
                                : POSITION_DELTA_SPEED_SCALER_RETRACT;

    const int directionSign = leader.dir == Direction::EXTEND ? 1 : -1;
    const int positionDeltaBoost =
        constrain(directionSign * positionDifference * scalarValue,
                  -MAX_POSITION_SCALE_VALUE, MAX_POSITION_SCALE_VALUE);

    // Serial.printf("Position Delta Boost: %d\n", positionDeltaBoost);

    const int u = speed - (proportionalTerm + integralTerm + derivativeTerm);

    const float maxDeltaSpeed = maxSpeedAdjustmentRate * deltaT *
                                0.001; // scaling down the adjustment rate
    int adjustedDeltaSpeed = constrain(u, -maxDeltaSpeed, maxDeltaSpeed);

    int adjustedSpeed = speed + adjustedDeltaSpeed + positionDeltaBoost;

    // Serial.printf("Adjusted Speed: %d\n", adjustedSpeed);

    if (debugEnabled) {
      // Debug prints
    }

    lastError = error;
    previousMeasurement = laggingMotorPosition;
    return constrain(adjustedSpeed, MIN_SPEED, MAX_SPEED);
  }

  void setFollowerMaxAccel(float newMaxAccel) {
    followerMaxAccel = newMaxAccel;
  }

  void setFollowerMaxSpeed(int newMaxSpeed) { followerMaxSpeed = newMaxSpeed; }

  /**
   * @brief Reports the PID parameters.
   *
   * @return void
   */
  void report() const {
    Serial.printf("\nPID Parameters:\n");
    Serial.println("---------------------------");
    Serial.printf("K_p: %d\n", K_p);
    Serial.printf("K_i: %f\n", K_i);
    Serial.printf("K_d: %f\n", K_d);
    Serial.printf("Position Difference: %f\n", positionDifference);
    Serial.println("---------------------------");
  }

  void setKd(float newKd) { K_d = newKd; }
  void setKi(float newKi) {
    if (fabs(newKi) < 1e-6) { // Checks if newKi is almost zero
      K_i = 0.0f;
      limMinInteg =
          -std::numeric_limits<float>::infinity(); // Changed to negative
                                                   // infinity
      limMaxInteg = std::numeric_limits<float>::infinity(); // Changed to
                                                            // positive infinity
    } else {
      K_i = newKi;
      limMinInteg = -uMax / K_i; // Recalculated the minimum limit
      limMaxInteg = uMax / K_i;
    }
    errorIntegral = constrain(errorIntegral, limMinInteg, limMaxInteg);
  }

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

/*! \file PidController.hpp */

#ifndef _PID_CONTROLLER_HPP__
#define _PID_CONTROLLER_HPP__

#include "defs.hpp"
#include <cstring>
#include <math.h>
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
  int uMax;                       /// Maximum magnitude of control signal
  float lastError = 0.0f;         /// The last error value
  float errorIntegral = 0.0f;     /// The  error integral value
  float limMinInteg, limMaxInteg; /// Anti Windup Integrator Limits
  float previousMeasurement;      /// The previous follower motor measurement
  float derivative = 0.0f;        /// The derivative term value
public:
  int K_p;                /// the controller path proportional gain
  float K_i = DEFAULT_KI; /// the controller's integral component
  float K_d = DEFAULT_KD; /// the controller's differential component
  float tau = 1.0f;       /// the controller's low-pass filter time constant
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
      : K_p(kp), K_i(ki), K_d(kd), tau(tau), uMax(uMax), lastError(0.0f),
        errorIntegral(0.0f), derivative(0.0f) {
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

  // A function to set the parameters
  void setParams(int kpIn, int uMaxIn = MAX_SPEED) {
    K_p = kpIn;
    uMax = uMaxIn;
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
  int adjustSpeed(Motor &leader, Motor &follower, int speed,
                  const float deltaT = 0.0f) {
    // Calculate the error between the target value and the current value
    const float leadingMotorPosition = leader.getNormalizedPos();
    const float laggingMotorPosition = follower.getNormalizedPos();
    const float positionDifference =
        laggingMotorPosition - leadingMotorPosition;
    const float error = fabs(positionDifference);
    const int proportionalTerm = static_cast<int>(error * K_p);
    // Serial.printf("Position Difference: %f\n", positionDifference);

    errorIntegral += error * deltaT;

    const float derivativeTerm = K_d * (fabs(error - lastError) / deltaT);
    const float integralTerm = K_i * errorIntegral;
    const float measurementDifference =
        laggingMotorPosition - previousMeasurement;

    // Calculate the control signal
    const int u =
        speed - static_cast<int>(proportionalTerm + integralTerm + derivative);

    if (debugEnabled) {
      Serial.printf("Proportional term: %f\n", proportionalTerm);
      Serial.printf("Derivative term: %f\n", derivativeTerm);
      Serial.printf("Integral term: %f\n", integralTerm);
      Serial.printf("u: %d\n", u);
    }

    // Adjust the speed within the range [0, uMax]
    int adjustedSpeed = static_cast<int>(u);
    adjustedSpeed = constrain(adjustedSpeed, 0, uMax);

    // Save the current error for the next iteration
    lastError = error;

    // Save previous measurement
    previousMeasurement = laggingMotorPosition;

    // Return the adjusted speed
    return adjustedSpeed;
  }

  /**
   * @brief Reports the PID parameters.
   *
   * @return void
   */
  void report() {
    Serial.printf("\nPID Parameters:\n");
    Serial.println("---------------------------");
    Serial.printf("K_p: %d\n", K_p);
    Serial.printf("K_i: %f\n", K_i);
    Serial.printf("K_d: %f\n", K_d);
    Serial.printf("Last Error: %f\n", lastError);
    Serial.printf("Error integral: %f\n", errorIntegral);
    Serial.printf("Max Speed: %d\n", uMax);
    Serial.println("---------------------------");
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

/*! \file PidController.hpp */

#ifndef _PID_CONTROLLER_HPP__
#define _PID_CONTROLLER_HPP__

#include "defs.hpp"
#include <cstring>
#include <math.h>
#include <stdio.h>

/** @class PIDController
 *
 *  @brief This is the  PID controller for motor synchronization
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
class PIDController {
private:
  int K_p;  // the controller path proportional gain
  int uMax; // Maximum magnitude of control signal

public:
  PIDController(float kp = 39500, float uMax = 255.0) : K_p(kp), uMax(uMax) {
    if (debugEnabled) {
      Serial.println("PID Controller Initialized");
      Serial.printf("PID Parameters:\n");
      Serial.printf("K_p: %d\n", K_p);
      Serial.printf("Max Speed: %d\n", uMax);
      Serial.println("---------------------------");
    }
  }

  // A function to set the parameters
  void setParams(int kpIn, int uMaxIn = 255.0) {
    K_p = kpIn;
    uMax = uMaxIn;
  }

  /**
   * Compute the adjusted speed based on the current value, target value, and
   * speed.
   *
   * @param leader The leader motor
   * @param follower The follower motor
   * @param speed The reference to the speed variable
   * @return The adjusted speed
   */
  int adjustSpeed(const Motor &leader, const Motor &follower, int speed) {
    // Calculate the error between the target value and the current value
    const float leadingMotorPosition = leader.getNormalizedPos();
    const float laggingMotorPosition = follower.getNormalizedPos();
    const float error = fabs(leadingMotorPosition - laggingMotorPosition);

    // Calculate the control signal
    const float u = speed - (error * K_p);

    // Calculate the motor power using a ternary operator
    int adjustedSpeed = static_cast<int>(fabs(u));
    adjustedSpeed = adjustedSpeed > uMax ? uMax : adjustedSpeed;

    return adjustedSpeed;
  }

  /**
   * Reports the PID parameters.
   *
   * @return void
   */
  void report() {
    Serial.printf("\nPID Parameters:\n");
    Serial.println("---------------------------");
    Serial.printf("K_p: %d\n", K_p);
    Serial.printf("Max Value: %d\n\n", uMax);
  }
};

#endif // _PID_CONTROLLER_HPP__

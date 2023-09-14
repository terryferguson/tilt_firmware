/*! \file PidController.hpp */

#ifndef _PID_CONTROLLER_HPP__
#define _PID_CONTROLLER_HPP__

#include "defs.hpp"
#include <cstring>
#include <math.h>
#include <stdio.h>


class PIDController {
private:
  float kp;               // the controller path proportional gain
  float ti;               // the controller's integrator time constant
  float td;               // the controller's derivative time constant
  float uMax;             // Maximum magnitude of control signal
  float ePrev, eIntegral; // Storage

public:
  PIDController(float kp = 0.1, float ti = 0.002, float td = 0.01,
                float uMax = 255.0)
      : kp(kp), ti(ti), td(td), uMax(uMax), ePrev(0.0), eIntegral(0.0) {}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float uMaxIn = 255.0) {
    kp = kpIn;
    td = kdIn;
    ti = kiIn;
    uMax = uMaxIn;
  }

  /**
   * A function to compute the control signal
   *
   * @param value The current value
   * @param target The target value
   * @param deltaT The time step
   * @param speed The reference to the speed variable
   * @param dir The reference to the direction variable
   */
  void evaluate(int value, int target, float deltaT, int &speed,
                Direction &dir) {
    // Calculate the error between the target value and the current value
    int e = target - value;

    // Calculate the derivative of the error
    float dedt = static_cast<float>(e - ePrev) / deltaT;

    // Calculate the integral of the error
    eIntegral += e * deltaT;

    // Calculate the control signal
    float u = kp * e + td * dedt + ti * eIntegral;

    // Calculate the motor power
    speed = static_cast<int>(fabs(u));
    speed = speed > uMax ? uMax : speed;

    // Determine the motor direction based on the control signal
    dir = u < 0   ? Direction::RETRACT
          : u > 0 ? Direction::EXTEND
                  : Direction::STOP;

    // Store the current error for the next iteration
    ePrev = e;
  }

  void report(const int value, const int target, const float deltaT,
              const int speed, const Direction dir) const {
    char buf[256]; // Diagnostic messages
    sprintf(buf,
            "PID params: value:%d target: %d, deltaT: %f, speed: %d, dir: %s",
            value, target, deltaT, speed, directions[static_cast<int>(dir)]);
    Serial.println(buf);
  }
};

#endif // _PID_CONTROLLER_HPP__

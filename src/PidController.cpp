#include "PIDController.hpp"
#include "SystemState.hpp"
#include <Arduino.h>

extern SystemState systemState;

/**
 * @brief This is the PID controller for motor synchronization
 *
 * @param kp The proportional gain
 * @param ki The integral gain
 * @param kd The derivative gain
 * @param tau The controller's low-pass filter time constant
 * @param uMax Max control signal (speed) for the motors
 */
PIDController::PIDController(const int kp, const float ki, const float kd,
                             const int uMax)
    : K_p(kp), K_i(ki), K_d(kd), uMax(uMax),
      followerMaxSpeed(0.9 * uMax), // Initialize followerMaxSpeed
      limMinInteg(-std::numeric_limits<float>::infinity()), // Or another
                                                            // appropriate value
      limMaxInteg(std::numeric_limits<float>::infinity()),  // Or another
                                                            // appropriate value
      errorIntegral(limMinInteg), // Now it is safe to use limMinInteg
      filteredSpeed(0.0f) {

  if (systemState.debugEnabled) {
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
void PIDController::setParams(const int kpIn, const float kiIn,
                              const float kdIn, const int uMaxIn) {
  K_p = kpIn;
  K_i = kiIn;
  K_d = kdIn;
  uMax = uMaxIn;

  if (fabs(kiIn) < 1e-6) { // Checks if Ki is almost zero
    K_i = 0.0f;
    limMinInteg = -std::numeric_limits<float>::infinity(); // Changed to
                                                           // negative infinity
    limMaxInteg = std::numeric_limits<float>::infinity();  // Changed to
                                                           // positive infinity
  } else {
    K_i = kiIn;
    limMinInteg = -uMax / K_i; // Recalculated the minimum limit
    limMaxInteg = uMax / K_i;
  }
  errorIntegral = constrain(errorIntegral, limMinInteg, limMaxInteg);
}

int PIDController::adjustSpeed(Motor &leader, Motor &follower, const int speed,
                               const float deltaT) {

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
    filteredDerivative = alpha * filteredDerivative + (1 - alpha) * derivative;
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

  if (systemState.debugEnabled) {
    // Debug prints
  }

  lastError = error;
  previousMeasurement = laggingMotorPosition;
  return constrain(adjustedSpeed, MIN_SPEED, MAX_SPEED);
}

/**
 * @brief Reports the PID parameters.
 *
 * @return void
 */
void PIDController::report() const {
  if (!systemState.debugEnabled) {
    return;
  }

  Serial.printf("\nPID Parameters:\n");
  Serial.println("---------------------------");
  Serial.printf("K_p: %d\n", K_p);
  Serial.printf("K_i: %f\n", K_i);
  Serial.printf("K_d: %f\n", K_d);
  Serial.printf("Position Difference: %f\n", positionDifference);
  Serial.println("---------------------------");
}

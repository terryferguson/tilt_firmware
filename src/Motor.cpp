#include "Motor.hpp"
#include "Arduino.h"

Motor::Motor() {}

/// @brief The constructor for the motor controlled by the microcontroller
/// @param name The name of this motor for debug prints
/// @param rpwm The right PWM signal pin
/// @param lpwm The left PWM signal pin
/// @param r_en The right PWM enable pin
/// @param l_en The left PWM enable pin
/// @param hall_1 The pin for hall sensor 1
/// @param hall_2  The pin for hall sensor 2
/// @param currentSensePin The pin for current sensor
/// @param totalPulses The total number of pulses from full retraction to full
/// extension
/// @param freq The frequency of the PWM signal
/// @param defSpeed The default motor speed
/// @param pwmRes The PWM bitdepth resolution
Motor::Motor(const char *name, const MotorPin rpwm, const MotorPin lpwm,
             const MotorPin r_en, const MotorPin l_en, const MotorPin hall_1,
             const MotorPin hall_2, const adc1_channel_t currentSensePin,
             const int totalPulses, const int freq, const int defSpeed,
             const int pwmRes, const int bottomLimitPin, const int currentLimit,
             const int alarmCurrentLimit, const int stopBuffer)
    : rPWM_Pin(rpwm), lPWM_Pin(lpwm), r_EN_Pin(r_en), l_EN_Pin(l_en),
      hall_1_Pin(hall_1), hall_2_Pin(hall_2), currentSensePin(currentSensePin),
      totalPulseCount(totalPulses), frequency(freq), speed(defSpeed),
      pwmResolution(pwmRes), bottomLimitPin(bottomLimitPin),
      bottomCurrentLimit(currentLimit), currentAlarmLimit(alarmCurrentLimit),
      stopBuffer(stopBuffer), outOfRange(false) {
  /// Copy name of linear actuator into ID field
  strncpy(id, name, sizeof(id) - 1);
  id[sizeof(id) - 1] = '\0';
}

/// @brief Initialize motor
void Motor::initialize() {
  // At least two channels are needed for the linear actuator motor
  if (currentPWMChannel > -1 && currentPWMChannel < 14) {
    pwmRChannel = currentPWMChannel++;
    pwmLChannel = currentPWMChannel++;
  }

  ledcSetup(pwmRChannel, frequency, pwmResolution);
  ledcSetup(pwmLChannel, frequency, pwmResolution);

  motorAttachPin(rPWM_Pin, pwmRChannel);
  Serial.printf("Attaching pin %d to RPWM Channel %d\n", rPWM_Pin, pwmRChannel);
  motorAttachPin(lPWM_Pin, pwmLChannel);
  Serial.printf("Attaching pin %d to LPWM Channel %d\n\n", lPWM_Pin,
                pwmLChannel);

  pinMode(bottomLimitPin, INPUT_PULLUP);

  motorPinMode(r_EN_Pin, OUTPUT);
  motorPinMode(l_EN_Pin, OUTPUT);

  motorPinWrite(r_EN_Pin, HIGH);
  motorPinWrite(l_EN_Pin, HIGH);

  ledcWrite(pwmRChannel, 0);
  ledcWrite(pwmLChannel, 0);

  distanceSensor.attachSingleEdge(static_cast<int>(hall_1_Pin),
                                  static_cast<int>(hall_2_Pin));
  distanceSensor.clearCount();
  pos = distanceSensor.getCount();
  currentSense.initialize(currentSensePin);

  Serial.printf("Motor: %s\n"
                "-------------------\n"
                "Frequency:    %5d\n"
                "Resolution:   %5d\n"
                "Speed:        %5d\n"
                "Position:     %5d\n"
                "R_EN Pin:     %5d\n"
                "L_EN Pin:     %5d\n"
                "RPWM Pin:     %5d\n"
                "LPWM Pin:     %5d\n"
                "Hall 1 Pin:   %5d\n"
                "Hall 2 Pin:   %5d\n"
                "Max Position: %5d\n\n"
                "Stop Buffer: %5d pulses\n\n"
                "Alarm Current: %5d mA\n\n",
                id, frequency, pwmResolution, speed, pos, r_EN_Pin, l_EN_Pin,
                rPWM_Pin, lPWM_Pin, hall_1_Pin, hall_2_Pin, totalPulseCount,
                stopBuffer, currentAlarmLimit);
  Serial.printf("RPWM Channel %d - LPWM Channel: %d\n\n", pwmRChannel,
                pwmLChannel);
}

/**
 * Drives the motor in the specified direction at the specified speed.
 *
 * @param motorDirection the direction in which to drive the motor, either
 * "EXTEND" or "RETRACT"
 * @param specifiedSpeed the speed at which to drive the motor, defaults to 0 if
 * not specified
 *
 * @throws None
 */
void Motor::drive(const Direction motorDirection, const int specifiedSpeed) {
  // Set the drive speed based on the specified speed or the default speed
  const int driveSpeed = specifiedSpeed > 0 ? specifiedSpeed : speed;

  motorPinWrite(r_EN_Pin, HIGH);
  motorPinWrite(l_EN_Pin, HIGH);

  switch (motorDirection) {
  case Direction::EXTEND:
    // Drive the motor in the extend direction
    ledcWrite(pwmRChannel, driveSpeed);
    ledcWrite(pwmLChannel, 0);
    break;
  case Direction::RETRACT:
    // Drive the motor in the retract direction
    ledcWrite(pwmRChannel, 0);
    ledcWrite(pwmLChannel, driveSpeed);
    break;

  default:
    break;
  }

  // Update the last position variable
  lastPos = pos;
}

/**
 * Toggles the direction of the motor between extending and stopping.
 *
 * @param N/A
 *
 * @return N/A
 *
 * @throws None
 */
void Motor::extend() {
  // Works as a toggle
  dir = (dir != Direction::EXTEND) ? Direction::EXTEND : Direction::STOP;
}

/**
 * Toggles the direction of the motor between RETRACT and STOP.
 *
 * @throws None
 */
void Motor::retract() {
  // Works as a toggle
  dir = (dir != Direction::RETRACT) ? Direction::RETRACT : Direction::STOP;
}

/**
 * @brief Disables the motor.
 * If the motor is already stopped, this function does nothing.
 * Otherwise, it stops the motor, sets the speed to 0, and logs a message.
 */
void Motor::disable() {
  if (dir != Direction::STOP) {
    dir = Direction::STOP;
    motorPinWrite(r_EN_Pin, HIGH);
    motorPinWrite(l_EN_Pin, HIGH);
    speed = 0;
    Serial.printf("Disabled motor: %s\n", id);
  }
}

/**
 * @brief Update the state of the motor.
 *
 * @param newSpeed the new speed value to set (default: MAX_SPEED + 1)
 *
 * @throws None
 */
void Motor::update(const int newSpeed) {
  pos = distanceSensor.getCount();

  bool goingPastBottom = false;
  // Check if the motor is going past the bottom limit
  if (!homing) {
    goingPastBottom = (pos < stopBuffer) && dir == Direction::RETRACT;
  } else {
    speed = speed <= MIN_MOTOR_TRAVEL_SPEED ? speed : MIN_MOTOR_TRAVEL_SPEED;
    goingPastBottom = hitBottom();
    // Serial.printf("Motor current: %d\n", getCurrent());
  }

  // Check if the motor is going past the top limit
  const bool goingPastTop = topReached() && dir == Direction::EXTEND;

  // Check whether the motor is out of range
  outOfRange = goingPastTop || goingPastBottom;

  // If the motor is out of range or in the STOP direction, stop it
  if (outOfRange || dir == Direction::STOP) {
    if (!homing && (dir == Direction::STOP) && pos < 0) {
      zero();
    }
    dir = Direction::STOP;
    ledcWrite(pwmRChannel, 0);
    ledcWrite(pwmLChannel, 0);
    motorPinWrite(r_EN_Pin, HIGH);
    motorPinWrite(l_EN_Pin, HIGH);
    return;
  }

  // Set the motor speed based on the input
  if (newSpeed > MAX_SPEED || newSpeed < 0) {
    drive(dir, this->speed);
  } else {
    drive(dir, newSpeed);
  }
}
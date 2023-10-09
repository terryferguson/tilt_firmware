/*! \file defs.hpp */

#ifndef _DEFS_HPP_
#define _DEFS_HPP_

#define countof(a) (sizeof(a) / sizeof(*(a)))

#define FORMAT_SPIFFS_IF_FAILED true
#include "Commands.hpp"
#include "ControlPins.hpp"
#include "Direction.hpp"
#include "MotorPins.hpp"

//@brief String representations of the motor roles at instantiation
const char *motor_roles[2] = {"LEADER", "FOLLOWER"};

#define NUM_POSITION_SLOTS 5
const char *save_position_slot_names[NUM_POSITION_SLOTS] = {
    "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
};

//@brief Storage for position in hall sensor pusles relative to initial position
// when powered on
int savedPositions[NUM_POSITION_SLOTS] = {0, 0, 0, 0, 0};

/// @brief Indicates whether debug messages should be sent to serial
bool debugEnabled = false;

#define PWM_FREQUENCY 20000

#define PWM_RESOLUTION_BITS 8

#define DEFAULT_MOTOR_SPEED 255

#define MOTOR_END_OF_RANGE_SPEED 155

#define MOTOR_END_OF_RANGE_SPEED_DELTA                                         \
  (DEFAULT_MOTOR_SPEED - MOTOR_END_OF_RANGE_SPEED)

#define MILLIS_IN_SEC 1000
#define MICROS_IN_MS 1000
#define MICROS_IN_SEC (MILLIS_IN_SEC * MICROS_IN_MS)

#define SOFT_MOVEMENT_TIME_MS 16000

#define SOFT_MOVEMENT_MICROS (SOFT_MOVEMENT_TIME_MS * MICROS_IN_MS)

#define SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS 800

#define SOFT_MOVEMENT_UPDATE_STEPS                                             \
  (SOFT_MOVEMENT_MICROS / SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS)

#define MAX_SPEED ((1 << (PWM_RESOLUTION_BITS)) - 1)

#define MIN_SPEED 0

#define MAX_ADC_VALUE (1 << (ADC_RESOLUTION_BITS)-1)

#define CURRENT_ALARM_AMOUNT 1275
#define CURRENT_OFFSET 150
#define CURRENT_UPDATE_INTERVAL 250000
#define CURRENT_ALARM_DELAY 20000000
#define ALARM_REVERSE_AMOUNT 30
#define FOLLOWER_ALARM_REVERSE_AMOUNT (ALARM_REVERSE_AMOUNT + 30)

#define LEADER_BUFFER 3
#define FOLLOWER_BUFFER 3

#define DESYNC_TOLERANCE 9

#define PID_ALPHA 4.7619f
#define DEFAULT_KP 333000
#define RETRACT_KP 333000
#define STOP_KP 333000
#define EXTEND_RAMP_KP 70000
#define RETRACT_RAMP_KP 70000
#define DEFAULT_KI (DEFAULT_KP / PID_ALPHA)
#define DEFAULT_KD (DEFAULT_KP / (PID_ALPHA * 7))

/** @brief Whether PID is on or off */
bool pid_on = true;

/** @brief Whether limit range is on or off */
bool limit_range = true;

#endif // _DEFS_HPP_

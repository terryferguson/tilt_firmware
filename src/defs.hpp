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

#define PWM_FREQUENCY 22500

#define PWM_RESOLUTION_BITS 8

#define ADC_RESOLUTION_BITS 12

#define DEFAULT_MOTOR_SPEED 230

#define MOTOR_END_OF_RANGE_SPEED 172

#define MOTOR_END_OF_RANGE_SPEED_DELTA                                         \
  (DEFAULT_MOTOR_SPEED - MOTOR_END_OF_RANGE_SPEED)

#define MICROS_IN_MS 1000

#define SOFT_MOVEMENT_TIME_MS 1450

#define SOFT_MOVEMENT_MICROS (SOFT_MOVEMENT_TIME_MS * MICROS_IN_MS)

#define SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS 925

#define SOFT_MOVEMENT_UPDATE_STEPS                                             \
  (SOFT_MOVEMENT_MICROS / SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS)

#define MAX_SPEED ((1 << (PWM_RESOLUTION_BITS)) - 1)

#define MIN_SPEED 0

#define MAX_ADC_VALUE (1 << (ADC_RESOLUTION_BITS))

#define ALARM_REVERSE_AMOUNT 30

#define CURRENT_UPDATE_INTERVAL 250000

#define CURRENT_ALARM_DELAY 20000000

#define CURRENT_OFFSET 50

#define LEADER_BUFFER 7
#define FOLLOWER_BUFFER 3

#define DEFAULT_KP 100000
#define RETRACT_KP 100000
#define STOP_KP 100000
#define EXTEND_RAMP_KP 100000
#define RETRACT_RAMP_KP 75000

#define DEFAULT_KI 4000.0f

#define DEFAULT_KD 750.0f

/** @brief Whether PID is on or off */
bool pid_on = true;

/** @brief Whether limit range is on or off */
bool limit_range = true;

#endif // _DEFS_HPP_

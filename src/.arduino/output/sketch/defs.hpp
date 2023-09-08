#line 1 "/home/terry/Projects/motor_control_firmware/defs.hpp"
/*! \file defs.hpp */

#ifndef _DEFS_HPP_
#define _DEFS_HPP_

#define FORMAT_SPIFFS_IF_FAILED true

#include "Commands.hpp"
#include "MotorPins.hpp"
#include "ControlPins.hpp"
#include "Direction.hpp"

//@brief String representations of the motor roles at instantiation
const char *motor_roles[2] = {"LEADER", "FOLLOWER"};

#define NUM_POSITION_SLOTS 5
const char *save_position_slot_names[NUM_POSITION_SLOTS] = {
    "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
};

//@brief Storage for position in hall sensor pusles relative to initial position
//when powered on
int savedPositions[NUM_POSITION_SLOTS] = {0, 0, 0, 0, 0};

/// @brief Indicates whether debug messages should be sent to serial
bool debugEnabled = true;

#define PWM_FREQUENCY 15000

#define PWM_RESOLUTION_BITS 8

#define ADC_RESOLUTION_BITS 12

#define DEFAULT_MOTOR_SPEED 192

#define MICROS_IN_MS 1000

#define SOFT_MOVEMENT_TIME_MS 2000

#define SOFT_MOVEMENT_MICROS (SOFT_MOVEMENT_TIME_MS * MICROS_IN_MS)

#define SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS 20000

#define SOFT_MOVEMENT_UPDATE_STEPS                                             \
  (SOFT_MOVEMENT_MICROS / SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS)

#define MAX_SPEED (1 << (PWM_RESOLUTION_BITS)-1)

#define MIN_SPEED (1 << (PWM_RESOLUTION_BITS)-1) * -1

#define MAX_ADC_VALUE (1 << (ADC_RESOLUTION_BITS))

bool pid_on = true;

#endif // _DEFS_HPP_

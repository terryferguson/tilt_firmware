/*! \file defs.hpp */
#ifndef _DEFS_HPP_
#define _DEFS_HPP_

#define countof(a) (sizeof(a) / sizeof(*(a)))

#include "Commands.hpp"
#include "ControlPins.hpp"
#include "Direction.hpp"
#include "MotorPins.hpp"

extern int currentPWMChannel;

/// @brief String representations of the directions
extern const char *directions[];

/// @brief String representations of the motor roles at instantiation
extern const char *motor_roles[];

/** @brief Number of position slots supported by this firmware */
constexpr int NUM_POSITION_SLOTS = 5;

/** @brief String representations of the names of position slots */
extern const char *save_position_slot_names[];

/**
 * @brief Storage for position in hall sensor pusles relative to initial
 * position when powered on
 */
extern int savedPositions[];

/** @brief Whether PID is on or off */
extern bool pid_on;

/** @brief Whether limit range is on or off */
extern bool limit_range;

/// @brief Indicates whether debug messages should be sent to serial
extern bool debugEnabled;

constexpr int FORMAT_SPIFFS_IF_FAILED = true;

/** @brief The number of motors controlled by this system */
constexpr int NUMBER_OF_MOTORS = 2;

/** @brief Number of pulses for leader motor's maximum extension */
constexpr int LEADER_MAX_PULSES = 2845;

/** @brief Number of pulses for follower motor's maximum extension */
constexpr int FOLLOWER_MAX_PULSES = 2845;

/** @brief The frequency of the PWM signal sent to the motor controllers */
constexpr int PWM_FREQUENCY = 20000;

/** @brief The resolution of the PWM signal in bits (provides
 * 2^`PWM_RESOLUTION_BITS` of possible levels) */
constexpr int PWM_RESOLUTION_BITS = 8;

/** @brief The resolution of the ADC used in bits */
constexpr int ADC_RESOLUTION_BITS = 12;

/** @brief The default speed of the motors given in PWM value */
constexpr int DEFAULT_MOTOR_SPEED = (1 << PWM_RESOLUTION_BITS) - 1;

/** @brief The motor speed at the extremes of the range. Should be <
 * `DEFAULT_MOTOR_SPEED` */
constexpr int MOTOR_END_OF_RANGE_SPEED = 155;

/** @brief Minimum travel speed */
constexpr int MIN_MOTOR_TRAVEL_SPEED = 105;

/** @brief The difference in speed values between
 * `DEFAULT_MOTOR_SPEED` and `MOTOR_END_OF_RANGE_SPEED` */
constexpr int MOTOR_END_OF_RANGE_SPEED_DELTA =
    (DEFAULT_MOTOR_SPEED - MOTOR_END_OF_RANGE_SPEED);

/** @brief Set position buffer in hall pulses */
constexpr int SET_POSITION_BUFFER = 10;

/** @brief The number of milliseconds in a second */
constexpr int MILLIS_IN_SEC = 1000;

/** @brief The number of microseconds in a millisecond */
constexpr int MICROS_IN_MS = 1000;

/** @brief The number of microseconds in a second */
constexpr int MICROS_IN_SEC = (MILLIS_IN_SEC * MICROS_IN_MS);

/** @brief The number of milliseconds over which the soft movement occurs */
constexpr int SOFT_MOVEMENT_TIME_MS = 500;

/** @brief The number of microseconds over which the soft movement occurs */
constexpr int SOFT_MOVEMENT_MICROS = (SOFT_MOVEMENT_TIME_MS * MICROS_IN_MS);

/** @brief The minimum interval between PWM updates in microseconds */
constexpr int SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS =
    SOFT_MOVEMENT_MICROS / 2000;

/** @brief The maximum number of PWM updates over which the soft movement occurs
 */
constexpr int SOFT_MOVEMENT_UPDATE_STEPS =
    (SOFT_MOVEMENT_MICROS / SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS);

/** @brief The maximum speed value that can be represented by PWM */
constexpr int MAX_SPEED = ((1 << (PWM_RESOLUTION_BITS)) - 1);

/** @brief The minimum speed value that can be represented by PWM */
constexpr int MIN_SPEED = 0;

/** The logic level voltage of the ADC */
constexpr float ADC_LOGIC_VOLTAGE = 3.3f;

/** @brief The maximum ADC value that can be represented by the ADC resolution
 */
constexpr int MAX_ADC_VALUE = (1 << (ADC_RESOLUTION_BITS)) - 1;

/** @brief Maximum current in milliamps allowed before the system halts */
constexpr int CURRENT_LIMIT = 2000;

/** @brief The current offset between the two motors. */
constexpr int CURRENT_OFFSET = 17;

/** @brief The minimum interval in microseconds between current reading updates.
 */
constexpr int CURRENT_UPDATE_INTERVAL = 5000;

/** @brief The minimum time the motors must be moving before enabling the
 * current alarm. */
constexpr int CURRENT_ALARM_DELAY = 3000000;

/** @brief The number of hall pulses to back up leader motor after hitting
 * bottom on homing routine */
constexpr int ALARM_REVERSE_AMOUNT = 15;

/** @brief The number of hall pulses to back up follower motor after hitting
 * bottom on homing routine */
constexpr int FOLLOWER_ALARM_REVERSE_AMOUNT = (ALARM_REVERSE_AMOUNT + 30);

/** @brief A buffer accounting for the lag in hall pulses between a stop request
 * and the leader motor physically stopping */
constexpr int LEADER_BUFFER = 3;

/** @brief A buffer accounting for the lag in hall pulses between a stop request
 * and the follower motor physically stopping */
constexpr int FOLLOWER_BUFFER = 3;

/** @brief The number of hall pulse difference between the leader and follower
 * motors before triggering an immediate halt */
constexpr int DESYNC_TOLERANCE = 20;

/** @brief The alpha ratio value used to PID calculation parameters */
constexpr float PID_ALPHA = 33.333333f;

/** @brief The default propotional gain used in PID calculation */
constexpr int DEFAULT_KP = 79999;

/** @brief The propotional gain used in PID calculation for extension */
constexpr int RETRACT_KP = 79999;

/** @brief The propotional gain used in PID calculation for stopping */
constexpr int STOP_KP = 79999;

/** @brief The propotional gain used in PID calculation for extension ramping */
constexpr int EXTEND_RAMP_KP = 79999;

/** @brief The propotional gain used in PID calculation for retraction ramping
 */
constexpr int RETRACT_RAMP_KP = 79999;

/** @brief The integral gain used in PID calculation */
constexpr float DEFAULT_KI = ((DEFAULT_KP / PID_ALPHA) * 6);

/** @brief The derivative gain used in PID calculation */
constexpr float DEFAULT_KD = (DEFAULT_KP / (PID_ALPHA * 1.5));

/** @brief The tolerance percentage for the current increase before alarm */
constexpr int CURRENT_INCREASE_TOLERANCE_PERCENTAGE = 30;

/** @brief The current increase multiplier based on the current increase
 * tolerance percentage */
constexpr float CURRENT_INCREASE_MULTIPLIER =
    1 + (CURRENT_INCREASE_TOLERANCE_PERCENTAGE / 100.0f);

/** @brief Soft stop time in milliseconds */
constexpr int SOFT_STOP_TIME_MS = 160;

#endif // _DEFS_HPP_

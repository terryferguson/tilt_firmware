/*! \file defs.hpp */
#ifndef _DEFS_HPP_
#define _DEFS_HPP_
#define countof(x)                                                             \
  ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

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

/** @brief String representations of the names of configuration slots */
extern const char *save_configuration_slot_names[];

/**
 * @brief Storage for position in hall sensor pusles relative to initial
 * position when powered on
 */
extern int savedPositions[];

/**
 * @brief Storage for configuration position in hall sensor pusles relative to
 * initial position when powered on
 */
extern int savedConfigurations[];

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
constexpr int LEADER_MAX_PULSES = 8085;

/** @brief Number of pulses for follower motor's maximum extension */
constexpr int FOLLOWER_MAX_PULSES = 8085;

/** @brief The frequency of the PWM signal sent to the motor controllers */
constexpr int PWM_FREQUENCY = 20000;

/** @brief The resolution of the PWM signal in bits (provides
 * 2^`PWM_RESOLUTION_BITS` of possible levels) */
constexpr int PWM_RESOLUTION_BITS = 8;

/** @brief The resolution of the ADC used in bits */
constexpr int ADC_RESOLUTION_BITS = 12;

/** @brief The default speed of the motors given in PWM value */
constexpr int DEFAULT_MOTOR_SPEED = 255;

/** @brief The motor speed at the extremes of the range. Should be <
 * `DEFAULT_MOTOR_SPEED` */
constexpr int MOTOR_END_OF_RANGE_SPEED = 155;

/** @brief Minimum travel speed */
constexpr int MIN_MOTOR_TRAVEL_SPEED = 155;

/** @brief The difference in speed values between
 * `DEFAULT_MOTOR_SPEED` and `MOTOR_END_OF_RANGE_SPEED` */
constexpr int MOTOR_END_OF_RANGE_SPEED_DELTA =
    (DEFAULT_MOTOR_SPEED - MOTOR_END_OF_RANGE_SPEED);

/** @brief Set position buffer in hall pulses */
constexpr int SET_POSITION_BUFFER = 10;

/** @brief The number of milliseconds in a second */
constexpr unsigned long MILLIS_IN_SEC = 1000UL;

/** @brief The number of microseconds in a millisecond */
constexpr unsigned long MICROS_IN_MS = 1000UL;

/** @brief The number of microseconds in a second */
constexpr unsigned long MICROS_IN_SEC = (MILLIS_IN_SEC * MICROS_IN_MS);

/** @brief The number of milliseconds over which the soft movement occurs */
constexpr unsigned long SOFT_MOVEMENT_TIME_MS = 1000UL;

/** @brief The number of microseconds over which the soft movement occurs */
constexpr unsigned long SOFT_MOVEMENT_MICROS =
    (SOFT_MOVEMENT_TIME_MS * MICROS_IN_MS);

/** @brief The minimum interval between PWM updates in microseconds */
constexpr unsigned long SOFT_MOVEMENT_PWM_UPDATE_INTERVAL_MICROS =
    SOFT_MOVEMENT_MICROS / 1000UL;

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
constexpr int CURRENT_LIMIT = 2000UL;

/** @brief The minimum interval in microseconds between current reading updates.
 */
constexpr unsigned long CURRENT_UPDATE_INTERVAL = 5000UL;

/** @brief The minimum time the motors must be moving before enabling the
 * current alarm. */
constexpr unsigned long CURRENT_ALARM_DELAY = SOFT_MOVEMENT_MICROS + 1000000UL;

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
constexpr int DEFAULT_KP = 163000;

/** @brief The propotional gain used in PID calculation for extension */
constexpr int RETRACT_KP = 53000;

/** @brief The propotional gain used in PID calculation for stopping */
constexpr int STOP_KP = 173000;

/** @brief The propotional gain used in PID calculation for extension ramping */
constexpr int EXTEND_RAMP_KP = 15000;

/** @brief The propotional gain used in PID calculation for retraction ramping
 */
constexpr int RETRACT_RAMP_KP = 15000;

/** @brief The integral gain used in PID calculation */
constexpr float DEFAULT_KI = ((DEFAULT_KP / PID_ALPHA) * 10);

/** @brief The derivative gain used in PID calculation */
constexpr float DEFAULT_KD = (DEFAULT_KP / (PID_ALPHA * 4));

/** @brief The tolerance percentage for the current increase before alarm */
constexpr int CURRENT_INCREASE_TOLERANCE_PERCENTAGE = 45;

/** @brief The current increase multiplier based on the current increase
 * tolerance percentage */
constexpr float CURRENT_INCREASE_MULTIPLIER =
    1 + (CURRENT_INCREASE_TOLERANCE_PERCENTAGE / 100.0f);

/** @brief Soft stop time in milliseconds */
constexpr unsigned long SOFT_STOP_TIME_MS = 150UL;

/** @brief Number of pulses difference before transitioning to stopping state */
constexpr int DESIRED_POSITION_BUFFER = 50;

#endif // _DEFS_HPP_

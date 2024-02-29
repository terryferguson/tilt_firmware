/*! \file Commands.hpp */

#ifndef _COMMANDS_HPP_
#define _COMMANDS_HPP_

#include <cstdint>

/** @enum CommandType20
 *
 *
 *  @class CommandType
 *
 *  @brief Represents the types of commands recognized by the firmware
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 * @version 0.1
 */
enum class Command : std::uint32_t {
  /** Command to tell motors to retract - 17 */
  RETRACT = 17,

  /** Command to tell motors to extend - 18 */
  EXTEND,

  /** Command to tell tell the motor controller to report its state - 19 */
  REPORT,

  /** Command to tell the motor controller to stop - 20 */
  STOP,

  /** Save value to stored position slot 1 - 21 */
  SAVE_TILT_1,

  /** Save value to stored position slot 2 - 22 */
  SAVE_TILT_2,

  /** Save value to stored position slot 3 - 23 */
  SAVE_TILT_3,

  /** Save value to stored position slot 4 - 24 */
  SAVE_TILT_4,

  /** Save value to stored position slot 5 - 25 */
  SAVE_TILT_5,

  /** Get value from stored position slot 1 - 26 */
  GET_TILT_1,

  /** Get value from stored position slot 2 - 27 */
  GET_TILT_2,

  /** Get value from stored position slot 3 - 28 */
  GET_TILT_3,

  /** Get value from stored position slot 4 - 29 */
  GET_TILT_4,

  /** Get value from stored position slot 5 - 30 */
  GET_TILT_5,

  /** Command to tell tell the motor controller to reset position counters - 31
   */
  ZERO,

  /** Command to tell tell the microcontroller to reset - 32 */
  SYSTEM_RESET,

  /** Command to tell tell the microcontroller to turn off PID control - 33 */
  TOGGLE_PID,

  /** Home the linear actuators - 34 */
  HOME,

  /** Command to toggle the limit switch - 35 */
  TOGGLE_LIMIT_RANGE,

  /** Command to read the limit switches - 36 */
  READ_LIMIT,

  /** Set a position in hall pulses - 37 */
  SET_POSITION,

  /** Set current alarm value - 38 */
  SET_CURRENT_ALARM,

  /** Read state - 39 */
  READ_STATE,

  /** Save value to stored config slot 1 - 40 */
  SAVE_CONFIG_1,

  /** Save value to stored config slot 2 - 41 */
  SAVE_CONFIG_2,

  /** Save value to stored config slot 3 - 42 */
  SAVE_CONFIG_3,

  /** Save value to stored config slot 4 - 43 */
  SAVE_CONFIG_4,

  /** Save value to stored config slot 5 - 44 */
  SAVE_CONFIG_5,

  /** Get value from stored config slot 1 - 45 */
  RESTORE_CONFIG_1,

  /** Get value from stored config slot 2 - 46 */
  RESTORE_CONFIG_2,

  /** Get value from stored config slot 3 - 47 */
  RESTORE_CONFIG_3,

  /** Get value from stored config slot 4 - 48 */
  RESTORE_CONFIG_4,

  /** Get value from stored config slot 5 - 49 */
  RESTORE_CONFIG_5,

  /** Set the speed - 50 */
  SET_SPEED,
};

#endif // _COMMANDS_HPP_

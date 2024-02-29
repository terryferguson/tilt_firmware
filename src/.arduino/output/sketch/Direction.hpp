#line 1 "/home/terry/Projects/motor_control_firmware/Direction.hpp"
/*! \file Direction.hpp */

#ifndef _DIRECTION_HPP_
#define _DIRECTION_HPP_

/** @enum Direction
 *
 * @class Direction
 *
 * @brief  Direction values for the direction indicator
 *  for the motor controller and the motors themselves.
 *
 * @author Terry Paul Ferguson
 * @author terry@terryferguson.us
 *
 */
enum class Direction {
  /** Motor is turning for extensions */
  EXTEND = 0,

  /** Motor is stopped */
  STOP,

  /** Motor is turning for retraction */
  RETRACT
};

/// @brief String representations of the directions
const char *directions[3] = {"EXTEND", "STOP", "RETRACT"};

#endif // _DIRECTION_HPP_

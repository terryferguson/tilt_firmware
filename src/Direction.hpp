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

#endif // _DIRECTION_HPP_

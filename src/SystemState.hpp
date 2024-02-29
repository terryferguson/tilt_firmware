#ifndef _SYSTEM_STATE_HPP_
#define _SYSTEM_STATE_HPP_

#include "MotorController.hpp"
#include "defs.hpp"
#include <Preferences.h>

struct SystemState {
  /** @brief Whether PID is on or off */
  bool pid_on = true;

  /** @brief Whether limit range is on or off */
  bool limit_range = true;

  /// @brief Indicates whether debug messages should be sent to serial
  bool debugEnabled = true;

  /** @brief Indicates whether the current alarm is set */
  bool currentAlarmSet = false;

  /** @brief Whether the current alarm has been triggered */
  bool alarmTriggered = false;

  int systemSpeed = DEFAULT_MOTOR_SPEED;

  Preferences systemState;
  MotorController *controller = nullptr;

  int leaderMotorPosition = 0;
  int followerMotorPosition = 0;

  void Initialize(MotorController *controller);

  void Serialize();
  void Deserialize();

  void SaveMotorPostions();
  void LoadMotorPositions();

  void DisableAlarm();
  void EnableAlarm();
};

#endif // _SYSTEM_STATE_HPP_

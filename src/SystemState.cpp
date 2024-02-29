#include "SystemState.hpp"

void SystemState::Initialize(MotorController *controller) {
  this->controller = controller;
}

void SystemState::Serialize() {
  systemState.begin("system", false);
  systemState.putBool("pid_on", pid_on);
  systemState.putBool("limit_range", limit_range);
  systemState.putBool("debugEnabled", debugEnabled);
  systemState.putBool("currentAlarmSet", currentAlarmSet);
  systemState.putBool("alarmTriggered", alarmTriggered);
  systemState.putInt("systemSpeed", systemSpeed);
  systemState.putInt("leader", leaderMotorPosition);
  systemState.putInt("follower", followerMotorPosition);

  if (debugEnabled) {
    Serial.println("Serializing state");
    Serial.println("**************************************************");
    Serial.print("PID on: ");
    Serial.println(pid_on);
    Serial.print("Limit range: ");
    Serial.println(limit_range);
    Serial.print("Debug enabled: ");
    Serial.println(debugEnabled);
    Serial.print("Current alarm set: ");
    Serial.println(currentAlarmSet);
    Serial.print("Alarm triggered: ");
    Serial.println(alarmTriggered);
    Serial.print("System speed: ");
    Serial.println(systemSpeed);
    Serial.print("Leader motor position: ");
    Serial.println(leaderMotorPosition);
    Serial.print("Follower motor position: ");
    Serial.println(followerMotorPosition);
    Serial.println("**************************************************");
  }
  systemState.end();
}

void SystemState::Deserialize() {
  systemState.begin("system", false);
  pid_on = systemState.getBool("pid_on", true);
  limit_range = systemState.getBool("limit_range", true);
  debugEnabled = systemState.getBool("debugEnabled", true);
  currentAlarmSet = systemState.getBool("currentAlarmSet", false);
  alarmTriggered = systemState.getBool("alarmTriggered", false);
  leaderMotorPosition = systemState.getInt("leader", 0);
  followerMotorPosition = systemState.getInt("follower", 0);
  systemSpeed = systemState.getInt("systemSpeed", DEFAULT_MOTOR_SPEED);
  if (nullptr != controller) {
    controller->motors[MotorController::LEADER].setPos(leaderMotorPosition);
    controller->motors[MotorController::FOLLOWER].setPos(followerMotorPosition);
  }

  if (debugEnabled) {
    Serial.println("Deserialized state");
    Serial.println("**************************************************");
    Serial.print("PID on: ");
    Serial.println(pid_on);
    Serial.print("Limit range: ");
    Serial.println(limit_range);
    Serial.print("Debug enabled: ");
    Serial.println(debugEnabled);
    Serial.print("Current alarm set: ");
    Serial.println(currentAlarmSet);
    Serial.print("Alarm triggered: ");
    Serial.println(alarmTriggered);
    Serial.print("System speed: ");
    Serial.println(systemSpeed);
    Serial.print("Leader motor position: ");
    Serial.println(leaderMotorPosition);
    Serial.print("Follower motor position: ");
    Serial.println(followerMotorPosition);
    Serial.println("**************************************************");
  }
  systemState.end();
}

void SystemState::SaveMotorPostions() {
  if (nullptr != controller) {
    auto motors = controller->motors;
    leaderMotorPosition = motors[MotorController::LEADER].pos;
    followerMotorPosition = motors[MotorController::FOLLOWER].pos;

    Serialize();
  }
}

void SystemState::LoadMotorPositions() {
  systemState.begin("system", false);
  leaderMotorPosition = systemState.getInt("leader", 0);
  followerMotorPosition = systemState.getInt("follower", 0);

  if (nullptr != controller) {
    controller->motors[MotorController::LEADER].setPos(leaderMotorPosition);
    controller->motors[MotorController::FOLLOWER].setPos(followerMotorPosition);
  }
  systemState.end();
}

void SystemState::DisableAlarm() {
  alarmTriggered = false;
  Serialize();
}

void SystemState::EnableAlarm() {
  alarmTriggered = true;
  Serialize();
}

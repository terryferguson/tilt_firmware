#include <Arduino.h>
// #include <AsyncTCP.h>
// #include <ESP32Encoder.h>
// #include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ctime>
// #include <random>
#include <stdint.h>

#include "Commands.hpp"
#include "Motor.hpp"
#include "MotorController.hpp"
#include "PinMacros.hpp"
#include "StateController.hpp"
#include "SystemState.hpp"
#include "defs.hpp"

typedef void (*CommandFunc)();

void display_motor_info(void);
void display_network_info(void);

unsigned long lastTimestamp = 0UL;
unsigned long lastPrintTimeStamp = 0UL;
constexpr unsigned long MIN_PRINT_TIME_DELTA = 200000UL;
const char *directions[3] = {"EXTEND", "STOP", "RETRACT"};
/// @brief String representations of the motor roles at instantiation
const char *motor_roles[2] = {"LEADER", "FOLLOWER"};

/** @brief String representations of the names of position slots */
const char *save_position_slot_names[NUM_POSITION_SLOTS] = {
    "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
};
const char *save_configuration_slot_names[NUM_POSITION_SLOTS] = {
    "tilt-config-1", "tilt-config-2", "tilt-config-3",
    "tilt-config-4", "tilt-config-5",
};
int currentPWMChannel = 0;

SystemState systemState;

/**
 * @brief Storage for position in hall sensor pusles relative to initial
 * position when powered on
 */
int savedPositions[NUM_POSITION_SLOTS] = {0, 0, 0, 0, 0};

/**
 * @brief Storage for position in hall sensor pusles relative to initial
 * position when powered on
 */
int savedConfigurations[NUM_POSITION_SLOTS] = {0, 0, 0, 0, 0};

/** @brief Whether PID is on or off */
bool pid_on = true;

/** @brief Whether limit range is on or off */
bool limit_range = true;

/// @brief Indicates whether debug messages should be sent to serial
bool debugEnabled = true;

/**
 * @brief Creates a MotorController object with the specified PWM frequency, PWM
 * resolution bits, and default motor speed.
 *
 * @param frequency The PWM frequency for the motor controller.
 * @param resolutionBits The PWM resolution bits for the motor controller.
 * @param defaultSpeed The default motor speed for the motor controller.
 *
 */
MotorController motor_controller(PWM_FREQUENCY, PWM_RESOLUTION_BITS,
                                 DEFAULT_MOTOR_SPEED);

StateController state_controller;

#define RESTORE_POSITION(slot)                                                 \
  motor_controller.setPos(savedPositions[slot - 1]);                           \
  const Direction directionToMove = motor_controller.getRestoreDirection();    \
  if (directionToMove != Direction::STOP) {                                    \
    state_controller.OnStarting(directionToMove);                              \
    Serial.print("Restored position from slot: ");                             \
    Serial.println(slot);                                                      \
  }

#define RESTORE_CONFIG_POSITION(slot)                                          \
  motor_controller.setPos(savedConfigurations[slot - 1]);                      \
  const Direction directionToMove = motor_controller.getRestoreDirection();    \
  if (directionToMove != Direction::STOP) {                                    \
    state_controller.OnStarting(directionToMove);                              \
    Serial.print("Restored configuration from slot: ");                        \
    Serial.println(slot);                                                      \
  }

#define SAVE_POSITION(slot)                                                    \
  motor_controller.savePosition(slot);                                         \
                                                                               \
  Serial.print("Saved position to slot: ");                                    \
  Serial.println(slot);

#define SAVE_CONFIG_POSITION(slot)                                             \
  motor_controller.saveConfiguration(slot);                                    \
                                                                               \
  Serial.print("Saved configuration to slot: ");                               \
  Serial.println(slot);

/**
 * @brief Extends the motors.
 *
 * @throws None
 */
void extend() { state_controller.OnStarting(Direction::EXTEND); }

/**
 * @brief Retracts the motors.
 *
 * @throws None
 */
void retract() { state_controller.OnStarting(Direction::RETRACT); }

/**
 * @brief Stops the motors.
 *
 * @throws None
 */
void stop() { state_controller.OnStopping(); }

/**
 * @brief Displays the motor information.
 *
 * @throws None
 */
void report() { motor_controller.report(); }

void saveTilt1() { SAVE_POSITION(1); }
void saveTilt2() { SAVE_POSITION(2); }
void saveTilt3() { SAVE_POSITION(3); }
void saveTilt4() { SAVE_POSITION(4); }
void saveTilt5() { SAVE_POSITION(5); }

void getTilt1() { RESTORE_POSITION(1); }
void getTilt2() { RESTORE_POSITION(2); }
void getTilt3() { RESTORE_POSITION(3); }
void getTilt4() { RESTORE_POSITION(4); }
void getTilt5() { RESTORE_POSITION(5); }

void zero() { motor_controller.zero(); }
void systemReset() {
  systemState.SaveMotorPostions();
  systemState.Serialize();
  ESP.restart();
}
void togglePid() { pid_on = !pid_on; }
void home() { state_controller.OnHome(); }
void toggleLimitRange() { limit_range = !limit_range; }
void readLimit() {}

void saveTiltConfig1() { SAVE_CONFIG_POSITION(1); }
void saveTiltConfig2() { SAVE_CONFIG_POSITION(2); }
void saveTiltConfig3() { SAVE_CONFIG_POSITION(3); }
void saveTiltConfig4() { SAVE_CONFIG_POSITION(4); }
void saveTiltConfig5() { SAVE_CONFIG_POSITION(5); }

void getTiltConfig1() { RESTORE_CONFIG_POSITION(1); }
void getTiltConfig2() { RESTORE_CONFIG_POSITION(2); }
void getTiltConfig3() { RESTORE_CONFIG_POSITION(3); }
void getTiltConfig4() { RESTORE_CONFIG_POSITION(4); }
void getTiltConfig5() { RESTORE_CONFIG_POSITION(5); }

void setSpeed() {
  if (Serial.available() > 0) {
    const auto newSpeed = Serial.parseInt();
    systemState.systemSpeed = newSpeed;
    motor_controller.setSpeed(newSpeed);
  }
}

/**
 * Sets the current alarm value based on the input received from the Serial
 * port.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */

void setCurrentAlarm() {
  if (Serial.available() > 0) {
    int newCurrent = Serial.parseInt();
    motor_controller.minCurrent = newCurrent;
  }
}

/**
 * @brief Sets the position of the motor controller based on input from the
 * Serial port.
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void setPosition() {
  if (Serial.available() > 0) {
    const auto new_position = Serial.parseInt();
    const auto pos = motor_controller.getPos();

    motor_controller.setPos(new_position);

    if (pos > new_position) {
      state_controller.OnStarting(Direction::RETRACT);
    } else if (pos < new_position) {
      state_controller.OnStarting(Direction::EXTEND);
    }
  }
}

/**
 * Reports the current state of the motor controller.
 *
 * Fields in order are:
 *   1. Current Position of leader motor
 *   2. Max pulses of leader motor
 *   3. Desired Position
 *   4. Speed
 *   5. Whether current alarm is set
 *   6. Alarm Triggered
 *
 * @param None
 *
 * @return None
 *
 * @throws None
 */
void readState() {
  char buf[256];

  /*
  Reports the current state of the motor controller
  Fields in order are:
    1. Current Position of leader motor
    2. Max pulses of leader motor
    3. Desired Position
    4. Speed
    5. Whether current alarm is set
    6. Alarm Triggered
  */
  snprintf(buf, 255, "%d,%d,%d,%d,%d,%d\n", motor_controller.getPos(),
           LEADER_MAX_PULSES, motor_controller.getDesiredPosition(),
           motor_controller.speed, motor_controller.currentAlarmSet,
           motor_controller.alarmTriggered);
  Serial.println(buf);
}

CommandFunc commands[] = {
    retract,
    extend,
    report,
    stop,
    saveTilt1,
    saveTilt2,
    saveTilt3,
    saveTilt4,
    saveTilt5,
    getTilt1,
    getTilt2,
    getTilt3,
    getTilt4,
    getTilt5,
    zero,
    systemReset,
    togglePid,
    home,
    toggleLimitRange,
    readLimit,
    setPosition,
    setCurrentAlarm,
    readState,
    saveTiltConfig1,
    saveTiltConfig2,
    saveTiltConfig3,
    saveTiltConfig4,
    saveTiltConfig5,
    getTiltConfig1,
    getTiltConfig2,
    getTiltConfig3,
    getTiltConfig4,
    getTiltConfig5,
    setSpeed,
};

/**
 * @brief Initializes the setup of the the firmware.
 *
 * @return void
 *
 * @throws None
 */
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(3);

  motor_controller.initialize();

  // Initialize SPIFFS
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  state_controller.setController(&motor_controller);
  systemState.Initialize(&motor_controller);
  systemState.Deserialize();

  lastTimestamp = micros();
}

/**
 * @brief Loop function that handles serial commands, updates the state machine,
 * and displays motor information.
 *
 * @return void
 */
void loop() {
  if (Serial.available() > 0) {
    const auto serialInput = Serial.parseInt();

    if (systemState.debugEnabled) {
      Serial.printf("Serial input received: %d\n", serialInput);
    }

    const auto commandInput = serialInput - 17;

    if (systemState.debugEnabled) {
      Serial.printf("Command received: %d\n", commandInput);
    }

    if (commandInput >= 0 && commandInput < countof(commands)) {
      commands[commandInput]();
    }
  }
  const unsigned long timestamp = micros();
  motor_controller.deltaT = ((timestamp - lastTimestamp) / 1.0e6);
  const unsigned long printDeltaTime = timestamp - lastPrintTimeStamp;

  state_controller.update();
  if (printDeltaTime > MIN_PRINT_TIME_DELTA) {
    display_motor_info();
    lastPrintTimeStamp = timestamp;
  }

  lastTimestamp = micros();
}
/**
 * @brief Displays the motor information if debug mode is enabled.
 *
 * @throws None
 */
void display_motor_info(void) {
  char buf[256];
  snprintf(buf, 255, "`%d,%d,%d,%d,%d,%d~", motor_controller.getPos(),
           LEADER_MAX_PULSES, motor_controller.getDesiredPosition(),
           motor_controller.speed, motor_controller.currentAlarmSet,
           systemState.alarmTriggered);
  Serial.println(buf);
}

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
#include "defs.hpp"

typedef void (*CommandFunc)();

void display_motor_info(void);
void display_network_info(void);

long lastTimestamp = 0L;
long lastPrintTimeStamp = 0L;
constexpr long MIN_PRINT_TIME_DELTA = 1000000L;
const char *directions[3] = {"EXTEND", "STOP", "RETRACT"};
/// @brief String representations of the motor roles at instantiation
const char *motor_roles[2] = {"LEADER", "FOLLOWER"};

/** @brief String representations of the names of position slots */
const char *save_position_slot_names[NUM_POSITION_SLOTS] = {
    "tilt-1", "tilt-2", "tilt-3", "tilt-4", "tilt-5",
};
int currentPWMChannel = 0;

/**
 * @brief Storage for position in hall sensor pusles relative to initial
 * position when powered on
 */
int savedPositions[NUM_POSITION_SLOTS] = {0, 0, 0, 0, 0};

/** @brief Whether PID is on or off */
bool pid_on = true;

/** @brief Whether limit range is on or off */
bool limit_range = true;

/// @brief Indicates whether debug messages should be sent to serial
bool debugEnabled = false;

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

void saveTilt1() { SERIAL_SAVE_POSITION(1); }
void saveTilt2() { SERIAL_SAVE_POSITION(2); }
void saveTilt3() { SERIAL_SAVE_POSITION(3); }
void saveTilt4() { SERIAL_SAVE_POSITION(4); }
void saveTilt5() { SERIAL_SAVE_POSITION(5); }

void getTilt1() { RESTORE_POSITION(1); }
void getTilt2() { RESTORE_POSITION(2); }
void getTilt3() { RESTORE_POSITION(3); }
void getTilt4() { RESTORE_POSITION(4); }
void getTilt5() { RESTORE_POSITION(5); }

void zero() { motor_controller.zero(); }
void systemReset() { ESP.restart(); }
void togglePid() { pid_on = !pid_on; }
void home() { state_controller.OnHome(); }
void toggleLimitRange() { limit_range = !limit_range; }
void readLimit() {}
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
    int new_position = Serial.parseInt();
    int pos = motor_controller.getPos();

    motor_controller.setPos(new_position);

    if (pos > new_position) {
      state_controller.OnStarting(Direction::RETRACT);
    } else if (pos < new_position) {
      state_controller.OnStarting(Direction::EXTEND);
    }
  }
}

CommandFunc commands[] = {retract,
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
                          setCurrentAlarm};

/**
 * @brief Initializes the setup of the the firmware.
 *
 * @return void
 *
 * @throws None
 */
void setup() {
  Serial.begin(115200);

  motor_controller.initialize();

  // Initialize SPIFFS
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  state_controller.setController(&motor_controller);

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
    const int commandInput = Serial.parseInt() - 17;

    if (commandInput >= 0 && commandInput < countof(commands)) {
      commands[commandInput]();
    }
  }
  const long timestamp = micros();
  motor_controller.deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
  const int printDeltaTime = timestamp - lastPrintTimeStamp;

  state_controller.update();
  if (printDeltaTime > MIN_PRINT_TIME_DELTA &&
      !motor_controller.motorsStopped()) {
    display_motor_info();
    lastPrintTimeStamp = timestamp;
  }

  lastTimestamp = timestamp;
}
/**
 * @brief Displays the motor information if debug mode is enabled.
 *
 * @throws None
 */
void display_motor_info(void) { motor_controller.report(); }

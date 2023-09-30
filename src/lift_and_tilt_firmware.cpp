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
#include "RouteMacros.hpp"
#include "defs.hpp"

typedef void (*CommandFunc)();

void display_motor_info(void);
void display_network_info(void);

long lastTimestamp = 0L;
long lastPrintTimeStamp = 0L;
const long minPrintTimeDelta = 200000L;

#define MICROS_IN_SECONDS (1 * 1000 * 1000)

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

/**
 * @brief Extends the motors.
 *
 * @throws None
 */
void extend() { motor_controller.extend(); }

/**
 * @brief Retracts the motors.
 *
 * @throws None
 */
void retract() { motor_controller.retract(); }

/**
 * @brief Stops the motors.
 *
 * @throws None
 */
void stop() { motor_controller.stop(); }

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
void home() { motor_controller.home(); }
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
    motor_controller.setPos(new_position);
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

  lastTimestamp = micros();
}

/**
 * @brief Loop function that handles serial commands, updates the motor
 * controller, and displays motor information.
 *
 * @return void
 */
void loop() {
  int fBottom, lBottom, fTop, lTop = 0;

  if (Serial.available() > 0) {
    const int commandInput = Serial.parseInt() - 17;

    Serial.printf("Command input: %d\n", commandInput);

    if (commandInput >= 0 && commandInput < countof(commands)) {
      commands[commandInput]();
    }
  }
  const long timestamp = micros();
  const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
  const int printDeltaTime = timestamp - lastPrintTimeStamp;

  if (!motor_controller.motorsStopped() &&
      (printDeltaTime > minPrintTimeDelta)) {
    display_motor_info();
  }

  motor_controller.update(deltaT);

  lastTimestamp = timestamp;
}
/**
 * @brief Displays the motor information if debug mode is enabled.
 *
 * @throws None
 */
void display_motor_info(void) {
  if (debugEnabled) {
    motor_controller.report();
  }
}

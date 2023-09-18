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
#include "CurrentSettings.hpp"
#include "Motor.hpp"
#include "MotorController.hpp"
#include "PinMacros.hpp"
#include "RouteMacros.hpp"
#include "defs.hpp"

void display_motor_info(void);
void display_network_info(void);

long lastTimestamp = 0L;
long lastPrintTimeStamp = 0L;
const long minPrintTimeDelta = 500000L;

#define MICROS_IN_SECONDS (1 * 1000 * 1000)

MotorController motor_controller(PWM_FREQUENCY, PWM_RESOLUTION_BITS,
                                 DEFAULT_MOTOR_SPEED);

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

void loop() {
  int fBottom, lBottom, fTop, lTop = 0;

  if (Serial.available() > 0) {
    Command cmd = static_cast<Command>(Serial.parseInt());

    switch (cmd) {
    case Command::RETRACT:
      motor_controller.retract();
      break;
    case Command::EXTEND:
      motor_controller.extend();
      break;
    case Command::STOP:
      motor_controller.stop();
      break;
    case Command::REPORT:
      motor_controller.report();
      break;
    case Command::SAVE_TILT_1:
      // Read parameter
      SERIAL_SAVE_POSITION(1)
      break;
    case Command::SAVE_TILT_2:
      // Read parameter
      SERIAL_SAVE_POSITION(2)
      break;
    case Command::SAVE_TILT_3:
      // Read parameter
      SERIAL_SAVE_POSITION(3)
      break;
    case Command::SAVE_TILT_4:
      // Read parameter
      SERIAL_SAVE_POSITION(4)
      break;
    case Command::SAVE_TILT_5:
      // Read parameter
      SERIAL_SAVE_POSITION(5)
      break;
    case Command::GET_TILT_1:
      RESTORE_POSITION(1)
      break;
    case Command::GET_TILT_2:
      RESTORE_POSITION(1)
      break;
    case Command::GET_TILT_3:
      RESTORE_POSITION(2)
      break;
    case Command::GET_TILT_4:
      RESTORE_POSITION(3)
      break;
    case Command::GET_TILT_5:
      RESTORE_POSITION(4)
      break;
    case Command::ZERO:
      motor_controller.zero();
      break;
    case Command::SYSTEM_RESET:
      ESP.restart();
      break;
    case Command::TOGGLE_PID:
      pid_on = !pid_on;
      break;
    case Command::HOME:
      motor_controller.home();
      break;
    case Command::TOGGLE_LIMIT_RANGE:
      limit_range = !limit_range;
      break;
    case Command::READ_LIMIT:
      lBottom = digitalRead(MOTOR1_LIMIT);
      fBottom = digitalRead(MOTOR2_LIMIT);
      lTop = digitalRead(MOTOR1_TLIMIT);
      fTop = digitalRead(MOTOR2_TLIMIT);
      Serial.printf("Bottom 1: %d, Bottom 2: %d, Top 1: %d, Top 2: %d\n",
                    lBottom, fBottom, lTop, fTop);
      break;
    default:
      break;
    }
  }

  // ws.cleanupClients();

  const long timestamp = micros();
  const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
  const int printDeltaTime = timestamp - lastPrintTimeStamp;

  if (!motor_controller.motorsStopped() &&
      (printDeltaTime > minPrintTimeDelta)) {
    display_motor_info();
  }

  motor_controller.update(deltaT);
}

void display_motor_info(void) {
  if (debugEnabled) {
    motor_controller.report();
  }
}

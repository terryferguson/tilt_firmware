#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESP32Encoder.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <ctime>
#include <random>
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

#define VAL_IN_RANGE(var, min, max) (var >= min && var <= max)

const char *PARAM_INPUT_1 = "pos";
const char *VAL_PARAM = "val";
// const char *ssid = "PLDTHOMEFIBRCDge4";
// const char *password = "PLDTWIFI6Jg4G";

const char *ssid = "MySpectrumWiFi30-5G";
const char *password = "superocean537";

long lastTimestamp = 0L;
long lastPrintTimeStamp = 0L;
const long minPrintTimeDelta = 200000L;

#define MICROS_IN_SECONDS (1 * 1000 * 1000)

MotorController motor_controller(PWM_FREQUENCY, PWM_RESOLUTION_BITS,
                                 DEFAULT_MOTOR_SPEED);

void display_network_info(void);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a websocket server for updates
AsyncWebSocket ws("/ws");

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

String makeJson() {
  String speed(motor_controller.speed);
    String ki(motor_controller.K_i);
    String kp(motor_controller.K_p);
    String leader_pos(motor_controller.motors[0].pos);
    String follower_pos(motor_controller.motors[1].pos);
    String leader_current(motor_controller.leaderCurrent);
    String follower_current(motor_controller.followerCurrent);
    String min_current(motor_controller.minCurrent);
    String alarm_current_velocity(motor_controller.alarmCurrentVelocity);
    String limit_range(limit_range ? "true" : "false");
    String pid_on(pid_on ? "true" : "false");
    String direction(directions[static_cast<int>(motor_controller.systemDirection)]);
    String leader_current_velocity(motor_controller.leaderCurrentVelocity);
    String follower_current_velocity(motor_controller.followerCurrentVelocity);

    String response = "{\"type\":\"stats\",\"leader_current\":" + leader_current +
                      ",\"follower_current\":" + follower_current +
                      ",\"speed\":" + speed +
                      ",\"ki\":" + ki +
                      ",\"limit_range\":\"" + limit_range + "\"" +
                      ",\"pid_on\":\"" + pid_on + "\"" +
                      ",\"system_direction\":\"" + direction + "\"" +
                      ",\"kp\":" + kp +
                      ",\"min_current\":" + min_current +
                      ",\"alarm_current_velocity\":" + alarm_current_velocity +
                       ",\"leader_current_velocity\":" + leader_current_velocity +
                      ",\"follower_current_velocity\":" + follower_current_velocity +
                      ",\"leader_pos\": " + leader_pos +
                      ",\"follower_pos\": " + follower_pos + "}";
    return response;  
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {}

/**
 * Handles events from an asynchronous WebSocket.
 *
 * @param server The AsyncWebSocket server instance.
 * @param client The AsyncWebSocketClient instance representing the client.
 * @param type The type of event.
 * @param arg A pointer to additional event-specific data.
 * @param data A pointer to the received data.
 * @param len The length of the received data.
 */
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(),
                  client->remoteIP().toString().c_str());
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA)
  {
    handleWebSocketMessage(arg, data, len);
  }
}

void setup()
{
  Serial.begin(115200);

  motor_controller.initialize();

  Serial.println("Attempting to connect to wifi.");

  // Connect to wifi
  WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println("");

  // put your setup code here, to run once:
  Serial.println("Online");
  display_network_info();

  // Initialize SPIFFS
  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  srand(std::time(nullptr));

  // Route for root / web page
  server.on("/", HTTP_GET, DEF_HANDLER(STATIC_FILE("/index.html", String())));

  server.on("/style.css", HTTP_GET,
            DEF_HANDLER(STATIC_FILE("/style.css", "text/css")));

  server.on("style.css", HTTP_GET,
            DEF_HANDLER(STATIC_FILE("/style.css", "text/css")));

  server.on("/extend", HTTP_GET,
            DEF_HANDLER(MOTOR_COMMAND(extend, "Extending")));
  server.on("/retract", HTTP_GET,
            DEF_HANDLER(MOTOR_COMMAND(retract, "Retracting")));
  server.on("/reset", HTTP_GET, DEF_HANDLER(ESP.restart();));
  server.on("/toggle-limit-range", HTTP_GET, DEF_HANDLER(limit_range = !limit_range;));
  server.on("/stop", HTTP_GET, DEF_HANDLER(MOTOR_COMMAND(stop, "Stopping")));

  server.on("/get-tilt/1", HTTP_GET,
            DEF_HANDLER(LOAD_SAVED_POSITION(0, "Loading saved position 1")));
  server.on("/get-tilt/2", HTTP_GET,
            DEF_HANDLER(LOAD_SAVED_POSITION(1, "Loading saved position 2")));
  server.on("/get-tilt/3", HTTP_GET,
            DEF_HANDLER(LOAD_SAVED_POSITION(2, "Loading saved position 3")));
  server.on("/get-tilt/4", HTTP_GET,
            DEF_HANDLER(LOAD_SAVED_POSITION(3, "Loading saved position 4")));
  server.on("/get-tilt/5", HTTP_GET,
            DEF_HANDLER(LOAD_SAVED_POSITION(4, "Loading saved position 5")));

  server.on("/set-tilt/1", HTTP_GET, DEF_HANDLER(SET_POS_HANDLER(1)));
  server.on("/set-tilt/2", HTTP_GET, DEF_HANDLER(SET_POS_HANDLER(2)));
  server.on("/set-tilt/3", HTTP_GET, DEF_HANDLER(SET_POS_HANDLER(3)));
  server.on("/set-tilt/4", HTTP_GET, DEF_HANDLER(SET_POS_HANDLER(4)));
  server.on("/set-tilt/5", HTTP_GET, DEF_HANDLER(SET_POS_HANDLER(5)));

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String inputMessage1;

    // GET input1 value on
    // <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      const int new_pos = inputMessage1.toInt();
      motor_controller.setPos(new_pos);
    } else {
      inputMessage1 = "No message sent";
    }

    request->send(200, "text/plain", inputMessage1); });

  server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String speedText;

    // GET input1 value on
    // <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(VAL_PARAM)) {
      speedText = request->getParam(VAL_PARAM)->value();
      const int newSpeed = speedText.toInt();
      if VAL_IN_RANGE (newSpeed, 0, 255) {
        motor_controller.defaultSpeed = newSpeed;
        motor_controller.speed = newSpeed;

        Serial.printf("Motor controller speed: %d\n", motor_controller.speed);
      }
    } else {
      speedText = "No message sent";
    }

    request->send(200, "text/plain", speedText); });

  server.on("/kp", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String kpInput;

    kpInput = request->getParam(VAL_PARAM)->value();
    const int newKp = kpInput.toInt(); // Assuming kpInput is a string
    if (VAL_IN_RANGE(newKp, 0, 400000)) {
      motor_controller.K_p = newKp;
      Serial.printf("Motor controller Kp: %d\n", motor_controller.K_p);
    } else {
      kpInput = "No message sent";
    }

    request->send(200, "text/plain", kpInput); });

  server.on("/ki", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String kiInput;

    // GET input1 value on
    // <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(VAL_PARAM)) {
      kiInput = request->getParam(VAL_PARAM)->value();
      const float newKi = kiInput.toFloat();
      if (VAL_IN_RANGE(newKi, 0, 10.0)) {
        motor_controller.K_i = newKi;
        Serial.printf("Motor controller Ki: %f\n", motor_controller.K_i);
      }
    } else {
      kiInput = "No message sent";
    }

    request->send(200, "text/plain", kiInput); });

  server.on("/get-stats", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String response = makeJson();

    request->send(200, "application/json", response.c_str()); });

  server.on("/min-current", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String minCurrentInput;

    minCurrentInput = request->getParam(VAL_PARAM)->value();
    const int newMinCurrent = minCurrentInput.toInt();
    if (VAL_IN_RANGE(newMinCurrent, 1, 4000)) {
      motor_controller.minCurrent = newMinCurrent;
      Serial.printf("New min current: %d\n", motor_controller.minCurrent );
    } else {
      minCurrentInput = "No message sent";
    }

    request->send(200, "text/plain", minCurrentInput); });

  server.on("/alarm-current-velocity", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String alarmVelocity;

    alarmVelocity = request->getParam(VAL_PARAM)->value();
    const int newAlarmCurrentVelocity = alarmVelocity.toInt();
    if (VAL_IN_RANGE(newAlarmCurrentVelocity, 50, 2500)) {
      motor_controller.alarmCurrentVelocity = newAlarmCurrentVelocity;
      Serial.printf("Alarm current velocity in mA/ms: %d\n", motor_controller.alarmCurrentVelocity);
    } else {
      alarmVelocity = "No message sent";
    }

    request->send(200, "text/plain", alarmVelocity); });

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  // motor_controller.home();

  lastTimestamp = micros();
  server.begin();
}

void loop()
{
  int fBottom, lBottom = 0;

  if (Serial.available() > 0)
  {
    Command cmd = static_cast<Command>(Serial.parseInt());

    switch (cmd)
    {
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
      // motor_controller.home();
      break;
    case Command::TOGGLE_LIMIT_RANGE:
      limit_range = !limit_range;
      break;
    case Command::READ_LIMIT:
      lBottom = digitalRead(MOTOR1_LIMIT);
      fBottom = digitalRead(MOTOR2_LIMIT);
      Serial.printf("Bottom 1: %d, Bottom 2: %d\n", lBottom, fBottom);
      break;
    default:
      break;
    }
  }

  // ws.cleanupClients();

  const long timestamp = micros();
  const float deltaT = ((float)(timestamp - lastTimestamp) / 1.0e6);
  const int printDeltaTime = timestamp - lastPrintTimeStamp;

  // SET_TO_ANALOG_PIN(KP_POT_PIN, motor_controller.K_p, 0, 100000);
  // FSET_TO_ANALOG_PIN(KP_POT_PIN, motor_controller.K_i, 0.0f, 2.0f);
  // SET_TO_ANALOG_PIN(CURRENT_TOLERANCE_PIN,
  // motor_controller.currentIncreaseTolerance, 0, CURRENT_INCREASE_LIMIT_MAX);

  if (!motor_controller.isStopped() && (printDeltaTime > minPrintTimeDelta))
  {
    display_motor_info();
    lastPrintTimeStamp = timestamp;

    const String response = makeJson();

    ws.textAll(response);
  }

  motor_controller.update(deltaT);
}

void display_motor_info(void)
{
  if (debugEnabled)
  {
    motor_controller.report();
  }
}

void display_network_info(void)
{
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("[*] Network information for ");
    Serial.println(ssid);
    Serial.print("[+] BSSID : ");
    Serial.println(WiFi.BSSIDstr());
    Serial.print("[+] Gateway IP : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[+] Subnet Mask : ");
    Serial.println(WiFi.subnetMask());
    Serial.print("[+] RSSI : ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dB");
    Serial.print("[+] ESP32 IP : ");
    Serial.println(WiFi.localIP());
  }
}

/*! \file RouteMacros.hpp */

#ifndef _ROUTE_MACROS_HPP__
#define _ROUTE_MACROS_HPP__

#define SET_TILT(n)                                                            \
  if (request->hasParam(PARAM_INPUT_1)) {                                      \
    inputMessage1 = request->getParam(PARAM_INPUT_1)->value();                 \
    const int new_pos = inputMessage1.toInt();                                       \
    motor_controller.savePosition(n, new_pos);                                 \
  } else {                                                                     \
    inputMessage1 = "Error: No position sent.";                                \
  }

#define DEF_HANDLER(func) [](AsyncWebServerRequest *request) { func }

#define LOAD_SAVED_POSITION(position, response_text)                           \
  motor_controller.setPos(savedPositions[position]);                           \
  request->send(200, "text/plain", response_text);

#define MOTOR_COMMAND(command, response_text)                                  \
  motor_controller.command();                                                  \
  request->send(200, "text/plain", response_text);

#define SET_POS_HANDLER(slot)                                                  \
  String inputMessage1;                                                        \
  SET_TILT(slot)                                                               \
  request->send(200, "text/plain", inputMessage1);

#define STATIC_FILE(filename, file_type)                                       \
  request->send(SPIFFS, filename, file_type);

#endif // _ROUTE_MACROS_HPP__

#ifndef _DEBUGGING_HPP_
#define _DEBUGGING_HPP_

#include <Arduino.h>

#define DebugPrint(message)                                                    \
  if (systemState.debugEnabled) {                                              \
    Serial.print(millis());                                                    \
    Serial.print(": ");                                                        \
    Serial.print(__PRETTY_FUNCTION__);                                         \
    Serial.print(' ');                                                         \
    Serial.print(__LINE__);                                                    \
    Serial.print(": ");                                                        \
    Serial.print(message);                                                     \
  }

#define DebugPrintln(message)                                                  \
  if (systemState.debugEnabled) {                                              \
    Serial.print(millis());                                                    \
    Serial.print(": ");                                                        \
    Serial.print(__PRETTY_FUNCTION__);                                         \
    Serial.print(' ');                                                         \
    Serial.print(__LINE__);                                                    \
    Serial.print(": ");                                                        \
    Serial.println(message);                                                   \
  }

#endif // _DEBUGGING_HPP_
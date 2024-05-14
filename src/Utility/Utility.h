#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>

class Utility {
public:
    template <typename T>
    static void serialOutput(T info);    // Outputs debug information to serial
    static void triggerPower(int pin, bool state);  // Triggers a digital pin HIGH or LOW
};

template <typename T>
void Utility::serialOutput(T info) {
  #ifdef DEBUG
  Serial.println(info);
  #endif
}

#endif

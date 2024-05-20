#include "Utility.h"

 
void Utility::triggerPower(int pin, bool state) {
    digitalWrite(pin, state ? HIGH : LOW);  // More concise and clear
}


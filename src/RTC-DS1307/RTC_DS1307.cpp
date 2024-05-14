#include "RTC_DS1307.h"
#include <Arduino.h>

RTC_DS1307_LogicHub::RTC_DS1307_LogicHub() {
    // Constructor
}

bool RTC_DS1307_LogicHub::begin() {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        return false;
    }
    return checkRunning();
}

bool RTC_DS1307_LogicHub::checkRunning() {
    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running, setting time to compiled time.");
        setTimeToCompiled();
        return false;
    }
    return true;
}

bool RTC_DS1307_LogicHub::setTime(DateTime dt) {
    rtc.adjust(dt);
    return true;
}

DateTime RTC_DS1307_LogicHub::now() {
    return rtc.now();
}

time_t RTC_DS1307_LogicHub::getEpochTime() {
    return now().unixtime();
}

bool RTC_DS1307_LogicHub::correctTime(DateTime correctTime) {
    if (now().unixtime() != correctTime.unixtime()) {
        Serial.println("Correcting RTC time...");
        rtc.adjust(correctTime);
        return true;
    }
    return false;
}

bool RTC_DS1307_LogicHub::syncTimeWithServer(DateTime serverTime) {
    if (now().unixtime() != serverTime.unixtime()) {
        Serial.println("Syncing time with server...");
        rtc.adjust(serverTime);
        return true;
    }
    return false;
}

void RTC_DS1307_LogicHub::setTimeToCompiled() {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void RTC_DS1307_LogicHub::printTime() {
    Serial.println(getFormattedTime());
}

String RTC_DS1307_LogicHub::getFormattedTime() {
    DateTime now = rtc.now();
    char buffer[20];  // Ensure the buffer is large enough for the formatted string
    sprintf(buffer, "%02d-%02d-%02dT%02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    return String(buffer);
}


void RTC_DS1307_LogicHub::printFutureTime(int days, int hours, int minutes, int seconds) {
    DateTime future = now() + TimeSpan(days, hours, minutes, seconds);
    Serial.print("Future time: ");
    // Again, use const_cast if safe to do so
    Serial.println(future.toString(const_cast<char*>("YY-MM-DDTHH:MM:SS")));
}

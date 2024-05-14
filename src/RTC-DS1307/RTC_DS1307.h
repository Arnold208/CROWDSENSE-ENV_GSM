#ifndef RTC_DS1307_H
#define RTC_DS1307_H

#include <RTClib.h>

class RTC_DS1307_LogicHub {
public:
    RTC_DS1307_LogicHub();
    bool begin(); // Returns true if RTC begins successfully
    bool checkRunning(); // Checks if RTC is running, sets time if not, returns status
    DateTime now();
    void printTime();
    void printFutureTime(int days, int hours, int minutes, int seconds);
    bool setTime(DateTime dt);
    time_t getEpochTime();
    String getFormattedTime();
    bool correctTime(DateTime correctTime);
    bool syncTimeWithServer(DateTime serverTime);
    void setTimeToCompiled(); // Sets the RTC time to the compiled time

private:
    RTC_DS1307 rtc;
    const char* daysOfTheWeek[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
};

#endif

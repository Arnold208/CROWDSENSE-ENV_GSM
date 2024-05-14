#include <Arduino.h>
#include <ArduinoHttpClient.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"
#include "FS.h"
#include "SD.h"
#include "Adafruit_LTR390.h"
#include "SDCardManager/SDCardManager.h"
#include "SettingsManager/SettingsManager.h"
#include "RTC-DS1307/RTC_DS1307.h"
#include "Utility/Utility.h"

#define PWR 2

bool rtcInit = false;

// Initialize the Sensors

RTC_DS1307_LogicHub rtc;

void setup()
{
  pinMode(PWR, OUTPUT);
  Utility::triggerPower(PWR, true);
  Serial.begin(9600);

  delay(5000);

  rtcInit = rtc.begin();
  if (!rtcInit)
  {
    Utility::serialOutput("RTC setup failed!");
  }
  else{
    Utility::serialOutput(rtc.getFormattedTime());
    Utility::serialOutput( rtc.getEpochTime());

  }
}

void loop()
{

  delay(2000);
}
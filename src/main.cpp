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


bool RtcInit = false;

// Initialize the Sensors

RTC_DS1307_LogicHub rtc;


void serialOutput(String info){
  #ifdef DEBUG
  Serial.println(info);
  #endif
}


void setup()
{
  Serial.begin(9600);
   if (!rtc.begin()) {
        serialOutput("RTC setup failed!");
    }
  else{
    RtcInit = !RtcInit;
    serialOutput("RTC is active");
  }
}

void loop()
{


delay(2000);

}
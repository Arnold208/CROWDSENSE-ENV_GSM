#define TINY_GSM_MODEM_SIM800

#include <Arduino.h>

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"
#include "RTClib.h"
#include "FS.h"
#include "SD.h"
#include "Adafruit_LTR390.h"
#include "SDCardManager/SDCardManager.h"
#include "SettingsManager/SettingsManager.h"
#include <LiquidCrystal_I2C.h>

#define PWR_CTRL 2
#define BAT 15
#define rxPin 14
#define txPin 4

const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char server[] = "cctelemetry-dev.azurewebsites.net";
const char resource[] = "/telemetry";
const int port = 80;

String jsonPayload;
const char *jsonPayloadChar;
LiquidCrystal_I2C lcd(0x27, 16, 2);
JsonDocument telemetry;
SoftwareSerial SerialDT(txPin, rxPin);

const int analogPin = 15;
const int solar = 13;
const float minVoltage = 3.122;
const float maxVoltage = 4.1;

void readPower_levels()
{
  int adcValue = analogRead(analogPin);
  int solarValue = analogRead(solar);
  float battery_out = adcValue * (3.122 / 4095.0);
  float batteryVoltage = battery_out * 2.0;
  float batteryPercentage = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage) * 100;
  float solar_out = solarValue * (3.1 / 4095.0);
  float solarVoltage = solar_out * 2.0;

  #ifdef DEBUG
  Serial.print("ADC Value: ");
  Serial.print(adcValue);
  Serial.print(", Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V, Battery Percentage: ");
  Serial.print(batteryPercentage, 2);
  Serial.print("%");
  Serial.print("Solar Voltage: ");
  Serial.print(solarVoltage, 2);
  Serial.println("V");
  #endif

  telemetry['s'] = solarVoltage;
  telemetry['b'] = batteryPercentage;
  
  delay(1000);
}

void setup()
{
  Serial.begin(9600);

  #ifdef DEBUG
  Serial.println("DEBUG MODE: System is starting up...");
  #endif
}

void loop()
{
}
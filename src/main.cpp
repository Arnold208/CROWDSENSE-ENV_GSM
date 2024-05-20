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
#include "PMS.h"


#define PWR 2
#define SD_CS_PIN 5

bool rtcInit = false;
bool bmeInit = false;
bool vemlInit = false;
bool ltrInit = false;
bool sdInit = false;

// Initialize the Sensors
RTC_DS1307_LogicHub rtc;
Adafruit_BME680 bme;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_LTR390 ltr = Adafruit_LTR390();
SDCardManager sdCardManager;
SettingsManager settingsManager(SD);

SoftwareSerial pmsSerial(16, 17);  // RX, TX pins; Choose pins based on your hardware

PMS pms(pmsSerial);
PMS::DATA data;


void pm_data(){
  if (pms.read(data))
  {
    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(data.PM_AE_UG_1_0);

    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(data.PM_AE_UG_2_5);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(data.PM_AE_UG_10_0);

    Serial.println();
  }
}
void setup()
{
  pinMode(PWR, OUTPUT);
  Utility::triggerPower(PWR, true);

  Serial.begin(9600);
  pmsSerial.begin(9600);    // Start the software serial for PMS
  
  // Initialize PMS library
  //pms.passiveMode();   // GPIO2 (D4 pin on ESP-12E Development Board)

  delay(5000);

  Utility::serialOutput("");
  Utility::serialOutput("Debugging and Testing Mode");

  bool sdInit = SD.begin(SD_CS_PIN);
  if (!sdInit ) {
        Utility::serialOutput("SD card initialization failed.");
    }
  else{
    Utility::serialOutput("SD Card Ready");
  }


  rtcInit = rtc.begin();
  if (!rtcInit)
  {
    Utility::serialOutput("RTC setup failed!");
  }
  else
  {
    rtc.setTimeToCompiled();
    Utility::serialOutput(rtc.getFormattedTime());
    Utility::serialOutput(rtc.getEpochTime());
  }

  bmeInit = bme.begin();
  if (!bmeInit)
  {
    Utility::serialOutput("Bme680 setup failed!");
  }
  else
  {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);

    delay(2000);
    if (!bme.performReading())
    {
      Utility::serialOutput("Failed to perform reading :(");
    }
    Utility::serialOutput(bme.temperature);
    Utility::serialOutput(bme.pressure / 100.0);
    Utility::serialOutput(bme.humidity);
    Utility::serialOutput(bme.gas_resistance / 1000.0);
  }

  vemlInit = veml.begin();
  if (!vemlInit)
  {
    Utility::serialOutput("Veml7700 setup failed!");
  }
  else
  {
    veml.setGain(VEML7700_GAIN_1_8);
    veml.setIntegrationTime(VEML7700_IT_100MS);
    veml.setLowThreshold(10000);
    veml.setHighThreshold(20000);
    veml.interruptEnable(true);

    delay(1000);
    Utility::serialOutput(veml.readALS());
    Utility::serialOutput(veml.readWhite());
    Utility::serialOutput(veml.readLux());
  }

  ltrInit = ltr.begin();
  if (!ltrInit)
  {
    Utility::serialOutput("LTR-390 setup failed!");
  }
  else
  {
    ltr.setMode(LTR390_MODE_UVS);
    ltr.setGain(LTR390_GAIN_3);
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    ltr.setThresholds(100, 1000);
    ltr.configInterrupt(true, LTR390_MODE_UVS);

    Utility::serialOutput(ltr.readUVS());
  }

}

void loop()
{ 
    pm_data();


  delay(300);
 }
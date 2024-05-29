#define TINY_GSM_MODEM_SIM800

#include <Arduino.h>
#include <ArduinoHttpClient.h>
#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
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
#define rxPin 14
#define txPin 4
#define EID "2e2"
#define API_KEY "2jkl23A"

bool rtcInit = false;
bool bmeInit = false;
bool vemlInit = false;
bool ltrInit = false;
bool sdInit = false;

String jsonPayload;

// Initialize the Sensors
RTC_DS1307_LogicHub rtc;
Adafruit_BME680 bme;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_LTR390 ltr = Adafruit_LTR390();
SDCardManager sdCardManager;
SettingsManager settingsManager(SD);
JsonDocument telemetry;
SoftwareSerial pmsSerial(16, 17); // RX, TX pins; Choose pins based on your hardware
SoftwareSerial SerialDT(txPin, rxPin);

PMS pms(pmsSerial);
PMS::DATA data;

// Connection Details
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details
const char server[] = "cctelemetry-dev.azurewebsites.net"; // Replace with your server
const char resource[] = "/telemetry";                      // Replace with your resource path
const int port = 80;

TinyGsm modem(SerialDT);
TinyGsmClient client(modem);
HttpClient http(client, server, port);

bool isGprsConnected = false;

void connect_gprs()
{

  Utility::serialOutput("Initializing modem...");
  modem.restart();
  delay(2000);
  if (!modem.restart())
  {
    Utility::serialOutput("Modem restart failed!");
    return; // Exit the function if modem restart fails
  }

  // GPRS connection
  Serial.print("Connecting to APN: ");
  Utility::serialOutput(apn);

  bool connected = modem.gprsConnect(apn, gprsUser, gprsPass);
  if (!connected)
  {
    Utility::serialOutput("GPRS connection failed on first attempt");
    Utility::serialOutput("Retrying...");

    // First retry logic
    connected = modem.gprsConnect(apn, gprsUser, gprsPass);
    if (!connected)
    {
      Utility::serialOutput("GPRS connection failed on second attempt");
      Utility::serialOutput("Waiting for 2 minutes before final retry...");

      // Non-blocking delay using millis()
      unsigned long startTime = millis();
      while (millis() - startTime < 2 * 60 * 1000)
      {
        // Perform other tasks here if necessary
      }

      // Final retry attempt
      connected = modem.gprsConnect(apn, gprsUser, gprsPass);
      if (!connected)
      {
        Utility::serialOutput("GPRS connection failed on third attempt");
        // Handle the failure here, possibly reset the module or trigger an alert
        // TODO:
        //  Check battery voltage
        //  If it is not enough to send save data and sleep to charge
        //  if is is enough to send restart GSM and retey
        //  In the case of service provider error, stay off GSM
        modem.restart();
        return; // Exit the function if the connection fails after all retries
      }
    }
  }

  if (modem.isGprsConnected())
  {
    Utility::serialOutput("GPRS connected");
  }
  else
  {
    Utility::serialOutput("GPRS connection status could not be verified");
  }
}

void postTelemetry()
{

  serializeJson(telemetry, jsonPayload);

  const char *jsonPayloadChar = jsonPayload.c_str();

  Utility::serialOutput(jsonPayloadChar);

  delay(500);

  Serial.print(F("Performing HTTP POST request... "));

  http.beginRequest();

  http.post(resource, "application/json", jsonPayloadChar);

  // http.sendHeader("Authorization", "Bearer " + String(API_KEY));

  http.sendHeader("Content-Type", "application/json");

  // http.sendHeader("Content-Length", String(strlen(jsonPayloadChar)));

  http.beginBody();

  http.print(jsonPayloadChar);

  http.endRequest();

  int status = http.responseStatusCode();

  Utility::serialOutput("Response status code: ");

  Utility::serialOutput(status);

  if (status == 200)
  {
    Utility::serialOutput("Data sent successfully");
    return;
  }

  String responseBody = http.responseBody();

  Utility::serialOutput("Response:");

  Utility::serialOutput(responseBody);

  http.stop();

  Utility::serialOutput("Server disconnected");

  modem.gprsDisconnect();

  Utility::serialOutput("GPRS disconnected");

  isGprsConnected = false;

  // Sleep for a while before sending the next request
  delay(3000);
}

void pm_data()
{
  if (pms.read(data))
  {
    Utility::serialOutput("PM 1.0 (ug/m3): ");
    Utility::serialOutput(data.PM_AE_UG_1_0);
    Utility::serialOutput("\n");

    Utility::serialOutput("PM 2.5 (ug/m3): ");
    Utility::serialOutput(data.PM_AE_UG_2_5);
    Utility::serialOutput("\n");

    Utility::serialOutput("PM 10.0 (ug/m3): ");
    Utility::serialOutput(data.PM_AE_UG_10_0);

    Utility::serialOutput("\n");
  }
}

void setup()
{
  pinMode(PWR, OUTPUT);
  Utility::triggerPower(PWR, true);

  Serial.begin(9600);

  delay(5000);

  pmsSerial.begin(9600); // Start the software serial for PMS
  SerialDT.begin(9600);

  // Initialize PMS library
  // pms.passiveMode();   // GPIO2 (D4 pin on ESP-12E Development Board)

  Utility::serialOutput("");
  Utility::serialOutput("Debugging and Testing Mode");

  bool sdInit = SD.begin(SD_CS_PIN);
  if (!sdInit)
  {
    Utility::serialOutput("SD card initialization failed.");
  }
  else
  {
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
  // connect_gprs();
  // pm_data();
  //postTelemetry()

  delay(300);
}
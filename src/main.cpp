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
#define Bat 26
#define Solar 25
#define EID "2e2"
#define API_KEY "2jkl23A"

bool rtcInit = false;
bool bmeInit = false;
bool vemlInit = false;
bool ltrInit = false;
bool sdInit = false;

String jsonPayload;
String Status = "";

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

// PMS pms(pmsSerial);
// PMS::DATA data;

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

struct pms5003data
{

  uint16_t framelen;

  uint16_t pm10_standard, pm25_standard, pm100_standard;

  uint16_t pm10_env, pm25_env, pm100_env;

  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;

  uint16_t unused;

  uint16_t checksum;
};

struct pms5003data data;

boolean readPMSdata(Stream *s)
{
  if (!s->available())
  {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42)
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32)
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum)
  {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

void PM()
{

  if (readPMSdata(&pmsSerial))
  {

    Serial.println();

    Utility::serialOutput("Concentration Units (environmental)");

    Serial.print("PM 1.0: ");
    Serial.print(data.pm10_env);

    Serial.print("\t\tPM 2.5: ");
    Serial.print(data.pm25_env);

    Serial.print("\t\tPM 10: ");
    Serial.println(data.pm100_env);

    telemetry["p0"] = data.pm10_env;

    telemetry["p2"] = data.pm25_env;

    telemetry["p1"] = data.pm100_env;

    Status += "0";
  }
  else
  {
    // appendFile(SD, "/logs/logs.txt", "Air Quality Sensor Test: [ ] Fail" );
    telemetry["p0"] = 0;

    telemetry["p2"] = 0;

    telemetry["p1"] = 0;

    Status += "1";
  }

  delay(50);
}

void getVoltage()
{
  int rawBattInput = analogRead(26);             // Read raw ADC value from pin 26
  float voltage = (rawBattInput / 4095.0) * 3.3; // Convert raw value to voltage

  // Adjust for the voltage divider
  float R1 = 0.986; // Resistor 1 in kOhms
  float R2 = 0.984; // Resistor 2 in kOhms

  // Voltage divider formula: V_in = V_out * (R1 + R2) / R2
  float batteryVoltage = voltage * (R1 + R2) / R2;

  // Output the calculated battery voltage
  Utility::serialOutput(batteryVoltage);
  telemetry["b"] = batteryVoltage;


  int rawSolarInput = analogRead(25);
  float voltageSolar = (rawSolarInput / 4095.0) * 3.3; // Convert raw value to voltage
  // Adjust for the voltage divider
  float r1 = 10; // Resistor 1 in kOhms
  float r2 = 4;  // Resistor 2 in kOhms
  float SolarVoltage = voltageSolar * (r1 + r2) / r2;
  telemetry["sp"] = SolarVoltage;
   Utility::serialOutput(SolarVoltage);
}

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
  Utility::serialOutput("Connecting to APN: ");
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

void setup()
{
  pinMode(PWR, OUTPUT);
  Utility::triggerPower(PWR, true);

  Serial.begin(9600);

  delay(5000);

  pmsSerial.begin(9600); // Start the software serial for PMS
  SerialDT.begin(9600);

  Utility::serialOutput("");
  Utility::serialOutput("Debugging and Testing Mode");

  bool sdInit = SD.begin(SD_CS_PIN);
  if (!sdInit)
  {
    Utility::serialOutput("SD card initialization failed.");
    Status += "1";
  }
  else
  {
    Utility::serialOutput("SD Card Ready");
    Status += "0";
  }

  bmeInit = bme.begin();
  if (!bmeInit)
  {
    Utility::serialOutput("Bme680 setup failed!");
    Status += "1";
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
      Status += "1";
    }
    Utility::serialOutput(bme.temperature);
    Utility::serialOutput(bme.pressure / 100.0);
    Utility::serialOutput(bme.humidity);
    // Utility::serialOutput(bme.gas_resistance / 1000.0);

    telemetry["t"] = bme.temperature;
    telemetry["p"] = (bme.pressure / 100.0);
    telemetry["h"] = (bme.humidity);
    Status += "0";
  }

  vemlInit = veml.begin();
  if (!vemlInit)
  {
    Utility::serialOutput("Veml7700 setup failed!");
    Status += "1";
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

    telemetry["l"] = veml.readLux();

    Status += "0";
  }

  ltrInit = ltr.begin();
  if (!ltrInit)
  {
    Utility::serialOutput("LTR-390 setup failed!");
    Status += "1";
  }
  else
  {
    ltr.setMode(LTR390_MODE_UVS);
    ltr.setGain(LTR390_GAIN_3);
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    ltr.setThresholds(100, 1000);
    ltr.configInterrupt(true, LTR390_MODE_UVS);

    telemetry["uv"] = ltr.readUVS();
    Utility::serialOutput(ltr.readUVS());
    Status += "0";
  }

  PM();

  delay(10000);

  PM();

  delay(10000);

  PM();

  getVoltage();

  rtcInit = rtc.begin();
  if (!rtcInit)
  {
    Utility::serialOutput("RTC setup failed!");
    Status += "1";
  }
  else
  {
    rtc.setTimeToCompiled();
    Utility::serialOutput(rtc.getFormattedTime());
    Utility::serialOutput(rtc.getEpochTime());
    telemetry["d"] = rtc.getEpochTime();
    Status += "0";
  }

  Utility::serialOutput(Status);
  telemetry["e"] = Status;
  telemetry["i"] = EID;

  serializeJson(telemetry, jsonPayload);

  const char *jsonPayloadChar = jsonPayload.c_str();

  Utility::serialOutput(jsonPayloadChar);
}

void loop()
{
  // connect_gprs();
  // pm_data();
  // postTelemetry()
getVoltage();
  delay(600);
}
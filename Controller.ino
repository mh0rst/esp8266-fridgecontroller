#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>

#define COMPRESSOR_GPIO 5
#define DEFROST_GPIO 4
#define TEMPERATURE_SENSORS_GPIO 14

#define TEMPERATURE_RESOLUTION 10
const unsigned long TEMPERATURE_REQUEST_INTERVAL_MILLIS = 30000; // 30 seconds
#define MAX_TEMPERATURE_REQUEST_RETRIES 3

/* Minimum delay between motor starts */
const unsigned long STARTUP_DELAY_MILLIS = 120 * 1000; // 2 minutes
/* Defrost interval */
const unsigned long DEFROST_EVERY_MILLIS = 2 * 86400 * 1000; // 2 days
/* Waiting period after defrost heater is switched off */
const unsigned long DEFROST_COOLDOWN_MILLIS = 300 * 1000; // 5 minutes
/* Defrost time */
const unsigned long DEFROST_DURATION_MILLIS = 1500 * 1000 + DEFROST_COOLDOWN_MILLIS; // 30 minutes
/* Defrost backoff if compressor is currently running */
const unsigned long DEFROST_COMPRESSOR_MIN_RUNTIME_MILLIS = 600 * 1000; // 10 minutes
/* Failsafe compressor start interval */
const unsigned long FAILSAFE_START_EVERY_MILLIS = 1800 * 1000; // 30 minutes
/* Failsafe compressor runtime */
const unsigned long FAILSAFE_RUN_FOR_MILLIS = 7200 * 1000; // 120 minutes

// Temperature in Celsius
#define POSITIVE_HYSTERESIS 1
#define NEGATIVE_HYSTERESIS 0.75
#define DEFAULT_FRIDGE_TEMP 7.25
#define DEFAULT_FREEZER_TEMP -18
#define TEMPERATURE_CORRECTION_FRIDGE 0
#define TEMPERATURE_CORRECTION_FREEZER -3.5

DeviceAddress fridgeTempAddr = {0x28, 0x56, 0x02, 0x57, 0x04, 0x3F, 0x3C, 0xE9}; // TEMP_0
DeviceAddress freezerTempAddr = {0x28, 0x3A, 0xEE, 0x57, 0x04, 0x9D, 0x3C, 0xD3}; // TEMP_1

#define WEBSERVER_PORT 80

OneWire oneWire(TEMPERATURE_SENSORS_GPIO);
DallasTemperature sensors(&oneWire);
WiFiManager wifiManager;
ESP8266WebServer server(WEBSERVER_PORT);

bool failsafeMode = false;
bool inDefrost = false;
bool compressorOn = false;
bool heaterOn = false;

float fridgeTemp;
float freezerTemp;
float targetFridgeTemp;
float targetFreezerTemp;

unsigned long lastCompressorShutdown = 0;
unsigned long lastTemperatureUpdate = 0;
unsigned long lastDefrost = 0;
unsigned long defrostStart = 0;
unsigned long compressorStart = 0;
unsigned long temperatureRetries = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Fridge Temp Controller");

  pinMode(COMPRESSOR_GPIO, OUTPUT);
  pinMode(DEFROST_GPIO, OUTPUT);
  switchCompressor(false);
  switchHeater(false);

  initTempSensors();

  targetFridgeTemp = DEFAULT_FRIDGE_TEMP;
  targetFreezerTemp = DEFAULT_FREEZER_TEMP;
  
  WiFi.mode(WIFI_STA);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect("FridgeSetup", "asdfghjk");

  // GET routes
  server.on("/info", handleInfoRequest);
  // POST routes
  server.on("/temp", handleTempRequest);
  server.on("/reset", handleReset);
  server.on("/defrost", handleDefrost);
  server.begin();
}

void initTempSensors() {
  sensors.begin();
  if (!sensors.isConnected(fridgeTempAddr)) {
    Serial.println("Could not find fridge temperature sensor");
    failsafeMode = true;
  }
  if (!sensors.isConnected(freezerTempAddr)) {
    Serial.println("Could not find freezer temperature sensor");
    failsafeMode = true;
  }
  if (failsafeMode) {
    lastCompressorShutdown = millis() - FAILSAFE_START_EVERY_MILLIS;
    return;
  } else {
    lastTemperatureUpdate = millis() - TEMPERATURE_REQUEST_INTERVAL_MILLIS;
  }
  sensors.setResolution(fridgeTempAddr, TEMPERATURE_RESOLUTION);
  sensors.setResolution(freezerTempAddr, TEMPERATURE_RESOLUTION);
}


void loop() {
  if (!failsafeMode) {
    requestTemperatures();
  }
  if (!inDefrost && shouldDefrost()) {
    if (compressorOn) {
      switchCompressor(false);
    }
    switchHeater(true);
    defrostStart = millis();
    inDefrost = true;
  }
  if (inDefrost) {
    if (heaterOn && millis() - defrostStart > DEFROST_DURATION_MILLIS - DEFROST_COOLDOWN_MILLIS) {
      switchHeater(false);
    }
    if (millis() - defrostStart > DEFROST_DURATION_MILLIS) {
      lastDefrost = millis();
      inDefrost = false;
    }
  } else {
    if (!compressorOn && shouldStartCompressor()) {
      switchCompressor(true);
    }
    if (compressorOn && shouldStopCompressor()) {
      switchCompressor(false);
    }
  }
  wifiManager.process();
  server.handleClient();
  if (server.client().status() == CLOSED) {
    delay(1);
  }
}

void requestTemperatures() {
  if (millis() - lastTemperatureUpdate > TEMPERATURE_REQUEST_INTERVAL_MILLIS) {
    sensors.requestTemperatures();
    float tempC = sensors.getTempC(fridgeTempAddr);
    bool failure = false;
    if (tempC == DEVICE_DISCONNECTED_C) 
    {
      Serial.println("Could not read fridge temperature");
      failure = true;
    } else {
      fridgeTemp = tempC + TEMPERATURE_CORRECTION_FRIDGE;
    }
    tempC = sensors.getTempC(freezerTempAddr);
    if (tempC == DEVICE_DISCONNECTED_C) 
    {
      Serial.println("Could not read freezer temperature");
      failure = true;
    } else {
      freezerTemp = tempC + TEMPERATURE_CORRECTION_FREEZER;
    }
    if (failure) {
      if (temperatureRetries >= MAX_TEMPERATURE_REQUEST_RETRIES) {
        failsafeMode = true;
        lastCompressorShutdown = millis() - FAILSAFE_START_EVERY_MILLIS;
      } else {
        temperatureRetries++;
      }
    } else {
      temperatureRetries = 0;
    }
    lastTemperatureUpdate = millis();
  }
}

bool shouldStartCompressor() {
  unsigned long now = millis();
  if (now - lastCompressorShutdown > STARTUP_DELAY_MILLIS) {
    if (failsafeMode) {
      return now - lastCompressorShutdown > FAILSAFE_START_EVERY_MILLIS;
    }
    return (fridgeTemp > targetFridgeTemp + POSITIVE_HYSTERESIS
          || freezerTemp > targetFreezerTemp + POSITIVE_HYSTERESIS);
  }
  return false;
}

bool shouldDefrost() {
  unsigned long now = millis();
  return now - lastDefrost > DEFROST_EVERY_MILLIS && (!compressorOn || now - compressorStart > DEFROST_COMPRESSOR_MIN_RUNTIME_MILLIS);
}

bool shouldStopCompressor() {
  if (failsafeMode) {
    return millis() - compressorStart > FAILSAFE_RUN_FOR_MILLIS;
  }
  return fridgeTemp < targetFridgeTemp - NEGATIVE_HYSTERESIS
      && freezerTemp < targetFreezerTemp - NEGATIVE_HYSTERESIS;
}

void switchCompressor(bool state) {
  Serial.print("Switching compressor ");
  Serial.println(state ? "on" : "off");
  compressorOn = state;
  digitalWrite(COMPRESSOR_GPIO, state);
  if (state) {
    compressorStart = millis();
  } else {
    lastCompressorShutdown = millis();
  }
}

void switchHeater(bool state) {
  Serial.print("Switching defrost heater ");
  Serial.println(state ? "on" : "off");
  heaterOn = state;
  digitalWrite(DEFROST_GPIO, state);
}

void handleInfoRequest() {
  String json = "{\"temperatures\": {\"fridge\": ";
          json.concat(fridgeTemp);
          json += ", \"freezer\": ";
          json.concat(freezerTemp);
          json += "}, \"targetTemperatures\": {\"fridge\": ";
          json.concat(targetFridgeTemp);
          json += ", \"freezer\": ";
          json.concat(targetFreezerTemp);
          json += "}, \"stats\": {\"now\": ";
          json.concat(millis());
          json += ", \"compressorStart\": ";
          json.concat(compressorStart);
          json += ", \"lastCompressorShutdown\": ";
          json.concat(lastCompressorShutdown);
          json += ", \"defrostStart\": ";
          json.concat(defrostStart);
          json += ", \"lastDefrost\": ";
          json.concat(lastDefrost);
          json += "}, \"compressorOn\": ";
          json.concat(compressorOn ? "true" : "false");
          json += ", \"inDefrost\": ";
          json.concat(inDefrost ? "true" : "false");
          json += ", \"heaterOn\": ";
          json.concat(heaterOn ? "true" : "false");
          json += ", \"failSafe\": ";
          json.concat(failsafeMode ? "true" : "false");
          json += "}";
  server.send(200, "application/json", json);
}

void handleTempRequest() {
  if (server.method() == HTTP_POST) {
    for (uint8_t i = 0; i < server.args(); i++) {
      if (server.argName(i).equals("targetFridgeTemp")) {
        targetFridgeTemp = atof(server.arg(i).c_str());
      } else if (server.argName(i).equals("targetFreezerTemp")) {
        targetFreezerTemp = atof(server.arg(i).c_str());
      }
    }
  }
}

void handleReset() {
  if (server.method() == HTTP_POST) {
    ESP.restart();
  }
}

void handleDefrost() {
  if (server.method() == HTTP_POST) {
    for (uint8_t i = 0; i < server.args(); i++) {
      if (server.argName(i).equals("forceStart") && !inDefrost) {
        lastDefrost = millis() - DEFROST_EVERY_MILLIS;
        if (compressorOn) {
          compressorStart = millis() - DEFROST_COMPRESSOR_MIN_RUNTIME_MILLIS;
        }
      } else if (server.argName(i).equals("forceStop") && inDefrost) {
        defrostStart = millis() - DEFROST_DURATION_MILLIS;
        if (failsafeMode) {
          lastCompressorShutdown = millis() - FAILSAFE_START_EVERY_MILLIS;
        } else {
          lastCompressorShutdown = millis() - STARTUP_DELAY_MILLIS;
        }
      }
    }
  }
}
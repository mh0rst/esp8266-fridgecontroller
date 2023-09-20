#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>

#define COMPRESSOR_GPIO 5

#define TEMPERATURE_SENSORS_GPIO 14
#define TEMPERATURE_RESOLUTION 10

#define TEMPERATURE_REQUEST_INTERVAL_MILLIS 30000
#define MAX_TEMPERATURE_REQUEST_RETRIES 3
//#define STARTUP_DELAY_MILLIS 300 * 1000 // 5 Minutes
#define STARTUP_DELAY_MILLIS 10 * 1000 // Original controller handles this
#define FAILSAFE_START_EVERY_MILLIS 1800 * 1000 // 30 minutes
#define FAILSAFE_RUN_FOR_MILLIS 7200 * 1000 // 120 minutes

// Temperature in Celsius
#define POSITIVE_HYSTERESIS 1
#define NEGATIVE_HYSTERESIS 1
#define DEFAULT_FRIDGE_TEMP 7
#define DEFAULT_FREEZER_TEMP -18
#define TEMPERATURE_CORRECTION_FRIDGE 0
#define TEMPERATURE_CORRECTION_FREEZER -3,5


OneWire oneWire(TEMPERATURE_SENSORS_GPIO);
DallasTemperature sensors(&oneWire);
WiFiManager wifiManager;
ESP8266WebServer server(80);

DeviceAddress fridgeTempAddr = {0x28, 0x56, 0x02, 0x57, 0x04, 0x3F, 0x3C, 0xE9}; // TEMP_0
DeviceAddress freezerTempAddr = {0x28, 0x3A, 0xEE, 0x57, 0x04, 0x9D, 0x3C, 0xD3}; // TEMP_1

bool failsafeMode = false;
bool compressorOn = false;

float fridgeTemp;
float freezerTemp;
float targetFridgeTemp;
float targetFreezerTemp;

unsigned long lastCompressorShutdown = 0;
unsigned long lastTemperatureUpdate = - TEMPERATURE_REQUEST_INTERVAL_MILLIS; // Request temps on start immediately
unsigned long temperatureRetries = 0;
unsigned long failsafeStart = 0;
unsigned long failsafeStop = - FAILSAFE_START_EVERY_MILLIS; // Start immediately in failsafe mode

void setup() {
  Serial.begin(115200);
  Serial.println("Fridge Temp Controller");

  pinMode(COMPRESSOR_GPIO, OUTPUT);
  switchCompressor(false);

  initTempSensors();

  targetFridgeTemp = DEFAULT_FRIDGE_TEMP;
  targetFreezerTemp = DEFAULT_FREEZER_TEMP;
  
  WiFi.mode(WIFI_STA);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect("FridgeSetup", "asdfghjk");

  server.on("/temp", handleTempRequest);
  server.on("/reset", handleReset);
  server.begin();
}

void initTempSensors() {
  sensors.begin();
  if (!sensors.isConnected(fridgeTempAddr)) {
    Serial.println("Could not find fridge temperature sensor");
    failsafeMode = true;
    return;
  }
  if (!sensors.isConnected(freezerTempAddr)) {
    Serial.println("Could not find freezer temperature sensor");
    failsafeMode = true;
    return;
  }
  sensors.setResolution(fridgeTempAddr, TEMPERATURE_RESOLUTION);
  sensors.setResolution(freezerTempAddr, TEMPERATURE_RESOLUTION);
}


void loop() {
  if (failsafeMode) {
    unsigned long now = millis();
    if (!compressorOn && now - lastCompressorShutdown > STARTUP_DELAY_MILLIS && now - failsafeStop > FAILSAFE_START_EVERY_MILLIS) {
      switchCompressor(true);
      failsafeStart = now;
    }
    if (compressorOn && now - failsafeStart > FAILSAFE_RUN_FOR_MILLIS) {
      switchCompressor(false);
      failsafeStop = now;
    }
  } else {
    requestTemperatures();
    if (!compressorOn && shouldStartCompressor()) {
      switchCompressor(true);
    }
    if (compressorOn && shouldStopCompressor()) {
      switchCompressor(false);
    }
  }
  wifiManager.process();
  server.handleClient();
  if(server.client().status() == CLOSED) {
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
  return millis() - lastCompressorShutdown > STARTUP_DELAY_MILLIS
      && (fridgeTemp > targetFridgeTemp + POSITIVE_HYSTERESIS
       || freezerTemp > targetFreezerTemp + POSITIVE_HYSTERESIS);
}

bool shouldStopCompressor() {
  return fridgeTemp < targetFridgeTemp - NEGATIVE_HYSTERESIS
      && freezerTemp < targetFreezerTemp - NEGATIVE_HYSTERESIS;
}

void switchCompressor(bool state) {
  Serial.print("Switching compressor ");
  Serial.println(state ? "on" : "off");
  compressorOn = state;
  // Using PNP transistor, LOW switches it on
  digitalWrite(COMPRESSOR_GPIO, !state);
  if(!state) {
    lastCompressorShutdown = millis();
  }
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
  } else {
    String json = "{\"temperatures\": {\"fridge\": ";
           json.concat(fridgeTemp);
           json += ", \"freezer\": ";
           json.concat(freezerTemp);
           json += "}, \"targetTemperatures\": {\"fridge\": ";
           json.concat(targetFridgeTemp);
           json += ", \"freezer\": ";
           json.concat(targetFreezerTemp);
           json += "}, \"compressorOn\": ";
           json.concat(compressorOn ? "true" : "false");
           json += ", \"failSafe\": ";
           json.concat(failsafeMode ? "true" : "false");
           json += "}";
    server.send(200, "application/json", json);

  }
}

void handleReset() {
  if (server.method() == HTTP_POST) {
    ESP.restart();
  }
}
#include <Arduino.h>
#include <WiFi.h>
#include "ArduinoJson.h"
#include "hoermann.h"
#include <ESPAsyncWebServer.h>
#include "../../private/credentials.h"
/*
// Sample Contents of "credentials.sh"
const char* ssid = "Wifi SSID";
const char* password = "Wifi Password";
*/

static const char* TAG_WIFI = "WIFI";


void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  ESP_LOGI(TAG_WIFI,"Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  ESP_LOGI(TAG_WIFI, "WiFi connected");
  ESP_LOGI(TAG_WIFI, "IP address: ");
  ESP_LOGI(TAG_WIFI, WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  ESP_LOGI(TAG_WIFI, "Disconnected from WiFi access point");
  ESP_LOGI(TAG_WIFI, "WiFi lost connection. Reason: %s",info.disconnected.reason);
  ESP_LOGI(TAG_WIFI, "Trying to Reconnect");
  WiFi.begin(ssid, password);
}

String sysInfoToJson(){
  // Process Request to get SysInfo
  DynamicJsonDocument root(1024);
  root["freemem"] = ESP.getFreeHeap();    
  root["hostname"] = WiFi.getHostname();
  root["ip"] = WiFi.localIP().toString();
  root["ssid"] = String(ssid);
  root["wifistatus"] = WiFi.status();
  root["resetreason"] =esp_reset_reason();    
  String output;
  serializeJson(root,output);
  return output;
}

AsyncWebServer server(80);

void setupWebServer(){
  server.on("/status", HTTP_GET, [] (AsyncWebServerRequest *request) {
      request->send(200, "application/json", hoermannEngine->state->toStatusJson());
  });
  server.on("/sysinfo", HTTP_GET, [] (AsyncWebServerRequest *request) {
      request->send(200, "application/json", sysInfoToJson());
  });
  server.on("/command/stopDoor", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->stopDoor();request->send(202);});
  server.on("/command/closeDoor", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->closeDoor();request->send(202);});
  server.on("/command/openDoor", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->openDoor();request->send(202);});
  server.on("/command/toogleDoor", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->toogleDoor();request->send(202);});
  server.on("/command/halpPositionDoor", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->halpPositionDoor();request->send(202);});
  server.on("/command/ventilationPositionDoor", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->ventilationPositionDoor();request->send(202);});
  server.on("/command/toogleLight", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->toogleLight();request->send(202);});
  server.on("/command/turnLight", HTTP_POST, [] (AsyncWebServerRequest *request) { hoermannEngine->turnLight(request->hasParam("on"));request->send(202);});

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Not found ;(");
  });

  server.begin();
}


/****************************************************************************************
 * setup mcu
 ****************************************************************************************/
void setup(){
  
  hoermannEngine->setup();
  
  WiFi.disconnect(true);
  delay(1000);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.setHostname("garagentor");
  WiFi.begin(ssid, password);

  setupWebServer();
}

// mainloop
void loop(){
  delay(1000);
}

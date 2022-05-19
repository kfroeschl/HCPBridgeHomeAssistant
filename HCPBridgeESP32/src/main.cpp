#include <Arduino.h>
#include <WiFi.h>
#include "ArduinoJson.h"
#include "hciemulator.h"
#include "EspMQTTClient.h"
#include "../../../private/credentials.h"
/*
// Sample Contents of "credentials.sh"
const char* ssid = "Wifi SSID";
const char* password = "Wifi Password";
const char* mqtt_server = "mqttserver.localdomain";
const int mqtt_port = 1883;
const char* mqtt_username = "mqtt_username";
const char* mqtt_password = "mqtt_password";
*/

const String deviceMqttPath = "hoermann/supramatic4";

const char* mqtt_clientId = deviceMqttPath.c_str();
const char* deviceBusStatus = (deviceMqttPath + "/status").c_str();
const char* deviceBusSysInfoRequest = (deviceMqttPath + "/sysInfoRequest").c_str();
const char* deviceBusSysInfoReply = (deviceMqttPath + "/sysInfo").c_str();
const char* deviceBusCommands = (deviceMqttPath + "/commands").c_str();
const char* deviceBusAvailable = (deviceMqttPath + "/available").c_str();

const String homeAssistantConfigPrefix = "homeassistant";

// switch relay sync to the lamp
// e.g. the Wifi Relay Board U4648
#define USERELAY

// use alternative uart pins 
//#define SWAPUART

#define RS485 Serial

#define PIN_TXD 17 // UART 2 TXT - G17
#define PIN_RXD 16 // UART 2 RXD - G16

// Relay Board parameters
#define ESP8266_GPIO2    2 // Blue LED.
#define ESP8266_GPIO4    4 // Relay control.
#define ESP8266_GPIO5    5 // Optocoupler input.
#define LED_PIN          ESP8266_GPIO2

// Hörmann HCP2 based on modbus rtu @57.6kB 8E1
HCIEmulator emulator(&RS485);

EspMQTTClient client(
  ssid,
  password,
  mqtt_server,
  mqtt_username,
  mqtt_password,
  mqtt_clientId,
  mqtt_port
);

volatile unsigned long lastCall = 0;
volatile unsigned long maxPeriod = 0;

// called by ESPAsyncTCP-esphome:SyncClient.cpp (see patch) instead of delay to avoid connection breaks
void DelayHandler(void){
    emulator.poll();
}

String translateState(int stateCode){
  switch (stateCode)
  {
  case 1:
    return "opening";
  case 2:
    return "closing";
  case 32:
    return "open";
  case 64:
    return "closed";
  default:
    return "stoped";
  }
}

void pushStateToMqtt(){
  const SHCIState& doorstate = emulator.getState();
  if (doorstate.valid){
    DynamicJsonDocument root(1024);
    root["valid"] = doorstate.valid;
    root["doorstate"] = translateState(doorstate.doorState);
    root["doorposition"] = doorstate.doorCurrentPosition;
    root["doortarget"] = doorstate.doorTargetPosition;
    root["lamp"] = doorstate.lampOn;
    root["debug"] = doorstate.reserved;
    root["lastresponse"] = emulator.getMessageAge()/1000;    
    root["looptime"] = maxPeriod;    

    lastCall = maxPeriod = 0;

    String output;
    serializeJson(root,output);
    client.publish(deviceBusStatus, output, true);
  }
}

// switch GPIO4 und GPIO2 sync to the lamp
void onStatusChanged(const SHCIState& state){
  //see https://ucexperiment.wordpress.com/2016/12/18/yunshan-esp8266-250v-15a-acdc-network-wifi-relay-module/
  //Setting GPIO4 high, causes the relay to close the NO contact with
  if(state.valid){    
      digitalWrite( ESP8266_GPIO4, state.lampOn ); 
      digitalWrite(LED_PIN, state.lampOn);
      pushStateToMqtt();
  }else
  {
      digitalWrite( ESP8266_GPIO4, false ); 
      digitalWrite(LED_PIN, false);
  }
}

// toggle lamp to expected state
void switchLamp(bool on){
  bool toggle = (on && !emulator.getState().lampOn) || (!on && emulator.getState().lampOn);
  if(toggle){
    emulator.toggleLamp();
  }    
}


void modBusPolling( void * parameter) {
  while(true){
      if(lastCall>0){
          maxPeriod = _max(micros()-lastCall,maxPeriod);
      }
      lastCall=micros();
      emulator.poll();  
      vTaskDelay(1);    
  }
  vTaskDelete(NULL);
}


TaskHandle_t modBusTask;

// setup mcu
void setup(){
  
  //setup modbus
  RS485.begin(57600,SERIAL_8E1,PIN_RXD,PIN_TXD);
  #ifdef SWAPUART
    RS485.swap();  
  #endif  

  xTaskCreatePinnedToCore(
      modBusPolling, /* Function to implement the task */
      "ModBusTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      //1,  /* Priority of the task */
      configMAX_PRIORITIES -1,
      &modBusTask,  /* Task handle. */
      1); /* Core where the task should run */
  
  //setup relay board
#ifdef USERELAY
  pinMode( ESP8266_GPIO4, OUTPUT );       // Relay control pin.
  pinMode( ESP8266_GPIO5, INPUT_PULLUP ); // Input pin.
  pinMode( LED_PIN, OUTPUT );             // ESP8266 module blue L
  digitalWrite( ESP8266_GPIO4, 0 );
  digitalWrite(LED_PIN,0);
  emulator.onStatusChanged(onStatusChanged);
#endif
}

void onCommandReceived(const String &payload){
  // Process commands from MQTT
  if (payload == "CLOSE"){
    emulator.closeDoor();
  } else if (payload == "OPEN"){
    emulator.openDoor();
  } else if (payload == "STOP"){
      emulator.stopDoor();
  } else if (payload == "VENTILATION"){
      emulator.ventilationPosition();
  } else if (payload == "HALF_OPEN"){
      emulator.openDoorHalf();
  } else if (payload == "LAMP_TOOGLE"){
      emulator.toggleLamp();
  } else if (payload == "LAMP_ON"){
      switchLamp(true);
  } else if (payload == "LAMP_OFF"){
      switchLamp(false);
  }
}

void pushSysInfo(const String &prequestPayload){
  // Process Request to get SysInfo
  DynamicJsonDocument root(1024);
  root["freemem"] = ESP.getFreeHeap();    
  root["hostname"] = client.getMqttClientName();
  root["ip"] = WiFi.localIP().toString();
  root["ssid"] = String(ssid);
  root["wifistatus"] = WiFi.status();
  root["resetreason"] =esp_reset_reason();    
  String output;
  serializeJson(root,output);
  client.publish(deviceBusSysInfoReply, output);
}

void createHomeAssistantAutoConfig(){
    DynamicJsonDocument root(1024);
    root["name"] = "Garegentor";
    root["plattform"] = "mqtt";
    root["device_class"] = "garage";
    root["availability"][0]["topic"] = deviceBusAvailable;
    root["command_topic"] = deviceBusCommands;
    root["position_topic"] = "hoermann/supramatic4/status";
    root["position_template"] = "{{ value_json.doorposition }}";
    root["position_open"] = 200;
    root["position_closed"] = 0;
    root["state_topic"] = deviceBusStatus;
    root["value_template"] = "{{ value_json.doorstate }}";
    String output;
    serializeJson(root,output);
    String haDeviceId = ("" + deviceMqttPath);
    haDeviceId.replace('/', '_'); 
    client.publish(homeAssistantConfigPrefix + "/cover/" + haDeviceId + "/config", 
      output, true);
}

void onConnectionEstablished()
{
  client.enableHTTPWebUpdater();
  client.subscribe(deviceBusSysInfoRequest, pushSysInfo);
  client.subscribe(deviceBusCommands, onCommandReceived);
  client.enableLastWillMessage(deviceBusAvailable,"offline", true);
  client.publish(deviceBusAvailable,"online", true);
  createHomeAssistantAutoConfig();
  pushSysInfo("");
}


// mainloop
void loop(){
  client.loop();
}
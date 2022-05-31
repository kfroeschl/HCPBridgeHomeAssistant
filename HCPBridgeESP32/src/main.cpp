#include <Arduino.h>
#include <WiFi.h>
#include "ArduinoJson.h"
#include "hciemulator.h"
#include "PubSubClient.h"
#include "../../../private/credentials.h"
/*
// Sample Contents of "credentials.sh"
const char* ssid = "Wifi SSID";
const char* password = "Wifi Password";
const char* mqtt_server = "mqttserver.localdomain";
const short mqtt_port = 1883;
const char* mqtt_username = "mqtt_username";
const char* mqtt_password = "mqtt_password";
*/

const char* deviceMqttPath = "hoermann/supramatic4";

const char* mqtt_clientId = deviceMqttPath;
const char* deviceBusStatus = "hoermann/supramatic4/status";
const char* deviceBusSysInfoRequest = "hoermann/supramatic4/sysInfoRequest";
const char* deviceBusSysInfoReply = "hoermann/supramatic4/sysInfo";
const char* deviceBusCommands = "hoermann/supramatic4/commands";
const char* deviceBusAvailable = "hoermann/supramatic4/available";

const String homeAssistantConfigPrefix = "homeassistant";

// switch relay sync to the lamp
// e.g. the Wifi Relay Board U4648
#define USERELAY

// use alternative uart pins 
//#define SWAPUART

#define RS485 Serial2

#define PIN_TXD 17 // UART 2 TXT - G17
#define PIN_RXD 16 // UART 2 RXD - G16

// Relay Board parameters
#define ESP8266_GPIO2    2 // Blue LED.
#define ESP8266_GPIO4    4 // Relay control.
#define ESP8266_GPIO5    5 // Optocoupler input.
#define LED_PIN          ESP8266_GPIO2

// Hörmann HCP2 based on modbus rtu @57.6kB 8E1
HCIEmulator emulator(&RS485);

WiFiClient espClient;
PubSubClient client(espClient);

volatile unsigned long lastCall = 0;
volatile unsigned long maxPeriod = 0;

void pushStateToMqtt();

// called by ESPAsyncTCP-esphome:SyncClient.cpp (see patch) instead of delay to avoid connection breaks
void DelayHandler(void){
    emulator.poll();
}

void setup_wifi() {
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/****************************************************************************************
 * State Handling Modbus
 ****************************************************************************************/

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

/****************************************************************************************
 * Commands in MQTT
 ****************************************************************************************/
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
    client.publish(deviceBusStatus, output.c_str(), true);
  }
}

void pushSysInfo(){
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
  client.publish(deviceBusSysInfoReply, output.c_str());
}

void createHomeAssistantAutoConfig(){
    DynamicJsonDocument root(1024);
    root["name"] = "Garegentor";
    root["plattform"] = "mqtt";
    root["device_class"] = "garage";
    root["availability"][0]["topic"] = deviceBusAvailable;
    root["command_topic"] = deviceBusCommands;
    root["position_topic"] = deviceBusStatus;
    root["position_template"] = "{{ value_json.doorposition }}";
    root["position_open"] = 200;
    root["position_closed"] = 0;
    root["state_topic"] = deviceBusStatus;
    root["value_template"] = "{{ value_json.doorstate }}";
    String output;
    serializeJson(root,output);
    String haDeviceId(deviceMqttPath);
    haDeviceId.replace('/', '_'); 
    String haConfigTopic = homeAssistantConfigPrefix + "/cover/" + haDeviceId + "/config";
    client.publish(haConfigTopic.c_str(), output.c_str(), true);
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

void callback(char* topic, byte* message, unsigned int length) {
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  if (String(topic)==deviceBusCommands){
    onCommandReceived(messageTemp);
  } else if (String(topic)==deviceBusSysInfoRequest){
    pushSysInfo();
  }
}

/****************************************************************************************
 * setup mcu
 ****************************************************************************************/
void setup(){
  Serial.begin(115200);
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

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //ArduinoOTA.begin(WiFi.localIP(), mqtt_clientId, mqtt_password, InternalStorage);
}


/****************************************************************************************
 * Connection Handling Loop
 ****************************************************************************************/
void onConnectionEstablished(){ 
  client.publish(deviceBusAvailable,"online", true);
  pushSysInfo();
  createHomeAssistantAutoConfig();
  client.subscribe(deviceBusSysInfoRequest);
  client.subscribe(deviceBusCommands);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_clientId,mqtt_username,mqtt_password,deviceBusAvailable,0,true,"offline")) {
      onConnectionEstablished();
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// mainloop
void loop(){
  //ArduinoOTA.poll();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
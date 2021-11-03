#include <Arduino.h>
#include "esp_system.h"

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>   ////OTA

//// NorVi Session ////

#include "SSD1306.h"
#define ANALOG_PIN_0  32
SSD1306  display(0x3C, 16, 17);
int analog_value = 0;
String adcString;

//// END NorVi Session ////

char data[100];
char datart[100];
char datact[100];
char dataev[100];

//Wi-Fi User name & Password: IoT & BIT210821k ///// Guest & BIR830741y ///// I4.0 & factoryI4.0
//String ssid() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* WSSID = doc["SSID"]; // "testt"
//  return WSSID;
//}

//String password() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* Password = doc["Password"]; // "testt"
//  return Password;
//}
//MQTT Broker IP 192.168.74.72  //13.10.0.87 ///for test: broker.hivemq.com


//const char* mqtt_server() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* MQServer = doc["MQServer"]; // "testt"
//  return MQServer;
//}
//
//const char* mcName() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* MCName = doc["MCName"]; // "testt"
//  return MCName;
//}
//
//const char* mcLoca() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* MCPlant = doc["MCPlant"]; // "testt"
//  return MCPlant;
//}
//
//const char* mqtt_user() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* MQUser = doc["MQUser"]; // "testt"
//  return MQUser;
//}
//
//const char* mqtt_pass() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* MQPass = doc["MQPass"]; // "testt"
//  return MQPass;
//}
//
//const char* mqtt_port() {
//  String read_spiffs = "";
//  read_spiffs = readFile("setting.json");  //////Read from SPIFFS
//  StaticJsonDocument<512> doc;
//  DeserializationError error = deserializeJson(doc, read_spiffs);
//  const char* MQPort = doc["MQPort"]; // "1883"
//  return MQPort;
//}

//Wi-Fi User name & Password: IoT & BIT210821k
const char* ssid     = "Unknow"; //Guest ---- I4.0
const char* password = "IKjirawaT"; //BIR830741y ---- factoryI4.0

//MQTT Broker IP 192.168.74.72  //13.10.0.87 ///for test: broker.hivemq.com

const char* mqtt_server = "broker.hivemq.com";

String mcName = "m06";
String mcLoca = "R2";
String mcBuilding = "R2";

#define mqtt_user "" //beltonoee
#define mqtt_pass "" //beltonoee
#define mqtt_port 1883
#define mqtt_topic "test"
#define mqtt_runtime "R2/rt/m06"
#define mqtt_downtime "R2/idlt/m06"
#define mqtt_counting "R2/ct/m06"
#define mqtt_event_runtime "R2/ev/m06"
#define willTopic "R2/st/m06"
#define mqtt_json "R2/json/m06"

char* willMessage = "disconnect MQTT";

AsyncWebServer server(80);
WiFiClient wifiClient;
PubSubClient client(wifiClient);
/////////////////////////////////////////////////////////////

//uint64_t macAddress = ESP.getEfuseMac();
//uint64_t macAddressTrunc = macAddress << 40;
//chipID = macAddressTrunc >> 40;

char serialNo[16];
uint64_t chipid = ESP.getEfuseMac(); // The chip ID is essentially its MAC address(length: 6 bytes).
uint16_t chip = (uint16_t)(chipid >> 32);

const int buttonPin = 33;   //I.2 runtime
const int counterpin = 22; //I.6 counting

const int ledstate = 27;
const int ledmqstate = 23;

int currentstate = 0;
int previousstate = 0;
int evcurstate = 0;

int counting;

int rrtstate = 0;

int dtState = 0;
int rtState = 0;

///used insted of dalay function
const long ctTime = 10000;
unsigned long preTime = 0;

unsigned long previousMillis = 0;
const long interval = 5000;

unsigned long preHigh = 0;
unsigned long preLow = 0;
const long intervalHigh = 1000;
const long intervalLow = 6000;

int evrt = 0;
int evdt = 0;
const int evinterval = 1000;
int evpreT = 0;
int evstate = 0;

String statusCheckRT;
String receieveFromIP = "";



void callback(char* topic, byte * payload, unsigned int length) {
  Serial.println("-------new message from broker-----");
  Serial.print("channel:");
  Serial.println(topic);
  Serial.print("data:");
  Serial.write(payload, length);
  Serial.println();
}

void publishParameter() {
  String read_spiffs = "";
  read_spiffs = readFile("setting.json");  //////Read from SPIFFS

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, read_spiffs);

  const char* WSSID = doc["SSID"]; // "IoT"
  const char* Password = doc["Password"]; // "BIT210821k"
  const char* MQServer = doc["MQServer"]; // "192.168.74.72"
  const char* MQPort = doc["MQPort"]; // "1883"
  const char* MQUser = doc["MQUser"]; // "oee"
  const char* MQPass = doc["MQPass"]; // "testt"
  const char* MCName = doc["MCName"]; // "m04"
  const char* MCPlant = doc["MCPlant"]; // "R2"
  const char* MCBuild = doc["MCBuild"]; // "R2"

  String LocalIP = String() + "IP:" + WiFi.localIP()[0] + "." + WiFi.localIP()[1] + "." + WiFi.localIP()[2] + "." + WiFi.localIP()[3];
  DynamicJsonDocument  doc(1024);
  JsonObject control =  doc.to<JsonObject>();;
  control["SSID"] = WSSID;
  control["MQPort"] = MQPort;
  control["mcName"] = MCName;
  control["location"] = MCPlant;
  control["deviceIP"] = LocalIP;
  control["deviceSN"] = String(serialNo);
  JsonObject objData = doc.createNestedObject("data");
  objData["runtime"] = datart;
  objData["counter"] = datact;
  objData["event"] = dataev;
  objData["mcstatus"] = statusCheckRT;
  control["RSSI"] = WiFi.RSSI();
  char bufferSend[300] ;
  serializeJson(doc, bufferSend);
  client.publish(mqtt_json, bufferSend, true);
}

int checkState() {
  int buttonState = 0;
  int rtState = 0;
  int ZState = 0;
  buttonState = digitalRead(buttonPin); //pin 15
  //  rtState = digitalRead(runTimepin); //pin 27
  //Serial.println(buttonState);
  if (buttonState == LOW) {
    ZState = 1;
  }
  else if (buttonState == HIGH) {
    ZState = 0;
  }
  return ZState;
}

void run_time_state() {            //// auto sending loop used instead of delay function rt&dt
  unsigned long crTime = millis();
  if (crTime - preTime >= ctTime) {
    preTime = crTime;
    rrtstate = digitalRead(buttonPin); // I.2 runtime
    int rttstate = checkState();
    sprintf(datart, "%d", rttstate);
    client.publish(mqtt_runtime, datart);
    publishData();
    if (rttstate = 1) {
      statusCheckRT = "stop";
      update_display();
    }
    if (rttstate = 0) {
      statusCheckRT = "running";
      update_display();
    }
    update_display();
  }
}

void counter_state () {
  counting = 0;
  currentstate = digitalRead(counterpin); // I.6 counting
  unsigned long currenT = millis();

  if (currentstate == LOW && currenT - preHigh >= intervalHigh && previousstate == 0) {
    counting = 1;
    Serial.print("Counter:");
    Serial.println(counting);
    sprintf(datact, "%d", counting);
    client.publish(mqtt_counting, datact);
    previousstate = 1;
    publishData();
    preHigh = currenT;
    preLow = preHigh;
  }
  else if (currentstate == HIGH && currenT - preLow >= intervalLow && previousstate == 1) {
    previousstate = 0;
    preLow = currenT;
  }
}

int writeToFile(String filename, String strtxt) {
  File file = SPIFFS.open("/setting.json", "w");
  if (!file) {
    Serial.println("Error opening file for writing");
    return (0);
  }
  int bytesWritten = file.println(strtxt);
  file.close();
  return (bytesWritten);
}

String readFile(String filename) {
  String str = "";
  char ch;
  File file = SPIFFS.open("/setting.json", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return (str);
  }

  while (file.available()) {
    ch = file.read();
    if (ch != 0x0a) {
      str += ch;
    }
  }
  file.close();
  //     str.trim();
  return (str);
}

void Read_parameter_from_SPIFFS() {
  /////////////////////////////////////Read_parameter_from_SPIFFS.Read/////////////////////////////
  String read_spiffs = "";
  read_spiffs = readFile("setting.json");  //////Read from SPIFFS

  StaticJsonDocument<512> doc;

  DeserializationError error = deserializeJson(doc, read_spiffs);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  const char* WSSID = doc["SSID"]; // "IoT"
  const char* Password = doc["Password"]; // "BIT210821k"
  const char* MQServer = doc["MQServer"]; // "192.168.74.72"
  const char* MQPort = doc["MQPort"]; // "1883"
  const char* MQUser = doc["MQUser"]; // "oee"
  const char* MQPass = doc["MQPass"]; // "testt"
  const char* MCName = doc["MCName"]; // "m04"
  const char* MCPlant = doc["MCPlant"]; // "R2"
  const char* MCBuild = doc["MCBuild"]; // "R2"

  //  FFSSSID = String(WSSID);
  //  FFSPassword = String(Password);
  //  FFSMQServer = String(MQServer);
  //  FFSMQPort = String(MQPort);
  //  FFSMQUser = String(MQUser);
  //  FFSMQPass = String(MQPass);
  //  FFSMCName = String(MCName);
  //  FFSMCPlant = String(MCPlant);
  //  FFSMCBuild = String(MCBuild);
  /////////////////////////////////////Read_parameter_from_SPIFFS.Read/////////////////////////////
}

void Writing_parameter_from_web_server() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", "text/html", false);
  });

  server.on("/espReset", HTTP_GET, [](AsyncWebServerRequest * request) {
    ESP.restart();
    request->send(200, "text/plain", "ESP32 has been reset.");
  });

  server.on("/update", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "/update", "text/plain", "Go to update page.");
  });

  server.on("/getParameter", HTTP_GET, [](AsyncWebServerRequest * request) {
    Read_parameter_from_SPIFFS();
    String LocalIP = String() + "IP:" + WiFi.localIP()[0] + "." + WiFi.localIP()[1] + "." + WiFi.localIP()[2] + "." + WiFi.localIP()[3];
    DynamicJsonDocument  doc(1024);
    JsonObject control =  doc.to<JsonObject>();;
    control["mcName"] = mcName;
    control["location"] = mcLoca;
    control["deviceIP"] = LocalIP;
    control["deviceSN"] = String(serialNo);
    JsonObject objData = doc.createNestedObject("data");
    objData["runtime"] = datart;
    objData["counter"] = datact;
    objData["event"] = dataev;
    objData["mcstatus"] = statusCheckRT;
    control["RSSI"] = WiFi.RSSI();
    char bufferSend[300] ;
    serializeJson(doc, bufferSend);
    client.publish(mqtt_json, bufferSend, true);
    request->send(200, "text/plain", "Parameter has sent.");
  });

  server.on("/showpara", HTTP_GET, [](AsyncWebServerRequest * request) {
    String str = "";
    int paramsNr = request->params();
    str = "";
    int wf = 0;
    String strIP = "";

    DynamicJsonDocument  doc(1024);
    JsonObject pconvert =  doc.to<JsonObject>();;

    AsyncWebParameter* p = request->getParam(0);
    AsyncWebParameter* p1 = request->getParam(1);
    AsyncWebParameter* p2 = request->getParam(2);
    AsyncWebParameter* p3 = request->getParam(3);
    AsyncWebParameter* p4 = request->getParam(4);
    AsyncWebParameter* p5 = request->getParam(5);
    AsyncWebParameter* p6 = request->getParam(6);
    AsyncWebParameter* p7 = request->getParam(7);
    AsyncWebParameter* p8 = request->getParam(8);

    pconvert[p->name()] = p->value();
    pconvert[p1->name()] = p1->value();
    pconvert[p2->name()] = p2->value();
    pconvert[p3->name()] = p3->value();
    pconvert[p4->name()] = p4->value();
    pconvert[p5->name()] = p5->value();
    pconvert[p6->name()] = p6->value();
    pconvert[p7->name()] = p7->value();
    pconvert[p8->name()] = p8->value();

    char bufSend[300];
    serializeJson(doc, bufSend);

    wf = writeToFile("setting.json", bufSend); //////Write to SPIFFS
    Read_parameter_from_SPIFFS();
    request->send(200, "text/plain", "Parameter has been set!\n\n" + String(str));
  });

}


void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(ssid.c_str(), password.c_str());
}

void device_restart() {
  analog_value = analogRead(ANALOG_PIN_0);
  //  Serial.println(analog_value);
  int pretimer = 0;
  int dummy = 0;
  int justcheck = 0;
  const long targettimer = 1000;
  unsigned long crtimer = millis();
  if (3300 > analog_value && analog_value > 2650) {
    justcheck = 1;
  }
  if (crtimer - pretimer >= targettimer && justcheck == 1) {
    pretimer = crtimer;
    dummy = dummy + 1;
    ESP.restart();
  }
  else {
    justcheck = 0;
    dummy = 0;
  }
}


void setup() {
  snprintf(serialNo, sizeof(serialNo), "SN:m06-%04X%08X", chip, (uint32_t)chipid);
  Serial.begin(115200);

  bool success = SPIFFS.begin();

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawLogBuffer(0, 0);
  display.display();

  pinMode(ledstate, OUTPUT);
  pinMode(ledmqstate, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(counterpin, INPUT_PULLUP);

  //  digitalWrite(buttonPin,HIGH);
  //  digitalWrite(counterpin,HIGH);

  if (!SPIFFS.begin()) {
    Serial.println("Error while mounting SPIFFS");
    return;
  }


  WiFi.disconnect(true);
  delay(100);
  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  //  Read_parameter_from_SPIFFS();
  WiFi.begin((const char*)ssid.c_str(), (const char*)password.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(ledstate, HIGH);
    delay(100);
    digitalWrite(ledstate, LOW);
    delay(1000);
  }

  update_display();

  //----------------------- MQTT -----------------------//
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  //----------------------- MQTT -----------------------//

  Writing_parameter_from_web_server();
  AsyncElegantOTA.begin(&server);   ////OTA
  server.begin();
}


void loop() {
  AsyncElegantOTA.loop();   ////OTA
  //  Read_parameter_from_SPIFFS();

  if (!client.connected()) {
    if (millis() - previousMillis > interval ) {
      previousMillis = millis();
      Serial.print("MQTT connection... ");
      String clientId = String() + "CNCm06R2-" + String(serialNo) + "-";
      clientId += String(random(0xffff), HEX);
      if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass, willTopic, 0, true, willMessage))  {
        client.publish(willTopic, willMessage, true);
        Serial.println("Connected");
      }
      else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
      }
    }
  }

  counter_state();
  run_time_state();
  update_display();
  client.loop();
}

void update_display() {   //  display.drawString(0, 0, "CNC-m06");
  display.clear();

  String LocalIP = String() + "IP:" + WiFi.localIP()[0] + "." + WiFi.localIP()[1] + "." + WiFi.localIP()[2] + "." + WiFi.localIP()[3];
  display.drawString(0, 0, LocalIP);

  adcString = String(serialNo);
  display.drawString(0, 15, adcString);

  String WiFiRSSI = String() + "RSSI:" + WiFi.RSSI();
  display.drawString(0, 30, WiFiRSSI);

  String StChk = String() + "Status:" + statusCheckRT;
  display.drawString(0, 45, StChk);

  adcString = String();
  display.drawString(30, 45, adcString);

  display.display();
}

void publishData() {
  String LocalIP = String() + "IP:" + WiFi.localIP()[0] + "." + WiFi.localIP()[1] + "." + WiFi.localIP()[2] + "." + WiFi.localIP()[3];
  DynamicJsonDocument  doc(1024);
  JsonObject control =  doc.to<JsonObject>();;
  control["mcName"] = mcName;
  control["location"] = mcLoca;
  control["deviceIP"] = LocalIP;
  control["deviceSN"] = String(serialNo);
  JsonObject objData = doc.createNestedObject("data");
  objData["runtime"] = datart;
  objData["counter"] = datact;
  objData["event"] = dataev;
  objData["mcstatus"] = statusCheckRT;
  control["RSSI"] = WiFi.RSSI();
  char bufferSend[300] ;
  serializeJson(doc, bufferSend);
  client.publish(mqtt_json, bufferSend, true);
}

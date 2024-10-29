#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncHTTPUpdateServer.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <PicoMQTT.h>
#include <ESPmDNS.h>

AsyncWebServer server(80);
ESPAsyncHTTPUpdateServer updateServer;
PicoMQTT::Client mqtt("10.0.0.77");

Adafruit_ADS1115 ads;
float fwd = 0.0f;
float rev = 0.0f;
float vswr = 1.0f;
float vswr_max = 1.5;
float rev_max = 1.0;
float fwd_max = 50.0;
float fwd_min = 0.001; // roughly 1mW
float rev_min = 0.1;

static const float multiplier = 0.1875f;
static const adsGain_t gain = GAIN_TWO;
static const float callibration = 2.7/6.9; //12.85*2.7*1.2/6.9; //12.85*6.9/2.7;
static const int RELAY_CLOSE = 32;
bool active_key = true;
bool auto_unlock = true;
int cooldown = 2000;

unsigned int previousMillis = 0;
unsigned const int interval = 30*1000;

void addFloat(JsonDocument *doc, String name, float value) {
    (*doc)[name].set(String(value,3));
}

void addInt(JsonDocument *doc, String name, int value) {
    (*doc)[name].set(value);
}

void addString(JsonDocument *doc, String name, String value) {
    (*doc)[name].set(value);
}

void addBool(JsonDocument *doc, String name, bool value) {
  if (value) {
    (*doc)[name].set("true");
  } else {
    (*doc)[name].set("false");
  }
}

String getStatusObject() {
  StaticJsonDocument<1024> doc;
  addFloat(&doc,"fwd",fwd);
  addFloat(&doc,"rev",rev);
  addFloat(&doc,"vswr",vswr);
  addBool(&doc,"active_key",active_key);
  addBool(&doc,"auto_unlock",auto_unlock);
  char buffer[1024];
  serializeJson(doc, buffer);
  return String(buffer);
}

void readSwr() {
    char msg[1024];
    int16_t fwd_d_raw = ads.readADC_Differential_0_1(); //ads.readADC_SingleEnded(0);
    int16_t rev_d_raw = ads.readADC_Differential_2_3(); //ads.readADC_SingleEnded(2);
    if (fwd_d_raw == fwd_d_raw && rev_d_raw == rev_d_raw) {
        float fwd_v = ads.computeVolts(fwd_d_raw) * callibration;
        float rev_v = ads.computeVolts(rev_d_raw) * callibration;
        fwd = 50*fwd_v*fwd_v;
        rev = 50*rev_v*rev_v;
        if (fwd > fwd_min) {
          float q = rev_v/fwd_v;
          vswr = (1.0f+q)/(1.0f-q);
        } else {
          vswr = 1.0f;
        }
        mqtt.publish("esp32swr/raw",getStatusObject());
    }
}

void lock() {
    active_key = false;
    Serial.println("locking...");
    digitalWrite(RELAY_CLOSE,true);
    if (auto_unlock) delay(cooldown);
}

void unlock() {
    active_key = true;
    digitalWrite(RELAY_CLOSE,false);
}

void checkLimits() {
    if (fwd > fwd_max || rev > rev_max || (vswr > vswr_max && rev > rev_min)) {
        lock();
    } else if (!active_key && auto_unlock) {
        unlock();
    }
}

void getStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;
  addFloat(&doc,"fwd",fwd);
  addFloat(&doc,"rev",rev);
  addFloat(&doc,"vswr",vswr);
  addBool(&doc,"active_key",active_key);
  addBool(&doc,"auto_unlock",auto_unlock);
  char buffer[1024];
  serializeJson(doc, buffer);
  
  request->send(200, "application/json", buffer);
}

void getSettings(AsyncWebServerRequest *request) {
    StaticJsonDocument<1024> doc;
    char buffer[1024];
    addFloat(&doc,"fwd_max",fwd_max);
    addFloat(&doc,"rev_max",rev_max);
    addFloat(&doc,"vswr_max",vswr_max);
    addFloat(&doc,"rev_min",rev_min);
    addInt(&doc,"cooldown",cooldown);
    addBool(&doc,"auto_unlock",auto_unlock);
    serializeJson(doc, buffer);
    request->send(200, "application/json", buffer);
}

bool handleSettings(AsyncWebServerRequest *request, uint8_t *datas) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc,datas);
    if (doc.containsKey("fwd_max")) {
        fwd_max = doc["fwd_max"].as<float>();
    }
    if (doc.containsKey("rev_max")) {
        rev_max = doc["rev_max"].as<float>();
    }
    if (doc.containsKey("rev_min")) {
        rev_max = doc["rev_min"].as<float>();
    }
    if (doc.containsKey("vswr_max")) {
        vswr_max = doc["vswr_max"].as<float>();
    }
    if (doc.containsKey("cooldown")) {
        cooldown = doc["cooldown"].as<int>();
    }
    if (doc.containsKey("auto_unlock")) {
        auto_unlock = doc["auto_unlock"].as<bool>();
    }
    return true;
}

void setupApi() {
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    getStatus(request);
  });
  server.on("/api/settings",HTTP_GET,[](AsyncWebServerRequest *request){
    getSettings(request);
  });
  server.on("/api/settings",HTTP_POST,[](AsyncWebServerRequest *request){
    getSettings(request);
  },NULL,
  [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    if (!handleSettings(request,data)) {
        request->send(500,"text/plain","Error updating settings");
    }
  });
  server.on("/api/unlock", HTTP_GET, [](AsyncWebServerRequest *request){
    unlock();
    getStatus(request);
  });
  server.on("/api/disable", HTTP_GET, [](AsyncWebServerRequest *request){
    auto_unlock = false;
    lock();
    getStatus(request);
  });
  server.on("/api/enable", HTTP_GET, [](AsyncWebServerRequest *request){
    auto_unlock = true;
    getStatus(request);
  });
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/s/", SPIFFS, "/");
  updateServer.setup(&server);
  updateServer.setup(&server,OTA_USER,OTA_PASSWORD);
  server.begin();
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(1500); 


  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // it is a good practice to make sure your code sets wifi mode how you want it.

  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
  // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
  // then goes into a blocking loop awaiting configuration and will return success result

  bool res;
  res = wm.autoConnect(); // auto generated AP name from chipid
  // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
  // res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

  if(!res) {
      Serial.println("Failed to connect");
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("Connected...yeey :)");
  }

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  pinMode(RELAY_CLOSE,OUTPUT);
  ads.setGain(gain);
  ads.setDataRate(RATE_ADS1115_860SPS);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    ESP.restart();
  }

  mqtt.subscribe("#", [](const char * topic, const char * payload) {
    Serial.printf("Received message in topic '%s': %s\n", topic, payload);
  });

  mqtt.begin();

  if (!MDNS.begin("esp32swr")) {   // Set the hostname to "esp32swr.local"
    Serial.println("Error setting up MDNS responder!");
    delay(1000);
  } else {
    Serial.println("mDNS responder started");
  }
  MDNS.addService("http", "tcp", 80);
  setupApi();
}


void loop() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
  readSwr();
  checkLimits();
  mqtt.loop();
  delay(500);
}
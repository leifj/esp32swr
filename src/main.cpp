#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>

AsyncWebServer server(80);
Adafruit_ADS1115 ads;
float fwd = 0;
float rev = 0;
float vswr = 0;
float vswr_max = 1.5;
float rev_max = 1.0;
float fwd_max = 50.0;

static const float multiplier = 0.1875f;
static const adsGain_t gain = GAIN_TWOTHIRDS;
static const float callibration = 12.85*6.9/2.7;
static const int RELAY_CLOSE = 32;
bool active_key = true;
bool auto_unlock = true;
int cooldown = 2000;

unsigned int previousMillis = 0;
unsigned const int interval = 30*1000;

void addFloat(JsonDocument *doc, String name, float value) {
    (*doc)[name].set(value);
}

void addInt(JsonDocument *doc, String name, int value) {
    (*doc)[name].set(value);
}

void addBool(JsonDocument *doc, String name, bool value) {
  if (value) {
    (*doc)[name].set("true");
  } else {
    (*doc)[name].set("false");
  }
}

void readSwr() {
    int16_t fwd_d_raw = ads.readADC_SingleEnded(0);
    int16_t rev_d_raw = ads.readADC_SingleEnded(2);
    if (fwd_d_raw == fwd_d_raw && rev_d_raw == rev_d_raw) {
        fwd = ads.computeVolts(fwd_d_raw) * callibration;
        rev = ads.computeVolts(rev_d_raw) * callibration;
        if (fwd < 0) fwd = 0.0;
        if (rev < 0) rev = 0.0;
        float q = sqrtf(rev/fwd);
        vswr = (1.0f+q)/(1.0f-q);
        Serial.printf("fwd=%f, rev=%f, q=%f, vswr=%f\n",fwd,rev,q,vswr);
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
    if (fwd > fwd_max || rev > rev_max || vswr > vswr_max || vswr < 1.0f) {
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
  char buffer[1024];
  serializeJson(doc, buffer);
  
  request->send(200, "application/json", buffer);
}

void getSettings(AsyncWebServerRequest *request) {
    Serial.println("getSettings");
    StaticJsonDocument<1024> doc;
    char buffer[1024];
    addFloat(&doc,"fwd_max",fwd_max);
    addFloat(&doc,"rev_max",rev_max);
    addFloat(&doc,"vswr_max",vswr_max);
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
  server.on("/api/unlock", HTTP_GET, [](AsyncWebServerRequest *request){
    unlock();
    getStatus(request);
  });
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    Serial.printf("onRequestBody %s\n",data);
    if (request->url() == "/api/settings" && request->method() == HTTP_POST) {
        if (!handleSettings(request,data)) {
            request->send(500,"text/plain","Error updating settings");
        }
        getSettings(request);
    }
  });
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/static/", SPIFFS, "/");
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
  delay(1000);
}
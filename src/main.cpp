#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Update.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <PicoMQTT.h>
#include <ESPmDNS.h>

#define FW_VERSION "1.1"

AsyncWebServer server(80);
PicoMQTT::Client mqtt("10.0.0.3");

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

// --- OTA -----------------------------------------------------------------
// Custom handler instead of ESPAsyncHTTPUpdateServer. The library starts a
// filesystem update with SPIFFS.totalBytes(), which is smaller than the
// partition; the full-size spiffs.bin is then truncated and the filesystem is
// corrupted. Using UPDATE_SIZE_UNKNOWN makes Update use the partition size.
static bool otaAuthorized = false;
static bool otaFilesystem = false;
static bool otaFailed = false;
static String otaError;

static void otaFail() {
  otaFailed = true;
  otaError = Update.errorString();
  Serial.printf("OTA error: %s\n", otaError.c_str());
  Update.abort();
  if (otaFilesystem) {
    // Best effort: the image may be unusable after a partial write.
    SPIFFS.begin(false);
  }
}

static const char otaPage[] PROGMEM = R"rawliteral(
<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>esp32swr update</title>
<style>body{font-family:system-ui,sans-serif;max-width:34rem;margin:3rem auto;padding:0 1rem}
fieldset{margin:1rem 0}button{margin-top:.5rem}</style>
<h1>esp32swr update</h1>
<form method="post" action="/update?name=firmware" enctype="multipart/form-data">
<fieldset><legend>Firmware</legend><input type="file" name="firmware" accept=".bin" required><br><button>Upload firmware</button></fieldset></form>
<form method="post" action="/update?name=filesystem" enctype="multipart/form-data">
<fieldset><legend>Filesystem</legend><input type="file" name="filesystem" accept=".bin" required><br><button>Upload filesystem</button></fieldset></form>
<p><a href="/">Back to status</a></p>
)rawliteral";

static void otaRequest(AsyncWebServerRequest *request) {
  if (!otaAuthorized) {
    request->requestAuthentication();
    return;
  }
  if (otaFailed || Update.hasError()) {
    String msg = F("Update failed: ");
    msg += otaError.length() ? otaError : Update.errorString();
    request->send(500, "text/plain", msg);
    return;
  }
  request->send(200, "text/plain", F("Update Success! Rebooting..."));
  delay(1000);
  ESP.restart();
}

static void otaUpload(AsyncWebServerRequest *request, String filename,
                      size_t index, uint8_t *data, size_t len, bool final) {
  if (index == 0) {
    otaFailed = false;
    otaError = "";
    otaFilesystem = false;
    otaAuthorized = request->authenticate(OTA_USER, OTA_PASSWORD);
    if (!otaAuthorized) {
      return;
    }

    String kind = "firmware";
    if (request->hasParam("name")) {
      kind = request->getParam("name")->value();
    }
    otaFilesystem = (kind == "filesystem");

    if (otaFilesystem) {
      // Unmount before writing raw sectors: a mounted SPIFFS keeps stale
      // metadata in RAM and will not mount cleanly after the raw write.
      SPIFFS.end();
    }

    if (!Update.begin(UPDATE_SIZE_UNKNOWN, otaFilesystem ? U_SPIFFS : U_FLASH)) {
      otaFail();
      return;
    }
  }
  if (otaAuthorized && !otaFailed) {
    if (len && !Update.write(data, len)) {
      otaFail();
      return;
    }
    if (final && !Update.end(true)) {
      otaFail();
      return;
    }
  }
}

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
        if (fwd > fwd_min) {
          mqtt.publish("esp32swr/raw",getStatusObject());
        }
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
  addString(&doc,"version",FW_VERSION);
  doc["uptime"] = millis() / 1000;
  doc["rssi"] = WiFi.RSSI();
  doc["heap"] = ESP.getFreeHeap();
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
        rev_min = doc["rev_min"].as<float>();
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
  // Registered after the static handlers: ESPAsyncWebServer checks handlers
  // newest-first, so these win over the serveStatic fallbacks.
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(204);  // no favicon; stop browsers from re-fetching errors
  });
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "not found");
  });
  // Authenticated OTA at /update (firmware + filesystem)
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->authenticate(OTA_USER, OTA_PASSWORD)) {
      return request->requestAuthentication();
    }
    request->send_P(200, "text/html", otaPage);
  });
  server.on("/update", HTTP_POST, otaRequest, otaUpload);
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
      // Post-connect ritual, same as the ATU-100 / ESPels: keep the radio
      // out of modem sleep so HTTP responses stay fast on a marginal link.
      WiFi.setHostname("esp32swr");
      WiFi.setAutoReconnect(true);
      WiFi.persistent(true);
      WiFi.setSleep(false);
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

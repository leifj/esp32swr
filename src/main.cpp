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

#define FW_VERSION "1.2"

AsyncWebServer server(80);
PicoMQTT::Client mqtt("10.0.0.3");

Adafruit_ADS1115 ads;

// Latest averaged reading (published on the API and MQTT).
float fwd = 0.0f;
float rev = 0.0f;
float vswr = 1.0f;
bool reading_valid = false;

float vswr_max = 1.5;
float rev_max = 1.0;
float fwd_max = 50.0;
float fwd_min = 0.001; // roughly 1mW
float rev_min = 0.1;

// --- measurement configuration -------------------------------------------
static const adsGain_t gain = GAIN_TWO;   // +-2.048 V full scale
// Per-channel detector slope. Forward and reverse couplers have different
// coupling factors, so they get independent constants.
static const float CAL_FWD = 2.7/6.9;
static const float CAL_REV = 2.7/6.9;
static const float POWER_SCALE = 50.0f;   // W per volt^2 at the detector
static const int ADC_WINDOW = 16;         // samples averaged per reading
static const unsigned long LOOP_INTERVAL_MS = 100;    // ~7-10 Hz
static const unsigned long PUBLISH_INTERVAL_MS = 100;

// Zero-RF baselines measured at boot (volts after calibration).
float off_fwd = 0.0f;
float off_rev = 0.0f;

// One measurement window: average powers plus the forward peak, which is
// what the protection path uses (a single average sample can miss an
// overshoot between loop passes).
struct Reading {
  float fwd;
  float rev;
  float fwdPeak;
  float vswr;
  bool valid;
};

static const int RELAY_CLOSE = 32;
bool active_key = true;
bool auto_unlock = true;
int cooldown = 2000;
unsigned long unlock_at = 0;

unsigned int previousMillis = 0;
unsigned const int interval = 30*1000;
unsigned long previousPublish = 0;

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
  addBool(&doc,"valid",reading_valid);
  char buffer[1024];
  serializeJson(doc, buffer);
  return String(buffer);
}

// Average both channels with no RF to remove the detector offset. If the
// readings look like real RF (someone transmitting at boot), keep zero
// offsets rather than calibrating against a live signal.
void calibrateOffsets() {
    const int N = 32;
    float fwdSum = 0.0f;
    float revSum = 0.0f;
    for (int i = 0; i < N; i++) {
        fwdSum += ads.computeVolts(ads.readADC_Differential_0_1()) * CAL_FWD;
        revSum += ads.computeVolts(ads.readADC_Differential_2_3()) * CAL_REV;
    }
    float fwdAvg = fwdSum / N;
    float revAvg = revSum / N;
    if (fabsf(fwdAvg) < 0.10f && fabsf(revAvg) < 0.10f) {
        off_fwd = fwdAvg;
        off_rev = revAvg;
        Serial.printf("Offset calibration: fwd %.4f V, rev %.4f V\n", off_fwd, off_rev);
    } else {
        Serial.println("Offset calibration skipped: RF detected at boot");
    }
}

// One measurement window: discard the first conversion after the mux change
// (ADS1115 settling), then average ADC_WINDOW samples per channel. Negative
// samples are clamped so noise/offset can never square into phantom power,
// and the forward peak is tracked for the protection path.
Reading measureOnce() {
    ads.readADC_Differential_0_1();
    ads.readADC_Differential_2_3();

    float fwdSum = 0.0f;
    float revSum = 0.0f;
    float fwdPeak = 0.0f;
    for (int i = 0; i < ADC_WINDOW; i++) {
        // Detector polarity depends on the coupler winding orientation, so
        // take the magnitude after offset removal. Clamping to zero instead
        // would freeze a polarity-reversed channel at 0 W (and SWR at 1.0).
        float fwdV = fabsf(ads.computeVolts(ads.readADC_Differential_0_1()) * CAL_FWD - off_fwd);
        float revV = fabsf(ads.computeVolts(ads.readADC_Differential_2_3()) * CAL_REV - off_rev);
        float fwdW = POWER_SCALE * fwdV * fwdV;
        float revW = POWER_SCALE * revV * revV;
        fwdSum += fwdW;
        revSum += revW;
        if (fwdW > fwdPeak) fwdPeak = fwdW;
    }
    Reading r;
    r.fwd = fwdSum / ADC_WINDOW;
    r.rev = revSum / ADC_WINDOW;
    r.fwdPeak = fwdPeak;
    r.valid = (r.fwd > fwd_min);
    if (r.valid) {
        // SWR from the power ratio. q >= 1 means reflected >= forward (open
        // or short load, or a noisy sample): report the sentinel instead of
        // dividing by zero and producing inf/NaN.
        float q = sqrtf(r.rev / r.fwd);
        r.vswr = (q >= 1.0f) ? 99.0f : (1.0f + q) / (1.0f - q);
    } else {
        r.vswr = 1.0f;
    }
    return r;
}

// Non-blocking lock: the relay closes immediately and the auto-unlock
// deadline is checked from loop(), so measurements, the web UI and MQTT
// keep running during a fault.
void lock(unsigned long now) {
    if (active_key) {
        Serial.println("locking...");
    }
    active_key = false;
    digitalWrite(RELAY_CLOSE, HIGH);
    if (auto_unlock) {
        unlock_at = now + cooldown;
    }
}

void unlock() {
    active_key = true;
    digitalWrite(RELAY_CLOSE, LOW);
}

void checkLimits(const Reading &r, unsigned long now) {
    bool fault = r.fwdPeak > fwd_max ||
                 (r.valid && (r.rev > rev_max || (r.vswr > vswr_max && r.rev > rev_min)));
    if (fault) {
        lock(now);
    } else if (!active_key && auto_unlock && (long)(now - unlock_at) >= 0) {
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
  addBool(&doc,"valid",reading_valid);
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
    lock(millis());
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
  calibrateOffsets();

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

  Reading r = measureOnce();
  fwd = r.fwd;
  rev = r.rev;
  vswr = r.vswr;
  reading_valid = r.valid;

  checkLimits(r, currentMillis);

  // Publish at the loop rate (~7-10 Hz) while there is measurable forward
  // power; quisk marks the SWR label stale after 1.5 s of silence.
  if (r.valid && (currentMillis - previousPublish >= PUBLISH_INTERVAL_MS)) {
    previousPublish = currentMillis;
    mqtt.publish("esp32swr/raw", getStatusObject());
  }

  mqtt.loop();
  delay(LOOP_INTERVAL_MS);
}

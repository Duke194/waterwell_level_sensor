#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <numeric>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <algorithm>
#include <vector>

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "settings.h"

#include <Preferences.h>

#define echoPin 5
#define trigPin 26
#define LED_BUILTIN 2
#define uS_TO_S_FACTOR 1000000
#define MQTT_RETRY_LIMIT 5
#define NO_SLEEP_MEASURE_INTERVAL_MS 10000
#define MQTT_RESTART_COMMAND_TOPIC "carport/sensor/rainwater_well_1/control/restart"
#define MQTT_IP_STATE_TOPIC "carport/sensor/rainwater_well_1/status/ip"
#define MQTT_VOLUME_L_TOPIC "carport/sensor/rainwater_well_1/metrics/volume_l"
#define MQTT_VOLUME_M3_TOPIC "carport/sensor/rainwater_well_1/metrics/volume_m3"
#define MQTT_DISTANCE_CM_TOPIC "carport/sensor/rainwater_well_1/metrics/distance_cm"
#define MQTT_AREA_M2_TOPIC "carport/sensor/rainwater_well_1/metrics/area_m2"

#ifndef BATTERY_MODE_DEFAULT
#define BATTERY_MODE_DEFAULT true
#endif

#ifndef PLAUSIBILITY_OUTLIER_DELTA_CM
#define PLAUSIBILITY_OUTLIER_DELTA_CM 6
#endif

#ifndef PLAUSIBILITY_MAX_STEP_CM
#define PLAUSIBILITY_MAX_STEP_CM 20
#endif

#ifndef PLAUSIBILITY_RELAX_RATE_CM_PER_MIN
#define PLAUSIBILITY_RELAX_RATE_CM_PER_MIN 8
#endif

#ifndef PLAUSIBILITY_DISABLE_AFTER_MIN
#define PLAUSIBILITY_DISABLE_AFTER_MIN 10
#endif

#ifndef BELOW_MIN_HITS_THRESHOLD
#define BELOW_MIN_HITS_THRESHOLD 3
#endif

#ifndef BELOW_MIN_STREAK_THRESHOLD
#define BELOW_MIN_STREAK_THRESHOLD 2
#endif

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);
WebServer webServer(80);
Preferences preferences;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int distance_prev = 0;
RTC_DATA_ATTR bool prev_wifistate = false;
RTC_DATA_ATTR int lastPlausibleDistance = 0;
RTC_DATA_ATTR uint32_t lastPlausibleBootCount = 0;
RTC_DATA_ATTR uint8_t belowMinCycleStreak = 0;

bool pingDone = false;
bool webMonitorRunning = false;
bool webMonitorForcedByFailure = false;
bool otaAuthFailed = false;
int mqttRetryCount = 0;
int lastMeasuredDistance = 0;
unsigned long lastWiFiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;
unsigned long lastNoSleepMeasureMs = 0;

constexpr size_t LOG_LINES = 40;
constexpr size_t LOG_LEN = 128;
char logBuffer[LOG_LINES][LOG_LEN];
size_t logHead = 0;
bool logWrapped = false;

struct RuntimeSettings
{
  int time_to_sleep;
  int heartbeat;
  int wifi_timeout;
  int sensor_timeout;
  int buffer_size;
  int sensor_range_min;
  int sensor_range_max;
  float calibration_offset;
  float area;
  float volume_well;
  char mqtt_host[64];
  int mqtt_port;
  char mqtt_user[64];
  char mqtt_pass[80];
  bool web_monitor_enabled;
  bool battery_mode;
} settings;

void startWebMonitor();
void stopWebMonitor();

void logLine(const char *fmt, ...)
{
  char line[LOG_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  Serial.println(line);
  strlcpy(logBuffer[logHead], line, LOG_LEN);
  logHead = (logHead + 1) % LOG_LINES;
  if (logHead == 0)
  {
    logWrapped = true;
  }
}

String getLogs()
{
  String out;
  const size_t count = logWrapped ? LOG_LINES : logHead;
  const size_t start = logWrapped ? logHead : 0;
  for (size_t i = 0; i < count; ++i)
  {
    const size_t idx = (start + i) % LOG_LINES;
    out += logBuffer[idx];
    out += "\n";
  }
  return out;
}

void loadDefaults()
{
  settings.time_to_sleep = TIME_TO_SLEEP;
  settings.heartbeat = HEARTBEAT;
  settings.wifi_timeout = WIFI_TIMEOUT;
  settings.sensor_timeout = SENSOR_TIMEOUT;
  settings.buffer_size = BUFFER_SIZE;
  settings.sensor_range_min = SENSOR_RANGE_MIN;
  settings.sensor_range_max = SENSOR_RANGE_MAX;
  settings.calibration_offset = CALIBRATION_OFFSET;
  settings.area = AREA;
  settings.volume_well = VOLUME_WELL;
  strlcpy(settings.mqtt_host, MQTT_HOST, sizeof(settings.mqtt_host));
  settings.mqtt_port = MQTT_PORT;
  strlcpy(settings.mqtt_user, MQTT_USER, sizeof(settings.mqtt_user));
  strlcpy(settings.mqtt_pass, MQTT_PASS, sizeof(settings.mqtt_pass));
  settings.web_monitor_enabled = true;
  settings.battery_mode = BATTERY_MODE_DEFAULT;
}

void loadSettingsFromNvs()
{
  loadDefaults();
  preferences.begin("wwls_cfg", true);
  settings.time_to_sleep = preferences.getInt("sleep", settings.time_to_sleep);
  settings.heartbeat = preferences.getInt("hb", settings.heartbeat);
  settings.wifi_timeout = preferences.getInt("wifi_to", settings.wifi_timeout);
  settings.sensor_timeout = preferences.getInt("sens_to", settings.sensor_timeout);
  settings.buffer_size = preferences.getInt("buf_sz", settings.buffer_size);
  settings.sensor_range_min = preferences.getInt("rng_min", settings.sensor_range_min);
  settings.sensor_range_max = preferences.getInt("rng_max", settings.sensor_range_max);
  settings.calibration_offset = preferences.getFloat("cal_off", settings.calibration_offset);
  settings.area = preferences.getFloat("area", settings.area);
  settings.volume_well = preferences.getFloat("vol", settings.volume_well);
  preferences.getString("mq_host", settings.mqtt_host, sizeof(settings.mqtt_host));
  settings.mqtt_port = preferences.getInt("mq_port", settings.mqtt_port);
  preferences.getString("mq_user", settings.mqtt_user, sizeof(settings.mqtt_user));
  preferences.getString("mq_pass", settings.mqtt_pass, sizeof(settings.mqtt_pass));
  settings.web_monitor_enabled = preferences.getBool("web_mon", settings.web_monitor_enabled);
  settings.battery_mode = preferences.getBool("bat_mode", settings.battery_mode);
  preferences.end();
}

void saveSettingsToNvs()
{
  preferences.begin("wwls_cfg", false);
  preferences.putInt("sleep", settings.time_to_sleep);
  preferences.putInt("hb", settings.heartbeat);
  preferences.putInt("wifi_to", settings.wifi_timeout);
  preferences.putInt("sens_to", settings.sensor_timeout);
  preferences.putInt("buf_sz", settings.buffer_size);
  preferences.putInt("rng_min", settings.sensor_range_min);
  preferences.putInt("rng_max", settings.sensor_range_max);
  preferences.putFloat("cal_off", settings.calibration_offset);
  preferences.putFloat("area", settings.area);
  preferences.putFloat("vol", settings.volume_well);
  preferences.putString("mq_host", settings.mqtt_host);
  preferences.putInt("mq_port", settings.mqtt_port);
  preferences.putString("mq_user", settings.mqtt_user);
  preferences.putString("mq_pass", settings.mqtt_pass);
  preferences.putBool("web_mon", settings.web_monitor_enabled);
  preferences.putBool("bat_mode", settings.battery_mode);
  preferences.end();
}

bool updateIntSetting(const char *payload, int &setting)
{
  const int value = atoi(payload);
  if (value != setting)
  {
    setting = value;
    return true;
  }
  return false;
}

bool updateFloatSetting(const char *payload, float &setting)
{
  const float value = atof(payload);
  if (value != setting)
  {
    setting = value;
    return true;
  }
  return false;
}

void publishMonitorState()
{
  if (mqttclient.connected())
  {
    mqttclient.publish(MQTT_MONITOR_STATE_TOPIC, webMonitorRunning ? "ON" : "OFF", true);
  }
}

void publishIpState()
{
  if (!mqttclient.connected() || WiFi.status() != WL_CONNECTED)
  {
    return;
  }
  const String ip = WiFi.localIP().toString();
  mqttclient.publish(MQTT_IP_STATE_TOPIC, ip.c_str(), true);
}

void publishMetricStates(int volume_l)
{
  if (!mqttclient.connected())
  {
    return;
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "%d", volume_l < 0 ? 0 : volume_l);
  mqttclient.publish(MQTT_VOLUME_L_TOPIC, buf, true);

  const float volume_m3 = (volume_l < 0 ? 0.0f : (float)volume_l / 1000.0f);
  dtostrf(volume_m3, 0, 3, buf);
  mqttclient.publish(MQTT_VOLUME_M3_TOPIC, buf, true);

  snprintf(buf, sizeof(buf), "%d", lastMeasuredDistance);
  mqttclient.publish(MQTT_DISTANCE_CM_TOPIC, buf, true);

  dtostrf(settings.area, 0, 3, buf);
  mqttclient.publish(MQTT_AREA_M2_TOPIC, buf, true);
}

void setWebMonitorEnabled(bool enable, bool persist)
{
  if (!enable)
  {
    // If user explicitly disables monitor, clear fallback latch.
    webMonitorForcedByFailure = false;
  }
  settings.web_monitor_enabled = enable;
  if (enable)
  {
    startWebMonitor();
  }
  else
  {
    stopWebMonitor();
  }
  if (persist)
  {
    saveSettingsToNvs();
  }
  publishMonitorState();
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char safePayload[128];
  const size_t copyLen = (length < sizeof(safePayload) - 1) ? length : sizeof(safePayload) - 1;
  memcpy(safePayload, payload, copyLen);
  safePayload[copyLen] = '\0';

  logLine("MQTT %s => %s", topic, safePayload);

  if (strncmp(topic, MQTT_SETTINGS_TOPIC, strlen(MQTT_SETTINGS_TOPIC)) == 0)
  {
    char *settingKey = topic + strlen(MQTT_SETTINGS_TOPIC) + 1;
    bool changed = false;
    if (strcmp(settingKey, "time_to_sleep") == 0)
      changed = updateIntSetting(safePayload, settings.time_to_sleep);
    else if (strcmp(settingKey, "heartbeat") == 0)
      changed = updateIntSetting(safePayload, settings.heartbeat);
    else if (strcmp(settingKey, "wifi_timeout") == 0)
      changed = updateIntSetting(safePayload, settings.wifi_timeout);
    else if (strcmp(settingKey, "sensor_timeout") == 0)
      changed = updateIntSetting(safePayload, settings.sensor_timeout);
    else if (strcmp(settingKey, "buffer_size") == 0)
      changed = updateIntSetting(safePayload, settings.buffer_size);
    else if (strcmp(settingKey, "sensor_range_min") == 0)
      changed = updateIntSetting(safePayload, settings.sensor_range_min);
    else if (strcmp(settingKey, "sensor_range_max") == 0)
      changed = updateIntSetting(safePayload, settings.sensor_range_max);
    else if (strcmp(settingKey, "calibration_offset") == 0)
      changed = updateFloatSetting(safePayload, settings.calibration_offset);
    else if (strcmp(settingKey, "area") == 0)
      changed = updateFloatSetting(safePayload, settings.area);
    else if (strcmp(settingKey, "volume_well") == 0)
      changed = updateFloatSetting(safePayload, settings.volume_well);
    else if (strcmp(settingKey, "battery_mode") == 0)
    {
      const bool newBatteryMode = (strcasecmp(safePayload, "ON") == 0 || strcmp(safePayload, "1") == 0 ||
                                   strcasecmp(safePayload, "TRUE") == 0);
      changed = (newBatteryMode != settings.battery_mode);
      settings.battery_mode = newBatteryMode;
    }

    if (changed)
    {
      saveSettingsToNvs();
      logLine("Setting %s updated", settingKey);
    }
  }
  else if (strcmp(topic, MQTT_SETTINGS_TOPIC "/ping") == 0)
  {
    pingDone = true;
  }
  else if (strcmp(topic, MQTT_MONITOR_COMMAND_TOPIC) == 0)
  {
    const bool enable = strcasecmp(safePayload, "ON") == 0 || strcmp(safePayload, "1") == 0;
    setWebMonitorEnabled(enable, true);
    logLine("Web monitor set via MQTT: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(topic, MQTT_RESTART_COMMAND_TOPIC) == 0)
  {
    logLine("Restart requested via MQTT");
    delay(300);
    ESP.restart();
  }
}

int measure_distance()
{
  std::vector<int> distances;
  int retries = 0;
  int belowMinHits = 0;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  while ((distances.size() < (size_t)settings.buffer_size && retries < settings.sensor_timeout) || CONT_MEASURE)
  {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);

    const unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    const int distance = round(float(duration) * 0.0343f / 2.0f + settings.calibration_offset);

    // Count below-min only for real echoes, not for pulse timeout (duration==0).
    if (duration > 0 && distance <= settings.sensor_range_min)
    {
      belowMinHits++;
    }

    if (distance > settings.sensor_range_min && distance < settings.sensor_range_max)
    {
      logLine("Distance: %d cm", distance);
      distances.push_back(distance);
    }
    delay(RETRY_DELAY);
    ++retries;
  }

  const int dynamicBelowMinThreshold = (settings.buffer_size / 2) > BELOW_MIN_HITS_THRESHOLD
                                           ? (settings.buffer_size / 2)
                                           : BELOW_MIN_HITS_THRESHOLD;
  const int minValidForOverride = (dynamicBelowMinThreshold / 2) > 0 ? (dynamicBelowMinThreshold / 2) : 1;
  const bool belowMinCycle = (belowMinHits >= dynamicBelowMinThreshold);
  if (belowMinCycle)
  {
    if (belowMinCycleStreak < 255)
    {
      belowMinCycleStreak++;
    }
  }
  else if (belowMinHits == 0)
  {
    belowMinCycleStreak = 0;
  }

  if (distances.empty())
  {
    if (belowMinCycleStreak >= BELOW_MIN_STREAK_THRESHOLD || belowMinHits >= dynamicBelowMinThreshold)
    {
      logLine("Below-min detected (hits=%d, streak=%u) -> forcing distance 0 cm", belowMinHits, belowMinCycleStreak);
      return 0;
    }
    logLine("No valid measurements received.");
    return 0;
  }

  std::sort(distances.begin(), distances.end());
  const int median = distances[distances.size() / 2];

  std::vector<int> plausible;
  plausible.reserve(distances.size());
  for (int value : distances)
  {
    if (abs(value - median) <= PLAUSIBILITY_OUTLIER_DELTA_CM)
    {
      plausible.push_back(value);
    }
  }

  int filtered = median;
  if (!plausible.empty())
  {
    filtered = round(std::accumulate(plausible.begin(), plausible.end(), 0.0) / plausible.size());
  }

  uint32_t elapsedSec = 0;
  if (lastPlausibleBootCount > 0 && bootCount > lastPlausibleBootCount)
  {
    elapsedSec = (uint32_t)(bootCount - lastPlausibleBootCount) * (uint32_t)settings.time_to_sleep;
  }
  else
  {
    elapsedSec = (uint32_t)(NO_SLEEP_MEASURE_INTERVAL_MS / 1000UL);
  }

  const uint32_t elapsedMin = elapsedSec / 60U;
  int dynamicMaxStep = PLAUSIBILITY_MAX_STEP_CM + (int)(elapsedMin * PLAUSIBILITY_RELAX_RATE_CM_PER_MIN);
  const bool clampDisabledByTime = elapsedMin >= PLAUSIBILITY_DISABLE_AFTER_MIN;

  if (!clampDisabledByTime && lastPlausibleDistance > 0 && abs(filtered - lastPlausibleDistance) > dynamicMaxStep)
  {
    logLine("Plausibility clamp: raw=%d cm prev=%d cm max_step=%d cm elapsed_min=%u",
            filtered, lastPlausibleDistance, dynamicMaxStep, (unsigned int)elapsedMin);
    filtered = lastPlausibleDistance;
  }

  const bool enoughValidDistances = (int)distances.size() >= minValidForOverride;
  if (!enoughValidDistances && (belowMinCycleStreak >= BELOW_MIN_STREAK_THRESHOLD || belowMinHits >= dynamicBelowMinThreshold))
  {
    logLine("Below-min override (hits=%d, streak=%u) -> forcing distance 0 cm", belowMinHits, belowMinCycleStreak);
    filtered = 0;
  }
  else
  {
    belowMinCycleStreak = 0;
  }

  lastPlausibleDistance = filtered;
  lastPlausibleBootCount = (uint32_t)bootCount;
  logLine("Average distance: %d cm (median=%d, kept=%d/%d)",
          filtered, median, (int)plausible.size(), (int)distances.size());

  const int volume_l_empty = round((filtered * 1e-1f) * (settings.area * 1e2f));
  int volume_l = round(settings.volume_well - volume_l_empty);
  if (volume_l < 0)
  {
    volume_l = 0;
  }
  logLine("Calculated volume: %d L (%.3f m3)", volume_l, (float)volume_l / 1000.0f);
  return filtered;
}

bool setup_wifi()
{
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  logLine("Connecting WiFi: %s", WIFI_SSID);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < settings.wifi_timeout * 10)
  {
    ++retries;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    logLine("WiFi failed");
    prev_wifistate = false;
    return false;
  }

  prev_wifistate = true;
  logLine("WiFi connected IP=%s", WiFi.localIP().toString().c_str());
  return true;
}

void publish_discovery_message()
{
  static StaticJsonDocument<768> doc;
  static char buffer[768];

  auto add_device = [&](JsonObject device)
  {
    JsonArray identifiers = device.createNestedArray("identifiers");
    identifiers.add(MQTT_CLIENT_ID);
    device["name"] = "Waterwell Level Sensor";
    device["manufacturer"] = "DIY";
    device["model"] = "ESP32 Waterwell";
  };

  auto publish_doc = [&](const char *topic)
  {
    const size_t payloadLen = measureJson(doc);
    const size_t written = serializeJson(doc, buffer, sizeof(buffer));
    const bool ok = mqttclient.publish(topic, buffer, true);
    logLine("Discovery publish: %s topic=%s state=%d len=%u written=%u",
            ok ? "OK" : "FAILED",
            topic,
            mqttclient.state(),
            (unsigned int)payloadLen,
            (unsigned int)written);
  };

  doc.clear();
  doc["name"] = "Waterwell Volume";
  doc["object_id"] = String(MQTT_CLIENT_ID) + "_volume";
  doc["state_topic"] = MQTT_STATE_TOPIC;
  doc["unit_of_measurement"] = "L";
  doc["value_template"] = "{{ value_json.volume }}";
  doc["icon"] = "mdi:water";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_volume";
  doc["availability_topic"] = MQTT_WILL_TOPIC;
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  add_device(doc.createNestedObject("device"));
  publish_doc(MQTT_DISCOVERY_TOPIC);

  doc.clear();
  doc["name"] = "Waterwell Neustart";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_restart";
  doc["command_topic"] = MQTT_RESTART_COMMAND_TOPIC;
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/button/rainwater_well_1_restart/config");

  doc.clear();
  doc["name"] = "Waterwell Web Monitor";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_monitor";
  doc["command_topic"] = MQTT_MONITOR_COMMAND_TOPIC;
  doc["state_topic"] = MQTT_MONITOR_STATE_TOPIC;
  doc["payload_on"] = "ON";
  doc["payload_off"] = "OFF";
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/switch/rainwater_well_1_monitor/config");

  doc.clear();
  doc["name"] = "Waterwell IP Adresse";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_ip";
  doc["state_topic"] = MQTT_IP_STATE_TOPIC;
  doc["icon"] = "mdi:ip-network";
  doc["entity_category"] = "diagnostic";
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/sensor/rainwater_well_1_ip/config");

  doc.clear();
  doc["name"] = "Waterwell Volumen Liter";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_volume_l";
  doc["state_topic"] = MQTT_VOLUME_L_TOPIC;
  doc["unit_of_measurement"] = "L";
  doc["device_class"] = "volume";
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/sensor/rainwater_well_1_volume_l/config");

  doc.clear();
  doc["name"] = "Waterwell Volumen m3";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_volume_m3";
  doc["state_topic"] = MQTT_VOLUME_M3_TOPIC;
  doc["unit_of_measurement"] = "m3";
  doc["device_class"] = "volume";
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/sensor/rainwater_well_1_volume_m3/config");

  doc.clear();
  doc["name"] = "Waterwell Abstand";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_distance_cm";
  doc["state_topic"] = MQTT_DISTANCE_CM_TOPIC;
  doc["unit_of_measurement"] = "cm";
  doc["device_class"] = "distance";
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/sensor/rainwater_well_1_distance_cm/config");

  doc.clear();
  doc["name"] = "Waterwell Flaeche";
  doc["unique_id"] = String(MQTT_CLIENT_ID) + "_area_m2";
  doc["state_topic"] = MQTT_AREA_M2_TOPIC;
  doc["unit_of_measurement"] = "m2";
  doc["icon"] = "mdi:ruler-square";
  add_device(doc.createNestedObject("device"));
  publish_doc("homeassistant/sensor/rainwater_well_1_area_m2/config");
}

void publish_state(int volume_l)
{
  if (volume_l < 0)
    volume_l = 0;

  StaticJsonDocument<128> doc;
  doc["volume"] = volume_l;
  doc["distance_cm"] = lastMeasuredDistance;
  char buffer[128];
  serializeJson(doc, buffer);
  mqttclient.publish(MQTT_STATE_TOPIC, buffer, true);
  publishMetricStates(volume_l);
}

bool connect_mqtt_once()
{
  mqttclient.setServer(settings.mqtt_host, settings.mqtt_port);
  mqttclient.setCallback(mqttCallback);
  mqttclient.setBufferSize(1024);

  logLine("MQTT connect %s:%d", settings.mqtt_host, settings.mqtt_port);
  if (!mqttclient.connect(MQTT_CLIENT_ID, settings.mqtt_user, settings.mqtt_pass, MQTT_WILL_TOPIC, 0, true, "offline"))
  {
    logLine("MQTT failed, state=%d", mqttclient.state());
    return false;
  }

  mqttclient.publish(MQTT_BIRTH_TOPIC, "online", true);
  publish_discovery_message();
  const bool subSettings = mqttclient.subscribe(MQTT_SETTINGS_TOPIC "/#");
  const bool subMonitor = mqttclient.subscribe(MQTT_MONITOR_COMMAND_TOPIC);
  const bool subRestart = mqttclient.subscribe(MQTT_RESTART_COMMAND_TOPIC);
  logLine("MQTT subscribe settings=%d monitor=%d restart=%d", subSettings ? 1 : 0, subMonitor ? 1 : 0, subRestart ? 1 : 0);
  mqttclient.publish(MQTT_SETTINGS_TOPIC "/ping", String(bootCount).c_str());
  publishMonitorState();
  publishIpState();
  return true;
}

bool connect_mqtt_with_retries(int maxRetries)
{
  for (int i = 0; i < maxRetries; ++i)
  {
    if (connect_mqtt_once())
    {
      mqttRetryCount = 0;
      pingDone = false;
      unsigned long startWait = millis();
      while (!pingDone && (millis() - startWait) < 5000)
      {
        mqttclient.loop();
        delay(10);
      }
      return true;
    }
    delay(1000);
  }
  mqttRetryCount += maxRetries;
  return false;
}

void handleRoot()
{
  String html;
  html.reserve(12000);
  html +=
      "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Waterwell Monitor</title>"
      "<style>body{font-family:Arial,Helvetica,sans-serif;background:#0c111d;color:#f4f5f7;margin:0;padding:24px;}"
      "h1{margin-top:0;}h2{font-size:1.1rem;margin-top:0;}"
      ".card{background:#141b2d;border-radius:12px;padding:16px;box-shadow:0 4px 12px rgba(0,0,0,0.4);max-width:780px;margin-bottom:16px;}"
      ".status{margin-top:12px;font-size:0.92rem;color:#9dc6ff;}"
      ".kv{display:grid;grid-template-columns:180px 1fr;gap:8px;margin-top:10px;font-size:0.95rem;}"
      ".kv .k{color:#9ab;}.kv .v{font-weight:600;}"
      ".form label{display:block;margin-top:8px;color:#9ab;font-size:0.9rem;}"
      ".form input{width:100%;padding:8px;margin-top:4px;background:#1b2336;border:1px solid #22283b;color:#f4f5f7;border-radius:4px;box-sizing:border-box;}"
      ".checkbox{display:flex;align-items:center;gap:8px;margin-top:10px;}"
      ".checkbox input{width:auto;}"
      ".form button{margin-top:12px;padding:8px 16px;background:#2563eb;color:#fff;border:none;border-radius:6px;cursor:pointer;}"
      ".form button:hover{background:#1d4ed8;}.msg{margin-left:8px;font-size:0.9rem;}"
      "pre{background:#0a0f1c;border-radius:8px;padding:12px;overflow:auto;max-height:320px;white-space:pre-wrap;word-break:break-word;}</style>"
      "</head><body>"
      "<div class='card'><h1>Waterwell Sensor Monitor</h1><div class='status' id='status'>Loading status...</div>"
      "<div class='kv'>"
      "<div class='k'>Boot count</div><div class='v' id='bootCount'>-</div>"
      "<div class='k'>Distance</div><div class='v' id='distance'>-</div>"
      "<div class='k'>IP address</div><div class='v' id='ip'>-</div>"
      "<div class='k'>Web monitor</div><div class='v' id='monitor'>-</div>"
      "<div class='k'>Battery mode</div><div class='v' id='batteryMode'>-</div>"
      "</div></div>"
      "<div class='card'><h2>Sensor Settings</h2><form class='form' id='settingsForm'>"
      "<label>Area (m2)</label><input name='area' id='area' type='number' step='0.01' value='";
  html += String(settings.area, 2);
  html += "'>"
          "<label>Well volume (L)</label><input name='volume_well' id='volumeWell' type='number' step='0.01' value='";
  html += String(settings.volume_well, 2);
  html += "'>"
          "<label>Sleep (s)</label><input name='time_to_sleep' id='sleepSec' type='number' value='";
  html += String(settings.time_to_sleep);
  html += "'>"
          "<label>Heartbeat (s)</label><input name='heartbeat' id='heartbeatSec' type='number' value='";
  html += String(settings.heartbeat);
  html += "'>"
          "<label>Average sample count</label><input name='buffer_size' id='bufferSize' type='number' min='1' max='100' value='";
  html += String(settings.buffer_size);
  html += "'>"
          "<label>Sensor range min (cm)</label><input name='sensor_range_min' id='sensorRangeMin' type='number' min='0' max='1000' value='";
  html += String(settings.sensor_range_min);
  html += "'>"
          "<label>Sensor range max (cm)</label><input name='sensor_range_max' id='sensorRangeMax' type='number' min='1' max='2000' value='";
  html += String(settings.sensor_range_max);
  html += "'>"
          "<label>Calibration offset (cm)</label><input name='calibration_offset' id='calibrationOffset' type='number' step='0.01' value='";
  html += String(settings.calibration_offset, 2);
  html += "'>"
          "<div class='checkbox'><input type='checkbox' id='batteryModeInput' name='battery_mode' value='1' ";
  html += settings.battery_mode ? "checked" : "";
  html += "><label for='batteryModeInput'>Battery mode</label></div>"
          "<button type='submit'>Save settings</button><span class='msg' id='settingsMsg'></span>"
          "</form></div>"
          "<div class='card'><h2>MQTT Settings</h2><form class='form' id='mqttForm'>"
          "<label>Host</label><input name='host' id='mqttHost' type='text' value='";
  html += String(settings.mqtt_host);
  html += "'>"
          "<label>Port</label><input name='port' id='mqttPort' type='number' value='";
  html += String(settings.mqtt_port);
  html += "'>"
          "<label>User</label><input name='user' id='mqttUser' type='text' value='";
  html += String(settings.mqtt_user);
  html += "'>"
          "<label>Password</label><input name='pass' id='mqttPass' type='password' placeholder='Leave empty to keep current password'>"
          "<button type='submit'>Save MQTT</button><span class='msg' id='mqttMsg'></span>"
          "</form></div>"
          "<div class='card'><h2>Firmware Update</h2><form class='form' method='POST' action='/ota/upload' enctype='multipart/form-data'>"
          "<label>Password</label><input name='pw' type='password'>"
          "<label>Firmware (.bin)</label><input type='file' name='update' accept='.bin'>"
          "<button type='submit'>Flash firmware</button></form></div>"
          "<div class='card'><h2>Live Logs</h2><pre id='logs'>Loading logs...</pre></div>"
          "<script>"
      "let initialLoaded=false;"
      "function setMsg(id,text,ok){const el=document.getElementById(id);el.textContent=text;el.style.color=ok?'#4ade80':'#f87171';}"
      "async function refreshStatus(){"
      "try{const res=await fetch('/status/json',{cache:'no-store'});if(!res.ok)throw new Error('status http '+res.status);"
      "const data=await res.json();document.getElementById('status').textContent='Last update: '+new Date().toLocaleTimeString();"
      "document.getElementById('bootCount').textContent=(data.bootCount!==undefined&&data.bootCount!==null)?data.bootCount:'-';"
      "document.getElementById('distance').textContent=((data.distance_cm!==undefined&&data.distance_cm!==null)?data.distance_cm:'-')+' cm';"
      "document.getElementById('ip').textContent=data.ip||'-';"
      "document.getElementById('monitor').textContent=data.monitor?'ON':'OFF';"
      "document.getElementById('batteryMode').textContent=data.settings&&data.settings.battery_mode?'ON':'OFF';"
      "if(!initialLoaded&&data.settings&&data.mqtt){"
      "document.getElementById('area').value=(data.settings.area!==undefined&&data.settings.area!==null)?data.settings.area:'';"
      "document.getElementById('volumeWell').value=(data.settings.volume_well!==undefined&&data.settings.volume_well!==null)?data.settings.volume_well:'';"
      "document.getElementById('sleepSec').value=(data.settings.time_to_sleep!==undefined&&data.settings.time_to_sleep!==null)?data.settings.time_to_sleep:'';"
      "document.getElementById('heartbeatSec').value=(data.settings.heartbeat!==undefined&&data.settings.heartbeat!==null)?data.settings.heartbeat:'';"
      "document.getElementById('bufferSize').value=(data.settings.buffer_size!==undefined&&data.settings.buffer_size!==null)?data.settings.buffer_size:'';"
      "document.getElementById('sensorRangeMin').value=(data.settings.sensor_range_min!==undefined&&data.settings.sensor_range_min!==null)?data.settings.sensor_range_min:'';"
      "document.getElementById('sensorRangeMax').value=(data.settings.sensor_range_max!==undefined&&data.settings.sensor_range_max!==null)?data.settings.sensor_range_max:'';"
      "document.getElementById('calibrationOffset').value=(data.settings.calibration_offset!==undefined&&data.settings.calibration_offset!==null)?data.settings.calibration_offset:'';"
      "document.getElementById('batteryModeInput').checked=!!data.settings.battery_mode;"
      "document.getElementById('mqttHost').value=(data.mqtt.host!==undefined&&data.mqtt.host!==null)?data.mqtt.host:'';"
      "document.getElementById('mqttPort').value=(data.mqtt.port!==undefined&&data.mqtt.port!==null)?data.mqtt.port:'';"
      "document.getElementById('mqttUser').value=(data.mqtt.user!==undefined&&data.mqtt.user!==null)?data.mqtt.user:'';"
      "initialLoaded=true;}"
      "}catch(e){document.getElementById('status').textContent='Status refresh failed';}}"
      "async function refreshLogs(){"
      "try{const res=await fetch('/logs',{cache:'no-store'});document.getElementById('logs').textContent=await res.text();}"
      "catch(e){document.getElementById('logs').textContent='Failed to load logs';}}"
      "document.getElementById('settingsForm').addEventListener('submit',async function(e){e.preventDefault();"
      "const p=new URLSearchParams();p.set('area',document.getElementById('area').value);p.set('volume_well',document.getElementById('volumeWell').value);"
      "p.set('time_to_sleep',document.getElementById('sleepSec').value);p.set('heartbeat',document.getElementById('heartbeatSec').value);"
      "p.set('buffer_size',document.getElementById('bufferSize').value);"
      "p.set('sensor_range_min',document.getElementById('sensorRangeMin').value);"
      "p.set('sensor_range_max',document.getElementById('sensorRangeMax').value);"
      "p.set('calibration_offset',document.getElementById('calibrationOffset').value);"
      "if(document.getElementById('batteryModeInput').checked){p.set('battery_mode','1');}"
      "const r=await fetch('/settings',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});"
      "setMsg('settingsMsg',r.ok?'Saved':'Save failed',r.ok);if(r.ok){refreshStatus();}});"
      "document.getElementById('mqttForm').addEventListener('submit',async function(e){e.preventDefault();"
      "const p=new URLSearchParams();p.set('host',document.getElementById('mqttHost').value);p.set('port',document.getElementById('mqttPort').value);"
      "p.set('user',document.getElementById('mqttUser').value);const pass=document.getElementById('mqttPass').value;if(pass){p.set('pass',pass);}"
      "const r=await fetch('/mqtt/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});"
      "setMsg('mqttMsg',r.ok?'Saved (reconnect)...':'Save failed',r.ok);if(r.ok){document.getElementById('mqttPass').value='';setTimeout(refreshStatus,1500);}});"
      "refreshStatus();refreshLogs();setInterval(refreshStatus,5000);setInterval(refreshLogs,3000);"
      "</script></body></html>";
  webServer.send(200, "text/html", html);
}

void handleStatusJson()
{
  StaticJsonDocument<512> doc;
  doc["bootCount"] = bootCount;
  doc["distance_cm"] = lastMeasuredDistance;
  doc["ip"] = WiFi.localIP().toString();
  doc["monitor"] = webMonitorRunning;
  JsonObject s = doc.createNestedObject("settings");
  s["time_to_sleep"] = settings.time_to_sleep;
  s["heartbeat"] = settings.heartbeat;
  s["wifi_timeout"] = settings.wifi_timeout;
  s["sensor_timeout"] = settings.sensor_timeout;
  s["buffer_size"] = settings.buffer_size;
  s["sensor_range_min"] = settings.sensor_range_min;
  s["sensor_range_max"] = settings.sensor_range_max;
  s["calibration_offset"] = settings.calibration_offset;
  s["area"] = settings.area;
  s["volume_well"] = settings.volume_well;
  s["battery_mode"] = settings.battery_mode;
  JsonObject m = doc.createNestedObject("mqtt");
  m["host"] = settings.mqtt_host;
  m["port"] = settings.mqtt_port;
  m["user"] = settings.mqtt_user;
  m["pass"] = "***";
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

void handleLogs()
{
  webServer.send(200, "text/plain", getLogs());
}

void handleSettingsPost()
{
  if (webServer.hasArg("area"))
    settings.area = webServer.arg("area").toFloat();
  if (webServer.hasArg("volume_well"))
    settings.volume_well = webServer.arg("volume_well").toFloat();
  if (webServer.hasArg("time_to_sleep"))
    settings.time_to_sleep = webServer.arg("time_to_sleep").toInt();
  if (webServer.hasArg("heartbeat"))
    settings.heartbeat = webServer.arg("heartbeat").toInt();
  if (webServer.hasArg("buffer_size"))
    settings.buffer_size = webServer.arg("buffer_size").toInt();
  if (webServer.hasArg("sensor_range_min"))
    settings.sensor_range_min = webServer.arg("sensor_range_min").toInt();
  if (webServer.hasArg("sensor_range_max"))
    settings.sensor_range_max = webServer.arg("sensor_range_max").toInt();
  if (webServer.hasArg("calibration_offset"))
    settings.calibration_offset = webServer.arg("calibration_offset").toFloat();

  if (settings.sensor_range_min < 0)
    settings.sensor_range_min = 0;
  if (settings.sensor_range_max <= settings.sensor_range_min)
    settings.sensor_range_max = settings.sensor_range_min + 1;
  if (settings.buffer_size < 1)
    settings.buffer_size = 1;
  settings.battery_mode = webServer.hasArg("battery_mode");
  saveSettingsToNvs();
  const int volume_l_empty = round((lastMeasuredDistance * 1e-1f) * (settings.area * 1e2f));
  const int volume_l = round(settings.volume_well - volume_l_empty);
  publishMetricStates(volume_l);
  webServer.send(200, "text/plain", "OK");
}

void handleMqttConfigPost()
{
  if (!webServer.hasArg("host") || !webServer.hasArg("port") || !webServer.hasArg("user"))
  {
    webServer.send(400, "text/plain", "host, port, user required");
    return;
  }
  strlcpy(settings.mqtt_host, webServer.arg("host").c_str(), sizeof(settings.mqtt_host));
  settings.mqtt_port = webServer.arg("port").toInt();
  strlcpy(settings.mqtt_user, webServer.arg("user").c_str(), sizeof(settings.mqtt_user));
  if (webServer.hasArg("pass") && webServer.arg("pass").length() > 0)
  {
    strlcpy(settings.mqtt_pass, webServer.arg("pass").c_str(), sizeof(settings.mqtt_pass));
  }
  saveSettingsToNvs();
  mqttclient.disconnect();
  webServer.send(200, "text/plain", "OK");
}

void handleOtaUpload()
{
  HTTPUpload &upload = webServer.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    otaAuthFailed = false;
    if (!webServer.hasArg("pw") || webServer.arg("pw") != OTA_SHARED_SECRET)
    {
      otaAuthFailed = true;
      Update.abort();
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (Update.end(true))
    {
      webServer.send(200, "text/plain", "OK rebooting");
      delay(500);
      ESP.restart();
    }
    else
    {
      Update.printError(Serial);
      webServer.send(500, "text/plain", "OTA failed");
    }
  }
}

void startWebMonitor()
{
  if (webMonitorRunning)
    return;

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/status/json", HTTP_GET, handleStatusJson);
  webServer.on("/logs", HTTP_GET, handleLogs);
  webServer.on("/settings", HTTP_POST, handleSettingsPost);
  webServer.on("/mqtt/config", HTTP_POST, handleMqttConfigPost);
  webServer.on(
      "/ota/upload", HTTP_POST,
      []()
      {
        if (otaAuthFailed)
        {
          webServer.send(401, "text/plain", "invalid password");
        }
      },
      handleOtaUpload);
  webServer.begin();
  webMonitorRunning = true;
  publishMonitorState();
  logLine("Web monitor started on port 80");
}

void stopWebMonitor()
{
  if (!webMonitorRunning)
    return;
  webServer.stop();
  webMonitorRunning = false;
  publishMonitorState();
  logLine("Web monitor stopped");
}

void deep_sleep()
{
  logLine("Going to sleep now");
  esp_wifi_stop();
  delay(20);
  Serial.flush();
  esp_sleep_enable_timer_wakeup(settings.time_to_sleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void printSettings()
{
  logLine("Settings: hb=%d sleep=%d wifi_to=%d sensor_to=%d buf=%d range=%d-%d area=%.2f volume=%.2f",
          settings.heartbeat, settings.time_to_sleep, settings.wifi_timeout, settings.sensor_timeout,
          settings.buffer_size, settings.sensor_range_min, settings.sensor_range_max, settings.area, settings.volume_well);
  logLine("Calibration offset: %.2f cm", settings.calibration_offset);
  logLine("Power profile: battery_mode=%s", settings.battery_mode ? "ON" : "OFF");
  logLine("MQTT: %s:%d user=%s", settings.mqtt_host, settings.mqtt_port, settings.mqtt_user);
}

void setup()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  nvs_flash_init();
  loadSettingsFromNvs();
  ++bootCount;
  printSettings();
  logLine("Boot number: %d", bootCount);

  lastMeasuredDistance = measure_distance();
  const bool shouldPublish = (lastMeasuredDistance != distance_prev && lastMeasuredDistance != 0) ||
                             ((bootCount * settings.time_to_sleep) % settings.heartbeat == 0) ||
                             !prev_wifistate;

  bool mqttOk = false;
  if (shouldPublish && setup_wifi())
  {
    mqttOk = connect_mqtt_with_retries(MQTT_RETRY_LIMIT);
    if (mqttOk)
    {
      const int volume_l_empty = round((lastMeasuredDistance * 1e-1f) * (settings.area * 1e2f));
      const int volume_l = round(settings.volume_well - volume_l_empty);
      publish_state(volume_l);
    }
  }

  if (!mqttOk)
  {
    mqttRetryCount = MQTT_RETRY_LIMIT;
  }

  if (mqttRetryCount >= MQTT_RETRY_LIMIT)
  {
    webMonitorForcedByFailure = true;
    setWebMonitorEnabled(true, false);
  }

  distance_prev = lastMeasuredDistance;

  if (settings.web_monitor_enabled || webMonitorForcedByFailure)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      setup_wifi();
    }
    startWebMonitor();
    return;
  }

  if (settings.battery_mode)
  {
    if (mqttclient.connected())
    {
      mqttclient.disconnect();
    }
    WiFi.mode(WIFI_OFF);
    deep_sleep();
    return;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    setup_wifi();
  }
  if (!mqttclient.connected())
  {
    connect_mqtt_once();
  }
  lastNoSleepMeasureMs = millis();
  logLine("Battery mode OFF: keep device awake");
}

void loop()
{
  const unsigned long now = millis();

  if (!webMonitorRunning)
  {
    if (!settings.battery_mode)
    {
      if (WiFi.status() != WL_CONNECTED && (now - lastWiFiRetryMs > 5000))
      {
        lastWiFiRetryMs = now;
        setup_wifi();
      }
      else if (WiFi.status() == WL_CONNECTED && !mqttclient.connected() && (now - lastMqttRetryMs > 5000))
      {
        lastMqttRetryMs = now;
        if (!connect_mqtt_once())
        {
          mqttRetryCount++;
          if (mqttRetryCount >= MQTT_RETRY_LIMIT)
          {
            setWebMonitorEnabled(true, false);
          }
        }
        else
        {
          mqttRetryCount = 0;
        }
      }
      else if (WiFi.status() == WL_CONNECTED && mqttclient.connected())
      {
        mqttclient.loop();
      }

      if (now - lastNoSleepMeasureMs >= NO_SLEEP_MEASURE_INTERVAL_MS)
      {
        lastNoSleepMeasureMs = now;
        lastMeasuredDistance = measure_distance();
        const int volume_l_empty = round((lastMeasuredDistance * 1e-1f) * (settings.area * 1e2f));
        const int volume_l = round(settings.volume_well - volume_l_empty);
        if (mqttclient.connected())
        {
          publish_state(volume_l);
        }
      }
      delay(50);
      return;
    }

    delay(1000);
    return;
  }

  webServer.handleClient();

  if (WiFi.status() != WL_CONNECTED)
  {
    if (now - lastWiFiRetryMs > 5000)
    {
      lastWiFiRetryMs = now;
      setup_wifi();
    }
  }
  else if (!mqttclient.connected())
  {
    if (now - lastMqttRetryMs > 5000)
    {
      lastMqttRetryMs = now;
      if (!connect_mqtt_once())
      {
        mqttRetryCount++;
      }
      else
      {
        mqttRetryCount = 0;
      }
      if (mqttRetryCount >= MQTT_RETRY_LIMIT)
      {
        setWebMonitorEnabled(true, false);
      }
    }
  }
  else
  {
    mqttclient.loop();
  }

  // Keep measurements running while web monitor is active.
  if (now - lastNoSleepMeasureMs >= NO_SLEEP_MEASURE_INTERVAL_MS)
  {
    lastNoSleepMeasureMs = now;
    lastMeasuredDistance = measure_distance();
    const int volume_l_empty = round((lastMeasuredDistance * 1e-1f) * (settings.area * 1e2f));
    const int volume_l = round(settings.volume_well - volume_l_empty);
    if (mqttclient.connected())
    {
      publish_state(volume_l);
    }
  }
}
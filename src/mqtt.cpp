#include "mqtt.h"

#include "config.h"
#include "diag.h"
#include "log.h"
#include "sensor.h"
#include "settings.h"
#include "web.h"

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

namespace
{
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool pingDone = false;
int failedAttempts = 0;

// Last published diagnostic texts. Cleared on connect so a new session republishes.
char lastStatus[32] = "";
char lastError[64] = "";

// Every path is derived from MQTT_NODE so a second board cannot drift.
struct MqttPaths
{
  char clientId[40];
  char settings[64];
  char ping[72];
  char state[64];
  char status[64];
  char restart[80];
  char reference[80];
  char monitorCommand[80];
  char monitorState[80];
  char ip[80];
  char diagStatus[80];
  char diagError[80];
  char firmware[80];
  char volumeL[80];
  char volumeM3[80];
  char distance[80];
  char area[80];
  char discoveryVolume[96];
  char discoveryRestart[96];
  char discoveryReference[96];
  char discoveryMonitor[96];
  char discoveryIp[96];
  char discoveryDiagStatus[104];
  char discoveryDiagError[104];
  char discoveryFirmware[104];
  char discoveryVolumeL[104];
  char discoveryVolumeM3[104];
  char discoveryDistance[104];
  char discoveryArea[104];
} path;

void joinRoot(char *dest, size_t destLen, const char *root, const char *suffix)
{
  snprintf(dest, destLen, "%s/%s", root, suffix);
}

void buildPaths()
{
  snprintf(path.clientId, sizeof(path.clientId), "ESP32_%s", MQTT_NODE);
  char root[48];
  snprintf(root, sizeof(root), "carport/sensor/%s", MQTT_NODE);
  joinRoot(path.settings, sizeof(path.settings), root, "settings");
  joinRoot(path.ping, sizeof(path.ping), path.settings, "ping");
  joinRoot(path.state, sizeof(path.state), root, "state");
  joinRoot(path.status, sizeof(path.status), root, "status");
  joinRoot(path.restart, sizeof(path.restart), root, "control/restart");
  joinRoot(path.reference, sizeof(path.reference), root, "control/reference");
  joinRoot(path.monitorCommand, sizeof(path.monitorCommand), root, "control/monitor");
  joinRoot(path.monitorState, sizeof(path.monitorState), root, "status/monitor");
  joinRoot(path.ip, sizeof(path.ip), root, "status/ip");
  joinRoot(path.diagStatus, sizeof(path.diagStatus), root, "status/state");
  joinRoot(path.diagError, sizeof(path.diagError), root, "status/error");
  joinRoot(path.firmware, sizeof(path.firmware), root, "status/firmware");
  joinRoot(path.volumeL, sizeof(path.volumeL), root, "metrics/volume_l");
  joinRoot(path.volumeM3, sizeof(path.volumeM3), root, "metrics/volume_m3");
  joinRoot(path.distance, sizeof(path.distance), root, "metrics/distance_cm");
  joinRoot(path.area, sizeof(path.area), root, "metrics/area_m2");
  snprintf(path.discoveryVolume, sizeof(path.discoveryVolume), "homeassistant/sensor/%s/config", MQTT_NODE);
  snprintf(path.discoveryRestart, sizeof(path.discoveryRestart), "homeassistant/button/%s_restart/config", MQTT_NODE);
  snprintf(path.discoveryReference, sizeof(path.discoveryReference), "homeassistant/button/%s_reference/config", MQTT_NODE);
  snprintf(path.discoveryMonitor, sizeof(path.discoveryMonitor), "homeassistant/switch/%s_monitor/config", MQTT_NODE);
  snprintf(path.discoveryIp, sizeof(path.discoveryIp), "homeassistant/sensor/%s_ip/config", MQTT_NODE);
  snprintf(path.discoveryDiagStatus, sizeof(path.discoveryDiagStatus), "homeassistant/sensor/%s_state/config", MQTT_NODE);
  snprintf(path.discoveryDiagError, sizeof(path.discoveryDiagError), "homeassistant/sensor/%s_error/config", MQTT_NODE);
  snprintf(path.discoveryFirmware, sizeof(path.discoveryFirmware), "homeassistant/sensor/%s_firmware/config", MQTT_NODE);
  snprintf(path.discoveryVolumeL, sizeof(path.discoveryVolumeL), "homeassistant/sensor/%s_volume_l/config", MQTT_NODE);
  snprintf(path.discoveryVolumeM3, sizeof(path.discoveryVolumeM3), "homeassistant/sensor/%s_volume_m3/config", MQTT_NODE);
  snprintf(path.discoveryDistance, sizeof(path.discoveryDistance), "homeassistant/sensor/%s_distance_cm/config", MQTT_NODE);
  snprintf(path.discoveryArea, sizeof(path.discoveryArea), "homeassistant/sensor/%s_area_m2/config", MQTT_NODE);
}

void publishDiscovery();
void publishIpState();
void publishMetricStates(int volumeL, int distanceCm);

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char safePayload[128];
  size_t copyLen = (length < sizeof(safePayload) - 1) ? length : sizeof(safePayload) - 1;
  memcpy(safePayload, payload, copyLen);
  safePayload[copyLen] = '\0';
  while (copyLen > 0 && (safePayload[copyLen - 1] == '\n' || safePayload[copyLen - 1] == '\r' || safePayload[copyLen - 1] == ' '))
  {
    safePayload[--copyLen] = '\0';
  }

  logLine("MQTT %s => %s", topic, safePayload);

  if (strncmp(topic, path.settings, strlen(path.settings)) == 0)
  {
    char *settingKey = topic + strlen(path.settings) + 1;
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
    else if (strcmp(settingKey, "sensor_to_max_fill_cm") == 0)
      changed = updateFloatSetting(safePayload, settings.sensor_to_max_fill_cm);
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
  else if (strcmp(topic, path.ping) == 0)
  {
    pingDone = true;
  }
  else if (strcmp(topic, path.monitorCommand) == 0)
  {
    const bool enable = strcasecmp(safePayload, "ON") == 0 || strcmp(safePayload, "1") == 0;
    // The command is retained on the broker, so a sleeping device receives it on its
    // next wake-up. Store it only when it changes to spare the flash.
    setWebMonitorEnabled(enable, enable != settings.web_monitor_enabled);
    logLine("Web monitor set via MQTT: %s", enable ? "ON" : "OFF");
  }
  else if (strcmp(topic, path.restart) == 0)
  {
    logLine("Restart requested via MQTT");
    delay(300);
    ESP.restart();
  }
  else if (strcmp(topic, path.reference) == 0)
  {
    // Empty payload is the retained command being cleared after a run.
    if (copyLen == 0)
    {
      return;
    }
    logLine("Reference measurement requested via MQTT");
    requestReferenceMeasurement();
  }
}

void publishDiscovery()
{
  static StaticJsonDocument<768> doc;
  static char buffer[768];

  auto addDevice = [&](JsonObject device)
  {
    JsonArray identifiers = device.createNestedArray("identifiers");
    identifiers.add(path.clientId);
    device["name"] = MQTT_DEVICE_NAME;
    device["manufacturer"] = "DIY";
    device["model"] = "ESP32";
  };

  auto publishDoc = [&](const char *topic)
  {
    const size_t payloadLen = measureJson(doc);
    const size_t written = serializeJson(doc, buffer, sizeof(buffer));
    bool ok = false;
    if (written == payloadLen && written + 1 < sizeof(buffer))
    {
      for (int attempt = 0; attempt < 5 && !ok; ++attempt)
      {
        ok = mqttClient.publish(topic, buffer, true);
        mqttClient.loop();
        delay(30);
      }
    }
    logLine("Discovery publish: %s topic=%s state=%d len=%u written=%u",
            ok ? "OK" : "FAILED",
            topic,
            mqttClient.state(),
            (unsigned int)payloadLen,
            (unsigned int)written);
  };

  // The old "Volume" sensor duplicated "Volumen Liter". An empty retained
  // config removes the stale entity from Home Assistant.
  mqttClient.publish(path.discoveryVolume, "", true);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Neustart";
  doc["unique_id"] = String(path.clientId) + "_restart";
  doc["command_topic"] = path.restart;
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryRestart);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Referenzmessung";
  doc["unique_id"] = String(path.clientId) + "_reference";
  doc["command_topic"] = path.reference;
  doc["payload_press"] = "PRESS";
  doc["icon"] = "mdi:crosshairs";
  // Retained so a sleeping device runs it on the next wake. The firmware
  // publishes an empty payload after the attempt, otherwise every wake
  // would skip the plausibility check.
  doc["retain"] = true;
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryReference);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Web Monitor";
  doc["unique_id"] = String(path.clientId) + "_monitor";
  doc["command_topic"] = path.monitorCommand;
  doc["state_topic"] = path.monitorState;
  doc["payload_on"] = "ON";
  doc["payload_off"] = "OFF";
  // Retained command: the broker hands it to the device whenever it wakes up and
  // subscribes. optimistic stays false so Home Assistant draws a toggle. With
  // optimistic true it treats the state as unknown and shows on/off buttons.
  doc["retain"] = true;
  doc["optimistic"] = false;
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryMonitor);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " IP Adresse";
  doc["unique_id"] = String(path.clientId) + "_ip";
  doc["state_topic"] = path.ip;
  doc["icon"] = "mdi:ip-network";
  doc["entity_category"] = "diagnostic";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryIp);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Status";
  doc["unique_id"] = String(path.clientId) + "_state";
  doc["state_topic"] = path.diagStatus;
  doc["icon"] = "mdi:led-on";
  doc["entity_category"] = "diagnostic";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryDiagStatus);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Fehler";
  doc["unique_id"] = String(path.clientId) + "_error";
  doc["state_topic"] = path.diagError;
  doc["icon"] = "mdi:alert-circle-outline";
  doc["entity_category"] = "diagnostic";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryDiagError);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Firmware";
  doc["unique_id"] = String(path.clientId) + "_firmware";
  doc["state_topic"] = path.firmware;
  doc["icon"] = "mdi:chip";
  doc["entity_category"] = "diagnostic";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryFirmware);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Volumen Liter";
  doc["unique_id"] = String(path.clientId) + "_volume_l";
  doc["state_topic"] = path.volumeL;
  doc["unit_of_measurement"] = "L";
  doc["device_class"] = "volume";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryVolumeL);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Volumen m3";
  doc["unique_id"] = String(path.clientId) + "_volume_m3";
  doc["state_topic"] = path.volumeM3;
  doc["unit_of_measurement"] = "m³";
  doc["device_class"] = "volume";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryVolumeM3);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Abstand";
  doc["unique_id"] = String(path.clientId) + "_distance_cm";
  doc["state_topic"] = path.distance;
  doc["unit_of_measurement"] = "cm";
  doc["device_class"] = "distance";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryDistance);

  doc.clear();
  doc["name"] = MQTT_DEVICE_NAME " Flaeche";
  doc["unique_id"] = String(path.clientId) + "_area_m2";
  doc["state_topic"] = path.area;
  doc["unit_of_measurement"] = "m²";
  doc["icon"] = "mdi:ruler-square";
  addDevice(doc.createNestedObject("device"));
  publishDoc(path.discoveryArea);
}

void publishIpState()
{
  if (!mqttClient.connected() || WiFi.status() != WL_CONNECTED)
  {
    return;
  }
  const String ip = WiFi.localIP().toString();
  mqttClient.publish(path.ip, ip.c_str(), true);
}

void publishMetricStates(int volumeL, int distanceCm)
{
  if (!mqttClient.connected())
  {
    return;
  }

  char buf[32];
  const int publishedVolume = volumeL < 0 ? 0 : volumeL;
  snprintf(buf, sizeof(buf), "%d", publishedVolume);
  mqttClient.publish(path.volumeL, buf, true);

  const float volumeM3 = (float)publishedVolume / 1000.0f;
  dtostrf(volumeM3, 0, 3, buf);
  mqttClient.publish(path.volumeM3, buf, true);

  snprintf(buf, sizeof(buf), "%d", distanceCm);
  mqttClient.publish(path.distance, buf, true);

  dtostrf(settings.area, 0, 3, buf);
  mqttClient.publish(path.area, buf, true);
}
} // namespace

bool mqttConnectOnce()
{
  buildPaths();
  mqttClient.setServer(settings.mqtt_host, settings.mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);

  logLine("MQTT connect %s:%d", settings.mqtt_host, settings.mqtt_port);
  if (!mqttClient.connect(path.clientId, settings.mqtt_user, settings.mqtt_pass, path.status, 0, true, "offline"))
  {
    logLine("MQTT failed, state=%d", mqttClient.state());
    return false;
  }

  mqttClient.publish(path.status, "online", true);
  mqttClient.publish(path.firmware, FW_VERSION, true);
  lastStatus[0] = '\0';
  lastError[0] = '\0';
  publishDiscovery();
  char settingsFilter[72];
  snprintf(settingsFilter, sizeof(settingsFilter), "%s/#", path.settings);
  const bool subSettings = mqttClient.subscribe(settingsFilter);
  const bool subMonitor = mqttClient.subscribe(path.monitorCommand);
  const bool subRestart = mqttClient.subscribe(path.restart);
  const bool subReference = mqttClient.subscribe(path.reference);
  logLine("MQTT subscribe settings=%d monitor=%d restart=%d reference=%d", subSettings ? 1 : 0, subMonitor ? 1 : 0,
          subRestart ? 1 : 0, subReference ? 1 : 0);
  if (!subMonitor)
  {
    // Without this subscription the switch can never turn the page back on.
    webForceMonitorOnMqttFailure();
  }
  mqttClient.publish(path.ping, String(currentBootCount()).c_str());
  mqttPublishMonitorState();
  publishIpState();
  return true;
}

bool mqttConnectWithRetries(int maxRetries)
{
  for (int i = 0; i < maxRetries; ++i)
  {
    if (mqttConnectOnce())
    {
      failedAttempts = 0;
      pingDone = false;
      const unsigned long startWait = millis();
      // Always drain for a moment. A ping can arrive before the retained
      // monitor command, and leaving early would drop that command.
      while ((millis() - startWait) < 3000)
      {
        mqttClient.loop();
        if (pingDone && (millis() - startWait) >= 1500)
        {
          break;
        }
        delay(10);
      }
      return true;
    }
    delay(1000);
  }
  failedAttempts += maxRetries;
  return false;
}

void mqttLoop()
{
  mqttClient.loop();
}

bool mqttConnected()
{
  return mqttClient.connected();
}

void mqttDisconnect()
{
  mqttClient.disconnect();
}

void mqttClearReferenceCommand()
{
  if (mqttClient.connected())
  {
    mqttClient.publish(path.reference, "", true);
  }
}

bool publishLastValidState()
{
  if (!hasValidDistance() || !mqttClient.connected())
  {
    return false;
  }

  const int distanceCm = lastValidDistanceCm();
  int volumeL = computeVolumeLitersFromDistance(distanceCm);
  if (volumeL < 0)
  {
    volumeL = 0;
  }

  StaticJsonDocument<128> doc;
  doc["volume"] = volumeL;
  doc["distance_cm"] = distanceCm;
  char buffer[128];
  serializeJson(doc, buffer);
  mqttClient.publish(path.state, buffer, true);
  publishMetricStates(volumeL, distanceCm);
  return true;
}

void mqttPublishMonitorState()
{
  if (mqttClient.connected())
  {
    mqttClient.publish(path.monitorState, webMonitorIsRunning() ? "ON" : "OFF", true);
  }
}

void mqttPublishDiagnostics(const char *status)
{
  if (!mqttClient.connected())
  {
    return;
  }
  if (strcmp(status, lastStatus) != 0 && mqttClient.publish(path.diagStatus, status, true))
  {
    strlcpy(lastStatus, status, sizeof(lastStatus));
  }
  const char *error = diagErrorText();
  if (strcmp(error, lastError) != 0 && mqttClient.publish(path.diagError, error, true))
  {
    strlcpy(lastError, error, sizeof(lastError));
  }
}

int mqttFailedAttempts()
{
  return failedAttempts;
}

void mqttSetFailedAttempts(int count)
{
  failedAttempts = count;
}

void mqttRegisterConnectFailure()
{
  ++failedAttempts;
}

void mqttResetFailures()
{
  failedAttempts = 0;
}

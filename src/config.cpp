#include "config.h"

#include "log.h"
#include "settings.h"

#include <Preferences.h>
#include <stdlib.h>
#include <string.h>

RuntimeSettings settings;
static Preferences preferences;

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
  settings.sensor_to_max_fill_cm = SENSOR_TO_MAX_FILL_CM;
  settings.area = AREA;
  settings.volume_well = VOLUME_WELL;
  strlcpy(settings.mqtt_host, MQTT_HOST, sizeof(settings.mqtt_host));
  settings.mqtt_port = MQTT_PORT;
  strlcpy(settings.mqtt_user, MQTT_USER, sizeof(settings.mqtt_user));
  strlcpy(settings.mqtt_pass, MQTT_PASS, sizeof(settings.mqtt_pass));
  settings.web_monitor_enabled = true;
  settings.battery_mode = BATTERY_MODE_DEFAULT;
  settings.dev_mode = false;
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
  settings.sensor_to_max_fill_cm = preferences.getFloat("maxfill_cm", settings.sensor_to_max_fill_cm);
  settings.area = preferences.getFloat("area", settings.area);
  settings.volume_well = preferences.getFloat("vol", settings.volume_well);
  preferences.getString("mq_host", settings.mqtt_host, sizeof(settings.mqtt_host));
  settings.mqtt_port = preferences.getInt("mq_port", settings.mqtt_port);
  preferences.getString("mq_user", settings.mqtt_user, sizeof(settings.mqtt_user));
  preferences.getString("mq_pass", settings.mqtt_pass, sizeof(settings.mqtt_pass));
  settings.web_monitor_enabled = preferences.getBool("web_mon", settings.web_monitor_enabled);
  settings.battery_mode = preferences.getBool("bat_mode", settings.battery_mode);
  settings.dev_mode = preferences.getBool("dev_mode", settings.dev_mode);
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
  preferences.putFloat("maxfill_cm", settings.sensor_to_max_fill_cm);
  preferences.putFloat("area", settings.area);
  preferences.putFloat("vol", settings.volume_well);
  preferences.putString("mq_host", settings.mqtt_host);
  preferences.putInt("mq_port", settings.mqtt_port);
  preferences.putString("mq_user", settings.mqtt_user);
  preferences.putString("mq_pass", settings.mqtt_pass);
  preferences.putBool("web_mon", settings.web_monitor_enabled);
  preferences.putBool("bat_mode", settings.battery_mode);
  preferences.putBool("dev_mode", settings.dev_mode);
  preferences.end();
}

void logRuntimeSettings()
{
  logLine("Settings: hb=%d sleep=%d wifi_to=%d sensor_to=%d buf=%d range=%d-%d area=%.2f volume=%.2f",
          settings.heartbeat, settings.time_to_sleep, settings.wifi_timeout, settings.sensor_timeout,
          settings.buffer_size, settings.sensor_range_min, settings.sensor_range_max, settings.area, settings.volume_well);
  logLine("Sensor->max-fill offset: %.2f cm", settings.sensor_to_max_fill_cm);
  logLine("Calibration offset: %.2f cm", settings.calibration_offset);
  logLine("Power profile: battery_mode=%s dev_mode=%s", settings.battery_mode ? "ON" : "OFF", settings.dev_mode ? "ON" : "OFF");
  logLine("MQTT: %s:%d user=%s", settings.mqtt_host, settings.mqtt_port, settings.mqtt_user);
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

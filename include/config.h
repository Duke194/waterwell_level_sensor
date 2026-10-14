#pragma once

#include <Arduino.h>

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
  float sensor_to_max_fill_cm;
  float area;
  float volume_well;
  char mqtt_host[64];
  int mqtt_port;
  char mqtt_user[64];
  char mqtt_pass[80];
  bool web_monitor_enabled;
  bool battery_mode;
  bool dev_mode;
};

extern RuntimeSettings settings;

void loadSettingsFromNvs();
void saveSettingsToNvs();
void logRuntimeSettings();
bool updateIntSetting(const char *payload, int &setting);
bool updateFloatSetting(const char *payload, float &setting);

// Stores settings.dev_mode in NVS and restarts the measure timer. Implemented in main.cpp.
void setDevMode(bool enabled);
bool devModeEnabled();

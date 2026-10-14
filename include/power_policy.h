#pragma once

// Deep sleep is a battery feature. With battery mode off the device never
// sleeps, no matter what the web monitor or an OTA update is doing. The
// measure interval is independent of all of this.
inline bool powerMayDeepSleep(bool battery_mode)
{
  return battery_mode;
}

// Without battery mode the device is always awake. In battery mode it only
// stays awake while somebody needs it: dev mode, a running or wanted web
// monitor, or an update that is not confirmed yet.
inline bool powerStayAwake(bool battery_mode, bool dev_mode, bool web_running, bool web_wanted, bool awaiting_ota_confirm)
{
  if (!powerMayDeepSleep(battery_mode))
  {
    return true;
  }
  return dev_mode || web_running || web_wanted || awaiting_ota_confirm;
}

#include "config.h"
#include "diag.h"
#include "log.h"
#include "mqtt.h"
#include "ota_boot.h"
#include "ota_image.h"
#include "power_policy.h"
#include "sensor.h"
#include "settings.h"
#include "status_led.h"
#include "web.h"

#include <Arduino.h>
#include <WiFi.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

namespace
{
constexpr uint64_t US_TO_S_FACTOR = 1000000ULL;
constexpr int MQTT_RETRY_LIMIT = 5;
constexpr uint32_t DEV_MEASURE_INTERVAL_MS = 5000;
constexpr uint32_t NETWORK_RETRY_MS = 5000;
constexpr uint32_t MEASURING_HOLD_MS = 1000;

RTC_DATA_ATTR int distancePrev = 0;
RTC_DATA_ATTR bool prevWifiState = false;

unsigned long lastWifiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;
unsigned long lastMeasureMs = 0;
unsigned long lastPublishMs = 0;
unsigned long measuringSinceMs = 0;
bool measuringHold = false;
bool referenceRequested = false;

uint32_t configuredIntervalMs()
{
  const int seconds = settings.time_to_sleep > 0 ? settings.time_to_sleep : 1;
  return (uint32_t)seconds * 1000UL;
}

uint32_t measureIntervalMs()
{
  if (settings.dev_mode)
  {
    return DEV_MEASURE_INTERVAL_MS;
  }
  return configuredIntervalMs();
}

bool heartbeatDue()
{
  if (settings.heartbeat <= 0)
  {
    return false;
  }
  return ((currentBootCount() * settings.time_to_sleep) % settings.heartbeat) == 0;
}

bool setupWifi()
{
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  logLine("Connecting WiFi: %s", WIFI_SSID);

  // One logical state for the whole attempt. Pulsing only blinks the lamp,
  // so MQTT does not flap between "WLAN verbindet" and "Schlaeft".
  statusLedSet(StatusLed::WifiConnecting);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < settings.wifi_timeout * 10)
  {
    ++retries;
    statusLedPulse();
    delay(50);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    logLine("WiFi failed");
    prevWifiState = false;
    return false;
  }

  prevWifiState = true;
  logLine("WiFi connected IP=%s", WiFi.localIP().toString().c_str());
  return true;
}

// Steady LED state derived from what is going on right now.
void showStatusLed()
{
  StatusLed state = StatusLed::Online;
  if (WiFi.status() != WL_CONNECTED)
  {
    state = StatusLed::WifiFailed;
  }
  else if (!mqttConnected())
  {
    state = StatusLed::MqttFailed;
  }
  else if (settings.dev_mode)
  {
    state = StatusLed::Dev;
  }
  // statusLedSet publishes the same text. Skip it while a measurement is still
  // on display, so the remote LED stays on "Messung" for at least a second.
  const unsigned long now = millis();
  if (measuringHold && now - measuringSinceMs < MEASURING_HOLD_MS)
  {
    return;
  }
  measuringHold = false;
  statusLedSet(state);
}

void serviceNetwork(unsigned long now)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    if (now - lastWifiRetryMs > NETWORK_RETRY_MS)
    {
      lastWifiRetryMs = now;
      setupWifi();
    }
    return;
  }

  if (!mqttConnected())
  {
    if (now - lastMqttRetryMs > NETWORK_RETRY_MS)
    {
      lastMqttRetryMs = now;
      statusLedSet(StatusLed::MqttConnecting);
      if (!mqttConnectOnce())
      {
        mqttRegisterConnectFailure();
        if (mqttFailedAttempts() >= MQTT_RETRY_LIMIT)
        {
          webForceMonitorOnMqttFailure();
        }
      }
      else
      {
        mqttResetFailures();
      }
    }
    return;
  }

  mqttLoop();
}

void deepSleep()
{
  if (!powerMayDeepSleep(settings.battery_mode))
  {
    logLine("Deep sleep refused: battery mode is off");
    return;
  }
  logLine("Going to sleep now");
  // An RGB LED keeps its last color during deep sleep.
  statusLedSet(StatusLed::Off);
  esp_wifi_stop();
  delay(20);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)settings.time_to_sleep * US_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void goToBatterySleep()
{
  if (!powerMayDeepSleep(settings.battery_mode))
  {
    logLine("Deep sleep refused: battery mode is off");
    return;
  }
  if (mqttConnected())
  {
    // Publish "Schlaeft" and switch the lamp off while the socket is still up.
    statusLedSet(StatusLed::Off);
    delay(100);
    mqttDisconnect();
  }
  WiFi.mode(WIFI_OFF);
  deepSleep();
}

bool stayAwake()
{
  return powerStayAwake(settings.battery_mode, settings.dev_mode, webMonitorIsRunning(),
                        settings.web_monitor_enabled || webMonitorForcedByFailure());
}
} // namespace

void setDevMode(bool enabled)
{
  if (settings.dev_mode == enabled)
  {
    return;
  }
  settings.dev_mode = enabled;
  saveSettingsToNvs();
  // Do not measure right away: a measurement blocks the loop for up to ~1.5 s
  // and the page refreshes status immediately after this request.
  lastMeasureMs = millis();
  if (enabled)
  {
    logLine("DEV mode ON: measuring every 5 s, MQTT every %d s", settings.time_to_sleep);
  }
  else
  {
    logLine("DEV mode OFF: measuring every %d s", settings.time_to_sleep);
  }
}

bool devModeEnabled()
{
  return settings.dev_mode;
}

void requestReferenceMeasurement()
{
  referenceRequested = true;
}

bool referenceMeasurementPending()
{
  return referenceRequested;
}

bool runReferenceMeasurement()
{
  referenceRequested = false;
  int measuredCm = 0;
  statusLedSet(StatusLed::Measuring);
  measuringHold = true;
  measuringSinceMs = millis();
  logLine("Reference measurement");
  const bool ok = measureDistance(measuredCm, true);
  // Stamp this after the blocking read. A timestamp from before the burst
  // can sit behind the loop's captured millis() and wrap the interval check.
  lastMeasureMs = millis();
  measuringSinceMs = lastMeasureMs;
  if (mqttConnected())
  {
    // One press, one attempt. Clearing on failure too stops a dead sensor
    // from bypassing the check on every later wake.
    mqttClearReferenceCommand();
  }
  if (!ok)
  {
    logLine("Reference measurement failed");
    return false;
  }
  distancePrev = measuredCm;
  if (mqttConnected())
  {
    publishLastValidState();
    lastPublishMs = millis();
  }
  return true;
}

void setup()
{
  statusLedBegin();
  Serial.begin(115200);

  nvs_flash_init();
  sensorBegin();
  loadSettingsFromNvs();
  otaBootInit();
  incrementBootCount();
  logRuntimeSettings();
  logLine("Firmware %s, boot number: %d, reset reason: %d", FW_VERSION, currentBootCount(), (int)esp_reset_reason());

  int measuredCm = 0;
  statusLedSet(StatusLed::Measuring);
  bool measuredOk = measureDistance(measuredCm);
  const bool changed = measuredOk && measuredCm != distancePrev;
  const bool shouldPublish = hasValidDistance() && (changed || heartbeatDue() || !prevWifiState);

  // Always connect, not only when there is something to publish: retained MQTT
  // commands (web monitor switch, reference measurement) are only delivered
  // while connected, which is the only chance to reach a device that spends
  // its time in deep sleep.
  const bool wifiUp = setupWifi();
  bool mqttOk = false;
  bool publishedReference = false;
  if (wifiUp)
  {
    statusLedSet(StatusLed::MqttConnecting);
    mqttOk = mqttConnectWithRetries(MQTT_RETRY_LIMIT);
    // The boot sample already ran, before this command could arrive.
    if (referenceMeasurementPending())
    {
      if (runReferenceMeasurement())
      {
        measuredOk = true;
        measuredCm = lastValidDistanceCm();
        publishedReference = true;
      }
    }
    if (mqttOk && shouldPublish && !publishedReference)
    {
      publishLastValidState();
    }
  }
  showStatusLed();

  // Without Wi-Fi the web monitor could not be reached anyway, so only force it
  // when the network is up but the broker is not.
  if (wifiUp && !mqttOk)
  {
    mqttSetFailedAttempts(MQTT_RETRY_LIMIT);
  }

  if (mqttFailedAttempts() >= MQTT_RETRY_LIMIT)
  {
    webForceMonitorOnMqttFailure();
  }

  if (measuredOk)
  {
    distancePrev = measuredCm;
  }

  const unsigned long now = millis();
  lastMeasureMs = now;
  lastPublishMs = now;

  if (settings.web_monitor_enabled || webMonitorForcedByFailure() || settings.dev_mode)
  {
    // Wi-Fi and MQTT are retried by serviceNetwork() in loop().
    if (otaShouldForceWeb(settings.web_monitor_enabled, webMonitorForcedByFailure()))
    {
      startWebMonitor();
    }
    return;
  }

  if (powerMayDeepSleep(settings.battery_mode))
  {
    goToBatterySleep();
    return;
  }

  logLine("Battery mode OFF: keep device awake");
}

void loop()
{
  const unsigned long now = millis();

  if (mqttConnected())
  {
    // Apply a retained monitor command before the sleep decision. Otherwise
    // a sleeping device can miss ON and go straight back to sleep.
    mqttLoop();
  }

  if (webMonitorIsRunning())
  {
    webHandleClient();
  }

  // Reopen the listener after a Wi-Fi reconnect, the old socket may be dead.
  static bool wifiWasConnected = (WiFi.status() == WL_CONNECTED);
  const bool wifiNow = (WiFi.status() == WL_CONNECTED);
  if (wifiNow && !wifiWasConnected && webMonitorIsRunning())
  {
    webReopenListener();
  }
  wifiWasConnected = wifiNow;

  // A command that arrived in the mqttLoop above has to run before sleep.
  // The web button calls runReferenceMeasurement() itself and does not wait for this.
  if (referenceMeasurementPending())
  {
    runReferenceMeasurement();
  }

  if (!stayAwake())
  {
    // Only reachable in battery mode: nothing needs the device any more.
    goToBatterySleep();
    return;
  }

  serviceNetwork(now);

  // millis() here, not the captured `now`: a reference read stamps lastMeasureMs
  // after `now`, and an unsigned subtract would wrap and measure again at once.
  if (referenceMeasurementPending())
  {
    runReferenceMeasurement();
  }
  else if (millis() - lastMeasureMs >= measureIntervalMs())
  {
    lastMeasureMs = millis();
    int measuredCm = 0;
    // Publish before the read blocks the loop, then keep the state visible.
    statusLedSet(StatusLed::Measuring);
    measuringHold = true;
    measuringSinceMs = lastMeasureMs;
    if (measureDistance(measuredCm))
    {
      distancePrev = measuredCm;
    }
  }

  // millis(), not the captured `now`: a reference publish stamps lastPublishMs
  // after `now`, and an unsigned subtract would wrap and publish again at once.
  if (millis() - lastPublishMs >= configuredIntervalMs())
  {
    lastPublishMs = millis();
    if (mqttConnected())
    {
      publishLastValidState();
    }
  }

  showStatusLed();
  delay(webMonitorIsRunning() ? 10 : 50);
}

#include "status_led.h"

#include "mqtt.h"

#include <Arduino.h>

namespace
{
bool known = false;
bool lampLit = true;
StatusLed current = StatusLed::Off;

#ifdef STATUS_LED_RGB

// Percent of full brightness. The LED is lit all the time while awake.
#ifndef STATUS_LED_BRIGHTNESS_PERCENT
#define STATUS_LED_BRIGHTNESS_PERCENT 10
#endif

struct Rgb
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

Rgb colorFor(StatusLed state)
{
  switch (state)
  {
  case StatusLed::WifiConnecting:
    return {0, 0, 255};
  case StatusLed::WifiFailed:
    return {255, 0, 0};
  case StatusLed::MqttConnecting:
    return {0, 255, 255};
  case StatusLed::MqttFailed:
    return {255, 80, 0};
  case StatusLed::Measuring:
    return {255, 255, 255};
  case StatusLed::Dev:
    return {200, 0, 255};
  case StatusLed::Online:
    return {0, 255, 0};
  case StatusLed::Off:
  default:
    return {0, 0, 0};
  }
}

void write(StatusLed state)
{
  const Rgb c = colorFor(state);
  neopixelWrite(RGB_BUILTIN,
                (uint8_t)(c.r * STATUS_LED_BRIGHTNESS_PERCENT / 100),
                (uint8_t)(c.g * STATUS_LED_BRIGHTNESS_PERCENT / 100),
                (uint8_t)(c.b * STATUS_LED_BRIGHTNESS_PERCENT / 100));
}

void begin() {}

#else

constexpr int LED_PIN = 2;

void write(StatusLed state)
{
  digitalWrite(LED_PIN, state == StatusLed::WifiConnecting ? HIGH : LOW);
}

void begin()
{
  pinMode(LED_PIN, OUTPUT);
}

#endif

// Physical lamp only. The non-RGB board has no color, so any requested
// state lights GPIO2. Callers that blink must not go through statusLedSet.
void writeBlink(StatusLed state, bool on)
{
#ifdef STATUS_LED_RGB
  write(on ? state : StatusLed::Off);
#else
  (void)state;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif
}
} // namespace

void statusLedBegin()
{
  begin();
  known = false;
  statusLedSet(StatusLed::Off);
}

const char *statusLedName(StatusLed state)
{
  switch (state)
  {
  case StatusLed::WifiConnecting:
    return "WLAN verbindet";
  case StatusLed::WifiFailed:
    return "WLAN Fehler";
  case StatusLed::MqttConnecting:
    return "MQTT verbindet";
  case StatusLed::MqttFailed:
    return "MQTT Fehler";
  case StatusLed::Measuring:
    return "Messung";
  case StatusLed::Dev:
    return "Online (DEV)";
  case StatusLed::Online:
    return "Online";
  case StatusLed::Off:
  default:
    return "Schlaeft";
  }
}

void statusLedSet(StatusLed state)
{
  if (known && state == current)
  {
    return;
  }
  known = true;
  current = state;
  lampLit = true;
  write(state);
  mqttPublishDiagnostics(statusLedName(state));
}

void statusLedPulse()
{
  lampLit = !lampLit;
  write(lampLit ? current : StatusLed::Off);
}

void statusLedBlinkFor(StatusLed state, uint32_t durationMs)
{
  constexpr uint32_t halfPeriodMs = 250;
  const uint32_t started = millis();
  bool on = false;
  while ((uint32_t)(millis() - started) < durationMs)
  {
    on = !on;
    writeBlink(state, on);
    const uint32_t elapsed = (uint32_t)(millis() - started);
    if (elapsed >= durationMs)
    {
      break;
    }
    const uint32_t remain = durationMs - elapsed;
    delay(remain < halfPeriodMs ? remain : halfPeriodMs);
  }
}

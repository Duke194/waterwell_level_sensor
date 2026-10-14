#pragma once

// Status LED. The ESP32-S3 build (STATUS_LED_RGB) drives the on-board RGB LED.
// The classic ESP-WROOM-32 build keeps its plain LED on GPIO2: it is on only
// while Wi-Fi is connecting and off in every other state.
enum class StatusLed
{
  Off,            // Asleep or nothing to show
  WifiConnecting, // Blue. The lamp blinks locally; the logical state stays this one.
  WifiFailed,     // Red: no Wi-Fi
  MqttConnecting, // Cyan: Wi-Fi is up, connecting to the broker
  MqttFailed,     // Orange: Wi-Fi is up, broker is not reachable
  OtaPending,     // Yellow: new firmware is not confirmed yet
  Measuring,      // White flash while the sensor is read
  Dev,            // Magenta: DEV mode, online
  Online          // Green: everything fine
};

void statusLedBegin();
// Writes to the LED only when the state changed, and publishes that same text
// to the MQTT status topic when the broker is reachable.
void statusLedSet(StatusLed state);
// Toggles the lamp on and off without changing the logical state or MQTT.
void statusLedPulse();

// Short readable text for a state, shown as diagnostic status in Home Assistant.
const char *statusLedName(StatusLed state);

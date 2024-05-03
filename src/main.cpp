#include <vector>
#include <numeric>
#include <math.h>
#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <ArduinoJson.h>

#include <PubSubClient.h>

#include "settings.h"

#define echoPin 5     // default: 15 //13 // attach pin D13 Arduino to pin Echo of JSN-SR04T
#define trigPin 26    // default: 27 //12 //attach pin D12 Arduino to pin Trig of JSN-SR04T
#define LED_BUILTIN 2 // This is 2 for ESP-WROOM-32

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);

bool done = false;
byte msgBuf[64];
uint32_t msgLen;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int distance_prev = 0;
RTC_DATA_ATTR bool prev_wifistate = false;
// Settings structure definition
RTC_DATA_ATTR struct Settings
{
  int time_to_sleep = TIME_TO_SLEEP;
  int heartbeat = HEARTBEAT;
  int wifi_timeout = WIFI_TIMEOUT;
  int sensor_timeout = SENSOR_TIMEOUT;
  int buffer_size = BUFFER_SIZE;
  float area = AREA;
  float volume_well = VOLUME_WELL;
} settings;

// Function to update integer settings
bool updateIntSetting(const char *payload, int &setting)
{
  int value = atoi(payload);
  if (value != setting)
  {
    setting = value;
    return true;
  }
  return false;
}

// Function to update float settings
bool updateFloatSetting(const char *payload, float &setting)
{
  float value = atof(payload);
  if (value != setting)
  {
    setting = value;
    return true;
  }
  return false;
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.printf("Received message on topic %s: %.*s\n", topic, length, payload);

  if (length > 63)
  { // Check payload length to avoid buffer overflow
    Serial.println("Payload is too long!");
    return;
  }

  // Ensure null-terminated string for safe string operations
  char safePayload[65]; // Allocate buffer with space for null terminator
  memcpy(safePayload, payload, length);
  safePayload[length] = '\0'; // Null-terminate the copied payload

  if (strncmp(topic, MQTT_SETTINGS_TOPIC, strlen(MQTT_SETTINGS_TOPIC)) == 0)
  {
    char *settingKey = topic + strlen(MQTT_SETTINGS_TOPIC) + 1; // Move past the settings topic

    // Determine and update the appropriate setting based on the topic
    if (strcmp(settingKey, "time_to_sleep") == 0)
    {
      if (updateIntSetting(safePayload, settings.time_to_sleep))
        Serial.printf("Updated time_to_sleep to %d\n", settings.time_to_sleep);
    }
    else if (strcmp(settingKey, "heartbeat") == 0)
    {
      if (updateIntSetting(safePayload, settings.heartbeat))
        Serial.printf("Updated heartbeat to %d\n", settings.heartbeat);
    }
    else if (strcmp(settingKey, "wifi_timeout") == 0)
    {
      if (updateIntSetting(safePayload, settings.wifi_timeout))
        Serial.printf("Updated wifi_timeout to %d\n", settings.wifi_timeout);
    }
    else if (strcmp(settingKey, "sensor_timeout") == 0)
    {
      if (updateIntSetting(safePayload, settings.sensor_timeout))
        Serial.printf("Updated sensor_timeout to %d\n", settings.sensor_timeout);
    }
    else if (strcmp(settingKey, "buffer_size") == 0)
    {
      if (updateIntSetting(safePayload, settings.buffer_size))
        Serial.printf("Updated buffer_size to %d\n", settings.buffer_size);
    }
    else if (strcmp(settingKey, "area") == 0)
    {
      if (updateFloatSetting(safePayload, settings.area))
        Serial.printf("Updated area to %.2f\n", settings.area);
    }
    else if (strcmp(settingKey, "volume_well") == 0)
    {
      if (updateFloatSetting(safePayload, settings.volume_well))
        Serial.printf("Updated volume_well to %.2f liters\n", settings.volume_well);
    }
  }
  else if (strcmp(topic, MQTT_SETTINGS_TOPIC "/ping") == 0)
  {
    done = true;
    Serial.printf("Ping received: %d\n", atoi(safePayload));
  }
}

int measure_distance()
{
  std::vector<int> distances;
  unsigned long duration = 0;
  int distance = 0;
  int retries = 0;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  while (distances.size() < settings.buffer_size && retries < settings.sensor_timeout || CONT_MEASURE)
  {
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000); // Using a timeout of 30000 microseconds

    distance = round(float(duration) * 0.0343 / 2.0 + CALIBRATION_OFFSET);

    if (distance > SENSOR_RANGE_MIN && distance < SENSOR_RANGE_MAX)
    {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      distances.push_back(distance);
    }
    else
    {
      // Serial.println("Distance out of range.");
    }
    delay(RETRY_DELAY);
    ++retries;
  }

  int average_distance = 0;
  if (!distances.empty())
  {
    average_distance = round(accumulate(distances.begin(), distances.end(), 0.0) / distances.size());
  }
  else
  {
    Serial.println("No valid measurements received.");
  }

  Serial.print("Average Distance: ");
  Serial.print(average_distance);
  Serial.println(" cm");
  return average_distance;
}

int setup_wifi()
{
  // Make sure Wifi settings in flash are off so it doesn't start automagically at next boot
  if (WiFi.getMode() != WIFI_OFF)
  {
    printf("Wifi wasn't off!\n");
    WiFi.persistent(true);
    WiFi.mode(WIFI_OFF);
  }

  // Connect to wifi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to " WIFI_SSID);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < settings.wifi_timeout * 10)
  {
    Serial.print(".");
    ++retries;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Failed!!");
    prev_wifistate = false;
    return 0;
  }

  prev_wifistate = true;
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Connected");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  return 1;
}

void publish_discovery_message()
{
  StaticJsonDocument<512> doc;
  doc["name"] = MQTT_CLIENT_ID;
  doc["state_topic"] = MQTT_STATE_TOPIC;
  doc["unit_of_measurement"] = "L";
  doc["value_template"] = "{{ value_json.volume }}";
  doc["device_class"] = "volume";
  doc["unique_id"] = MQTT_CLIENT_ID;
  doc["availability_topic"] = MQTT_WILL_TOPIC;
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";

  char buffer[512];
  serializeJson(doc, buffer);
  // Serial.println("Discovery message: ");
  // Serial.println(buffer);
  if (!mqttclient.publish(MQTT_DISCOVERY_TOPIC, buffer, true))
  {
    Serial.println("Failed to publish discovery message!");
  }
  else
  {
    Serial.println("Published discovery message!");
  }
}

void publish_state(int volume_l)
{
  if (volume_l < 0)
    volume_l = 0;

  Serial.print("Publishing to " MQTT_STATE_TOPIC ": ");
  Serial.print(volume_l);
  Serial.println(" litres");

  StaticJsonDocument<100> doc;
  doc["volume"] = volume_l;
  char buffer[100];
  serializeJson(doc, buffer);
  mqttclient.publish(MQTT_STATE_TOPIC, buffer, true);
  delay(100);
}

bool connect_mqtt()
{
  mqttclient.setServer(MQTT_HOST, MQTT_PORT);
  mqttclient.setCallback(mqttCallback);
  mqttclient.setBufferSize(512);

  Serial.print("Connecting to MQTT server ");
  Serial.print(MQTT_HOST);
  Serial.print("...");

  // Attempt to connect with LWT
  if (mqttclient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, MQTT_WILL_TOPIC, 0, true, "online"))
  {
    Serial.println("Connected to MQTT Broker!");
    delay(100);
    publish_discovery_message();

    mqttclient.subscribe(MQTT_SETTINGS_TOPIC "/#");
    delay(100); // Wait for subscription acknowledgment

    Serial.println("Publishing ping");
    mqttclient.publish(MQTT_SETTINGS_TOPIC "/ping", String(bootCount).c_str());
    unsigned long startWait = millis();
    while (!done && (millis() - startWait) < 20000)
    {
      mqttclient.loop();
      delay(10); // Prevent tight loop, allow processing time
    }
    if (!done)
    {
      Serial.println("Ping not received, giving up!!");
    }
    else
    {
      Serial.println("Ping received");
      done = false;
    }

    return true;
  }
  else
  {
    Serial.print("Connection to MQTT Broker failed! ERROR_CODE: ");
    Serial.println(mqttclient.state()); // Print the connection error code for troubleshooting
    return false;
  }
}

void deep_sleep()
{
  Serial.println("Going to sleep now");

  // Explicitely stop wifi before sleep otherwise reconnect on wake fails
  esp_wifi_stop();

  delay(20);
  Serial.flush();

  // Set wakeup timer
  esp_sleep_enable_timer_wakeup(settings.time_to_sleep * uS_TO_S_FACTOR);

  // Enter deep sleep
  esp_deep_sleep_start();
}

void printSettings()
{
  Serial.println("Settings:");
  Serial.print("- heartbeat: ");
  Serial.println(settings.heartbeat);
  Serial.print("- time_to_sleep: ");
  Serial.println(settings.time_to_sleep);
  Serial.print("- wifi_timeout: ");
  Serial.println(settings.wifi_timeout);
  Serial.print("- sensor_timeout: ");
  Serial.println(settings.sensor_timeout);
  Serial.print("- buffer_size: ");
  Serial.println(settings.buffer_size);
  Serial.print("- area (m^2): ");
  Serial.println(settings.area);
  Serial.print("- volume of the well (liters): ");
  Serial.println(settings.volume_well);
}

void setup()
{
  done = false;
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);  // Sets the echoPin as an INPUT
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);                            // // Serial Communication is starting with 9600 of baud rate speed
  Serial.println("\nUltrasonic Sensor SR04M-2"); // print some text in Serial Monitor
  printSettings();

  // Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  int distance = measure_distance();
  if ((distance != distance_prev && distance != 0) ||                   // don't send if no change or invalid measurement
      (bootCount * settings.time_to_sleep) % settings.heartbeat == 0 || // but do send on heartbeat (even if invalid)
      !prev_wifistate)                                                  // or when previous wifi connect attempt failed
  {
    if (setup_wifi() && connect_mqtt())
    {
      int volume_l_empty = round((distance * 1e-1) * (settings.area * 1e2)); // We need to convert distance (cm to dm) and area (m² to dm²). This yield the volume in dm³ (litres)
      int volume_l = round(settings.volume_well - volume_l_empty);

      publish_state(volume_l);

      // disconnect mqtt and wifi
      mqttclient.disconnect();
      // WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // Save distance for reference
  distance_prev = distance;

  deep_sleep();
}

void loop()
{
  // Program will never come here as we enter deep sleep during setup.
}
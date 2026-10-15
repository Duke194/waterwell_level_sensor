#include "sensor.h"

#include "config.h"
#include "diag.h"
#include "log.h"
#include "plausibility.h"
#include "settings.h"

#include <Arduino.h>
#include <algorithm>
#include <math.h>
#include <numeric>
#include <vector>

namespace
{
// Overridden per board in platformio.ini. The ESP32-S3 DevKitC-1 does not
// break out GPIO26, so that build sets TRIG_PIN to 4.
#ifndef ECHO_PIN
#define ECHO_PIN 5
#endif
#ifndef TRIG_PIN
#define TRIG_PIN 26
#endif

#ifndef BELOW_MIN_HITS_THRESHOLD
#define BELOW_MIN_HITS_THRESHOLD 3
#endif

#ifndef BELOW_MIN_STREAK_THRESHOLD
#define BELOW_MIN_STREAK_THRESHOLD 2
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int lastPlausibleDistance = 0;
RTC_DATA_ATTR uint8_t belowMinCycleStreak = 0;
RTC_DATA_ATTR int lastValidDistance = 0;
RTC_DATA_ATTR bool validDistanceKnown = false;
RTC_DATA_ATTR int plausibilityPendingCm = 0;
RTC_DATA_ATTR uint8_t plausibilityPendingStreak = 0;

void logHeldValue(const char *reason)
{
  if (validDistanceKnown)
  {
    const int volumeL = computeVolumeLitersFromDistance(lastValidDistance);
    logLine("%s, holding last value: %d cm (%d L)", reason, lastValidDistance, volumeL);
    return;
  }
  logLine("%s, no previous value", reason);
}

void logBelowMinHold(const char *reason, int belowMinHits)
{
  if (validDistanceKnown)
  {
    const int volumeL = computeVolumeLitersFromDistance(lastValidDistance);
    logLine("%s (hits=%d, streak=%u), holding last value: %d cm (%d L)",
            reason, belowMinHits, belowMinCycleStreak, lastValidDistance, volumeL);
    return;
  }
  logLine("%s (hits=%d, streak=%u), no previous value", reason, belowMinHits, belowMinCycleStreak);
}
} // namespace

void sensorBegin()
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void incrementBootCount()
{
  ++bootCount;
}

int currentBootCount()
{
  return bootCount;
}

int computeVolumeLitersFromDistance(int distanceCm)
{
  // At max fill the sensor still sees a small remaining distance.
  float effectiveDistanceCm = (float)distanceCm - settings.sensor_to_max_fill_cm;
  if (effectiveDistanceCm < 0.0f)
  {
    effectiveDistanceCm = 0.0f;
  }

  const int volumeEmptyL = round((effectiveDistanceCm * 1e-1f) * (settings.area * 1e2f));
  int volumeL = round(settings.volume_well - volumeEmptyL);
  if (volumeL < 0)
  {
    volumeL = 0;
  }
  return volumeL;
}

int computeEffectiveDistanceCm(int distanceCm)
{
  float effectiveDistanceCm = (float)distanceCm - settings.sensor_to_max_fill_cm;
  if (effectiveDistanceCm < 0.0f)
  {
    effectiveDistanceCm = 0.0f;
  }
  return (int)round(effectiveDistanceCm);
}

bool hasValidDistance()
{
  return validDistanceKnown;
}

int lastValidDistanceCm()
{
  return lastValidDistance;
}

bool measureDistance(int &distanceCm)
{
  std::vector<int> distances;
  int retries = 0;
  int belowMinHits = 0;
  int echoCount = 0;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  while ((distances.size() < (size_t)settings.buffer_size && retries < settings.sensor_timeout) || CONT_MEASURE)
  {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);

    const unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    const int distance = round(float(duration) * 0.0343f / 2.0f + settings.calibration_offset);

    // Count below-min only for real echoes, not for pulse timeout (duration==0).
    if (duration > 0)
    {
      ++echoCount;
    }
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
      logBelowMinHold("Below-min detected", belowMinHits);
      diagSetError(ErrorSource::Sensor, "Sensor: Echo unter Mindestabstand");
    }
    else
    {
      logHeldValue("No valid measurement");
      diagSetError(ErrorSource::Sensor, echoCount == 0 ? "Sensor: kein Echo" : "Sensor: kein gueltiger Messwert");
    }
    return false;
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

  const bool enoughValidDistances = (int)distances.size() >= minValidForOverride;
  if (!enoughValidDistances && (belowMinCycleStreak >= BELOW_MIN_STREAK_THRESHOLD || belowMinHits >= dynamicBelowMinThreshold))
  {
    logBelowMinHold("Below-min override", belowMinHits);
    diagSetError(ErrorSource::Sensor, "Sensor: Echo unter Mindestabstand");
    return false;
  }

  PlausibilityState plausibility{lastPlausibleDistance, plausibilityPendingCm, plausibilityPendingStreak};
  const PlausibilityDecision decision = plausibilityDecide(plausibility, filtered, PLAUSIBILITY_MAX_STEP_CM,
                                                          PLAUSIBILITY_OUTLIER_DELTA_CM, PLAUSIBILITY_CONFIRM_COUNT);
  lastPlausibleDistance = plausibility.acceptedCm;
  plausibilityPendingCm = plausibility.pendingCm;
  plausibilityPendingStreak = plausibility.pendingStreak;

  diagClearError(ErrorSource::Sensor);
  belowMinCycleStreak = 0;
  lastValidDistance = decision.distanceCm;
  validDistanceKnown = true;
  distanceCm = decision.distanceCm;

  logLine("Measured distance: %d cm (median=%d, kept=%d/%d)",
          filtered, median, (int)plausible.size(), (int)distances.size());
  if (decision.held)
  {
    logLine("Plausibility hold: raw=%d cm prev=%d cm pending=%d cm streak=%u/%d",
            filtered, decision.distanceCm, decision.pendingCm, (unsigned int)decision.pendingStreak,
            PLAUSIBILITY_CONFIRM_COUNT);
  }
  else
  {
    logLine("Plausibility accept: %d cm", decision.distanceCm);
  }
  const int volumeL = computeVolumeLitersFromDistance(decision.distanceCm);
  logLine("Calculated volume: %d L (%.3f m3)", volumeL, (float)volumeL / 1000.0f);
  return true;
}

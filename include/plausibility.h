#pragma once

#include <stdint.h>

// Immediate step that is still tracked without confirmation. At 2 m2 that is
// 400 L per sample, which covers heavy rain on a normal roof. The allowance
// does not grow with time: a late one-off echo must not become the new level.
#ifndef PLAUSIBILITY_MAX_STEP_CM
#define PLAUSIBILITY_MAX_STEP_CM 20
#endif

// Samples in one burst, and later confirmation bursts, must sit in this band.
#ifndef PLAUSIBILITY_OUTLIER_DELTA_CM
#define PLAUSIBILITY_OUTLIER_DELTA_CM 6
#endif

// A larger jump is published only after this many burst results agree.
#ifndef PLAUSIBILITY_CONFIRM_COUNT
#define PLAUSIBILITY_CONFIRM_COUNT 3
#endif

// acceptedCm == 0 means no level has been published yet.
// pendingCm is the first sample of a jump that still needs confirmation.
// The band stays fixed on that first sample so a walking series cannot
// confirm itself by drifting a few centimetres each time.
struct PlausibilityState
{
  int acceptedCm;
  int pendingCm;
  uint8_t pendingStreak;
};

struct PlausibilityDecision
{
  int distanceCm;
  bool held;
  int pendingCm;
  uint8_t pendingStreak;
};

inline int plausibilityAbs(int value)
{
  return value < 0 ? -value : value;
}

// Updates state. A held decision keeps acceptedCm and returns that distance.
inline PlausibilityDecision plausibilityDecide(PlausibilityState &state, int candidateCm, int maxStepCm,
                                               int agreeDeltaCm, int confirmCount)
{
  if (confirmCount < 1)
  {
    confirmCount = 1;
  }

  if (state.acceptedCm <= 0)
  {
    state.acceptedCm = candidateCm;
    state.pendingCm = 0;
    state.pendingStreak = 0;
    return {candidateCm, false, 0, 0};
  }

  if (plausibilityAbs(candidateCm - state.acceptedCm) <= maxStepCm)
  {
    state.acceptedCm = candidateCm;
    state.pendingCm = 0;
    state.pendingStreak = 0;
    return {candidateCm, false, 0, 0};
  }

  const bool pendingOpen = state.pendingStreak > 0 && (int)state.pendingStreak < confirmCount && state.pendingCm > 0;
  if (pendingOpen && plausibilityAbs(candidateCm - state.pendingCm) <= agreeDeltaCm)
  {
    state.pendingStreak++;
  }
  else
  {
    state.pendingCm = candidateCm;
    state.pendingStreak = 1;
  }

  if ((int)state.pendingStreak >= confirmCount)
  {
    state.acceptedCm = candidateCm;
    state.pendingCm = 0;
    state.pendingStreak = 0;
    return {candidateCm, false, 0, 0};
  }

  return {state.acceptedCm, true, state.pendingCm, state.pendingStreak};
}

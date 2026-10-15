#pragma once

void sensorBegin();
void incrementBootCount();
int currentBootCount();

// Writes distanceCm and returns true only for a new in-range sample.
// A miss keeps the previous valid distance untouched.
// bypassPlausibility stores that sample as the new level even when it jumps
// far from the last one, and drops a pending confirmation. Range checks stay.
bool measureDistance(int &distanceCm, bool bypassPlausibility = false);

int computeVolumeLitersFromDistance(int distanceCm);
int computeEffectiveDistanceCm(int distanceCm);
bool hasValidDistance();
int lastValidDistanceCm();

#pragma once

void sensorBegin();
void incrementBootCount();
int currentBootCount();

// Writes distanceCm and returns true only for a new in-range sample.
// A miss keeps the previous valid distance untouched.
bool measureDistance(int &distanceCm);

int computeVolumeLitersFromDistance(int distanceCm);
int computeEffectiveDistanceCm(int distanceCm);
bool hasValidDistance();
int lastValidDistanceCm();

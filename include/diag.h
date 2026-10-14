#pragma once

// Firmware version as "vX.Y.Z". The build script takes it from the git tag.
#ifndef FW_VERSION
#define FW_VERSION "unknown"
#endif

// Each part of the firmware owns one error slot, so one part recovering does
// not hide an error of another part.
enum class ErrorSource
{
  Sensor,
  Ota,
  Count
};

// Sets the message of a slot. An empty or null message clears it.
void diagSetError(ErrorSource source, const char *message);
void diagClearError(ErrorSource source);

// First active error in slot order, or "Kein Fehler" when there is none.
const char *diagErrorText();

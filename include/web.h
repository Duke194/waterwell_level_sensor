#pragma once

#include <stdint.h>

void startWebMonitor();
void stopWebMonitor();
void webHandleClient();
bool webMonitorIsRunning();
uint32_t webMonitorStartedMs();
uint32_t webLastRequestMs();
bool webMonitorForcedByFailure();
void setWebMonitorEnabled(bool enable, bool persist);
void webForceMonitorOnMqttFailure();
void webReopenListener();

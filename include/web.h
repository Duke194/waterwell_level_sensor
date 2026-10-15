#pragma once

#include <stdint.h>

void startWebMonitor();
void stopWebMonitor();
void webHandleClient();
bool webMonitorIsRunning();
bool webMonitorForcedByFailure();
void setWebMonitorEnabled(bool enable, bool persist);
void webForceMonitorOnMqttFailure();
void webReopenListener();

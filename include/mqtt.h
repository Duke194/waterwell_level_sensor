#pragma once

bool mqttConnectOnce();
bool mqttConnectWithRetries(int maxRetries);
void mqttLoop();
bool mqttConnected();
void mqttDisconnect();

// Publishes the last in-range sample. No-op until one exists.
bool publishLastValidState();
// Drops a retained reference-measurement command so the next wake does not repeat it.
void mqttClearReferenceCommand();
void mqttPublishMonitorState();

// Publishes the status text and the current error text, each only when it
// differs from the last published value. No-op while MQTT is down.
void mqttPublishDiagnostics(const char *status);

int mqttFailedAttempts();
void mqttSetFailedAttempts(int count);
void mqttRegisterConnectFailure();
void mqttResetFailures();

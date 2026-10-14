#pragma once

#include "ota_image.h"

#include <stddef.h>
#include <stdint.h>

size_t otaNextSlotSize();
void otaBootInit();
bool otaBootAwaitingConfirm();
OtaBootDecision otaBootPoll(uint32_t uptime_ms, uint32_t web_started_ms, uint32_t last_served_ms);

#include "ota_boot.h"

#include "diag.h"
#include "log.h"

#include <esp_ota_ops.h>

namespace
{
bool awaitingConfirm = false;
bool confirmErrorLogged = false;
} // namespace

size_t otaNextSlotSize()
{
  const esp_partition_t *part = esp_ota_get_next_update_partition(nullptr);
  if (part == nullptr)
  {
    return 0;
  }
  return part->size;
}

void otaBootInit()
{
  awaitingConfirm = false;
  confirmErrorLogged = false;
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK)
  {
    return;
  }
  awaitingConfirm = state == ESP_OTA_IMG_PENDING_VERIFY;
  if (awaitingConfirm)
  {
    logLine("New firmware is unconfirmed, waiting for a real HTTP request before keeping it");
  }
}

bool otaBootAwaitingConfirm()
{
  return awaitingConfirm;
}

OtaBootDecision otaBootPoll(uint32_t uptime_ms, uint32_t web_started_ms, uint32_t last_served_ms)
{
  const OtaBootDecision decision = otaDecideBoot(awaitingConfirm, uptime_ms, web_started_ms, last_served_ms, OTA_CONFIRM_STABLE_MS, OTA_CONFIRM_TIMEOUT_MS);
  if (decision == OtaBootDecision::MarkValid)
  {
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
    {
      awaitingConfirm = false;
      diagClearError(ErrorSource::Ota);
      logLine("New firmware confirmed");
      return OtaBootDecision::MarkValid;
    }
    if (!confirmErrorLogged)
    {
      confirmErrorLogged = true;
      diagSetError(ErrorSource::Ota, "OTA: Bestaetigung fehlgeschlagen");
      logLine("Could not confirm new firmware");
    }
    return OtaBootDecision::StayPending;
  }
  if (decision == OtaBootDecision::RollBack)
  {
    logLine("New firmware was not reached over HTTP, rolling back");
    esp_ota_mark_app_invalid_rollback_and_reboot();
  }
  return decision;
}

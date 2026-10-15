#include "ota_boot.h"

#include "log.h"
#include "ota_image.h"

#include <esp_ota_ops.h>

#include <string.h>

// Kept in the app image so the other slot can show "Rollback vX.Y.Z".
// FW_VERSION is concatenated here, not passed through %s, so the bytes stay together.
const char kFwMark[] __attribute__((used)) = "waterwell-fw=" FW_VERSION;

namespace
{
char otherVersion[16];
bool otherVersionCached = false;
const char *otherVersionValue = nullptr;

bool findOtherVersion(const esp_partition_t *part)
{
  constexpr size_t kChunk = 1024;
  constexpr size_t kOverlap = 64;
  // The version literal lives in the first app segment, well under this.
  constexpr size_t kScanLimit = 256 * 1024;
  uint8_t buf[kChunk + kOverlap];
  size_t filled = 0;
  size_t offset = 0;
  const size_t limit = part->size < kScanLimit ? part->size : kScanLimit;

  while (offset < limit)
  {
    size_t want = kChunk;
    if (offset + want > limit)
    {
      want = limit - offset;
    }
    if (esp_partition_read(part, offset, buf + filled, want) != ESP_OK)
    {
      return false;
    }
    const size_t total = filled + want;
    if (otaCopyRollbackVersion(buf, total, otherVersion, sizeof(otherVersion)))
    {
      otherVersionValue = otherVersion;
      return true;
    }
    if (total > kOverlap)
    {
      memmove(buf, buf + total - kOverlap, kOverlap);
      filled = kOverlap;
    }
    else
    {
      filled = total;
    }
    offset += want;
  }
  return false;
}
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
  // Reference the mark so the linker keeps waterwell-fw=vX.Y.Z in the image.
  logLine("%s", kFwMark);
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (running == nullptr || esp_ota_get_state_partition(running, &state) != ESP_OK)
  {
    return;
  }
  if (state != ESP_OTA_IMG_PENDING_VERIFY)
  {
    return;
  }
  if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK)
  {
    logLine("New firmware kept");
  }
  else
  {
    logLine("Could not keep new firmware");
  }
}

const char *otaOtherVersion()
{
  if (otherVersionCached)
  {
    return otherVersionValue;
  }
  otherVersionCached = true;
  otherVersionValue = nullptr;
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  esp_app_desc_t desc;
  if (other == nullptr || esp_ota_get_partition_description(other, &desc) != ESP_OK)
  {
    return nullptr;
  }
  findOtherVersion(other);
  return otherVersionValue;
}

bool otaRollbackToOther()
{
  const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
  esp_app_desc_t desc;
  if (other == nullptr || esp_ota_get_partition_description(other, &desc) != ESP_OK)
  {
    return false;
  }
  return esp_ota_set_boot_partition(other) == ESP_OK;
}

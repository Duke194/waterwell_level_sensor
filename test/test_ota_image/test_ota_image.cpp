#include "ota_image.h"
#include "power_policy.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace
{
int g_failed = 0;
const char *g_test = "";

#define CHECK(cond)                                                                          \
  do                                                                                         \
  {                                                                                          \
    if (!(cond))                                                                             \
    {                                                                                        \
      std::fprintf(stderr, "FAIL %s: %s (%s:%d)\n", g_test, #cond, __FILE__, __LINE__);      \
      ++g_failed;                                                                            \
    }                                                                                        \
  } while (0)

void putU32(std::vector<uint8_t> &out, uint32_t value)
{
  out.push_back((uint8_t)(value & 0xFF));
  out.push_back((uint8_t)((value >> 8) & 0xFF));
  out.push_back((uint8_t)((value >> 16) & 0xFF));
  out.push_back((uint8_t)((value >> 24) & 0xFF));
}

std::vector<uint8_t> markerPayload()
{
  const char *marker = otaImageMarker();
  std::vector<uint8_t> payload(marker, marker + std::strlen(marker));
  while ((payload.size() % 4u) != 0)
  {
    payload.push_back(0);
  }
  return payload;
}

std::vector<uint8_t> buildImage(const std::vector<uint8_t> &payload, uint16_t chip, uint32_t entry, uint32_t load_addr, bool append_hash, uint8_t checksum_xor)
{
  std::vector<uint8_t> image(24, 0);
  image[0] = 0xE9;
  image[1] = 1;
  image[4] = (uint8_t)(entry & 0xFF);
  image[5] = (uint8_t)((entry >> 8) & 0xFF);
  image[6] = (uint8_t)((entry >> 16) & 0xFF);
  image[7] = (uint8_t)((entry >> 24) & 0xFF);
  image[12] = (uint8_t)(chip & 0xFF);
  image[13] = (uint8_t)((chip >> 8) & 0xFF);
  image[23] = append_hash ? 1 : 0;
  putU32(image, load_addr);
  putU32(image, (uint32_t)payload.size());
  image.insert(image.end(), payload.begin(), payload.end());

  uint8_t checksum = 0xEF;
  for (uint8_t byte : payload)
  {
    checksum = (uint8_t)(checksum ^ byte);
  }
  checksum = (uint8_t)(checksum ^ checksum_xor);
  while ((image.size() % 16u) != 15u)
  {
    image.push_back(0);
  }
  image.push_back(checksum);
  if (append_hash)
  {
    image.insert(image.end(), 32, 0xAB);
  }
  return image;
}

std::vector<uint8_t> validImage()
{
  return buildImage(markerPayload(), 0, 0x40081000u, 0x3FFB0000u, true, 0);
}

OtaImageCheck inspect(const std::vector<uint8_t> &image, size_t slot, size_t chunk)
{
  OtaImageParser parser;
  otaImageParserReset(&parser, slot);
  for (size_t offset = 0; offset < image.size(); offset += chunk)
  {
    const size_t n = std::min(chunk, image.size() - offset);
    if (!otaImageParserAdd(&parser, image.data() + offset, n))
    {
      return otaImageParserFinish(&parser);
    }
  }
  return otaImageParserFinish(&parser);
}

bool commitImage(const std::vector<uint8_t> &image, size_t slot, bool password_ok, bool begin_ok, bool write_ok, const char **reason)
{
  OtaUploadGate gate;
  if (!otaGateStart(&gate, password_ok, slot))
  {
    if (reason != nullptr)
    {
      *reason = otaGateReason(&gate);
    }
    return false;
  }
  if (!otaGateNoteFlashBegin(&gate, begin_ok))
  {
    if (reason != nullptr)
    {
      *reason = otaGateReason(&gate);
    }
    return false;
  }
  for (size_t offset = 0; offset < image.size(); offset += 50)
  {
    const size_t n = std::min<size_t>(50, image.size() - offset);
    if (!otaGateWrite(&gate, image.data() + offset, n))
    {
      if (reason != nullptr)
      {
        *reason = otaGateReason(&gate);
      }
      return false;
    }
    if (!otaGateNoteFlashWrite(&gate, write_ok))
    {
      if (reason != nullptr)
      {
        *reason = otaGateReason(&gate);
      }
      return false;
    }
  }
  const bool accepted = otaGateFinish(&gate);
  if (reason != nullptr)
  {
    *reason = otaGateReason(&gate);
  }
  return accepted;
}

bool loadFirmware(std::vector<uint8_t> &image, std::string &used_path)
{
  const char *paths[] = {
      ".pio/build/esp-wroom-32/firmware.bin",
      "firmware.bin",
      "../.pio/build/esp-wroom-32/firmware.bin",
      "../../.pio/build/esp-wroom-32/firmware.bin",
  };
  for (const char *path : paths)
  {
    FILE *file = std::fopen(path, "rb");
    if (file == nullptr)
    {
      continue;
    }
    if (std::fseek(file, 0, SEEK_END) != 0)
    {
      std::fclose(file);
      continue;
    }
    const long length = std::ftell(file);
    if (length <= 0)
    {
      std::fclose(file);
      continue;
    }
    std::rewind(file);
    image.resize((size_t)length);
    const size_t read = std::fread(image.data(), 1, image.size(), file);
    std::fclose(file);
    if (read != image.size())
    {
      image.clear();
      continue;
    }
    used_path = path;
    return true;
  }
  return false;
}

void test_valid_image_may_reboot()
{
  const char *reason = "";
  const std::vector<uint8_t> image = validImage();
  const bool accepted = commitImage(image, OTA_DEFAULT_SLOT_BYTES, true, true, true, &reason);
  CHECK(accepted);
  CHECK(std::strcmp(reason, "ok") == 0);
  CHECK(otaMayReboot(accepted, true));
  CHECK(!otaMayReboot(accepted, false));
}

void test_byte_at_a_time_still_accepts()
{
  const std::vector<uint8_t> image = validImage();
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 1);
  CHECK(check.structure_ok);
  CHECK(check.has_marker);
}

void test_image_without_hash_is_accepted()
{
  const std::vector<uint8_t> image = buildImage(markerPayload(), 0, 0x400D0020u, 0x400D0020u, false, 0);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 7);
  CHECK(check.structure_ok);
  CHECK(check.has_marker);
}

void test_garbage_is_not_an_image()
{
  const std::vector<uint8_t> garbage = {'n', 'o', 't', '-', 'a', '-', 'f', 'i', 'r', 'm', 'w', 'a', 'r', 'e'};
  const OtaImageCheck check = inspect(garbage, OTA_DEFAULT_SLOT_BYTES, 4);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "not an esp32 app image") == 0);
  CHECK(!otaMayReboot(false, true));
}

void test_wrong_chip_is_rejected()
{
  // 0x0009 is the ESP32-S3 chip id. This host build expects the ESP32 id 0.
  const std::vector<uint8_t> image = buildImage(markerPayload(), 0x0009, 0x40081000u, 0x3FFB0000u, true, 0);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 8);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "wrong chip") == 0);
}

void test_bad_entry_is_rejected()
{
  const std::vector<uint8_t> image = buildImage(markerPayload(), 0, 0, 0x3FFB0000u, true, 0);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 8);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "bad entry point") == 0);
}

void test_foreign_firmware_without_marker_is_rejected()
{
  std::vector<uint8_t> payload(16, 0x11);
  const std::vector<uint8_t> image = buildImage(payload, 0, 0x40081000u, 0x3FFB0000u, true, 0);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 5);
  CHECK(check.structure_ok);
  CHECK(!check.has_marker);
  const char *reason = "";
  CHECK(!commitImage(image, OTA_DEFAULT_SLOT_BYTES, true, true, true, &reason));
  CHECK(std::strcmp(reason, "not a waterwell firmware") == 0);
  CHECK(!otaMayReboot(false, true));
}

void test_bad_checksum_is_rejected()
{
  const std::vector<uint8_t> image = buildImage(markerPayload(), 0, 0x40081000u, 0x3FFB0000u, true, 0x01);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 9);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "bad image checksum") == 0);
}

void test_truncated_image_is_rejected()
{
  std::vector<uint8_t> image = validImage();
  image.pop_back();
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 6);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "truncated image") == 0);
}

void test_segment_outside_memory_is_rejected()
{
  const std::vector<uint8_t> image = buildImage(markerPayload(), 0, 0x40081000u, 0x08000000u, true, 0);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 8);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "segment outside esp32 memory") == 0);
}

void test_image_larger_than_slot_is_rejected()
{
  const std::vector<uint8_t> image = validImage();
  const OtaImageCheck check = inspect(image, 32, 8);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "image larger than ota slot") == 0);
}

void test_trailing_ff_padding_is_accepted()
{
  std::vector<uint8_t> image = validImage();
  image.insert(image.end(), 16, 0xFF);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 10);
  CHECK(check.structure_ok);
  CHECK(check.has_marker);
}

void test_trailing_garbage_is_rejected()
{
  std::vector<uint8_t> image = validImage();
  image.push_back(0x11);
  const OtaImageCheck check = inspect(image, OTA_DEFAULT_SLOT_BYTES, 10);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "trailing garbage") == 0);
}

void test_bad_password_never_commits()
{
  const char *reason = "";
  CHECK(!commitImage(validImage(), OTA_DEFAULT_SLOT_BYTES, false, true, true, &reason));
  CHECK(std::strcmp(reason, "invalid password") == 0);
  CHECK(!otaMayReboot(false, true));
}

void test_missing_partition_never_commits()
{
  const char *reason = "";
  CHECK(!commitImage(validImage(), 0, true, true, true, &reason));
  CHECK(std::strcmp(reason, "no ota partition") == 0);
}

void test_flash_begin_failure_never_commits()
{
  const char *reason = "";
  CHECK(!commitImage(validImage(), OTA_DEFAULT_SLOT_BYTES, true, false, true, &reason));
  CHECK(std::strcmp(reason, "flash begin failed") == 0);
  CHECK(!otaMayReboot(false, true));
}

void test_flash_write_failure_never_commits()
{
  const char *reason = "";
  CHECK(!commitImage(validImage(), OTA_DEFAULT_SLOT_BYTES, true, true, false, &reason));
  CHECK(std::strcmp(reason, "flash write failed") == 0);
  CHECK(!otaMayReboot(false, true));
}

void test_rejected_upload_can_be_retried()
{
  OtaUploadGate gate;
  CHECK(!otaGateStart(&gate, false, OTA_DEFAULT_SLOT_BYTES));
  const char *reason = "";
  CHECK(commitImage(validImage(), OTA_DEFAULT_SLOT_BYTES, true, true, true, &reason));
  CHECK(std::strcmp(reason, "ok") == 0);
}

void test_rollback_label_reads_the_version_in_front_of_the_log_line()
{
  const char marked[] = "waterwell-fw=v9.8.7";
  char version[16];
  CHECK(otaCopyRollbackVersion(reinterpret_cast<const uint8_t *>(marked), sizeof(marked), version, sizeof(version)));
  CHECK(std::strcmp(version, "v9.8.7") == 0);

  const char image[] = "xx\0v1.2.3\0Firmware %s, boot number: %d";
  CHECK(otaCopyRollbackVersion(reinterpret_cast<const uint8_t *>(image), sizeof(image), version, sizeof(version)));
  CHECK(std::strcmp(version, "v1.2.3") == 0);

  const char junk[] = "Firmware %s, boot number";
  CHECK(!otaCopyRollbackVersion(reinterpret_cast<const uint8_t *>(junk), sizeof(junk) - 1, version, sizeof(version)));
  const char foreign[] = "esp-idf\0Firmware %s, boot number";
  CHECK(!otaCopyRollbackVersion(reinterpret_cast<const uint8_t *>(foreign), sizeof(foreign) - 1, version, sizeof(version)));
}

void test_web_stays_up_when_the_monitor_is_wanted()
{
  CHECK(!otaShouldForceWeb(false, false));
  CHECK(otaShouldForceWeb(true, false));
  CHECK(otaShouldForceWeb(false, true));
  CHECK(otaShouldForceWeb(true, true));
}

void test_device_never_sleeps_without_battery_mode()
{
  CHECK(!powerMayDeepSleep(false));
  CHECK(powerMayDeepSleep(true));
  // Battery mode off: always awake, whatever the web monitor is doing.
  for (int mask = 0; mask < 8; ++mask)
  {
    CHECK(powerStayAwake(false, (mask & 1) != 0, (mask & 2) != 0, (mask & 4) != 0));
  }
  // Battery mode on: sleeps only when nothing needs the device.
  CHECK(!powerStayAwake(true, false, false, false));
  CHECK(powerStayAwake(true, true, false, false));
  CHECK(powerStayAwake(true, false, true, false));
  CHECK(powerStayAwake(true, false, false, true));
}

void test_s3_firmware_is_rejected_on_esp32()
{
  std::vector<uint8_t> firmware;
  const char *path = "update/waterwell-ota-v0.1.0-esp32-s3.bin";
  FILE *file = std::fopen(path, "rb");
  if (file == nullptr)
  {
    std::fprintf(stderr, "skip %s: %s not built\n", g_test, path);
    return;
  }
  if (std::fseek(file, 0, SEEK_END) != 0)
  {
    std::fclose(file);
    CHECK(false);
    return;
  }
  const long length = std::ftell(file);
  std::rewind(file);
  firmware.resize((size_t)length);
  const size_t read = std::fread(firmware.data(), 1, firmware.size(), file);
  std::fclose(file);
  CHECK(read == firmware.size());
  const OtaImageCheck check = inspect(firmware, OTA_DEFAULT_SLOT_BYTES, 4096);
  CHECK(!check.structure_ok);
  CHECK(std::strcmp(check.reason, "wrong chip") == 0);
  char rollbackVersion[16];
  CHECK(otaCopyRollbackVersion(firmware.data(), firmware.size(), rollbackVersion, sizeof(rollbackVersion)));
  CHECK(std::strcmp(rollbackVersion, "v0.1.0") == 0);
}

void test_built_firmware_is_safe_to_flash()
{
  std::vector<uint8_t> firmware;
  std::string path;
  if (!loadFirmware(firmware, path))
  {
    std::fprintf(stderr, "FAIL %s: build esp-wroom-32 first, firmware.bin was not found\n", g_test);
    ++g_failed;
    return;
  }

  const OtaImageCheck intact = inspect(firmware, OTA_DEFAULT_SLOT_BYTES, 3);
  CHECK(intact.structure_ok);
  CHECK(intact.has_marker);
  char rollbackVersion[16];
  CHECK(otaCopyRollbackVersion(firmware.data(), firmware.size(), rollbackVersion, sizeof(rollbackVersion)));
  CHECK(rollbackVersion[0] == 'v');
  CHECK(std::strcmp(intact.reason, "ok") == 0);
  if (!intact.structure_ok || !intact.has_marker)
  {
    std::fprintf(stderr, "  firmware %s (%u bytes) reason: %s\n", path.c_str(), (unsigned)firmware.size(), intact.reason);
  }

  std::vector<uint8_t> truncated = firmware;
  truncated.pop_back();
  const OtaImageCheck short_image = inspect(truncated, OTA_DEFAULT_SLOT_BYTES, 1024);
  CHECK(!short_image.structure_ok);

  std::vector<uint8_t> corrupt = firmware;
  corrupt[100] = (uint8_t)(corrupt[100] ^ 0xFF);
  const OtaImageCheck broken = inspect(corrupt, OTA_DEFAULT_SLOT_BYTES, 1024);
  CHECK(!broken.structure_ok);
  CHECK(!otaMayReboot(false, true));

  const char *reason = "";
  const bool accepted = commitImage(firmware, OTA_DEFAULT_SLOT_BYTES, true, true, true, &reason);
  CHECK(accepted);
  CHECK(otaMayReboot(accepted, true));
  CHECK(!otaMayReboot(accepted, false));
}
} // namespace

int main()
{
#define RUN(fn)   \
  do              \
  {               \
    g_test = #fn; \
    fn();         \
  } while (0)

  RUN(test_valid_image_may_reboot);
  RUN(test_byte_at_a_time_still_accepts);
  RUN(test_image_without_hash_is_accepted);
  RUN(test_garbage_is_not_an_image);
  RUN(test_wrong_chip_is_rejected);
  RUN(test_bad_entry_is_rejected);
  RUN(test_foreign_firmware_without_marker_is_rejected);
  RUN(test_bad_checksum_is_rejected);
  RUN(test_truncated_image_is_rejected);
  RUN(test_segment_outside_memory_is_rejected);
  RUN(test_image_larger_than_slot_is_rejected);
  RUN(test_trailing_ff_padding_is_accepted);
  RUN(test_trailing_garbage_is_rejected);
  RUN(test_bad_password_never_commits);
  RUN(test_missing_partition_never_commits);
  RUN(test_flash_begin_failure_never_commits);
  RUN(test_flash_write_failure_never_commits);
  RUN(test_rejected_upload_can_be_retried);
  RUN(test_rollback_label_reads_the_version_in_front_of_the_log_line);
  RUN(test_web_stays_up_when_the_monitor_is_wanted);
  RUN(test_device_never_sleeps_without_battery_mode);
  RUN(test_s3_firmware_is_rejected_on_esp32);
  RUN(test_built_firmware_is_safe_to_flash);

#undef RUN

  if (g_failed != 0)
  {
    std::fprintf(stderr, "%d check(s) failed\n", g_failed);
    return 1;
  }
  std::printf("ota image gate ok\n");
  return 0;
}

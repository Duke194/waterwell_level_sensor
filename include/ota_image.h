#pragma once

#include <stddef.h>
#include <stdint.h>

// Bytes that must appear in an app image before it may be flashed.
// The running firmware contains the same string, so a build of this
// project passes and a foreign ESP32 binary does not.
const char *otaImageMarker();

// Arduino default 4MB scheme: app0/app1 are each 0x140000 bytes.
constexpr size_t OTA_DEFAULT_SLOT_BYTES = 0x140000;

enum class OtaParsePhase : uint8_t
{
  Header = 0,
  SegHeader,
  SegData,
  ChecksumPad,
  Sha256,
  Trailing,
  Failed
};

struct OtaImageParser
{
  OtaParsePhase phase;
  size_t max_image_bytes;
  size_t offset;
  size_t seg_remaining;
  uint8_t segs_left;
  uint8_t hash_appended;
  uint8_t checksum;
  uint8_t header[24];
  uint8_t seg_header[8];
  uint8_t collect;
  uint8_t marker_matched;
  uint8_t sha_left;
  bool saw_marker;
  const char *reason;
};

struct OtaImageCheck
{
  bool structure_ok;
  bool has_marker;
  const char *reason;
};

struct OtaUploadGate
{
  bool password_ok;
  bool flash_begun;
  bool flash_failed;
  bool rejected;
  OtaImageParser parser;
  const char *reason;
};

void otaImageParserReset(OtaImageParser *parser, size_t max_image_bytes);
bool otaImageParserAdd(OtaImageParser *parser, const uint8_t *data, size_t len);
OtaImageCheck otaImageParserFinish(const OtaImageParser *parser);

bool otaGateStart(OtaUploadGate *gate, bool password_ok, size_t slot_size);
bool otaGateNoteFlashBegin(OtaUploadGate *gate, bool ok);
bool otaGateWrite(OtaUploadGate *gate, const uint8_t *data, size_t len);
bool otaGateNoteFlashWrite(OtaUploadGate *gate, bool ok);
bool otaGateFinish(OtaUploadGate *gate);
const char *otaGateReason(const OtaUploadGate *gate);

// Reboot only after the image was accepted and Update.end() succeeded.
// Calling end() switches the boot partition, so it must not run unless
// this would return true for a successful end.
bool otaMayReboot(bool image_accepted, bool update_end_ok);

// Copies "vX.Y.Z" into out. Prefers the literal "waterwell-fw=vX.Y.Z" that
// ota_boot.cpp keeps in the image. Older images only have the version C string
// immediately in front of "Firmware %s, boot number". Returns false when neither
// form is present.
bool otaCopyRollbackVersion(const uint8_t *data, size_t len, char *out, size_t out_len);

// The web listener stays up while the user asked for the monitor, or while
// MQTT failed and the page is the only way left to reach the device.
bool otaShouldForceWeb(bool monitor_enabled, bool failure_forced);

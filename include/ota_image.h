#pragma once

#include <stddef.h>
#include <stdint.h>

// Bytes that must appear in an app image before it may be flashed.
// The running firmware contains the same string, so a build of this
// project passes and a foreign ESP32 binary does not.
const char *otaImageMarker();

// Arduino default 4MB scheme: app0/app1 are each 0x140000 bytes.
constexpr size_t OTA_DEFAULT_SLOT_BYTES = 0x140000;

// Time the new image gets to prove the OTA page answers. Measured from the
// moment the web listener opens, or from boot if it never opens. A reset
// before confirm also rolls back, because the bootloader still has the
// image marked pending.
constexpr uint32_t OTA_CONFIRM_TIMEOUT_MS = 600000;

// A request only counts once the listener has been up this long, so a single
// packet at boot cannot confirm a server that dies immediately after.
constexpr uint32_t OTA_CONFIRM_STABLE_MS = 30000;

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

enum class OtaBootDecision
{
  NotPending,
  StayPending,
  MarkValid,
  RollBack
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

// web_started_ms / last_served_ms are millis() values, or 0 when that event
// has not happened. Confirm only after an HTTP response that was completed
// at least stable_ms after the listener opened. Roll back when that does
// not happen within timeout_ms.
OtaBootDecision otaDecideBoot(bool pending, uint32_t uptime_ms, uint32_t web_started_ms, uint32_t last_served_ms, uint32_t stable_ms, uint32_t timeout_ms);
bool otaShouldForceWeb(bool pending, bool monitor_enabled, bool failure_forced);

// A second upload would overwrite the only rollback slot. Refuse it until
// the running image has been confirmed.
bool otaMayAcceptUpload(bool awaiting_confirm);

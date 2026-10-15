#include "ota_image.h"

#include <string.h>

namespace
{
// Kept in the app image. otaImageMarker() is called from the web server
// so the linker cannot drop it. A flashable .bin of this project therefore
// contains these exact bytes.
const char kMarker[] __attribute__((used)) = "waterwell-ota-v1";

uint32_t readU32(const uint8_t *bytes)
{
  return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

bool addressInRange(uint32_t addr, uint32_t length, uint32_t low, uint32_t high)
{
  const uint64_t end = (uint64_t)addr + length;
  return (uint64_t)addr >= low && end <= high;
}

// Each build accepts only its own chip. CONFIG_IDF_TARGET_* is not visible in
// this file (nothing includes sdkconfig.h), so the S3 environment sets
// OTA_CHIP_ESP32S3 in platformio.ini. ESP32-S3 images use chip id 0x0009 and
// a different memory map than the ESP32 (chip id 0).
#if defined(OTA_CHIP_ESP32S3)
constexpr uint16_t kExpectedChip = 0x0009;

bool segmentFits(uint32_t addr, uint32_t length)
{
  if (length == 0)
  {
    return false;
  }
  if (addr == 0)
  {
    return true;
  }
  return addressInRange(addr, length, 0x3C000000u, 0x3E000000u) || // DROM
         addressInRange(addr, length, 0x3FC88000u, 0x3FD00000u) || // DRAM
         addressInRange(addr, length, 0x40370000u, 0x403E0000u) || // IRAM
         addressInRange(addr, length, 0x42000000u, 0x44000000u) || // IROM
         addressInRange(addr, length, 0x50000000u, 0x50002000u) || // RTC slow
         addressInRange(addr, length, 0x600FE000u, 0x60100000u);   // RTC fast
}

bool entryExecutable(uint32_t entry)
{
  return (entry >= 0x40370000u && entry < 0x403E0000u) ||
         (entry >= 0x42000000u && entry < 0x44000000u) ||
         (entry >= 0x600FE000u && entry < 0x60100000u);
}
#else
constexpr uint16_t kExpectedChip = 0;

bool segmentFits(uint32_t addr, uint32_t length)
{
  if (length == 0)
  {
    return false;
  }
  // esptool inserts padding segments at load address 0.
  if (addr == 0)
  {
    return true;
  }
  return addressInRange(addr, length, 0x3F400000u, 0x3F800000u) || // DROM
         addressInRange(addr, length, 0x3FF80000u, 0x3FF82000u) || // RTC DRAM
         addressInRange(addr, length, 0x3FFAE000u, 0x40000000u) || // DRAM
         addressInRange(addr, length, 0x40080000u, 0x400A0000u) || // IRAM
         addressInRange(addr, length, 0x400C0000u, 0x400C2000u) || // RTC IRAM
         addressInRange(addr, length, 0x400D0000u, 0x40400000u) || // IROM
         addressInRange(addr, length, 0x50000000u, 0x50002000u);   // RTC slow memory
}

bool entryExecutable(uint32_t entry)
{
  return (entry >= 0x40080000u && entry < 0x400A0000u) ||
         (entry >= 0x400C0000u && entry < 0x400C2000u) ||
         (entry >= 0x400D0000u && entry < 0x40400000u);
}
#endif

void noteMarker(OtaImageParser *parser, uint8_t byte)
{
  if (parser->saw_marker)
  {
    return;
  }
  if (byte == (uint8_t)kMarker[parser->marker_matched])
  {
    parser->marker_matched++;
    if (kMarker[parser->marker_matched] == '\0')
    {
      parser->saw_marker = true;
    }
    return;
  }
  parser->marker_matched = (byte == (uint8_t)kMarker[0]) ? 1 : 0;
}

bool fail(OtaImageParser *parser, const char *reason)
{
  parser->phase = OtaParsePhase::Failed;
  parser->reason = reason;
  return false;
}

bool acceptByte(OtaImageParser *parser, uint8_t byte)
{
  if (parser->offset >= parser->max_image_bytes)
  {
    return fail(parser, "image larger than ota slot");
  }
  noteMarker(parser, byte);

  switch (parser->phase)
  {
  case OtaParsePhase::Header:
    parser->header[parser->collect++] = byte;
    if (parser->collect == 1 && byte != 0xE9)
    {
      return fail(parser, "not an esp32 app image");
    }
    if (parser->collect == 24)
    {
      const uint8_t seg_count = parser->header[1];
      const uint16_t chip = (uint16_t)parser->header[12] | ((uint16_t)parser->header[13] << 8);
      const uint32_t entry = readU32(parser->header + 4);
      if (seg_count < 1 || seg_count > 16)
      {
        return fail(parser, "bad segment count");
      }
      if (chip != kExpectedChip)
      {
        return fail(parser, "wrong chip");
      }
      if (!entryExecutable(entry))
      {
        return fail(parser, "bad entry point");
      }
      if (parser->header[23] > 1)
      {
        return fail(parser, "bad image header");
      }
      parser->hash_appended = parser->header[23];
      parser->segs_left = seg_count;
      parser->collect = 0;
      parser->phase = OtaParsePhase::SegHeader;
    }
    break;

  case OtaParsePhase::SegHeader:
    parser->seg_header[parser->collect++] = byte;
    if (parser->collect == 8)
    {
      const uint32_t addr = readU32(parser->seg_header);
      const uint32_t length = readU32(parser->seg_header + 4);
      if ((length % 4u) != 0 || length == 0 || length >= 0x1000000u)
      {
        return fail(parser, "bad segment length");
      }
      if (!segmentFits(addr, length))
      {
        return fail(parser, "segment outside esp32 memory");
      }
      if (parser->offset >= parser->max_image_bytes || length > (parser->max_image_bytes - parser->offset - 1))
      {
        return fail(parser, "image larger than ota slot");
      }
      parser->seg_remaining = length;
      parser->phase = OtaParsePhase::SegData;
    }
    break;

  case OtaParsePhase::SegData:
    parser->checksum = (uint8_t)(parser->checksum ^ byte);
    parser->seg_remaining--;
    if (parser->seg_remaining == 0)
    {
      parser->segs_left--;
      parser->collect = 0;
      parser->phase = parser->segs_left == 0 ? OtaParsePhase::ChecksumPad : OtaParsePhase::SegHeader;
    }
    break;

  case OtaParsePhase::ChecksumPad:
    // Segment payloads are checksummed with initial value 0xEF.
    // The checksum byte sits at the next offset where offset % 16 == 15.
    // Bytes in between are padding and must be zero. They are not checksummed.
    if ((parser->offset % 16u) != 15u)
    {
      if (byte != 0)
      {
        return fail(parser, "bad image padding");
      }
    }
    else if (byte != parser->checksum)
    {
      return fail(parser, "bad image checksum");
    }
    else if (parser->hash_appended)
    {
      parser->phase = OtaParsePhase::Sha256;
      parser->sha_left = 32;
    }
    else
    {
      parser->phase = OtaParsePhase::Trailing;
    }
    break;

  case OtaParsePhase::Sha256:
    if (parser->sha_left == 0)
    {
      return fail(parser, "truncated image");
    }
    parser->sha_left--;
    if (parser->sha_left == 0)
    {
      parser->phase = OtaParsePhase::Trailing;
    }
    break;

  case OtaParsePhase::Trailing:
    if (byte != 0x00 && byte != 0xFF)
    {
      return fail(parser, "trailing garbage");
    }
    break;

  case OtaParsePhase::Failed:
    return false;
  }

  parser->offset++;
  return parser->phase != OtaParsePhase::Failed;
}
} // namespace

const char *otaImageMarker()
{
  return kMarker;
}

void otaImageParserReset(OtaImageParser *parser, size_t max_image_bytes)
{
  memset(parser, 0, sizeof(*parser));
  parser->phase = OtaParsePhase::Header;
  parser->max_image_bytes = max_image_bytes;
  parser->checksum = 0xEF;
  parser->reason = "ok";
}

bool otaImageParserAdd(OtaImageParser *parser, const uint8_t *data, size_t len)
{
  if (parser->phase == OtaParsePhase::Failed)
  {
    return false;
  }
  for (size_t i = 0; i < len; ++i)
  {
    if (!acceptByte(parser, data[i]))
    {
      parser->phase = OtaParsePhase::Failed;
      return false;
    }
  }
  return true;
}

OtaImageCheck otaImageParserFinish(const OtaImageParser *parser)
{
  OtaImageCheck check;
  check.structure_ok = false;
  check.has_marker = parser->saw_marker;
  check.reason = parser->reason != 0 ? parser->reason : "rejected";
  if (parser->phase == OtaParsePhase::Failed)
  {
    return check;
  }
  if (parser->phase != OtaParsePhase::Trailing)
  {
    check.reason = "truncated image";
    return check;
  }
  check.structure_ok = true;
  if (!parser->saw_marker)
  {
    check.reason = "not a waterwell firmware";
    return check;
  }
  check.reason = "ok";
  return check;
}

bool otaGateStart(OtaUploadGate *gate, bool password_ok, size_t slot_size)
{
  memset(gate, 0, sizeof(*gate));
  gate->password_ok = password_ok;
  gate->reason = "ok";
  if (!password_ok)
  {
    gate->rejected = true;
    gate->reason = "invalid password";
    return false;
  }
  if (slot_size == 0)
  {
    gate->rejected = true;
    gate->reason = "no ota partition";
    return false;
  }
  otaImageParserReset(&gate->parser, slot_size);
  return true;
}

bool otaGateNoteFlashBegin(OtaUploadGate *gate, bool ok)
{
  if (gate->rejected)
  {
    return false;
  }
  gate->flash_begun = ok;
  if (!ok)
  {
    gate->rejected = true;
    gate->reason = "flash begin failed";
    return false;
  }
  return true;
}

bool otaGateWrite(OtaUploadGate *gate, const uint8_t *data, size_t len)
{
  if (gate->rejected)
  {
    return false;
  }
  if (!otaImageParserAdd(&gate->parser, data, len))
  {
    gate->rejected = true;
    gate->reason = gate->parser.reason;
    return false;
  }
  return true;
}

bool otaGateNoteFlashWrite(OtaUploadGate *gate, bool ok)
{
  if (gate->rejected)
  {
    return false;
  }
  if (!ok)
  {
    gate->rejected = true;
    gate->flash_failed = true;
    gate->reason = "flash write failed";
    return false;
  }
  return true;
}

bool otaGateFinish(OtaUploadGate *gate)
{
  if (gate->rejected || !gate->password_ok || !gate->flash_begun || gate->flash_failed)
  {
    gate->rejected = true;
    if (gate->reason == 0 || strcmp(gate->reason, "ok") == 0)
    {
      gate->reason = "rejected";
    }
    return false;
  }
  const OtaImageCheck check = otaImageParserFinish(&gate->parser);
  if (!check.structure_ok || !check.has_marker)
  {
    gate->rejected = true;
    gate->reason = check.reason;
    return false;
  }
  gate->reason = "ok";
  return true;
}

const char *otaGateReason(const OtaUploadGate *gate)
{
  return gate->reason != 0 ? gate->reason : "rejected";
}

bool otaMayReboot(bool image_accepted, bool update_end_ok)
{
  return image_accepted && update_end_ok;
}

namespace
{
const char kFirmwareAnchor[] = "Firmware %s, boot number";
const char kFirmwareMark[] = "waterwell-fw=";
constexpr size_t kRollbackVersionMax = 15;

bool isRollbackVersion(const char *text, size_t len)
{
  if (len < 5 || len > kRollbackVersionMax || text[0] != 'v')
  {
    return false;
  }
  int part = 0;
  bool sawDigit = false;
  for (size_t i = 1; i < len; ++i)
  {
    const char c = text[i];
    if (c >= '0' && c <= '9')
    {
      sawDigit = true;
      continue;
    }
    if (c == '.' && sawDigit && part < 2)
    {
      ++part;
      sawDigit = false;
      continue;
    }
    return false;
  }
  return part == 2 && sawDigit;
}
} // namespace

bool copyRollbackVersionAt(const uint8_t *data, size_t start, size_t end, char *out, size_t out_len)
{
  const size_t versionLen = end - start;
  if (!isRollbackVersion(reinterpret_cast<const char *>(data + start), versionLen) || versionLen + 1 > out_len)
  {
    return false;
  }
  memcpy(out, data + start, versionLen);
  out[versionLen] = '\0';
  return true;
}

bool otaCopyRollbackVersion(const uint8_t *data, size_t len, char *out, size_t out_len)
{
  if (data == nullptr || out == nullptr || out_len < 6)
  {
    return false;
  }
  const size_t markLen = sizeof(kFirmwareMark) - 1;
  if (len >= markLen + 6)
  {
    for (size_t i = 0; i + markLen + 5 <= len; ++i)
    {
      if (memcmp(data + i, kFirmwareMark, markLen) != 0)
      {
        continue;
      }
      const size_t start = i + markLen;
      size_t end = start;
      while (end < len && data[end] != 0 && (end - start) <= kRollbackVersionMax)
      {
        ++end;
      }
      if (end < len && data[end] == 0 && copyRollbackVersionAt(data, start, end, out, out_len))
      {
        return true;
      }
    }
  }
  const size_t anchorLen = sizeof(kFirmwareAnchor) - 1;
  if (len < anchorLen + 2)
  {
    return false;
  }
  for (size_t i = 1; i + anchorLen <= len; ++i)
  {
    if (memcmp(data + i, kFirmwareAnchor, anchorLen) != 0 || data[i - 1] != 0)
    {
      continue;
    }
    const size_t end = i - 1;
    size_t start = end;
    while (start > 0 && data[start - 1] != 0 && (end - (start - 1)) <= kRollbackVersionMax)
    {
      --start;
    }
    if (start > 0 && data[start - 1] != 0)
    {
      continue;
    }
    if (copyRollbackVersionAt(data, start, end, out, out_len))
    {
      return true;
    }
  }
  return false;
}

bool otaShouldForceWeb(bool monitor_enabled, bool failure_forced)
{
  return monitor_enabled || failure_forced;
}

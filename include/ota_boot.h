#pragma once

#include <stddef.h>

size_t otaNextSlotSize();
// Keeps a freshly booted image. The bootloader marks it pending and would
// drop it on the next reset otherwise.
void otaBootInit();
// Version string from the inactive OTA slot, such as "v0.1.0", or null when
// that slot has no bootable image with a readable version. Cached.
const char *otaOtherVersion();
// Points the bootloader at the inactive slot. The caller reboots.
bool otaRollbackToOther();

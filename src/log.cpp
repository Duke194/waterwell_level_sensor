#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace
{
constexpr size_t LOG_LINES = 40;
constexpr size_t LOG_LEN = 128;
char logBuffer[LOG_LINES][LOG_LEN];
size_t logHead = 0;
bool logWrapped = false;
} // namespace

void logLine(const char *fmt, ...)
{
  char line[LOG_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  Serial.println(line);
  strlcpy(logBuffer[logHead], line, LOG_LEN);
  logHead = (logHead + 1) % LOG_LINES;
  if (logHead == 0)
  {
    logWrapped = true;
  }
}

String getLogs()
{
  String out;
  const size_t count = logWrapped ? LOG_LINES : logHead;
  const size_t start = logWrapped ? logHead : 0;
  for (size_t i = 0; i < count; ++i)
  {
    const size_t idx = (start + i) % LOG_LINES;
    out += logBuffer[idx];
    out += "\n";
  }
  return out;
}

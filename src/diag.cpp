#include "diag.h"

#include <string.h>

namespace
{
constexpr size_t MESSAGE_LEN = 64;
char messages[(size_t)ErrorSource::Count][MESSAGE_LEN] = {};
} // namespace

void diagSetError(ErrorSource source, const char *message)
{
  char *slot = messages[(size_t)source];
  if (message == nullptr)
  {
    slot[0] = '\0';
    return;
  }
  strlcpy(slot, message, MESSAGE_LEN);
}

void diagClearError(ErrorSource source)
{
  messages[(size_t)source][0] = '\0';
}

const char *diagErrorText()
{
  for (size_t i = 0; i < (size_t)ErrorSource::Count; ++i)
  {
    if (messages[i][0] != '\0')
    {
      return messages[i];
    }
  }
  return "Kein Fehler";
}

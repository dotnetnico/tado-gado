#include "web_guard.h"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace tadogado {

namespace {

char lower(char c) { return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); }

}  // namespace

bool hostAllowed(const char* host, const char* address, const char* name) {
  const char* colon = std::strchr(host, ':');
  const size_t length = colon != nullptr ? static_cast<size_t>(colon - host) : std::strlen(host);
  if (length == 0) {
    return false;
  }
  if (length == std::strlen(address) && std::memcmp(host, address, length) == 0) {
    return true;
  }
  const size_t nameLength = std::strlen(name);
  if (nameLength == 0 || length < nameLength) {
    return false;
  }
  for (size_t i = 0; i < nameLength; i++) {
    if (lower(host[i]) != name[i]) {
      return false;
    }
  }
  return length == nameLength || host[nameLength] == '.';
}

const TokenLockout::Entry* TokenLockout::find(uint32_t client) const {
  for (const Entry& entry : entries_) {
    if (entry.used && entry.client == client) {
      return &entry;
    }
  }
  return nullptr;
}

bool TokenLockout::locked(uint32_t client, uint64_t nowMs) const {
  const Entry* entry = find(client);
  return entry != nullptr && nowMs < entry->untilMs;
}

void TokenLockout::failed(uint32_t client, uint64_t nowMs) {
  Entry* entry = const_cast<Entry*>(find(client));
  if (entry == nullptr) {
    // A free place, or else the one whose wait is over first.
    entry = &entries_[0];
    for (Entry& candidate : entries_) {
      if (!candidate.used) {
        entry = &candidate;
        break;
      }
      if (candidate.untilMs < entry->untilMs) {
        entry = &candidate;
      }
    }
    *entry = Entry{true, client, 0, 0};
  }
  entry->delayMs = entry->delayMs == 0 ? kFirstDelayMs : std::min(entry->delayMs * 2, kMaxDelayMs);
  entry->untilMs = nowMs + entry->delayMs;
}

void TokenLockout::succeeded(uint32_t client) {
  Entry* entry = const_cast<Entry*>(find(client));
  if (entry != nullptr) {
    entry->used = false;
  }
}

}  // namespace tadogado

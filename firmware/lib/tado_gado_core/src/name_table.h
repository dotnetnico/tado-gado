// The names a user gives to devices, such as "Bedroom".
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "frames.h"

namespace tadogado {

constexpr size_t kMaxNameBytes = 24;

// Names end up in JSON, so anything that would need escaping there is refused.
inline bool validDeviceName(const char* name) {
  const size_t length = std::strlen(name);
  if (length > kMaxNameBytes) {
    return false;
  }
  for (size_t i = 0; i < length; i++) {
    const unsigned char c = static_cast<unsigned char>(name[i]);
    if (c < 0x20 || c == 0x7F || c == '"' || c == '\\') {
      return false;
    }
  }
  return true;
}

class NameTable {
 public:
  static constexpr size_t kCapacity = 32;

  struct Entry {
    bool used;
    uint8_t prefix[kPrefixLen];
    char name[kMaxNameBytes + 1];
  };

  // An empty name clears the entry. False for an invalid name or a full table.
  bool set(const uint8_t* prefix, const char* name) {
    if (!validDeviceName(name)) {
      return false;
    }
    Entry* entry = locate(prefix);
    if (name[0] == '\0') {
      if (entry != nullptr) {
        entry->used = false;
      }
      return true;
    }
    if (entry == nullptr) {
      entry = firstFree();
      if (entry == nullptr) {
        return false;
      }
    }
    entry->used = true;
    std::memcpy(entry->prefix, prefix, kPrefixLen);
    std::strcpy(entry->name, name);
    return true;
  }

  // nullptr if the device has no name.
  const char* find(const uint8_t* prefix) const {
    for (const Entry& entry : entries_) {
      if (entry.used && std::memcmp(entry.prefix, prefix, kPrefixLen) == 0) {
        return entry.name;
      }
    }
    return nullptr;
  }

  const Entry* entries() const { return entries_; }

 private:
  Entry* locate(const uint8_t* prefix) {
    for (Entry& entry : entries_) {
      if (entry.used && std::memcmp(entry.prefix, prefix, kPrefixLen) == 0) {
        return &entry;
      }
    }
    return nullptr;
  }

  Entry* firstFree() {
    for (Entry& entry : entries_) {
      if (!entry.used) {
        return &entry;
      }
    }
    return nullptr;
  }

  Entry entries_[kCapacity] = {};
};

}  // namespace tadogado

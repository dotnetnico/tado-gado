// CSL sample anchors: when each sleeping device opens its receiver.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "frames.h"

namespace tadogado {

struct Slot {
  uint64_t startUs;
  uint64_t endUs;
};

class AnchorTable {
 public:
  void update(const uint8_t* device, uint64_t ackUs, uint16_t phase) {
    Entry* entry = find(device);
    if (entry == nullptr) {
      entry = oldest();
    }
    const uint64_t back = (phase * kCslUnitUs) % kCslPeriodUs;
    entry->used = true;
    std::memcpy(entry->device, device, kPrefixLen);
    // Keep a whole period in hand, so an early timestamp cannot wrap below zero.
    entry->sampleUs = ackUs + kCslPeriodUs - back;
    entry->refreshedUs = ackUs;
    entry->misses = 0;
  }

  void miss(const uint8_t* device) {
    Entry* entry = find(device);
    if (entry != nullptr && ++entry->misses >= kMaxMisses) {
      entry->used = false;
    }
  }

  bool contains(const uint8_t* device) const { return find(device) != nullptr; }

  // The first train slot after `nowUs`. False if the anchor is missing or too old,
  // which means a train must cover a whole period.
  bool nextSlot(const uint8_t* device, uint64_t nowUs, Slot& out) const {
    const Entry* entry = find(device);
    if (entry == nullptr) {
      return false;
    }
    const uint64_t age = nowUs - entry->refreshedUs;
    const uint64_t guard = kGuardUs + age * kGuardDriftPpm / 1000000;
    if (guard >= kGuardMaxUs) {
      return false;
    }
    const uint64_t earliest = nowUs + guard;
    uint64_t sample = entry->sampleUs;
    if (earliest >= sample) {
      sample += ((earliest - sample) / kCslPeriodUs + 1) * kCslPeriodUs;
    }
    out.startUs = sample - guard;
    out.endUs = sample + guard;
    return true;
  }

  // The phase field of an ACK on behalf of the device. False with no anchor.
  bool phase(const uint8_t* device, uint64_t nowUs, uint16_t& out) const {
    const Entry* entry = find(device);
    if (entry == nullptr) {
      return false;
    }
    const uint64_t elapsed = (nowUs + kCslPeriodUs - entry->sampleUs % kCslPeriodUs) % kCslPeriodUs;
    out = static_cast<uint16_t>((elapsed / kCslUnitUs) % kCslPeriodUnits);
    return true;
  }

 private:
  static constexpr size_t kCapacity = 32;
  static constexpr uint8_t kMaxMisses = 4;  // As in the Tado firmware.
  static constexpr uint64_t kGuardUs = 40000;
  static constexpr uint64_t kGuardDriftPpm = 30;
  static constexpr uint64_t kGuardMaxUs = kCslPeriodUs / 2;

  struct Entry {
    bool used;
    uint8_t device[kPrefixLen];
    uint64_t sampleUs;
    uint64_t refreshedUs;
    uint8_t misses;
  };

  const Entry* find(const uint8_t* device) const {
    for (const Entry& entry : entries_) {
      if (entry.used && std::memcmp(entry.device, device, kPrefixLen) == 0) {
        return &entry;
      }
    }
    return nullptr;
  }

  Entry* find(const uint8_t* device) { return const_cast<Entry*>(static_cast<const AnchorTable*>(this)->find(device)); }

  Entry* oldest() {
    Entry* candidate = &entries_[0];
    for (Entry& entry : entries_) {
      if (!entry.used) {
        return &entry;
      }
      if (entry.refreshedUs < candidate->refreshedUs) {
        candidate = &entry;
      }
    }
    return candidate;
  }

  Entry entries_[kCapacity] = {};
};

}  // namespace tadogado

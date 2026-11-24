// Duplicate detection, without heap use. See docs/how-it-works.md.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

enum class Verdict : uint8_t { New, Retransmission, Echo };

// Hashes remembered for a fixed time. When full, the oldest entry goes first.
template <size_t Capacity>
class ExpiringHashes {
 public:
  void add(uint64_t hash, uint64_t expiryUs) {
    for (Entry& entry : entries_) {
      if (entry.expiryUs != 0 && entry.hash == hash) {
        entry.expiryUs = expiryUs;
        return;
      }
    }
    entries_[next_] = {hash, expiryUs};
    next_ = (next_ + 1) % Capacity;
  }

  bool contains(uint64_t hash, uint64_t nowUs) const {
    for (const Entry& entry : entries_) {
      if (entry.expiryUs > nowUs && entry.hash == hash) {
        return true;
      }
    }
    return false;
  }

 private:
  struct Entry {
    uint64_t hash;
    uint64_t expiryUs;
  };
  Entry entries_[Capacity] = {};
  size_t next_ = 0;
};

class FrameDeduplicator {
 public:
  Verdict classify(const uint8_t* frame, size_t length, uint64_t nowUs);
  // Record a frame this node puts on the air.
  void noteSent(const uint8_t* frame, size_t length, uint64_t nowUs);

  uint32_t statsNew = 0;
  uint32_t statsRetransmissions = 0;
  uint32_t statsEchoes = 0;

 private:
  static constexpr size_t kCapacity = 64;
  ExpiringHashes<kCapacity> echoes_;
  ExpiringHashes<kCapacity> datagrams_;
};

}  // namespace tadogado

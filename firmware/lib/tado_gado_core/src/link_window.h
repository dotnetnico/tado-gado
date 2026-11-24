// Counts of the last hour and of the last 24 hours, per device.
//
// Counting since boot mixes periods: a night in monitor and an hour of proxy end up in
// one figure. These windows slide, and the monitor clears them when the mode changes.
// All devices share the buckets, so the monitor moves them on once for everyone, and a
// device keeps only its counts: no heap, and a few hundred bytes per device.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

// The hour in buckets of 10 minutes, the day in buckets of 2 hours. The current bucket
// is still filling, so a window covers its length plus at most one bucket.
constexpr uint64_t kHourBucketUs = 600ULL * 1000000ULL;
constexpr size_t kHourBuckets = 7;
constexpr uint64_t kDayBucketUs = 7200ULL * 1000000ULL;
constexpr size_t kDayBuckets = 13;

// What one device did in one period. `Count` is small in a bucket and wide in a sum.
template <typename Count, typename Time>
struct LinkCountsOf {
  Count uplinks;
  Count uplinksAcked;
  Count uplinksUnacked;
  Count downlinks;
  Count downlinksAcked;
  Count downlinksUnacked;
  Count retransmissions;
  // Downlinks behind a long wake-up train: the bridge did not know when the device
  // listens. The others came behind a short train, aimed at a known sample.
  Count longTrainAcked;
  Count longTrainUnacked;
  Time airtimeUs;  // What the device sent, as this node heard it.
  Time trainUs;    // Wake-up trains for the device, as this node heard them.
};

using LinkBucket = LinkCountsOf<uint16_t, uint32_t>;
using LinkTotals = LinkCountsOf<uint32_t, uint64_t>;

// The fields that count frames, for code that treats them alike.
constexpr uint16_t LinkBucket::* const kLinkCountFields[] = {
    &LinkBucket::uplinks,         &LinkBucket::uplinksAcked,   &LinkBucket::uplinksUnacked,
    &LinkBucket::downlinks,       &LinkBucket::downlinksAcked, &LinkBucket::downlinksUnacked,
    &LinkBucket::retransmissions, &LinkBucket::longTrainAcked, &LinkBucket::longTrainUnacked,
};
constexpr uint32_t LinkTotals::* const kLinkTotalFields[] = {
    &LinkTotals::uplinks,         &LinkTotals::uplinksAcked,   &LinkTotals::uplinksUnacked,
    &LinkTotals::downlinks,       &LinkTotals::downlinksAcked, &LinkTotals::downlinksUnacked,
    &LinkTotals::retransmissions, &LinkTotals::longTrainAcked, &LinkTotals::longTrainUnacked,
};
constexpr size_t kLinkCountFieldCount = sizeof(kLinkCountFields) / sizeof(kLinkCountFields[0]);
static_assert(kLinkCountFieldCount == sizeof(kLinkTotalFields) / sizeof(kLinkTotalFields[0]), "same fields");

inline void saturatingIncrement(uint16_t& count) {
  if (count < UINT16_MAX) {
    count++;
  }
}

inline void saturatingAdd(uint32_t& total, uint64_t amount) {
  total = amount >= UINT32_MAX - total ? UINT32_MAX : static_cast<uint32_t>(total + amount);
}

inline void accumulate(LinkTotals& total, const LinkBucket& bucket) {
  for (size_t i = 0; i < kLinkCountFieldCount; i++) {
    total.*kLinkTotalFields[i] += bucket.*kLinkCountFields[i];
  }
  total.airtimeUs += bucket.airtimeUs;
  total.trainUs += bucket.trainUs;
}

// Which bucket of a ring is current. The owner clears the slots this hands back.
template <uint64_t BucketUs, size_t Buckets>
class BucketClock {
 public:
  static constexpr size_t kBuckets = Buckets;

  // Moves to the bucket of `nowUs`. Calls `clear(slot)` for each slot that starts a new
  // bucket, or for every slot after a gap longer than the window, or before the first.
  template <typename Clear>
  void advance(uint64_t nowUs, Clear&& clear) {
    const uint64_t index = nowUs / BucketUs;
    if (started_ && index == index_) {
      return;
    }
    if (!started_ || index < index_ || index - index_ >= Buckets) {
      for (size_t slot = 0; slot < Buckets; slot++) {
        clear(slot);
      }
    } else {
      for (uint64_t next = index_ + 1; next <= index; next++) {
        clear(static_cast<size_t>(next % Buckets));
      }
    }
    started_ = true;
    index_ = index;
  }

  size_t slot() const { return static_cast<size_t>(index_ % Buckets); }

  // Starts again from empty, as advance() does before the first bucket.
  void restart() { started_ = false; }

 private:
  bool started_ = false;
  uint64_t index_ = 0;
};

}  // namespace tadogado

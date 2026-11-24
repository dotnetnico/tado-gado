// Airtime over the last hour, and the transmit budget for the 1% duty cycle of the
// 868.0-868.6 MHz band.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

// Counts in buckets of `BucketSeconds`, so it needs no heap, and it frees a bucket
// late rather than early.
template <uint32_t BucketSeconds>
class HourlyAirtime {
 public:
  void record(uint64_t airtimeUs, uint64_t nowUs) {
    const uint32_t index = static_cast<uint32_t>(nowUs / kBucketUs);
    Bucket& bucket = buckets_[index % kBuckets];
    if (bucket.index != index) {
      bucket.index = index;
      bucket.airtimeUs = 0;
    }
    bucket.airtimeUs += static_cast<uint32_t>(airtimeUs);
  }

  uint64_t usedUs(uint64_t nowUs) const {
    const uint32_t index = static_cast<uint32_t>(nowUs / kBucketUs);
    uint64_t used = 0;
    for (const Bucket& bucket : buckets_) {
      if (bucket.airtimeUs != 0 && index - bucket.index <= kBuckets - 1) {
        used += bucket.airtimeUs;
      }
    }
    return used;
  }

 private:
  static_assert(BucketSeconds > 0 && BucketSeconds <= 600 && 3600 % BucketSeconds == 0, "whole buckets per hour");
  static constexpr uint64_t kBucketUs = BucketSeconds * 1000000ULL;
  static constexpr uint32_t kBuckets = 3600 / BucketSeconds + 1;

  struct Bucket {
    uint32_t index;
    uint32_t airtimeUs;
  };
  Bucket buckets_[kBuckets] = {};
};

class DutyCycleLimiter {
 public:
  // 0.9% of an hour: a margin under the legal 1%.
  static constexpr uint64_t kBudgetUs = 32400000;

  bool allows(uint64_t airtimeUs, uint64_t nowUs) const { return usedUs(nowUs) + airtimeUs <= kBudgetUs; }
  void record(uint64_t airtimeUs, uint64_t nowUs) { airtime_.record(airtimeUs, nowUs); }
  uint64_t usedUs(uint64_t nowUs) const { return airtime_.usedUs(nowUs); }

 private:
  HourlyAirtime<60> airtime_;
};

}  // namespace tadogado

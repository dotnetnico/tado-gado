// Finds the channel of an installation and the crystal trim of a CC1101 module.
//
// The radio task owns the retuning: it
// calls tick() and applies the channel and trim whenever that returns true.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

struct SurveyResult {
  uint8_t channel;
  int8_t fsctrl0;
  uint32_t frames;
  uint32_t crcOk;
};

// The trim that removes the measured offset. Returns false with no usable reading.
bool suggestFsctrl0(int current, const int8_t* readings, size_t count, int8_t& out);

class Survey {
 public:
  static constexpr size_t kChannels = 3;
  static constexpr size_t kTrims = 5;
  static constexpr size_t kResults = kChannels * kTrims;
  static constexpr size_t kMaxReadings = 64;

  void start(uint64_t nowUs, uint64_t dwellUs);
  void stop();
  // Advance the survey. Returns true when the radio must move to current().
  bool tick(uint64_t nowUs);
  void onFrame(bool crcOk, int8_t freqest);

  bool running() const { return running_; }
  bool finished() const { return finished_; }
  size_t resultCount() const { return finished_ ? kResults : index_; }
  const SurveyResult& result(size_t index) const { return results_[index]; }
  const SurveyResult& current() const { return results_[index_]; }
  size_t step() const { return index_; }

  // The best result, with a trim refined by FREQEST. False if no frame passed.
  bool best(SurveyResult& out) const;

 private:
  void beginStep(size_t index, uint64_t nowUs);

  SurveyResult results_[kResults] = {};
  int8_t readings_[kResults][kMaxReadings] = {};
  uint8_t readingCounts_[kResults] = {};
  size_t index_ = 0;
  uint64_t stepEndUs_ = 0;
  uint64_t dwellUs_ = 0;
  bool running_ = false;
  bool finished_ = false;
  bool pendingStart_ = false;
};

}  // namespace tadogado

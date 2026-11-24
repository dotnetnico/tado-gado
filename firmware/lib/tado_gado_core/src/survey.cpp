#include "survey.h"

#include <algorithm>
#include <cstdlib>

namespace tadogado {

namespace {
constexpr uint8_t kSurveyChannels[Survey::kChannels] = {26, 25, 27};
// FSCTRL0 steps of 1.587 kHz: about 50 kHz apart, the width the demodulator corrects.
constexpr int8_t kSurveyTrims[Survey::kTrims] = {0, 32, -32, 64, -64};
constexpr int kFreqestClip = 20;  // A reading this large has saturated.
}  // namespace

bool suggestFsctrl0(int current, const int8_t* readings, size_t count, int8_t& out) {
  int usable[Survey::kMaxReadings];
  size_t used = 0;
  for (size_t i = 0; i < count && used < Survey::kMaxReadings; i++) {
    if (std::abs(static_cast<int>(readings[i])) < kFreqestClip) {
      usable[used++] = readings[i];
    }
  }
  if (used == 0) {
    return false;
  }
  std::sort(usable, usable + used);
  // Like Python's statistics.median followed by round: an even count takes the
  // mean of the middle two, and halves go to the even neighbour.
  const int doubled = (used % 2 == 1) ? 2 * usable[used / 2] : usable[used / 2 - 1] + usable[used / 2];
  int median = doubled / 2;
  if (doubled % 2 != 0 && median % 2 != 0) {
    median += doubled > 0 ? 1 : -1;
  }
  const int corrected = current + median;
  out = static_cast<int8_t>(std::max(-128, std::min(127, corrected)));
  return true;
}

void Survey::start(uint64_t nowUs, uint64_t dwellUs) {
  dwellUs_ = dwellUs;
  finished_ = false;
  running_ = true;
  pendingStart_ = true;
  for (size_t channel = 0; channel < kChannels; channel++) {
    for (size_t trim = 0; trim < kTrims; trim++) {
      const size_t index = channel * kTrims + trim;
      results_[index] = {kSurveyChannels[channel], kSurveyTrims[trim], 0, 0};
      readingCounts_[index] = 0;
    }
  }
  beginStep(0, nowUs);
}

void Survey::stop() { running_ = false; }

void Survey::beginStep(size_t index, uint64_t nowUs) {
  index_ = index;
  stepEndUs_ = nowUs + dwellUs_;
}

bool Survey::tick(uint64_t nowUs) {
  if (!running_) {
    return false;
  }
  if (pendingStart_) {
    pendingStart_ = false;
    return true;
  }
  if (nowUs < stepEndUs_) {
    return false;
  }
  if (index_ + 1 >= kResults) {
    running_ = false;
    finished_ = true;
    return false;
  }
  beginStep(index_ + 1, nowUs);
  return true;
}

void Survey::onFrame(bool crcOk, int8_t freqest) {
  if (!running_) {
    return;
  }
  SurveyResult& result = results_[index_];
  result.frames++;
  if (crcOk) {
    result.crcOk++;
    if (readingCounts_[index_] < kMaxReadings) {
      readings_[index_][readingCounts_[index_]++] = freqest;
    }
  }
}

bool Survey::best(SurveyResult& out) const {
  size_t bestIndex = 0;
  for (size_t i = 1; i < kResults; i++) {
    if (results_[i].crcOk > results_[bestIndex].crcOk) {
      bestIndex = i;
    }
  }
  if (results_[bestIndex].crcOk == 0) {
    return false;
  }
  out = results_[bestIndex];
  int8_t refined = 0;
  if (suggestFsctrl0(out.fsctrl0, readings_[bestIndex], readingCounts_[bestIndex], refined)) {
    out.fsctrl0 = refined;
  }
  return true;
}

}  // namespace tadogado

#include "image_check.h"

#include <algorithm>
#include <cstring>

namespace tadogado {

namespace {
constexpr uint8_t kImageMagic = 0xE9;
constexpr uint8_t kChipEsp32S3 = 9;
constexpr size_t kChipOffset = 12;
constexpr size_t kDescriptionOffset = 32;
constexpr uint8_t kDescriptionMagic[4] = {0x32, 0x54, 0xCD, 0xAB};
constexpr size_t kPrefixLength = sizeof(kImageMarkerPrefix) - 1;

bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isVariantChar(char c) { return (c >= 'a' && c <= 'z') || isDigit(c); }
bool isVersionChar(char c) { return isDigit(c) || c == '.'; }
}  // namespace

void ImageCheck::feed(const uint8_t* data, size_t length) {
  const size_t header = std::min(length, kHeaderBytes - headerLength_);
  std::memcpy(header_ + headerLength_, data, header);
  headerLength_ += header;
  for (size_t i = 0; i < length; i++) {
    scan(static_cast<char>(data[i]));
  }
}

void ImageCheck::scan(char c) {
  if (state_ == State::Prefix) {
    // No proper start of the prefix is also an end of it, so a mismatch starts over.
    if (c != kImageMarkerPrefix[matched_]) {
      matched_ = 0;
    }
    if (c == kImageMarkerPrefix[matched_] && ++matched_ == kPrefixLength) {
      state_ = State::Variant;
      matched_ = 0;
    }
    return;
  }
  const bool variant = state_ == State::Variant;
  char* field = variant ? candidateVariant_ : candidateVersion_;
  if (matched_ < kMaxField && (variant ? isVariantChar(c) : isVersionChar(c))) {
    field[matched_++] = c;
    return;
  }
  field[matched_] = '\0';
  const bool ended = matched_ > 0 && c == (variant ? ':' : '\0');
  matched_ = 0;
  if (ended && variant) {
    state_ = State::Version;
    return;
  }
  state_ = State::Prefix;
  if (ended) {
    found_ = true;
    std::memcpy(foundVariant_, candidateVariant_, sizeof(foundVariant_));
    std::memcpy(version_, candidateVersion_, sizeof(version_));
  } else {
    scan(c);  // It may start the next prefix. Strings end with a NUL, so no prefix starts in a field.
  }
}

const char* ImageCheck::headerError() const {
  if (headerLength_ < kHeaderBytes || header_[0] != kImageMagic) {
    return "this is not a firmware image";
  }
  if (header_[kChipOffset] != kChipEsp32S3 || header_[kChipOffset + 1] != 0) {
    return "this image is for another chip";
  }
  if (std::memcmp(header_ + kDescriptionOffset, kDescriptionMagic, sizeof(kDescriptionMagic)) != 0) {
    return "this is the factory image: use the file that ends in -app.bin";
  }
  return nullptr;
}

const char* ImageCheck::error() const {
  const char* header = headerError();
  if (header != nullptr) {
    return header;
  }
  if (!found_) {
    return "this is not a tado-gado image of v0.11.0 or later";
  }
  if (std::strcmp(foundVariant_, variant_) != 0) {
    return "this image is for the other kind of board";
  }
  return nullptr;
}

}  // namespace tadogado

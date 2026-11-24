// Decides whether an uploaded file is firmware for this board, apart from the flash, so
// the native tests can run it.
//
// ESP-IDF checks the checksum and the chip of what was written. It does not know one
// board's flash from the other's, and it takes the factory image for an application,
// as that starts with the bootloader.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

// Every image holds one string "tado-gado-image:<variant>:<version>".
constexpr char kImageMarkerPrefix[] = "tado-gado-image:";

class ImageCheck {
 public:
  // The image and segment headers, and the first word of the application description.
  static constexpr size_t kHeaderBytes = 36;
  static constexpr size_t kMaxField = 15;

  // `variant` is the one this board runs.
  explicit ImageCheck(const char* variant) : variant_(variant) {}

  // The file, in order, in pieces of any size.
  void feed(const uint8_t* data, size_t length);

  bool headerComplete() const { return headerLength_ == kHeaderBytes; }
  // What is wrong, or nullptr. The header tells after kHeaderBytes, and the marker can
  // be anywhere, so only the whole file tells everything.
  const char* headerError() const;
  const char* error() const;
  const char* version() const { return version_; }

 private:
  enum class State : uint8_t { Prefix, Variant, Version };

  void scan(char c);

  const char* variant_;
  uint8_t header_[kHeaderBytes] = {};
  size_t headerLength_ = 0;
  State state_ = State::Prefix;
  size_t matched_ = 0;
  char candidateVariant_[kMaxField + 1] = {};
  char candidateVersion_[kMaxField + 1] = {};
  bool found_ = false;
  char foundVariant_[kMaxField + 1] = {};
  char version_[kMaxField + 1] = {};
};

}  // namespace tadogado

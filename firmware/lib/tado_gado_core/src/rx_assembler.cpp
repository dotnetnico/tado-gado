#include "rx_assembler.h"

#include <cstring>

namespace tadogado {

namespace {
constexpr uint8_t kFifoErrorBit = 0x80;
constexpr uint8_t kFifoBytesMask = 0x7F;
constexpr uint8_t kCrcOkBit = 0x80;
constexpr uint64_t kStallUs = 30000;  // A byte takes 160 us at 50 kBaud.
}  // namespace

float rssiToDbm(uint8_t raw) {
  const int value = raw >= 128 ? raw - 256 : raw;
  return value / 2.0f - 74.0f;
}

void RxAssembler::reset() {
  need_ = 0;
  have_ = 0;
}

bool RxAssembler::poll(uint64_t nowUs, RxFrame& out) {
  const uint8_t status = bus_.rxStatus();
  if (status & kFifoErrorBit) {
    overflows++;
    bus_.rearm();
    reset();
    return false;
  }

  size_t available = status & kFifoBytesMask;
  if (available == 0) {
    if (need_ != 0 && nowUs - lastByteUs_ > kStallUs) {
      stalls++;
      bus_.rearm();
      reset();
    }
    return false;
  }

  if (need_ == 0) {
    if (available < 2) {
      return false;  // Errata: the FIFO must not run empty while a packet arrives.
    }
    uint8_t length = 0;
    bus_.readFifo(&length, 1);
    available--;
    if (length == 0 || length > kMaxFrameLen) {
      badLengths++;
      bus_.rearm();
      reset();
      return false;
    }
    need_ = static_cast<size_t>(length) + 2;
    have_ = 0;
    lastByteUs_ = nowUs;
  }

  const size_t remaining = need_ - have_;
  const size_t take = available >= remaining ? remaining : available - 1;
  if (take > 0) {
    bus_.readFifo(buffer_ + have_, take);
    have_ += take;
    lastByteUs_ = nowUs;
  }
  if (have_ < need_) {
    return false;
  }
  finish(nowUs, out);
  reset();
  return true;
}

void RxAssembler::finish(uint64_t nowUs, RxFrame& out) {
  const size_t length = need_ - 2;
  std::memcpy(out.payload, buffer_, length);
  out.length = length;
  out.rssiDbm = rssiToDbm(buffer_[length]);
  out.lqi = buffer_[length + 1] & 0x7F;
  out.crcOk = (buffer_[length + 1] & kCrcOkBit) != 0;
  out.timestampUs = nowUs;
}

}  // namespace tadogado

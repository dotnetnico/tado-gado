// Builds frames from the RX FIFO while they arrive.
//
// A frame longer
// than the 64-byte FIFO stays intact, and the FIFO never runs empty in mid-packet,
// which the CC1101 errata forbids.
#pragma once

#include <cstddef>
#include <cstdint>

#include "frames.h"

namespace tadogado {

class RadioBus {
 public:
  virtual ~RadioBus() = default;
  // RXBYTES, with the overflow flag in bit 7.
  virtual uint8_t rxStatus() = 0;
  virtual void readFifo(uint8_t* out, size_t length) = 0;
  // Flush the FIFO and return to RX.
  virtual void rearm() = 0;
};

struct RxFrame {
  uint8_t payload[kMaxFrameLen];
  size_t length;
  float rssiDbm;
  uint8_t lqi;
  bool crcOk;
  uint64_t timestampUs;
};

float rssiToDbm(uint8_t raw);

class RxAssembler {
 public:
  explicit RxAssembler(RadioBus& bus) : bus_(bus) {}

  // Move bytes out of the FIFO. Returns true when `out` holds a complete frame.
  bool poll(uint64_t nowUs, RxFrame& out);
  bool busy() const { return need_ != 0; }
  void reset();

  uint32_t overflows = 0;
  uint32_t stalls = 0;
  uint32_t badLengths = 0;

 private:
  void finish(uint64_t nowUs, RxFrame& out);

  RadioBus& bus_;
  uint8_t buffer_[kMaxFrameLen + 2] = {};
  size_t need_ = 0;  // Payload plus two status bytes; 0 between frames.
  size_t have_ = 0;
  uint64_t lastByteUs_ = 0;
};

}  // namespace tadogado

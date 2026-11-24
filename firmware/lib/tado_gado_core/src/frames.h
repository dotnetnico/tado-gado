// MAC frames of the Tado 868 MHz network.
//
// Layouts, from freedo's docs/PROTOCOL.md (CappyT/freedo, Apache-2.0):
//   Data           fcf(2) seq dst_eui(8) src_prefix(5) ciphertext mic(4); 69 ec, or 79 ec with frame pending
//   Broadcast      fcf(2) seq ff ff src_eui(8) sec_ctrl counter(2 LE) ciphertext mic(4)
//   Enhanced ACK   42 ee seq dst_eui(8) src_eui(8) 04 0d phase(2 LE) period(2 LE) 80 3f fcs(2)
//   Wake-up        25 seq cd ab dst_short(2) 82 0e rendezvous(2 LE) 80 3f
// test_core/frame_vectors.inc holds frames as on air with their expected parse results.
#pragma once

#include <cstddef>
#include <cstdint>

namespace tadogado {

constexpr size_t kMaxFrameLen = 127;
constexpr size_t kPrefixLen = 5;
constexpr size_t kEuiLen = 8;
constexpr size_t kEnhAckLen = 29;
constexpr size_t kWakeupLen = 12;

constexpr uint64_t kCslUnitUs = 320;
constexpr uint64_t kCslPeriodUs = 1000000;
constexpr uint32_t kCslPeriodUnits = 3125;
constexpr uint64_t kWakeStepUs = 4000;

enum class FrameClass : uint8_t {
  Data,
  DataPending,
  Broadcast,
  EnhAck,
  Wakeup,
  BeaconRequest,
  Beacon,
  Unknown,
};

const char* frameClassName(FrameClass frameClass);
FrameClass classify(const uint8_t* frame, size_t length);

// CRC-16/CCITT reflected, init 0.
uint16_t macFcs(const uint8_t* data, size_t length);

uint32_t airtimeUs(size_t frameLength);

struct DataFrame {
  uint8_t seq;
  const uint8_t* dstEui;
  const uint8_t* srcPrefix;
  bool pending;
};

struct BroadcastFrame {
  uint8_t seq;
  const uint8_t* srcEui;
};

struct WakeUp {
  uint8_t seq;
  const uint8_t* dstShort;
  uint16_t rendezvous;
};

struct EnhAck {
  uint8_t seq;
  const uint8_t* dstEui;
  const uint8_t* srcEui;
  uint16_t phase;
  uint16_t period;
};

// Each parser returns false on a frame of another class, a bad length or a bad FCS.
// The pointers in the result point into `frame`.
bool parseData(const uint8_t* frame, size_t length, DataFrame& out);
bool parseBroadcast(const uint8_t* frame, size_t length, BroadcastFrame& out);
bool parseWakeup(const uint8_t* frame, size_t length, WakeUp& out);
bool parseEnhAck(const uint8_t* frame, size_t length, EnhAck& out);

void buildEnhAck(uint8_t seq, const uint8_t* dstEui, const uint8_t* srcEui, uint16_t phase, uint8_t out[kEnhAckLen]);
void buildWakeup(uint8_t seq, const uint8_t* dstShort, uint16_t rendezvous, uint8_t out[kWakeupLen]);

// 64-bit FNV-1a, for duplicate detection.
uint64_t hashBytes(const uint8_t* data, size_t length, uint64_t seed = 0xCBF29CE484222325ULL);

// Hash of what a MAC retransmission shares with the first frame: everything but
// the frame counter, the MAC FCS and the MIC.
uint64_t retransmissionHash(const uint8_t* frame, size_t length);

}  // namespace tadogado

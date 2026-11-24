#include "frames.h"

#include <cstring>

namespace tadogado {

namespace {

constexpr size_t kDataHeaderLen = 16;
constexpr size_t kMicLen = 4;
constexpr size_t kMinDataLen = kDataHeaderLen + kMicLen + 1;
constexpr size_t kMinBroadcastLen = 16 + kMicLen + 1;
constexpr size_t kPhyOverheadBytes = 9;  // Preamble 4, sync 2, length 1, PHY CRC 2.
constexpr uint32_t kDataRateBaud = 49987;
constexpr size_t kCounterStart = 4;  // Frame counter in the plaintext.
constexpr size_t kCounterEnd = 8;
constexpr size_t kFcsLen = 2;

uint16_t readLe16(const uint8_t* data) { return static_cast<uint16_t>(data[0] | (data[1] << 8)); }

}  // namespace

const char* frameClassName(FrameClass frameClass) {
  switch (frameClass) {
    case FrameClass::Data:
      return "data";
    case FrameClass::DataPending:
      return "data-pending";
    case FrameClass::Broadcast:
      return "broadcast";
    case FrameClass::EnhAck:
      return "enh-ack";
    case FrameClass::Wakeup:
      return "wakeup";
    case FrameClass::BeaconRequest:
      return "beacon-request";
    case FrameClass::Beacon:
      return "beacon";
    case FrameClass::Unknown:
      return "unknown";
  }
  return "unknown";
}

FrameClass classify(const uint8_t* frame, size_t length) {
  if (length < 2) {
    return length == 1 && frame[0] == 0x25 ? FrameClass::Wakeup : FrameClass::Unknown;
  }
  const uint8_t first = frame[0];
  const uint8_t second = frame[1];
  if (first == 0x69 && second == 0xEC) return FrameClass::Data;
  if (first == 0x79 && second == 0xEC) return FrameClass::DataPending;
  if ((first == 0x49 || first == 0x59) && second == 0xE8) return FrameClass::Broadcast;
  if (first == 0x42 && second == 0xEE) return FrameClass::EnhAck;
  if (first == 0x0B && second == 0xE8) return FrameClass::BeaconRequest;
  if (first == 0x18 && second == 0xEC) return FrameClass::Beacon;
  if (first == 0x25) return FrameClass::Wakeup;
  return FrameClass::Unknown;
}

uint16_t macFcs(const uint8_t* data, size_t length) {
  uint16_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

uint32_t airtimeUs(size_t frameLength) {
  const uint64_t bits = (frameLength + kPhyOverheadBytes) * 8ULL;
  return static_cast<uint32_t>(bits * 1000000ULL / kDataRateBaud);
}

bool parseData(const uint8_t* frame, size_t length, DataFrame& out) {
  const FrameClass frameClass = classify(frame, length);
  if (length < kMinDataLen || (frameClass != FrameClass::Data && frameClass != FrameClass::DataPending)) {
    return false;
  }
  out.seq = frame[2];
  out.dstEui = frame + 3;
  out.srcPrefix = frame + 11;
  out.pending = frameClass == FrameClass::DataPending;
  return true;
}

bool parseBroadcast(const uint8_t* frame, size_t length, BroadcastFrame& out) {
  if (length < kMinBroadcastLen || classify(frame, length) != FrameClass::Broadcast) {
    return false;
  }
  out.seq = frame[2];
  out.srcEui = frame + 5;
  return true;
}

bool parseWakeup(const uint8_t* frame, size_t length, WakeUp& out) {
  if (length != kWakeupLen || classify(frame, length) != FrameClass::Wakeup) {
    return false;
  }
  out.seq = frame[1];
  out.dstShort = frame + 4;
  out.rendezvous = readLe16(frame + 8);
  return true;
}

bool parseEnhAck(const uint8_t* frame, size_t length, EnhAck& out) {
  if (length != kEnhAckLen || classify(frame, length) != FrameClass::EnhAck) {
    return false;
  }
  if (readLe16(frame + 27) != macFcs(frame, 27)) {
    return false;
  }
  out.seq = frame[2];
  out.dstEui = frame + 3;
  out.srcEui = frame + 11;
  out.phase = readLe16(frame + 21);
  out.period = readLe16(frame + 23);
  return true;
}

void buildEnhAck(uint8_t seq, const uint8_t* dstEui, const uint8_t* srcEui, uint16_t phase, uint8_t out[kEnhAckLen]) {
  out[0] = 0x42;
  out[1] = 0xEE;
  out[2] = seq;
  std::memcpy(out + 3, dstEui, kEuiLen);
  std::memcpy(out + 11, srcEui, kEuiLen);
  out[19] = 0x04;
  out[20] = 0x0D;
  out[21] = static_cast<uint8_t>(phase);
  out[22] = static_cast<uint8_t>(phase >> 8);
  out[23] = static_cast<uint8_t>(kCslPeriodUnits);
  out[24] = static_cast<uint8_t>(kCslPeriodUnits >> 8);
  out[25] = 0x80;
  out[26] = 0x3F;
  const uint16_t fcs = macFcs(out, 27);
  out[27] = static_cast<uint8_t>(fcs);
  out[28] = static_cast<uint8_t>(fcs >> 8);
}

void buildWakeup(uint8_t seq, const uint8_t* dstShort, uint16_t rendezvous, uint8_t out[kWakeupLen]) {
  out[0] = 0x25;
  out[1] = seq;
  out[2] = 0xCD;
  out[3] = 0xAB;
  out[4] = dstShort[0];
  out[5] = dstShort[1];
  out[6] = 0x82;
  out[7] = 0x0E;
  out[8] = static_cast<uint8_t>(rendezvous);
  out[9] = static_cast<uint8_t>(rendezvous >> 8);
  out[10] = 0x80;
  out[11] = 0x3F;
}

uint64_t hashBytes(const uint8_t* data, size_t length, uint64_t seed) {
  uint64_t hash = seed;
  for (size_t i = 0; i < length; i++) {
    hash = (hash ^ data[i]) * 0x100000001B3ULL;
  }
  return hash;
}

uint64_t retransmissionHash(const uint8_t* frame, size_t length) {
  const size_t headerLen = length < kDataHeaderLen ? length : kDataHeaderLen;
  uint64_t hash = hashBytes(frame, headerLen);
  if (length < kDataHeaderLen + kMicLen) {
    return hash;
  }
  const uint8_t* body = frame + kDataHeaderLen;
  const size_t bodyLen = length - kDataHeaderLen - kMicLen;
  if (bodyLen <= kCounterEnd + kFcsLen) {
    return hashBytes(body, bodyLen, hash);
  }
  hash = hashBytes(body, kCounterStart, hash);
  return hashBytes(body + kCounterEnd, bodyLen - kCounterEnd - kFcsLen, hash);
}

}  // namespace tadogado

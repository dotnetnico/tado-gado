#include "pair_protocol.h"

#include <cstring>

#include "sha256.h"

namespace tadogado {

namespace {

constexpr uint8_t kMagic0 = 'T';
constexpr uint8_t kMagic1 = 'P';

void writeU32(uint8_t* out, uint32_t value) {
  for (int i = 0; i < 4; i++) {
    out[i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

uint32_t readU32(const uint8_t* in) {
  uint32_t value = 0;
  for (int i = 0; i < 4; i++) {
    value |= static_cast<uint32_t>(in[i]) << (8 * i);
  }
  return value;
}

static_assert(kPairHeaderLen + kPairMaxPayload <= kHmacMaxMessage, "the MAC covers the header and payload");

bool knownType(uint8_t type) {
  return type >= static_cast<uint8_t>(PairType::Hello) && type <= static_cast<uint8_t>(PairType::Name);
}

}  // namespace

size_t sealPair(const uint8_t* secret, size_t secretLength, const PairHeader& header, const uint8_t* payload,
                size_t payloadLength, uint8_t* out) {
  if (payloadLength > kPairMaxPayload) {
    return 0;
  }
  out[0] = kMagic0;
  out[1] = kMagic1;
  out[2] = static_cast<uint8_t>(header.type);
  std::memcpy(out + 3, header.nodeId, kNodeIdLen);
  writeU32(out + 9, header.senderBoot);
  writeU32(out + 13, header.receiverBoot);
  writeU32(out + 17, header.counter);
  if (payloadLength > 0) {
    std::memcpy(out + kPairHeaderLen, payload, payloadLength);
  }
  const size_t signedLength = kPairHeaderLen + payloadLength;
  uint8_t mac[kSha256Len];
  hmacSha256(secret, secretLength, out, signedLength, mac);
  std::memcpy(out + signedLength, mac, kPairMacLen);
  return signedLength + kPairMacLen;
}

bool openPair(const uint8_t* secret, size_t secretLength, const uint8_t* datagram, size_t length, PairHeader& header,
              const uint8_t*& payload, size_t& payloadLength) {
  if (length < kPairHeaderLen + kPairMacLen || length > kPairMaxDatagram || datagram[0] != kMagic0 ||
      datagram[1] != kMagic1 || !knownType(datagram[2])) {
    return false;
  }
  const size_t signedLength = length - kPairMacLen;
  uint8_t mac[kSha256Len];
  hmacSha256(secret, secretLength, datagram, signedLength, mac);
  uint8_t difference = 0;
  for (size_t i = 0; i < kPairMacLen; i++) {
    difference |= mac[i] ^ datagram[signedLength + i];
  }
  if (difference != 0) {
    return false;
  }
  header.type = static_cast<PairType>(datagram[2]);
  std::memcpy(header.nodeId, datagram + 3, kNodeIdLen);
  header.senderBoot = readU32(datagram + 9);
  header.receiverBoot = readU32(datagram + 13);
  header.counter = readU32(datagram + 17);
  payload = datagram + kPairHeaderLen;
  payloadLength = signedLength - kPairHeaderLen;
  return true;
}

void derivePairKey(const char* token, uint8_t out[kPairKeyLen], uint32_t iterations) {
  // Changing it changes every pair key: both nodes of a pair must then run the same version.
  static const char kLabel[] = "tado-gado pair key v1";
  pbkdf2Sha256(reinterpret_cast<const uint8_t*>(token), std::strlen(token), reinterpret_cast<const uint8_t*>(kLabel),
               sizeof(kLabel) - 1, iterations, out);
}

}  // namespace tadogado

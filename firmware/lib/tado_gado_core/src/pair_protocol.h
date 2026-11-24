// The messages between two nodes on the home network.
//
// A forged datagram could make a node transmit on the heating network, so each one
// carries an HMAC under a key derived from the admin token (derivePairKey). The ESP32
// has no clock to bound a replay, so each node counts its boots in flash, and frames
// name the boot of their receiver: a recording stops working as soon as the receiver
// restarts, and a counter stops a replay before that. A Hello from a lower boot than
// the last one seen is a replay, and is refused.
//
//   'T' 'P' type node_id(6) sender_boot(4) receiver_boot(4) counter(4) payload mac(16)
#pragma once

#include <cstddef>
#include <cstdint>

#include "frames.h"
#include "link_monitor.h"
#include "name_table.h"

namespace tadogado {

constexpr uint16_t kPairPort = 8868;
constexpr size_t kNodeIdLen = 6;
constexpr size_t kPairHeaderLen = 21;
constexpr size_t kPairMacLen = 16;
// count, then per device: prefix and flags.
constexpr size_t kPairProxiedEntryLen = kPrefixLen + 1;
constexpr size_t kPairProxiedMax = 1 + kMaxProxied * kPairProxiedEntryLen;
// count, then per ACK: prefix, seq and flags.
constexpr size_t kPairAckEntryLen = kPrefixLen + 2;
constexpr size_t kPairAcksPerMessage = 30;
constexpr size_t kPairAcksMax = 1 + kPairAcksPerMessage * kPairAckEntryLen;
// flags, prefix, then the name without its terminator.
constexpr size_t kPairNameMax = 1 + kPrefixLen + kMaxNameBytes;
constexpr size_t largerOf(size_t a, size_t b) { return a > b ? a : b; }
constexpr size_t kPairMaxPayload =
    largerOf(largerOf(kPairProxiedMax, kMaxFrameLen), largerOf(kPairAcksMax, kPairNameMax));
constexpr size_t kPairMaxDatagram = kPairHeaderLen + kPairMaxPayload + kPairMacLen;
constexpr size_t kPairMaxSecret = 64;
constexpr size_t kPairKeyLen = 32;
// PBKDF2 rounds for the pair key. A captured Hello lets an attacker test guesses of
// the token offline; each guess then costs this many HMACs. About 0.5 s on an ESP32-S3.
constexpr uint32_t kPairKeyIterations = 20000;

// The HMAC key of the pair link, from the admin token. A different label than any
// other use of the token, so the two never share a key.
void derivePairKey(const char* token, uint8_t out[kPairKeyLen], uint32_t iterations = kPairKeyIterations);

enum class PairType : uint8_t {
  Hello = 1,     // Broadcast: this node is here, and how well it hears the bridge.
  Uplink = 2,    // A frame for the bridge.
  Downlink = 3,  // A frame for a device.
  Mode = 4,      // Switch proxy on or off on both nodes.
  Proxied = 5,   // The list of devices to serve, and in which directions.
  Acks = 6,      // ACKs this node heard, so the other can count them too.
  Name = 7,      // The name of a device.
};

struct PairHeader {
  PairType type;
  uint8_t nodeId[kNodeIdLen];
  uint32_t senderBoot;
  uint32_t receiverBoot;
  uint32_t counter;
};

// Returns the datagram length, or 0 if the payload is too long. `out` holds kPairMaxDatagram.
size_t sealPair(const uint8_t* secret, size_t secretLength, const PairHeader& header, const uint8_t* payload,
                size_t payloadLength, uint8_t* out);

// Returns false unless the datagram is well formed and its MAC is right.
bool openPair(const uint8_t* secret, size_t secretLength, const uint8_t* datagram, size_t length, PairHeader& header,
              const uint8_t*& payload, size_t& payloadLength);

}  // namespace tadogado

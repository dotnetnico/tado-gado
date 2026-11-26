#include "pair_manager.h"

#include <algorithm>
#include <cstring>

namespace tadogado {

namespace {

constexpr uint64_t kHelloIntervalUs = 2000000;
constexpr uint64_t kPeerTimeoutUs = 10000000;   // Five missed hellos.
constexpr uint64_t kStartTimeoutUs = 10000000;  // Time the peer has to join in.
constexpr uint64_t kRssiFreshUs = 600000000;    // Traffic to the bridge can be sparse.
constexpr float kRoleHysteresisDb = 3.0f;       // Roles do not flip on a few dB of noise.

constexpr uint8_t kHasRssi = 1;
constexpr uint8_t kWantsProxy = 2;
constexpr uint8_t kHasAssignment = 4;
constexpr uint8_t kHasBridge = 8;
// flags rssi(2) bridge_node(6) bridge(5) budget_ms(2) free_jobs
constexpr size_t kCapacityOffset = 1 + 2 + kNodeIdLen + kPrefixLen;
constexpr size_t kHelloLen = kCapacityOffset + 3;
constexpr uint8_t kAckUplink = 1;
constexpr uint8_t kNameReplaces = 1;
// ACKs go out in batches: few datagrams, and news well within the 3 s a frame waits.
constexpr uint64_t kAcksIntervalUs = 250000;
constexpr size_t kNamesPerHello = 4;

bool sameId(const uint8_t* a, const uint8_t* b) { return std::memcmp(a, b, kNodeIdLen) == 0; }

}  // namespace

void PairManager::begin(const uint8_t* secret, size_t secretLength, const uint8_t* nodeId, uint32_t bootNumber) {
  secretLength_ = std::min(secretLength, kPairMaxSecret);
  if (secretLength_ > 0) {
    std::memcpy(secret_, secret, secretLength_);
  }
  std::memcpy(nodeId_, nodeId, kNodeIdLen);
  boot_ = bootNumber;
}

bool PairManager::peerAlive(uint64_t nowUs) const { return hasPeer_ && nowUs - peerSeenUs_ < kPeerTimeoutUs; }

bool PairManager::peerRssi(float& out) const {
  out = peerRssi_;
  return peerHasRssi_;
}

bool PairManager::ownRssi(uint64_t nowUs, float& out) const {
  if (!monitor_.hasBridge()) {
    return false;
  }
  const DeviceLink* bridge = monitor_.findLink(monitor_.bridge());
  if (bridge == nullptr || bridge->frames == 0 || nowUs - bridge->lastSeenUs > kRssiFreshUs) {
    return false;
  }
  out = bridge->rssiDbm;
  return true;
}

PairStatus PairManager::status(uint64_t nowUs) const {
  PairStatus out = {};
  out.enabled = enabled();
  out.hasPeer = hasPeer_;
  std::memcpy(out.peerId, peerId_, kNodeIdLen);
  out.peerAlive = peerAlive(nowUs);
  out.rolesDecided = hasAssignment_;
  out.peerRestarts = peerRestarts_;
  out.rejected = rejected_;
  out.hasOwnRssi = ownRssi(nowUs, out.ownRssi);
  out.hasPeerRssi = peerRssi(out.peerRssi);
  out.wanted = wanted_;
  out.role = role_;
  return out;
}

bool PairManager::isDecider() const { return std::memcmp(nodeId_, peerId_, kNodeIdLen) < 0; }

// ------------------------------------------------------------------------ Sending

void PairManager::send(PairType type, const uint8_t* payload, size_t length, bool broadcast) {
  if (!enabled()) {
    return;
  }
  PairHeader header = {};
  header.type = type;
  std::memcpy(header.nodeId, nodeId_, kNodeIdLen);
  header.senderBoot = boot_;
  header.receiverBoot = broadcast ? 0 : peerBoot_;
  header.counter = ++counter_;
  uint8_t datagram[kPairMaxDatagram];
  const size_t datagramLength = sealPair(secret_, secretLength_, header, payload, length, datagram);
  if (datagramLength == 0) {
    return;
  }
  if (broadcast) {
    transport_.broadcast(datagram, datagramLength);
  } else {
    transport_.sendToPeer(datagram, datagramLength);
  }
}

void PairManager::sendHello(uint64_t nowUs) {
  uint8_t payload[kHelloLen] = {};
  uint8_t flags = wanted_ ? kWantsProxy : 0;
  float rssi = 0;
  if (ownRssi(nowUs, rssi)) {
    flags |= kHasRssi;
    const int16_t halfDb = static_cast<int16_t>(rssi * 2);
    payload[1] = static_cast<uint8_t>(halfDb);
    payload[2] = static_cast<uint8_t>(static_cast<uint16_t>(halfDb) >> 8);
  }
  if (hasAssignment_) {
    flags |= kHasAssignment;
    std::memcpy(payload + 3, bridgeNode_, kNodeIdLen);
  }
  if (monitor_.hasBridge()) {
    flags |= kHasBridge;
    std::memcpy(payload + 3 + kNodeIdLen, monitor_.bridge(), kPrefixLen);
  }
  const Capacity capacity = engine_.capacity(nowUs);
  const uint64_t budgetMs = std::min<uint64_t>(capacity.budgetUs / 1000, 0xFFFF);
  payload[kCapacityOffset] = static_cast<uint8_t>(budgetMs);
  payload[kCapacityOffset + 1] = static_cast<uint8_t>(budgetMs >> 8);
  payload[kCapacityOffset + 2] = capacity.freeJobs;
  payload[0] = flags;
  send(PairType::Hello, payload, sizeof(payload), true);
}

void PairManager::sendMode(bool enabled) {
  const uint8_t value = enabled ? 1 : 0;
  send(PairType::Mode, &value, 1, false);
}

void PairManager::sendFrame(bool uplink, const uint8_t* frame, size_t length) {
  send(uplink ? PairType::Uplink : PairType::Downlink, frame, length, false);
}

void PairManager::syncProxied() {
  uint8_t payload[kPairProxiedMax];
  const size_t count = monitor_.proxiedCount();
  payload[0] = static_cast<uint8_t>(count);
  for (size_t i = 0; i < count; i++) {
    uint8_t* entry = payload + 1 + i * kPairProxiedEntryLen;
    std::memcpy(entry, monitor_.proxied(i), kPrefixLen);
    entry[kPrefixLen] = static_cast<uint8_t>(monitor_.proxiedDirection(i));
  }
  if (hasPeer_) {
    send(PairType::Proxied, payload, 1 + count * kPairProxiedEntryLen, false);
  }
}

void PairManager::sendAcks(uint64_t nowUs) {
  if (nowUs < nextAcksUs_) {
    return;
  }
  nextAcksUs_ = nowUs + kAcksIntervalUs;
  HeardAck acks[kPairAcksPerMessage];
  const size_t count = monitor_.takeHeardAcks(acks, kPairAcksPerMessage);
  if (count == 0) {
    return;
  }
  uint8_t payload[kPairAcksMax];
  payload[0] = static_cast<uint8_t>(count);
  for (size_t i = 0; i < count; i++) {
    uint8_t* entry = payload + 1 + i * kPairAckEntryLen;
    std::memcpy(entry, acks[i].device, kPrefixLen);
    entry[kPrefixLen] = acks[i].seq;
    entry[kPrefixLen + 1] = acks[i].uplink ? kAckUplink : 0;
  }
  send(PairType::Acks, payload, 1 + count * kPairAckEntryLen, false);
}

void PairManager::sendName(const uint8_t* prefix, const char* name) {
  if (hasPeer_) {
    sendNameEntry(prefix, name, true);
  }
}

void PairManager::sendNameEntry(const uint8_t* prefix, const char* name, bool replace) {
  uint8_t payload[kPairNameMax];
  const size_t length = std::min(std::strlen(name), kMaxNameBytes);
  payload[0] = replace ? kNameReplaces : 0;
  std::memcpy(payload + 1, prefix, kPrefixLen);
  std::memcpy(payload + 1 + kPrefixLen, name, length);
  send(PairType::Name, payload, 1 + kPrefixLen + length, false);
}

void PairManager::syncNames() {
  if (names_ == nullptr) {
    nameSyncNext_ = NameTable::kCapacity;
    return;
  }
  // After a new peer: all names, a few per Hello.
  if (nameSyncNext_ < NameTable::kCapacity) {
    size_t sent = 0;
    while (nameSyncNext_ < NameTable::kCapacity && sent < kNamesPerHello) {
      const NameTable::Entry& entry = names_->entries()[nameSyncNext_++];
      if (entry.used) {
        sendNameEntry(entry.prefix, entry.name, false);
        sent++;
      }
    }
    return;
  }
  // Then one name per Hello, round and round. UDP may lose the first round, and the
  // other node may not yet know us when it arrives; a later round brings what it missed.
  // A name the other node already has stays as it is there.
  for (size_t tried = 0; tried < NameTable::kCapacity; tried++) {
    const NameTable::Entry& entry = names_->entries()[nameRepeatNext_];
    nameRepeatNext_ = (nameRepeatNext_ + 1) % NameTable::kCapacity;
    if (entry.used) {
      sendNameEntry(entry.prefix, entry.name, false);
      return;
    }
  }
}

// ---------------------------------------------------------------------- Receiving

void PairManager::onDatagram(const uint8_t* data, size_t length, uint64_t nowUs) {
  if (!enabled()) {
    return;
  }
  PairHeader header;
  const uint8_t* payload = nullptr;
  size_t payloadLength = 0;
  if (!openPair(secret_, secretLength_, data, length, header, payload, payloadLength)) {
    rejected_++;
    return;
  }
  if (sameId(header.nodeId, nodeId_)) {
    return;  // Our own broadcast.
  }
  if (header.type == PairType::Hello) {
    onHello(header, payload, payloadLength, nowUs);
  } else {
    onPeerMessage(header, payload, payloadLength, nowUs);
  }
}

void PairManager::onHello(const PairHeader& header, const uint8_t* payload, size_t length, uint64_t nowUs) {
  if (length != kHelloLen) {
    rejected_++;
    return;
  }
  // A pair is two nodes: a third is ignored while the first is alive.
  if (hasPeer_ && !sameId(header.nodeId, peerId_) && peerAlive(nowUs)) {
    return;
  }
  const bool samePeer = hasPeer_ && sameId(header.nodeId, peerId_);
  // Boot numbers only go up, so a lower one is a recorded Hello played back. Taking
  // it as a restart would reset the counters and let old messages count again.
  if (samePeer && header.senderBoot < peerBoot_) {
    rejected_++;
    return;
  }
  if (samePeer && header.senderBoot == peerBoot_ && header.counter <= peerHelloCounter_) {
    rejected_++;
    return;
  }
  if (!samePeer || header.senderBoot != peerBoot_) {
    if (hasPeer_) {
      peerRestarts_++;
    }
    if (role_ != Role::Single) {
      // A restart stops both nodes: a power cycle must stop all transmissions.
      wanted_ = false;
      startPending_ = false;
    }
    hasAssignment_ = false;
    peerWanted_ = false;
    std::memcpy(peerId_, header.nodeId, kNodeIdLen);
    peerBoot_ = header.senderBoot;
    peerCounter_ = 0;
    hasPeer_ = true;
    nameSyncNext_ = 0;  // A new node, or a restarted one: it gets our names.
  }
  peerHelloCounter_ = header.counter;
  peerSeenUs_ = nowUs;
  transport_.rememberPeer();

  const uint8_t flags = payload[0];
  peerHasRssi_ = (flags & kHasRssi) != 0;
  if (peerHasRssi_) {
    const uint16_t raw = static_cast<uint16_t>(payload[1] | payload[2] << 8);
    peerRssi_ = static_cast<int16_t>(raw) / 2.0f;
  }
  peerWanted_ = (flags & kWantsProxy) != 0;
  if ((flags & kHasAssignment) != 0 && !isDecider() && !wanted_) {
    hasAssignment_ = true;
    std::memcpy(bridgeNode_, payload + 3, kNodeIdLen);
  }
  if ((flags & kHasBridge) != 0 && !monitor_.hasBridge()) {
    monitor_.setBridge(payload + 3 + kNodeIdLen);
  }
  Capacity capacity = {};
  capacity.budgetUs = (payload[kCapacityOffset] | payload[kCapacityOffset + 1] << 8) * 1000ULL;
  capacity.freeJobs = payload[kCapacityOffset + 2];
  engine_.setPeerCapacity(capacity, nowUs);
}

void PairManager::onPeerMessage(const PairHeader& header, const uint8_t* payload, size_t length, uint64_t nowUs) {
  if (!hasPeer_ || !sameId(header.nodeId, peerId_) || header.senderBoot != peerBoot_ || header.receiverBoot != boot_ ||
      header.counter <= peerCounter_) {
    rejected_++;
    return;
  }
  peerCounter_ = header.counter;
  peerSeenUs_ = nowUs;
  switch (header.type) {
    case PairType::Uplink:
    case PairType::Downlink:
      engine_.onPeerFrame(header.type == PairType::Uplink, payload, length, nowUs);
      return;
    case PairType::Mode:
      if (length == 1 && payload[0] == 1 && !transmitAllowed_) {
        peerWanted_ = false;
        sendMode(false);  // This node may not send here, so the other one must not start.
      } else if (length == 1 && payload[0] == 1) {
        peerWanted_ = true;
        if (hasAssignment_ && !wanted_) {
          updateRole(nowUs);
          wanted_ = true;
          // Answer at once: the first node must not promise delivery before we listen.
          sendMode(true);
        }
      } else if (length == 1) {
        peerWanted_ = false;
        wanted_ = false;
        startPending_ = false;
      }
      return;
    case PairType::Proxied:
      onProxied(payload, length);
      return;
    case PairType::Acks:
      onAcks(payload, length, nowUs);
      return;
    case PairType::Name:
      onName(payload, length);
      return;
    default:
      return;
  }
}

void PairManager::onProxied(const uint8_t* payload, size_t length) {
  if (length < 1 || payload[0] > kMaxProxied || length != 1 + static_cast<size_t>(payload[0]) * kPairProxiedEntryLen) {
    rejected_++;
    return;
  }
  for (size_t i = 0; i < payload[0]; i++) {
    if (payload[1 + i * kPairProxiedEntryLen + kPrefixLen] > kProxyDirectionLast) {
      rejected_++;  // A direction this version does not know: change nothing.
      return;
    }
  }
  bool same = payload[0] == monitor_.proxiedCount();
  for (size_t i = 0; same && i < payload[0]; i++) {
    const uint8_t* entry = payload + 1 + i * kPairProxiedEntryLen;
    const ProxyDirection direction = static_cast<ProxyDirection>(entry[kPrefixLen]);
    same = monitor_.isProxied(entry) && monitor_.proxiesUplink(entry) == (direction != ProxyDirection::ToValve) &&
           monitor_.proxiesDownlink(entry) == (direction != ProxyDirection::ToBridge);
  }
  if (same) {
    return;  // Nothing to write to flash.
  }
  monitor_.clearProxied();
  for (size_t i = 0; i < payload[0]; i++) {
    const uint8_t* entry = payload + 1 + i * kPairProxiedEntryLen;
    monitor_.setProxied(entry, true, static_cast<ProxyDirection>(entry[kPrefixLen]));
  }
  proxiedChanged_ = true;
}

void PairManager::onAcks(const uint8_t* payload, size_t length, uint64_t nowUs) {
  if (length < 1 || payload[0] > kPairAcksPerMessage ||
      length != 1 + static_cast<size_t>(payload[0]) * kPairAckEntryLen) {
    rejected_++;
    return;
  }
  for (size_t i = 0; i < payload[0]; i++) {
    const uint8_t* entry = payload + 1 + i * kPairAckEntryLen;
    monitor_.notePeerAck(entry, entry[kPrefixLen], (entry[kPrefixLen + 1] & kAckUplink) != 0, nowUs);
  }
}

void PairManager::onName(const uint8_t* payload, size_t length) {
  if (length < 1 + kPrefixLen || length > kPairNameMax) {
    rejected_++;
    return;
  }
  if (names_ == nullptr) {
    return;
  }
  const uint8_t* prefix = payload + 1;
  char name[kMaxNameBytes + 1];
  const size_t nameLength = length - 1 - kPrefixLen;
  std::memcpy(name, payload + 1 + kPrefixLen, nameLength);
  name[nameLength] = '\0';
  const char* current = names_->find(prefix);
  const bool replaces = (payload[0] & kNameReplaces) != 0;
  if (!replaces && current != nullptr) {
    return;  // Each node keeps its own name; the user's change is what replaces it.
  }
  const bool unchanged = current != nullptr ? std::strcmp(current, name) == 0 : name[0] == '\0';
  if (unchanged) {
    return;
  }
  if (std::strlen(name) != nameLength || !names_->set(prefix, name)) {
    rejected_++;  // A zero byte inside, a character JSON would need escaped, or a full table.
    return;
  }
  namesChanged_ = true;
}

bool PairManager::takeNamesChanged() {
  const bool changed = namesChanged_;
  namesChanged_ = false;
  return changed;
}

bool PairManager::takeProxiedChanged() {
  const bool changed = proxiedChanged_;
  proxiedChanged_ = false;
  return changed;
}

// ------------------------------------------------------------------ Decisions

void PairManager::elect(uint64_t nowUs) {
  if (!peerAlive(nowUs) || !isDecider() || wanted_ || peerWanted_) {
    return;
  }
  float mine = 0;
  float theirs = 0;
  const bool hasMine = ownRssi(nowUs, mine);
  const bool hasTheirs = peerRssi(theirs);
  if (!hasMine && !hasTheirs) {
    return;  // Neither hears the bridge: no way to tell yet.
  }
  bool iAmBridge;
  if (!hasMine || !hasTheirs) {
    iAmBridge = hasMine;
  } else if (hasAssignment_) {
    const bool wasBridge = sameId(bridgeNode_, nodeId_);
    const float margin = wasBridge ? theirs - mine : mine - theirs;
    iAmBridge = margin > kRoleHysteresisDb ? !wasBridge : wasBridge;
  } else {
    iAmBridge = mine >= theirs;
  }
  hasAssignment_ = true;
  std::memcpy(bridgeNode_, iAmBridge ? nodeId_ : peerId_, kNodeIdLen);
}

void PairManager::updateRole(uint64_t nowUs) {
  if (wanted_) {
    return;  // Roles never change while proxying.
  }
  if (!peerAlive(nowUs)) {
    hasAssignment_ = false;
    role_ = Role::Single;
  } else if (hasAssignment_) {
    role_ = sameId(bridgeNode_, nodeId_) ? Role::Bridge : Role::Valve;
  } else {
    role_ = Role::Single;
  }
}

void PairManager::apply(uint64_t nowUs) {
  const bool paired = role_ != Role::Single;
  const bool active = wanted_ && transmitAllowed_ && (!paired || (peerAlive(nowUs) && peerWanted_));
  if (engine_.proxyMode() != active) {
    if (active) {
      engine_.setRole(role_);
    }
    engine_.setProxyMode(active);
  }
}

bool PairManager::requestProxy(bool enabled, uint64_t nowUs) {
  if (!enabled) {
    wanted_ = false;
    startPending_ = false;
    peerWanted_ = false;
    if (hasPeer_) {
      sendMode(false);
    }
    return true;
  }
  if (!transmitAllowed_ || (peerAlive(nowUs) && !hasAssignment_)) {
    return false;
  }
  updateRole(nowUs);
  wanted_ = true;
  if (role_ != Role::Single) {
    startPending_ = true;
    startDeadlineUs_ = nowUs + kStartTimeoutUs;
    syncProxied();
    sendMode(true);
  }
  return true;
}

void PairManager::setTransmitAllowed(bool allowed, uint64_t nowUs) {
  transmitAllowed_ = allowed;
  if (!allowed && wanted_) {
    requestProxy(false, nowUs);
    apply(nowUs);
  }
}

void PairManager::tick(uint64_t nowUs) {
  if (enabled() && nowUs >= nextHelloUs_) {
    nextHelloUs_ = nowUs + kHelloIntervalUs;
    sendHello(nowUs);
    if (peerAlive(nowUs)) {
      syncNames();
    }
    if (startPending_ && wanted_ && !peerWanted_) {
      sendMode(true);  // The first one may be lost, as this is plain UDP.
    }
  }
  monitor_.setPeerLinked(enabled() && peerAlive(nowUs));
  if (enabled() && peerAlive(nowUs)) {
    sendAcks(nowUs);
  }
  elect(nowUs);
  updateRole(nowUs);

  if (wanted_ && role_ != Role::Single) {
    if (startPending_) {
      if (peerWanted_) {
        startPending_ = false;
      } else if (nowUs >= startDeadlineUs_) {
        wanted_ = false;  // The peer never joined.
        startPending_ = false;
      }
    } else if (!peerWanted_ && peerAlive(nowUs)) {
      wanted_ = false;  // The user stopped the peer, or it restarted.
    }
  }
  apply(nowUs);
}

}  // namespace tadogado

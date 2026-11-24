#include "link_monitor.h"

#include <cstring>

namespace tadogado {

namespace {
constexpr uint64_t kObserveAckUs = 25000;     // Time an uplink has to get its ACK.
constexpr uint64_t kTrainGapUs = 100000;      // Data frame behind a wake-up train.
constexpr uint64_t kPeerAckWaitUs = 3000000;  // The other node reports at least every 2 s.
// A device heard in this few frames, this long ago, and never in traffic with the bridge,
// makes way for a new one in a full table.
constexpr uint32_t kRareFrames = 2;
constexpr uint64_t kRareAgeUs = 600000000;
constexpr float kSmoothing = 0.2f;

bool samePrefix(const uint8_t* a, const uint8_t* b) { return std::memcmp(a, b, kPrefixLen) == 0; }

uint64_t ackKey(const uint8_t* device, uint8_t seq, bool uplink) {
  uint8_t key[kPrefixLen + 2];
  std::memcpy(key, device, kPrefixLen);
  key[kPrefixLen] = seq;
  key[kPrefixLen + 1] = uplink ? 1 : 0;
  return hashBytes(key, sizeof(key));
}

// A train is a wake-up frame every step. Counting from its length rather than from the
// frames heard keeps one missed frame from shortening it.
uint64_t trainAirtimeUs(uint64_t spanUs) { return (spanUs / kWakeStepUs + 1) * airtimeUs(kWakeupLen); }

void addTrains(TrainTotals& total, const TrainTotals& bucket) {
  total.bridgeUs += bucket.bridgeUs;
  total.peerUs += bucket.peerUs;
  total.unknownUs += bucket.unknownUs;
}
}  // namespace

void LinkMonitor::setBridge(const uint8_t* prefix) {
  std::memcpy(bridge_, prefix, kPrefixLen);
  hasBridge_ = true;
}

bool LinkMonitor::setProxied(const uint8_t* prefix, bool enabled, ProxyDirection direction) {
  for (size_t i = 0; i < proxiedCount_; i++) {
    if (samePrefix(proxied_[i], prefix)) {
      if (!enabled) {
        const size_t after = proxiedCount_ - i - 1;
        std::memmove(proxied_[i], proxied_[i + 1], after * kPrefixLen);
        std::memmove(&proxiedDirection_[i], &proxiedDirection_[i + 1], after * sizeof(ProxyDirection));
        proxiedCount_--;
      } else {
        proxiedDirection_[i] = direction;
      }
      return true;
    }
  }
  if (!enabled) {
    return true;
  }
  if (proxiedCount_ >= kMaxProxied) {
    return false;
  }
  std::memcpy(proxied_[proxiedCount_], prefix, kPrefixLen);
  proxiedDirection_[proxiedCount_] = direction;
  proxiedCount_++;
  return true;
}

const DeviceLink* LinkMonitor::findLink(const uint8_t* prefix) const {
  for (const DeviceLink& entry : devices_) {
    if (entry.used && samePrefix(entry.prefix, prefix)) {
      return &entry;
    }
  }
  return nullptr;
}

bool LinkMonitor::isProxied(const uint8_t* prefix) const {
  for (size_t i = 0; i < proxiedCount_; i++) {
    if (samePrefix(proxied_[i], prefix)) {
      return true;
    }
  }
  return false;
}

bool LinkMonitor::proxiesUplink(const uint8_t* prefix) const {
  for (size_t i = 0; i < proxiedCount_; i++) {
    if (samePrefix(proxied_[i], prefix)) {
      return proxiedDirection_[i] != ProxyDirection::ToValve;
    }
  }
  return false;
}

bool LinkMonitor::proxiesDownlink(const uint8_t* prefix) const {
  for (size_t i = 0; i < proxiedCount_; i++) {
    if (samePrefix(proxied_[i], prefix)) {
      return proxiedDirection_[i] != ProxyDirection::ToBridge;
    }
  }
  return false;
}

bool LinkMonitor::proxiesAnyDownlink() const {
  for (size_t i = 0; i < proxiedCount_; i++) {
    if (proxiedDirection_[i] != ProxyDirection::ToBridge) {
      return true;
    }
  }
  return false;
}

void LinkMonitor::noteHandedOver(const uint8_t* frame, size_t length, uint64_t nowUs) {
  handedOver_.add(hashBytes(frame, length), nowUs + kHandedOverTtlUs);
}

bool LinkMonitor::sentByOtherNode(const uint8_t* frame, size_t length, uint64_t nowUs) const {
  return handedOver_.contains(hashBytes(frame, length), nowUs);
}

const RecentFrame& LinkMonitor::recent(size_t index) const {
  return recent_[(recentNext_ + kRecentFrames - 1 - index) % kRecentFrames];
}

void LinkMonitor::onFrame(const RxFrame& rx) {
  nowUs_ = rx.timestampUs;
  advanceWindows(rx.timestampUs);
  counters_.framesRx++;
  if (!rx.crcOk) {
    counters_.crcErrors++;
    return;
  }
  const FrameClass frameClass = classify(rx.payload, rx.length);
  switch (frameClass) {
    case FrameClass::Wakeup: {
      WakeUp wake;
      if (parseWakeup(rx.payload, rx.length, wake)) {
        onWakeup(rx, wake);
      } else {
        counters_.fcsErrors++;
      }
      return;
    }
    case FrameClass::Data:
    case FrameClass::DataPending: {
      DataFrame data;
      if (parseData(rx.payload, rx.length, data)) {
        onData(rx, data);
      } else {
        counters_.fcsErrors++;
      }
      return;
    }
    case FrameClass::EnhAck: {
      EnhAck ack;
      if (parseEnhAck(rx.payload, rx.length, ack)) {
        onEnhAck(rx, ack);
      } else {
        counters_.fcsErrors++;
      }
      return;
    }
    case FrameClass::Broadcast: {
      BroadcastFrame broadcast;
      if (parseBroadcast(rx.payload, rx.length, broadcast)) {
        onBroadcast(rx, broadcast);
      } else {
        counters_.fcsErrors++;
      }
      return;
    }
    default:
      remember(rx, frameClass, nullptr, nullptr, nullptr);
      return;
  }
}

void LinkMonitor::onWakeup(const RxFrame& rx, const WakeUp& wake) {
  counters_.wakeups++;
  const bool continues = train_.open && wake.seq == train_.seq && rx.timestampUs - train_.lastUs < kTrainGapUs &&
                         std::memcmp(wake.dstShort, train_.dstShort, 2) == 0;
  if (continues) {
    train_.lastUs = rx.timestampUs;
  } else {
    closeTrain();
    train_.open = true;
    train_.claimed = false;
    train_.seq = wake.seq;
    std::memcpy(train_.dstShort, wake.dstShort, 2);
    train_.startUs = rx.timestampUs;
    train_.lastUs = rx.timestampUs;
  }
  wakeSeen_ = true;
  lastWakeUs_ = rx.timestampUs;
  lastWakeSeq_ = wake.seq;
}

uint64_t LinkMonitor::claimTrain(const RxFrame& rx, uint8_t seq, TrainSender sender, const uint8_t* src,
                                 const uint8_t* dst) {
  if (!train_.open || train_.claimed || train_.seq != seq || rx.timestampUs - train_.lastUs >= kTrainGapUs) {
    return 0;
  }
  const bool forAll = train_.dstShort[0] == 0xFF && train_.dstShort[1] == 0xFF;
  if (dst != nullptr && !forAll && (dst[0] != train_.dstShort[1] || dst[1] != train_.dstShort[0])) {
    return 0;  // A train for another device, with the same sequence number by chance.
  }
  train_.claimed = true;
  const uint64_t span = train_.lastUs - train_.startUs;
  DeviceLink* from = sender == TrainSender::Unknown || src == nullptr ? nullptr : link(src);
  DeviceLink* to = dst != nullptr ? link(dst) : nullptr;
  countTrain(sender, from, to, trainAirtimeUs(span));
  // At least one step, so a train of one frame counts as a train.
  return span + kWakeStepUs;
}

void LinkMonitor::closeTrain() {
  if (train_.open && !train_.claimed) {
    // No frame followed that this node heard, so its sender is unknown.
    const uint64_t span = train_.lastUs - train_.startUs;
    countTrain(TrainSender::Unknown, nullptr, deviceWithShort(train_.dstShort), trainAirtimeUs(span));
  }
  train_.open = false;
}

void LinkMonitor::countTrain(TrainSender sender, DeviceLink* from, DeviceLink* to, uint64_t airtimeUs) {
  TrainTotals& hour = hourTrains_[hourClock_.slot()];
  TrainTotals& day = dayTrains_[dayClock_.slot()];
  uint64_t TrainTotals::* field = sender == TrainSender::Bridge ? &TrainTotals::bridgeUs
                                  : sender == TrainSender::Peer ? &TrainTotals::peerUs
                                                                : &TrainTotals::unknownUs;
  hour.*field += airtimeUs;
  day.*field += airtimeUs;
  addTime(from, &LinkBucket::airtimeUs, airtimeUs);
  addTime(to, &LinkBucket::trainUs, airtimeUs);
}

DeviceLink* LinkMonitor::deviceWithShort(const uint8_t* dstShort) {
  // A wake-up frame names its device by the first two bytes of its address, reversed.
  DeviceLink* found = nullptr;
  for (DeviceLink& entry : devices_) {
    if (entry.used && entry.prefix[0] == dstShort[1] && entry.prefix[1] == dstShort[0]) {
      if (found != nullptr) {
        return nullptr;  // Two devices share it: no way to tell.
      }
      found = &entry;
    }
  }
  return found;
}

void LinkMonitor::learnBridge(const RxFrame& rx, const DataFrame& data) {
  // Only the bridge sends trains, so a Data frame behind one comes from it. Two trains
  // from the same sender are needed: one forged train must not name the bridge.
  if (hasBridge_ || !wakeSeen_) {
    return;
  }
  if (rx.timestampUs - lastWakeUs_ >= kTrainGapUs || lastWakeSeq_ != data.seq) {
    return;
  }
  if (bridgeVotes_ > 0 && std::memcmp(bridgeCandidate_, data.srcPrefix, kPrefixLen) == 0) {
    if (data.seq != bridgeCandidateSeq_ && ++bridgeVotes_ >= kBridgeVotes) {
      setBridge(data.srcPrefix);
    }
  } else {
    std::memcpy(bridgeCandidate_, data.srcPrefix, kPrefixLen);
    bridgeVotes_ = 1;
  }
  bridgeCandidateSeq_ = data.seq;
}

void LinkMonitor::onData(const RxFrame& rx, const DataFrame& data) {
  if (sentByOtherNode(rx.payload, rx.length, rx.timestampUs)) {
    claimTrain(rx, data.seq, TrainSender::Peer, nullptr, data.dstEui);
    counters_.echoes++;
    return;
  }
  learnBridge(rx, data);
  const bool fromBridge = hasBridge_ && samePrefix(data.srcPrefix, bridge_);
  const uint64_t train =
      claimTrain(rx, data.seq, fromBridge ? TrainSender::Bridge : TrainSender::Unknown, data.srcPrefix, data.dstEui);
  heard(link(data.srcPrefix), rx);
  remember(rx, classify(rx.payload, rx.length), data.srcPrefix, data.dstEui, &data.seq);
  if (!hasBridge_) {
    return;
  }

  const bool uplink = samePrefix(data.dstEui, bridge_);
  if (!uplink && !fromBridge) {
    return;
  }
  const uint8_t* device = uplink ? data.srcPrefix : data.dstEui;
  const Verdict verdict = dedup_.classify(rx.payload, rx.length, rx.timestampUs);
  if (verdict == Verdict::Echo) {
    counters_.echoes++;
    return;
  }

  DeviceLink* entry = link(device);
  if (entry != nullptr) {
    if (verdict == Verdict::Retransmission) {
      entry->retransmissions++;
      count(entry, &LinkBucket::retransmissions);
    }
    if (uplink) {
      entry->uplinks++;
      count(entry, &LinkBucket::uplinks);
    } else {
      entry->downlinks++;
      entry->lastDownlinkUs = rx.timestampUs;
      count(entry, &LinkBucket::downlinks);
    }
  }
  // The proxy answers these itself, and is deaf while it does.
  const bool answeredByProxy = proxyActive_ && (uplink ? proxiesUplink(device) : proxiesDownlink(device));
  if (!answeredByProxy) {
    observe(device, data.seq, uplink, !uplink && train >= kLongTrainUs, rx.timestampUs);
  }
}

bool LinkMonitor::ambiguousAck(const uint8_t* device, bool uplink) const {
  // In a pair the bridge node answers the bridge for a proxied device, and the valve
  // node answers the device for the bridge, if it serves the device's own frames.
  return pairActive_ && (uplink ? proxiesUplink(device) : proxiesDownlink(device));
}

void LinkMonitor::onEnhAck(const RxFrame& rx, const EnhAck& ack) {
  const bool toBridge = hasBridge_ && samePrefix(ack.dstEui, bridge_);
  const bool fromBridge = hasBridge_ && samePrefix(ack.srcEui, bridge_);
  if ((fromBridge && ambiguousAck(ack.dstEui, true)) || (toBridge && ambiguousAck(ack.srcEui, false))) {
    remember(rx, FrameClass::EnhAck, ack.srcEui, ack.dstEui, &ack.seq);
    return;
  }
  counters_.verifiedAcks++;
  heard(link(ack.srcEui), rx);
  remember(rx, FrameClass::EnhAck, ack.srcEui, ack.dstEui, &ack.seq);
  if (fromBridge) {
    rememberHeardAck(ack.dstEui, ack.seq, true);
    acknowledge(ack.dstEui, ack.seq, true, false);
  } else if (toBridge) {
    rememberHeardAck(ack.srcEui, ack.seq, false);
    acknowledge(ack.srcEui, ack.seq, false, false);
  }
}

void LinkMonitor::onBroadcast(const RxFrame& rx, const BroadcastFrame& broadcast) {
  if (sentByOtherNode(rx.payload, rx.length, rx.timestampUs)) {
    claimTrain(rx, broadcast.seq, TrainSender::Peer, nullptr, nullptr);
    counters_.echoes++;
    return;
  }
  const bool fromBridge = hasBridge_ && samePrefix(broadcast.srcEui, bridge_);
  claimTrain(rx, broadcast.seq, fromBridge ? TrainSender::Bridge : TrainSender::Unknown, broadcast.srcEui, nullptr);
  heard(link(broadcast.srcEui), rx);
  remember(rx, FrameClass::Broadcast, broadcast.srcEui, nullptr, &broadcast.seq);
}

void LinkMonitor::observe(const uint8_t* device, uint8_t seq, bool uplink, bool longTrain, uint64_t nowUs) {
  Observed* slot = nullptr;
  for (Observed& entry : observed_) {
    const bool same = entry.used && entry.seq == seq && entry.uplink == uplink && samePrefix(entry.device, device);
    if (same) {
      slot = &entry;
      break;
    }
    if (!entry.used && slot == nullptr) {
      slot = &entry;
    }
  }
  if (slot == nullptr) {
    return;  // More frames in flight than a real channel can carry.
  }
  slot->used = true;
  slot->waitingForPeer = false;
  slot->longTrain = longTrain;
  std::memcpy(slot->device, device, kPrefixLen);
  slot->seq = seq;
  slot->uplink = uplink;
  slot->timestampUs = nowUs;
}

void LinkMonitor::acknowledge(const uint8_t* device, uint8_t seq, bool uplink, bool byPeer) {
  for (Observed& entry : observed_) {
    if (entry.used && entry.seq == seq && entry.uplink == uplink && samePrefix(entry.device, device)) {
      settle(entry, true, byPeer);
      return;
    }
  }
}

void LinkMonitor::settle(Observed& entry, bool acked, bool byPeer) {
  entry.used = false;
  DeviceLink* found = link(entry.device);
  if (found == nullptr) {
    return;
  }
  if (entry.uplink) {
    (acked ? found->uplinksAcked : found->uplinksUnacked)++;
    count(found, acked ? &LinkBucket::uplinksAcked : &LinkBucket::uplinksUnacked);
  } else {
    (acked ? found->downlinksAcked : found->downlinksUnacked)++;
    count(found, acked ? &LinkBucket::downlinksAcked : &LinkBucket::downlinksUnacked);
    if (entry.longTrain) {
      (acked ? found->longTrainAcked : found->longTrainUnacked)++;
      count(found, acked ? &LinkBucket::longTrainAcked : &LinkBucket::longTrainUnacked);
    }
  }
  if (acked && byPeer) {
    found->ackedByPeer++;
  }
}

void LinkMonitor::rememberHeardAck(const uint8_t* device, uint8_t seq, bool uplink) {
  if (!peerLinked_) {
    return;
  }
  HeardAck& slot = heardAcks_[heardAckNext_];
  std::memcpy(slot.device, device, kPrefixLen);
  slot.seq = seq;
  slot.uplink = uplink;
  heardAckNext_ = (heardAckNext_ + 1) % kHeardAckCapacity;
  if (heardAckCount_ < kHeardAckCapacity) {
    heardAckCount_++;  // When full the oldest goes: news of a late ACK is worth less.
  }
}

size_t LinkMonitor::takeHeardAcks(HeardAck* out, size_t capacity) {
  size_t taken = 0;
  while (heardAckCount_ > 0 && taken < capacity) {
    const size_t oldest = (heardAckNext_ + kHeardAckCapacity - heardAckCount_) % kHeardAckCapacity;
    out[taken++] = heardAcks_[oldest];
    heardAckCount_--;
  }
  return taken;
}

void LinkMonitor::notePeerAck(const uint8_t* device, uint8_t seq, bool uplink, uint64_t nowUs) {
  advanceWindows(nowUs);
  peerAcks_.add(ackKey(device, seq, uplink), nowUs + kPeerAckTtlUs);
  acknowledge(device, seq, uplink, true);
}

void LinkMonitor::tick(uint64_t nowUs) {
  nowUs_ = nowUs;
  advanceWindows(nowUs);
  if (train_.open && nowUs - train_.lastUs >= kTrainGapUs) {
    closeTrain();
  }
  for (Observed& entry : observed_) {
    if (!entry.used) {
      continue;
    }
    const uint64_t age = nowUs - entry.timestampUs;
    if (!entry.waitingForPeer && age > kObserveAckUs) {
      if (peerAcks_.contains(ackKey(entry.device, entry.seq, entry.uplink), nowUs)) {
        settle(entry, true, true);
      } else if (peerLinked_) {
        entry.waitingForPeer = true;
      } else {
        settle(entry, false, false);
      }
    } else if (entry.waitingForPeer && age > kPeerAckWaitUs) {
      settle(entry, false, false);
    }
  }
}

void LinkMonitor::resetWindows(uint64_t nowUs) {
  // A pair also leaves proxy mode while the other node is out of reach. A short spell of
  // monitor like that says too little, so it does not replace a baseline.
  if (!proxyActive_ && (!hasBaseline_ || dayCoveredUs(nowUs) >= kMinBaselineUs)) {
    advanceWindows(nowUs);
    for (DeviceLink& entry : devices_) {
      entry.baseline = dayTotals(entry);
    }
    hasBaseline_ = true;
    baselineCoveredUs_ = dayCoveredUs(nowUs);
    baselineTakenUs_ = nowUs;
  }
  hourClock_.restart();
  dayClock_.restart();
  windowsSinceUs_ = nowUs;
  advanceWindows(nowUs);
}

void LinkMonitor::advanceWindows(uint64_t nowUs) {
  hourClock_.advance(nowUs, [this](size_t slot) {
    hourTrains_[slot] = TrainTotals{};
    for (DeviceLink& entry : devices_) {
      entry.hour[slot] = LinkBucket{};
    }
  });
  dayClock_.advance(nowUs, [this](size_t slot) {
    dayTrains_[slot] = TrainTotals{};
    for (DeviceLink& entry : devices_) {
      entry.day[slot] = LinkBucket{};
    }
  });
}

void LinkMonitor::count(DeviceLink* device, uint16_t LinkBucket::* field) {
  if (device != nullptr) {
    saturatingIncrement(device->hour[hourClock_.slot()].*field);
    saturatingIncrement(device->day[dayClock_.slot()].*field);
  }
}

void LinkMonitor::addTime(DeviceLink* device, uint32_t LinkBucket::* field, uint64_t airtimeUs) {
  if (device != nullptr) {
    saturatingAdd(device->hour[hourClock_.slot()].*field, airtimeUs);
    saturatingAdd(device->day[dayClock_.slot()].*field, airtimeUs);
  }
}

uint64_t LinkMonitor::hourCoveredUs(uint64_t nowUs) const {
  return windowCoveredUs(nowUs, windowsSinceUs_, kHourBucketUs, kHourBuckets);
}

uint64_t LinkMonitor::dayCoveredUs(uint64_t nowUs) const {
  return windowCoveredUs(nowUs, windowsSinceUs_, kDayBucketUs, kDayBuckets);
}

bool LinkMonitor::baseline(uint64_t& coveredUs, uint64_t& takenUs) const {
  coveredUs = baselineCoveredUs_;
  takenUs = baselineTakenUs_;
  return hasBaseline_;
}

LinkTotals LinkMonitor::hourTotals(const DeviceLink& device) const {
  LinkTotals total = {};
  for (const LinkBucket& bucket : device.hour) {
    accumulate(total, bucket);
  }
  return total;
}

LinkTotals LinkMonitor::dayTotals(const DeviceLink& device) const {
  LinkTotals total = {};
  for (const LinkBucket& bucket : device.day) {
    accumulate(total, bucket);
  }
  return total;
}

TrainTotals LinkMonitor::hourTrains() const {
  TrainTotals total = {};
  for (const TrainTotals& bucket : hourTrains_) {
    addTrains(total, bucket);
  }
  return total;
}

TrainTotals LinkMonitor::dayTrains() const {
  TrainTotals total = {};
  for (const TrainTotals& bucket : dayTrains_) {
    addTrains(total, bucket);
  }
  return total;
}

DeviceLink* LinkMonitor::link(const uint8_t* prefix) {
  DeviceLink* empty = nullptr;
  DeviceLink* rare = nullptr;
  for (DeviceLink& entry : devices_) {
    if (entry.used) {
      if (samePrefix(entry.prefix, prefix)) {
        return &entry;
      }
      // A device heard once or twice, long ago: often a frame of a neighbour's system,
      // or a corrupted address that passed the checks.
      // A far valve may never be heard itself: traffic with the bridge keeps it.
      const bool forgettable = entry.frames <= kRareFrames && entry.uplinks == 0 && entry.downlinks == 0 &&
                               entry.lastSeenUs < nowUs_ && nowUs_ - entry.lastSeenUs > kRareAgeUs &&
                               !(hasBridge_ && samePrefix(entry.prefix, bridge_)) && !isProxied(entry.prefix);
      if (forgettable && rare == nullptr) {
        rare = &entry;
      }
    } else if (empty == nullptr) {
      empty = &entry;
    }
  }
  if (empty == nullptr && rare != nullptr) {
    *rare = DeviceLink();  // All counts at zero, the windows too.
    empty = rare;
  }
  if (empty == nullptr) {
    counters_.devicesNotTracked++;
    return nullptr;
  }
  empty->used = true;
  std::memcpy(empty->prefix, prefix, kPrefixLen);
  return empty;
}

void LinkMonitor::heard(DeviceLink* device, const RxFrame& rx) {
  if (device == nullptr) {
    return;
  }
  const bool first = device->frames == 0;
  device->frames++;
  device->lastSeenUs = rx.timestampUs;
  device->rssiDbm = first ? rx.rssiDbm : device->rssiDbm + kSmoothing * (rx.rssiDbm - device->rssiDbm);
  const float lqi = static_cast<float>(rx.lqi);
  device->lqi = first ? lqi : device->lqi + kSmoothing * (lqi - device->lqi);
  addTime(device, &LinkBucket::airtimeUs, airtimeUs(rx.length));
}

void LinkMonitor::remember(const RxFrame& rx, FrameClass frameClass, const uint8_t* src, const uint8_t* dst,
                           const uint8_t* seq) {
  RecentFrame& entry = recent_[recentNext_];
  recentNext_ = (recentNext_ + 1) % kRecentFrames;
  if (recentCount_ < kRecentFrames) {
    recentCount_++;
  }
  entry.timestampUs = rx.timestampUs;
  entry.frameClass = frameClass;
  entry.length = static_cast<uint8_t>(rx.length);
  entry.rssiDbm = rx.rssiDbm;
  entry.lqi = rx.lqi;
  entry.hasAddresses = src != nullptr;
  entry.hasDestination = dst != nullptr;
  if (src != nullptr) {
    std::memcpy(entry.src, src, kPrefixLen);
  }
  if (dst != nullptr) {
    std::memcpy(entry.dst, dst, kPrefixLen);
  }
  entry.hasSeq = seq != nullptr;
  entry.seq = seq != nullptr ? *seq : 0;
}

}  // namespace tadogado

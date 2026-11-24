#include "proxy_engine.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tadogado {

namespace {

constexpr uint64_t kAckDelayUs = 5000;
constexpr uint64_t kAckLateLimitUs = 12000;
constexpr uint64_t kAckWaitUs = 30000;
constexpr uint64_t kRetryGapUs = 2000000;
constexpr uint8_t kMaxAttempts = 3;
constexpr uint64_t kAdvertisementIntervalUs = 600000000;
constexpr uint64_t kSolicitWindowUs = 5000000;
constexpr uint64_t kWakeQuietUs = 30000;  // A train this recent: the bridge is on air.
constexpr uint64_t kAckOnAirMarginUs = 3000;
constexpr uint64_t kAnchoredTrainSpanUs = 80000;
// A Hello comes every 2 s. Older news of the other node's capacity is not trusted.
constexpr uint64_t kPeerCapacityFreshUs = 5000000;
// An ACK this close to the device's usual level is taken as the device's own: the other
// node, in another room, is heard far weaker. It only feeds a count, never a decision.
constexpr float kOwnAnswerMarginDb = 6.0f;
// How far apart the device and the bridge node must be heard for an answer within the
// margin of the device to be its own.
constexpr float kOwnAnswerSeparationDb = 2.0f * kOwnAnswerMarginDb;
constexpr float kPeerAnswerSmoothing = 0.2f;
constexpr uint64_t kAnsweredDirectlyUs = 2000000;
const uint8_t kBroadcastShort[2] = {0xFF, 0xFF};

bool samePrefix(const uint8_t* a, const uint8_t* b) { return std::memcmp(a, b, kPrefixLen) == 0; }

// Airtime of a train over `spanUs`: a frame every step, and the last one with
// rendezvous 0, as sendTrain sends them.
uint64_t trainUs(uint64_t spanUs) { return (spanUs / kWakeStepUs + 1) * airtimeUs(kWakeupLen); }

uint64_t answerKey(const uint8_t* device, uint8_t seq) {
  uint8_t key[kPrefixLen + 1];
  std::memcpy(key, device, kPrefixLen);
  key[kPrefixLen] = seq;
  return hashBytes(key, sizeof(key));
}

}  // namespace

void ProxyEngine::setProxyMode(bool enabled) {
  if (enabled != proxyMode_) {
    monitor_.resetWindows(radio_.nowUs());
  }
  proxyMode_ = enabled;
  monitor_.setProxyActive(enabled, role_ != Role::Single);
  if (!enabled) {
    ackCount_ = 0;
    awaiting_ = nullptr;
    for (Job& job : jobs_) {
      job.used = false;
    }
  }
}

// ---------------------------------------------------------------- Receive path

void ProxyEngine::onFrame(const RxFrame& rx) {
  monitor_.onFrame(rx);
  if (!rx.crcOk) {
    return;
  }
  switch (classify(rx.payload, rx.length)) {
    case FrameClass::Wakeup: {
      WakeUp wake;
      if (parseWakeup(rx.payload, rx.length, wake)) {
        onWakeup(rx, wake);
      }
      return;
    }
    case FrameClass::Data:
    case FrameClass::DataPending: {
      DataFrame data;
      if (parseData(rx.payload, rx.length, data)) {
        onData(rx, data);
      }
      return;
    }
    case FrameClass::EnhAck: {
      EnhAck ack;
      if (parseEnhAck(rx.payload, rx.length, ack)) {
        onEnhAck(rx, ack);
      }
      return;
    }
    case FrameClass::Broadcast: {
      BroadcastFrame broadcast;
      if (parseBroadcast(rx.payload, rx.length, broadcast)) {
        onBroadcast(rx, broadcast);
      }
      return;
    }
    default:
      return;
  }
}

void ProxyEngine::onWakeup(const RxFrame& rx, const WakeUp&) {
  wakeSeen_ = true;
  lastWakeUs_ = rx.timestampUs;
}

void ProxyEngine::handOver(bool uplink, const RxFrame& rx) {
  if (peer_ != nullptr) {
    monitor_.noteHandedOver(rx.payload, rx.length, rx.timestampUs);
    peer_->sendFrame(uplink, rx.payload, rx.length);
  }
}

void ProxyEngine::onData(const RxFrame& rx, const DataFrame& data) {
  // An ACK to the other node would tell it that the frame arrived.
  if (!monitor_.hasBridge() || monitor_.sentByOtherNode(rx.payload, rx.length, rx.timestampUs)) {
    return;
  }
  const uint8_t* bridge = monitor_.bridge();
  const bool uplink = samePrefix(data.dstEui, bridge);
  if (!uplink && !samePrefix(data.srcPrefix, bridge)) {
    return;
  }
  const uint8_t* device = uplink ? data.srcPrefix : data.dstEui;
  const Verdict verdict = dedup_.classify(rx.payload, rx.length, rx.timestampUs);
  if (verdict == Verdict::Echo) {
    return;
  }
  const bool proxied = uplink ? monitor_.proxiesUplink(device) : monitor_.proxiesDownlink(device);
  if (proxyMode_ && proxied && serves(uplink)) {
    proxyData(rx, data, device, uplink, verdict);
  }
}

bool ProxyEngine::serves(bool uplink) const {
  return role_ == Role::Single || role_ == (uplink ? Role::Valve : Role::Bridge);
}

void ProxyEngine::onPeerFrame(bool uplink, const uint8_t* frame, size_t length, uint64_t nowUs) {
  dedup_.noteSent(frame, length, nowUs);
  if (!proxyMode_ || length > kMaxFrameLen) {
    return;
  }
  if (classify(frame, length) == FrameClass::Broadcast) {
    BroadcastFrame broadcast;
    if (parseBroadcast(frame, length, broadcast) &&
        addJob(frame, length, broadcast.seq, uplink ? broadcast.srcEui : nullptr, uplink, false, nowUs) == nullptr) {
      counters_.peerFramesDropped++;
    }
    return;
  }
  DataFrame data;
  if (parseData(frame, length, data)) {
    const uint8_t* device = uplink ? data.srcPrefix : data.dstEui;
    if (hasJob(device, data.seq, uplink)) {
      return;
    }
    Job* job = addJob(frame, length, data.seq, device, uplink, true, nowUs);
    if (job == nullptr) {
      // The other node ACKed a frame this node cannot take. Capacity in the Hellos
      // makes this rare; it shows on the dashboard when it happens.
      counters_.peerFramesDropped++;
    } else if (!uplink && answeredDirectly_.contains(answerKey(device, data.seq), nowUs)) {
      countAlreadyAnswered(*job);
    }
  }
}

void ProxyEngine::proxyData(const RxFrame& rx, const DataFrame& data, const uint8_t* device, bool uplink,
                            Verdict verdict) {
  uint8_t keyBytes[kPrefixLen + 2];
  std::memcpy(keyBytes, device, kPrefixLen);
  keyBytes[kPrefixLen] = data.seq;
  keyBytes[kPrefixLen + 1] = uplink ? 1 : 0;
  const uint64_t key = hashBytes(keyBytes, sizeof(keyBytes));

  // A retry of a frame we turned down is still ours to accept or refuse.
  const bool alreadyAccepted = verdict == Verdict::Retransmission && accepted_.contains(key, rx.timestampUs);
  if (!alreadyAccepted) {
    if (!canAccept(device, rx.length, uplink, rx.timestampUs)) {
      counters_.acksWithheld++;
      return;
    }
    accepted_.add(key, rx.timestampUs + 10000000);
    if (role_ != Role::Single) {
      // Until the next Hello, count what this frame takes from the other node.
      const uint64_t cost = peerCostUs(rx.length, uplink);
      peerCapacity_.budgetUs = peerCapacity_.budgetUs > cost ? peerCapacity_.budgetUs - cost : 0;
      peerCapacity_.freeJobs = peerCapacity_.freeJobs > 0 ? peerCapacity_.freeJobs - 1 : 0;
    }
  }

  uint8_t srcEui[kEuiLen];
  std::memcpy(srcEui, data.srcPrefix, kPrefixLen);
  std::memcpy(srcEui + kPrefixLen, data.dstEui + kPrefixLen, kEuiLen - kPrefixLen);

  const uint64_t due = rx.timestampUs + kAckDelayUs;
  const uint16_t phase = uplink ? bridgePhase(due) : devicePhase(device, due);
  if (ackCount_ < kMaxPendingAcks) {
    PendingAck& ack = acks_[(ackHead_ + ackCount_) % kMaxPendingAcks];
    ack.dueUs = due;
    ack.receivedUs = rx.timestampUs;
    std::memcpy(ack.device, device, kPrefixLen);
    buildEnhAck(data.seq, srcEui, data.dstEui, phase, ack.frame);
    ackCount_++;
  }

  if (alreadyAccepted) {
    return;
  }
  DeviceLink* link = monitor_.mutableLink(device);
  if (link != nullptr) {
    if (uplink) {
      link->proxiedUplinks++;
    } else {
      link->proxiedDownlinks++;
    }
  }
  if (role_ != Role::Single) {
    handOver(uplink, rx);
  } else if (!hasJob(device, data.seq, uplink)) {
    // Our ACK goes first on an uplink, and occupies the radio for this long.
    const uint64_t ready =
        uplink ? rx.timestampUs + kAckDelayUs + airtimeUs(kEnhAckLen) + kAckOnAirMarginUs : rx.timestampUs;
    addJob(rx.payload, rx.length, data.seq, device, uplink, true, ready);
  }
}

void ProxyEngine::onEnhAck(const RxFrame& rx, const EnhAck& ack) {
  if (!monitor_.hasBridge()) {
    return;
  }
  const uint8_t* bridge = monitor_.bridge();
  if (samePrefix(ack.srcEui, bridge)) {
    bridgePhaseKnown_ = true;
    bridgePhaseValue_ = ack.phase;
    bridgePhaseSeenUs_ = rx.timestampUs;
    acknowledge(ack.dstEui, ack.seq, true);
  } else if (samePrefix(ack.dstEui, bridge)) {
    // The other node of a pair may be heard answering for this device, with a phase
    // that says nothing about its samples. Only trust the answer to our own frame.
    const Job* job = awaiting_;
    const bool ownAnswer =
        job != nullptr && job->hasDevice && job->seq == ack.seq && samePrefix(job->device, ack.srcEui);
    if (role_ == Role::Single || ownAnswer) {
      anchors_.update(ack.srcEui, rx.timestampUs, ack.phase);
    }
    if (!ownAnswer) {
      noteAnsweredDirectly(rx, ack);
    }
    acknowledge(ack.srcEui, ack.seq, false);
  }
}

void ProxyEngine::acknowledge(const uint8_t* device, uint8_t seq, bool uplink) {
  Job* job = awaiting_;
  if (job != nullptr && job->uplink == uplink && job->hasDevice && job->seq == seq && samePrefix(job->device, device)) {
    awaiting_ = nullptr;
    delivered(*job);
    job->used = false;
    return;
  }
  if (uplink && role_ == Role::Single) {
    // The bridge heard the device directly: no need to forward. On a pair this ACK may
    // be the one the other node sent, which proves nothing of the bridge.
    for (Job& queued : jobs_) {
      if (queued.used && queued.uplink && queued.hasDevice && queued.seq == seq && samePrefix(queued.device, device) &&
          &queued != awaiting_) {
        queued.used = false;
      }
    }
  }
}

void ProxyEngine::noteAnsweredDirectly(const RxFrame& rx, const EnhAck& ack) {
  // Only the valve node hears this. The bridge node answers the bridge for the device
  // too, with a frame that looks the same: only the strength tells the two apart.
  if (!proxyMode_ || role_ != Role::Valve || !monitor_.proxiesDownlink(ack.srcEui)) {
    return;
  }
  const DeviceLink* link = monitor_.findLink(ack.srcEui);
  if (link == nullptr || link->frames == 0) {
    return;
  }
  if (std::fabs(rx.rssiDbm - link->rssiDbm) > kOwnAnswerMarginDb) {
    // Far from the device's own strength: the bridge node, as this node hears it.
    peerAnswerRssiDbm_ =
        peerAnswerKnown_ ? peerAnswerRssiDbm_ + kPeerAnswerSmoothing * (rx.rssiDbm - peerAnswerRssiDbm_) : rx.rssiDbm;
    peerAnswerKnown_ = true;
    return;
  }
  if (!canTellOwnAnswer(*link)) {
    return;
  }
  answeredDirectly_.add(answerKey(ack.srcEui, ack.seq), rx.timestampUs + kAnsweredDirectlyUs);
  for (Job& job : jobs_) {
    if (job.used && !job.uplink && job.hasDevice && job.seq == ack.seq && samePrefix(job.device, ack.srcEui)) {
      countAlreadyAnswered(job);
    }
  }
}

bool ProxyEngine::canTellOwnAnswer(const DeviceLink& link) const {
  return peerAnswerKnown_ && link.frames > 0 && std::fabs(link.rssiDbm - peerAnswerRssiDbm_) >= kOwnAnswerSeparationDb;
}

bool ProxyEngine::canTellOwnAnswer(const uint8_t* device) const {
  const DeviceLink* link = monitor_.findLink(device);
  return link != nullptr && canTellOwnAnswer(*link);
}

bool ProxyEngine::peerAnswerRssi(float& out) const {
  out = peerAnswerRssiDbm_;
  return peerAnswerKnown_;
}

void ProxyEngine::countAlreadyAnswered(Job& job) {
  if (job.alreadyAnswered) {
    return;
  }
  job.alreadyAnswered = true;
  DeviceLink* link = monitor_.mutableLink(job.device);
  if (link != nullptr) {
    link->alreadyAnswered++;
  }
}

void ProxyEngine::onBroadcast(const RxFrame& rx, const BroadcastFrame& broadcast) {
  if (!proxyMode_ || monitor_.proxiedCount() == 0) {
    return;
  }
  if (monitor_.sentByOtherNode(rx.payload, rx.length, rx.timestampUs) ||
      dedup_.classify(rx.payload, rx.length, rx.timestampUs) == Verdict::Echo) {
    return;
  }
  if (monitor_.hasBridge() && samePrefix(broadcast.srcEui, monitor_.bridge())) {
    if (!serves(false) || !monitor_.proxiesAnyDownlink()) {
      return;  // Devices served only to the bridge hear the bridge themselves.
    }
    // A Router Advertisement. Without one a device stops reporting after 20 minutes.
    const bool recent = advertisementSeen_ && rx.timestampUs - lastAdvertisementUs_ < kAdvertisementIntervalUs;
    const bool solicited = solicitSeen_ && rx.timestampUs - lastSolicitUs_ < kSolicitWindowUs;
    if (recent && !solicited) {
      return;
    }
    advertisementSeen_ = true;
    lastAdvertisementUs_ = rx.timestampUs;
    if (role_ == Role::Single) {
      addJob(rx.payload, rx.length, broadcast.seq, nullptr, false, false, rx.timestampUs);
    } else {
      handOver(false, rx);
    }
  } else if (monitor_.proxiesUplink(broadcast.srcEui) && serves(true)) {
    solicitSeen_ = true;
    lastSolicitUs_ = rx.timestampUs;
    if (role_ == Role::Single) {
      addJob(rx.payload, rx.length, broadcast.seq, broadcast.srcEui, true, false, rx.timestampUs);
    } else {
      handOver(true, rx);
    }
  }
}

// ------------------------------------------------------------------- Admission

bool ProxyEngine::canAccept(const uint8_t* device, size_t frameLength, bool uplink, uint64_t nowUs) const {
  if (ackCount_ >= kMaxPendingAcks) {
    return false;
  }
  const uint64_t committed = committedUs();
  if (role_ != Role::Single) {
    // The other node pays for the second hop, from its own budget.
    return duty_.allows(committed + airtimeUs(kEnhAckLen), nowUs) && peerCanTake(frameLength, uplink, nowUs);
  }
  if (countJobsFor(device, uplink) >= kMaxPerDevice) {
    return false;
  }
  bool freeSlot = false;
  for (const Job& job : jobs_) {
    freeSlot = freeSlot || !job.used;
  }
  if (!freeSlot) {
    return false;
  }
  uint64_t needed = airtimeUs(kEnhAckLen) + airtimeUs(frameLength);
  if (!uplink) {
    needed += trainAirtimeUs(device);
  }
  return duty_.allows(committed + needed, nowUs);
}

uint64_t ProxyEngine::committedUs() const {
  uint64_t total = ackCount_ * static_cast<uint64_t>(airtimeUs(kEnhAckLen));
  for (const Job& job : jobs_) {
    if (!job.used || &job == awaiting_) {
      continue;  // A job awaiting its ACK has been sent; a retry is not promised.
    }
    total += airtimeUs(job.length);
    if (!job.uplink) {
      total += job.hasDevice ? trainAirtimeUs(job.device) : trainUs(kCslPeriodUs);
    }
  }
  return total;
}

uint64_t ProxyEngine::peerCostUs(size_t frameLength, bool uplink) {
  // Without the other node's anchors, assume a full train.
  return airtimeUs(frameLength) + (uplink ? 0 : trainUs(kCslPeriodUs));
}

bool ProxyEngine::peerCanTake(size_t frameLength, bool uplink, uint64_t nowUs) const {
  return peerCapacityKnown_ && nowUs - peerCapacityUs_ < kPeerCapacityFreshUs && peerCapacity_.freeJobs > 0 &&
         peerCapacity_.budgetUs >= peerCostUs(frameLength, uplink);
}

bool ProxyEngine::idle() const {
  if (ackCount_ > 0 || awaiting_ != nullptr) {
    return false;
  }
  for (const Job& job : jobs_) {
    if (job.used) {
      return false;
    }
  }
  return true;
}

Capacity ProxyEngine::capacity(uint64_t nowUs) const {
  Capacity out = {};
  const uint64_t used = duty_.usedUs(nowUs) + committedUs();
  out.budgetUs = used < DutyCycleLimiter::kBudgetUs ? DutyCycleLimiter::kBudgetUs - used : 0;
  for (const Job& job : jobs_) {
    out.freeJobs += job.used ? 0 : 1;
  }
  return out;
}

void ProxyEngine::setPeerCapacity(const Capacity& capacity, uint64_t nowUs) {
  peerCapacityKnown_ = true;
  peerCapacity_ = capacity;
  peerCapacityUs_ = nowUs;
}

uint64_t ProxyEngine::trainAirtimeUs(const uint8_t* device) const {
  return trainUs(anchors_.contains(device) ? kAnchoredTrainSpanUs : kCslPeriodUs);
}

ProxyEngine::Job* ProxyEngine::addJob(const uint8_t* frame, size_t length, uint8_t seq, const uint8_t* device,
                                      bool uplink, bool needsAck, uint64_t nextTryUs) {
  for (Job& job : jobs_) {
    if (!job.used) {
      job = Job{};
      job.used = true;
      job.uplink = uplink;
      job.needsAck = needsAck;
      job.seq = seq;
      job.length = static_cast<uint8_t>(length);
      std::memcpy(job.frame, frame, length);
      job.order = nextOrder_++;
      job.nextTryUs = nextTryUs;
      if (device != nullptr) {
        job.hasDevice = true;
        std::memcpy(job.device, device, kPrefixLen);
      }
      return &job;
    }
  }
  return nullptr;
}

bool ProxyEngine::hasJob(const uint8_t* device, uint8_t seq, bool uplink) const {
  for (const Job& job : jobs_) {
    if (job.used && job.uplink == uplink && job.hasDevice && job.seq == seq && samePrefix(job.device, device)) {
      return true;
    }
  }
  return false;
}

size_t ProxyEngine::countJobs(bool uplink) const {
  size_t count = 0;
  for (const Job& job : jobs_) {
    count += job.used && job.uplink == uplink && &job != awaiting_ ? 1 : 0;
  }
  return count;
}

size_t ProxyEngine::countJobsFor(const uint8_t* device, bool uplink) const {
  size_t count = 0;
  for (const Job& job : jobs_) {
    count += job.used && job.uplink == uplink && job.hasDevice && samePrefix(job.device, device) ? 1 : 0;
  }
  return count;
}

// --------------------------------------------------------------- Transmit path

// The only place that keys the transmitter.
bool ProxyEngine::tx(const uint8_t* frame, size_t length, const uint8_t* device) {
  if (!proxyMode_) {
    return false;
  }
  const uint64_t air = airtimeUs(length);
  const uint64_t now = radio_.nowUs();
  if (!duty_.allows(air, now)) {
    counters_.dutyBlocked++;
    return false;
  }
  const bool ok = radio_.send(frame, length);
  duty_.record(air, now);
  DeviceLink* link = device != nullptr ? monitor_.mutableLink(device) : nullptr;
  if (link != nullptr) {
    link->airtime.record(air, now);
  }
  counters_.framesTx++;
  if (!ok) {
    counters_.txErrors++;
  }
  return ok;
}

bool ProxyEngine::tick() {
  const uint64_t now = radio_.nowUs();
  monitor_.tick(now);
  if (!proxyMode_) {
    return false;
  }
  if (ackCount_ > 0) {
    serviceAcks(now);
    return true;
  }
  if (awaiting_ != nullptr) {
    checkAwaiting(now);
    return true;
  }
  return startNextJob(now);
}

void ProxyEngine::serviceAcks(uint64_t nowUs) {
  const PendingAck& ack = acks_[ackHead_];
  if (nowUs < ack.dueUs) {
    return;
  }
  const uint64_t latency = nowUs - ack.receivedUs;
  const bool late = latency > kAckLateLimitUs;
  uint8_t frame[kEnhAckLen];
  uint8_t device[kPrefixLen];
  std::memcpy(frame, ack.frame, kEnhAckLen);
  std::memcpy(device, ack.device, kPrefixLen);
  ackHead_ = (ackHead_ + 1) % kMaxPendingAcks;
  ackCount_--;
  if (late) {
    counters_.acksLate++;
    return;
  }
  if (tx(frame, kEnhAckLen, device)) {
    counters_.acksSent++;
    latencyUs_[latencyNext_] = static_cast<uint16_t>(latency);
    latencyNext_ = (latencyNext_ + 1) % kLatencySamples;
    latencyCount_ = std::min(latencyCount_ + 1, kLatencySamples);
  }
}

void ProxyEngine::checkAwaiting(uint64_t nowUs) {
  if (nowUs < awaitDeadlineUs_) {
    return;
  }
  Job& job = *awaiting_;
  awaiting_ = nullptr;
  if (!job.uplink) {
    anchors_.miss(job.device);
  }
  if (job.attempts >= kMaxAttempts) {
    counters_.deliveriesFailed++;
    DeviceLink* link = monitor_.mutableLink(job.device);
    if (link != nullptr) {
      link->deliveryFailures++;
    }
    job.used = false;
    return;
  }
  job.nextTryUs = nowUs + kRetryGapUs;
  job.hasSlot = false;
  job.order = nextOrder_++;
}

void ProxyEngine::delivered(Job& job) {
  if (job.uplink) {
    counters_.uplinksForwarded++;
  } else {
    counters_.downlinksDelivered++;
  }
}

ProxyEngine::Job* ProxyEngine::nextJob(bool uplink, uint64_t nowUs) {
  Job* best = nullptr;
  for (Job& job : jobs_) {
    const bool due = job.used && job.uplink == uplink && job.nextTryUs <= nowUs;
    if (due && (best == nullptr || job.order < best->order)) {
      best = &job;
    }
  }
  return best;
}

bool ProxyEngine::startNextJob(uint64_t nowUs) {
  Job* job = nextJob(true, nowUs);
  if (job != nullptr) {
    sendUplink(*job);
    return true;
  }
  job = nextJob(false, nowUs);
  if (job != nullptr && readyForTrain(*job, nowUs)) {
    sendDownlink(*job);
    return true;
  }
  return false;
}

void ProxyEngine::sendUplink(Job& job) {
  // The bridge receiver is always on, so a frame needs no train.
  job.attempts++;
  dedup_.noteSent(job.frame, job.length, radio_.nowUs());
  const bool sent = tx(job.frame, job.length, job.hasDevice ? job.device : nullptr);
  if (!job.needsAck) {
    if (sent) {
      delivered(job);
    }
    job.used = false;
    return;
  }
  await(job);
}

bool ProxyEngine::readyForTrain(Job& job, uint64_t nowUs) {
  if (wakeSeen_ && nowUs - lastWakeUs_ < kWakeQuietUs) {
    job.hasSlot = false;  // The bridge is on air: pick the next sample.
    return false;
  }
  if (job.hasSlot && nowUs > job.slotStartUs + kWakeStepUs) {
    job.hasSlot = false;  // Another job held the radio past this slot.
  }
  if (!job.hasSlot) {
    Slot slot;
    if (job.hasDevice && anchors_.nextSlot(job.device, nowUs, slot)) {
      job.slotStartUs = slot.startUs;
      job.slotEndUs = slot.endUs;
    } else {
      job.slotStartUs = nowUs;
      job.slotEndUs = nowUs + kCslPeriodUs;
    }
    job.hasSlot = true;
  }
  return nowUs >= job.slotStartUs;
}

void ProxyEngine::sendDownlink(Job& job) {
  job.attempts++;
  counters_.trains++;
  bool sent = sendTrain(job);
  if (sent) {
    dedup_.noteSent(job.frame, job.length, radio_.nowUs());
    sent = tx(job.frame, job.length, job.hasDevice ? job.device : nullptr);
  }
  if (!job.needsAck) {
    if (sent) {
      counters_.advertisementsForwarded++;
    }
    job.used = false;
    return;
  }
  await(job);
}

bool ProxyEngine::sendTrain(Job& job) {
  // The radio cannot receive during a train. The bridge pays the same for its own.
  uint8_t shortAddress[2] = {kBroadcastShort[0], kBroadcastShort[1]};
  if (job.hasDevice) {
    shortAddress[0] = job.frame[4];
    shortAddress[1] = job.frame[3];
  }
  const uint8_t* device = job.hasDevice ? job.device : nullptr;
  uint8_t wake[kWakeupLen];
  const uint64_t start = radio_.nowUs();
  uint64_t index = 0;
  for (;;) {
    const uint64_t now = radio_.nowUs();
    if (now + kWakeStepUs > job.slotEndUs) {
      break;
    }
    const uint64_t units = (job.slotEndUs - now) / kCslUnitUs;
    buildWakeup(job.seq, shortAddress, static_cast<uint16_t>(std::min<uint64_t>(units, kCslPeriodUnits)), wake);
    if (!tx(wake, kWakeupLen, device)) {
      return false;
    }
    index++;
    radio_.sleepUntil(start + index * kWakeStepUs);
  }
  buildWakeup(job.seq, shortAddress, 0, wake);
  return tx(wake, kWakeupLen, device);
}

void ProxyEngine::await(Job& job) {
  awaiting_ = &job;
  awaitDeadlineUs_ = radio_.nowUs() + kAckWaitUs;
}

// --------------------------------------------------------------------- Helpers

uint16_t ProxyEngine::bridgePhase(uint64_t nowUs) const {
  // The bridge phase is a free-running counter of 320 us units.
  if (!bridgePhaseKnown_) {
    return static_cast<uint16_t>(nowUs / kCslUnitUs);
  }
  return static_cast<uint16_t>(bridgePhaseValue_ + (nowUs - bridgePhaseSeenUs_) / kCslUnitUs);
}

uint16_t ProxyEngine::devicePhase(const uint8_t* device, uint64_t nowUs) const {
  uint16_t phase = 0;
  if (anchors_.phase(device, nowUs, phase)) {
    return phase;
  }
  return static_cast<uint16_t>((nowUs / kCslUnitUs) % kCslPeriodUnits);
}

bool ProxyEngine::ackLatency(AckLatency& out) const {
  if (latencyCount_ == 0) {
    return false;
  }
  uint16_t sorted[kLatencySamples];
  std::copy(latencyUs_, latencyUs_ + latencyCount_, sorted);
  std::sort(sorted, sorted + latencyCount_);
  out.samples = latencyCount_;
  out.p50Ms = sorted[latencyCount_ / 2] / 1000.0f;
  out.p95Ms = sorted[latencyCount_ * 95 / 100] / 1000.0f;
  out.maxMs = sorted[latencyCount_ - 1] / 1000.0f;
  return true;
}

}  // namespace tadogado

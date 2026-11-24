// CSL proxy: carries frames between the bridge and devices it cannot reach.
//
// docs/how-it-works.md explains why a plain repeater cannot work. test/test_sim runs
// this code in a simulated Tado network.
//
// One node can serve both hops. In a pair the bridge node serves the bridge and the
// valve node the far valves, and the frames travel between them over the network.
//
// The engine starts in monitor mode, where it never calls RadioPort::send. A proxied
// ACK is a promise, so it gives one only for devices it was told to serve, and only
// when it has the queue space and the duty-cycle budget to deliver.
#pragma once

#include <cstddef>
#include <cstdint>

#include "anchors.h"
#include "dedup.h"
#include "duty_cycle.h"
#include "frames.h"
#include "link_monitor.h"
#include "rx_assembler.h"

namespace tadogado {

class RadioPort {
 public:
  virtual ~RadioPort() = default;
  // Transmit one frame and return to RX. Blocks while it is on air.
  virtual bool send(const uint8_t* frame, size_t length) = 0;
  virtual void sleepUntil(uint64_t timeUs) = 0;
  virtual uint64_t nowUs() = 0;
};

enum class Role : uint8_t {
  Single,  // One node serves both hops.
  Bridge,  // Near the Tado bridge, one half of a pair.
  Valve,   // Near the far valves, the other half.
};

// Hands a frame to the other node of a pair.
class PeerSink {
 public:
  virtual ~PeerSink() = default;
  virtual void sendFrame(bool uplink, const uint8_t* frame, size_t length) = 0;
};

struct ProxyCounters {
  uint32_t framesTx = 0;
  uint32_t acksSent = 0;
  uint32_t acksLate = 0;
  uint32_t acksWithheld = 0;
  uint32_t uplinksForwarded = 0;
  uint32_t downlinksDelivered = 0;
  uint32_t deliveriesFailed = 0;
  uint32_t trains = 0;
  uint32_t advertisementsForwarded = 0;
  uint32_t dutyBlocked = 0;
  uint32_t txErrors = 0;
  uint32_t peerFramesDropped = 0;
};

// What a node can still deliver: its duty-cycle budget left after the work it already
// promised, and its free job slots. The nodes of a pair tell each other, so neither
// ACKs a frame the other cannot deliver.
struct Capacity {
  uint64_t budgetUs;
  uint8_t freeJobs;
};

struct AckLatency {
  size_t samples;
  float p50Ms;
  float p95Ms;
  float maxMs;
};

class ProxyEngine {
 public:
  static constexpr size_t kMaxJobs = 32;
  static_assert(kMaxJobs >= 2 * kMaxProxied, "every device needs a job in each direction");
  static constexpr size_t kMaxPerDevice = 8;
  static constexpr size_t kMaxPendingAcks = 4;
  static constexpr size_t kLatencySamples = 128;

  ProxyEngine(RadioPort& radio, LinkMonitor& monitor) : radio_(radio), monitor_(monitor) {}

  // Leaving proxy mode drops all queued work. Call it from the same task as tick().
  void setProxyMode(bool enabled);
  bool proxyMode() const { return proxyMode_; }

  void setRole(Role role) { role_ = role; }
  Role role() const { return role_; }
  void setPeer(PeerSink* peer) { peer_ = peer; }

  void onFrame(const RxFrame& rx);
  // A frame from the other node of a pair, to deliver over this node's hop.
  void onPeerFrame(bool uplink, const uint8_t* frame, size_t length, uint64_t nowUs);
  // One pass of transmit work. Returns true if there was any. It may block for the
  // length of a wake-up train.
  bool tick();

  // True when nothing is promised or under way: no ACK due, no job queued or awaited.
  bool idle() const;
  Capacity capacity(uint64_t nowUs) const;
  // The capacity of the other node of a pair, from its latest Hello.
  void setPeerCapacity(const Capacity& capacity, uint64_t nowUs);

  const ProxyCounters& counters() const { return counters_; }
  const DutyCycleLimiter& duty() const { return duty_; }
  DutyCycleLimiter& duty() { return duty_; }
  const AnchorTable& anchors() const { return anchors_; }
  size_t uplinksQueued() const { return countJobs(true); }
  size_t downlinksQueued() const { return countJobs(false); }
  bool awaitingAck() const { return awaiting_ != nullptr; }
  bool ackLatency(AckLatency& out) const;
  // The valve node tells a device's own answer to the bridge from the bridge node's
  // answer for it by strength alone. False while that cannot be done for this device:
  // the bridge node's strength here is still unknown, or too close to the device's.
  bool canTellOwnAnswer(const uint8_t* device) const;
  // The strength at which this node hears the other node answer the bridge.
  bool peerAnswerRssi(float& out) const;

 private:
  struct Job {
    bool used;
    bool uplink;
    bool needsAck;
    bool hasDevice;
    bool hasSlot;
    bool alreadyAnswered;  // The device answered the bridge itself; counted once.
    uint8_t device[kPrefixLen];
    uint8_t seq;
    uint8_t attempts;
    uint8_t length;
    uint8_t frame[kMaxFrameLen];
    uint32_t order;
    uint64_t nextTryUs;
    uint64_t slotStartUs;
    uint64_t slotEndUs;
  };

  struct PendingAck {
    uint64_t dueUs;
    uint64_t receivedUs;
    uint8_t device[kPrefixLen];
    uint8_t frame[kEnhAckLen];
  };

  void onWakeup(const RxFrame& rx, const WakeUp& wake);
  void onData(const RxFrame& rx, const DataFrame& data);
  void proxyData(const RxFrame& rx, const DataFrame& data, const uint8_t* device, bool uplink, Verdict verdict);
  void onEnhAck(const RxFrame& rx, const EnhAck& ack);
  void acknowledge(const uint8_t* device, uint8_t seq, bool uplink);
  // The valve node counts frames to deliver that the device already had from the bridge.
  void noteAnsweredDirectly(const RxFrame& rx, const EnhAck& ack);
  void countAlreadyAnswered(Job& job);
  bool canTellOwnAnswer(const DeviceLink& link) const;
  void onBroadcast(const RxFrame& rx, const BroadcastFrame& broadcast);
  bool serves(bool uplink) const;

  bool canAccept(const uint8_t* device, size_t frameLength, bool uplink, uint64_t nowUs) const;
  uint64_t trainAirtimeUs(const uint8_t* device) const;
  // Airtime of the work accepted but not yet sent.
  uint64_t committedUs() const;
  // Airtime the other node of a pair spends to deliver a frame over its hop.
  static uint64_t peerCostUs(size_t frameLength, bool uplink);
  bool peerCanTake(size_t frameLength, bool uplink, uint64_t nowUs) const;
  Job* addJob(const uint8_t* frame, size_t length, uint8_t seq, const uint8_t* device, bool uplink, bool needsAck,
              uint64_t nextTryUs);
  bool hasJob(const uint8_t* device, uint8_t seq, bool uplink) const;
  size_t countJobs(bool uplink) const;
  size_t countJobsFor(const uint8_t* device, bool uplink) const;

  // `device` is the one the frame is for, or nullptr if it is for all of them.
  bool tx(const uint8_t* frame, size_t length, const uint8_t* device);
  void handOver(bool uplink, const RxFrame& rx);
  void serviceAcks(uint64_t nowUs);
  void checkAwaiting(uint64_t nowUs);
  void delivered(Job& job);
  bool startNextJob(uint64_t nowUs);
  Job* nextJob(bool uplink, uint64_t nowUs);
  bool readyForTrain(Job& job, uint64_t nowUs);
  void sendUplink(Job& job);
  void sendDownlink(Job& job);
  bool sendTrain(Job& job);
  void await(Job& job);
  uint16_t bridgePhase(uint64_t nowUs) const;
  uint16_t devicePhase(const uint8_t* device, uint64_t nowUs) const;

  RadioPort& radio_;
  LinkMonitor& monitor_;
  bool proxyMode_ = false;
  Role role_ = Role::Single;
  PeerSink* peer_ = nullptr;
  bool peerCapacityKnown_ = false;
  Capacity peerCapacity_ = {};
  uint64_t peerCapacityUs_ = 0;
  ProxyCounters counters_;
  DutyCycleLimiter duty_;
  AnchorTable anchors_;
  FrameDeduplicator dedup_;
  ExpiringHashes<64> accepted_;
  // ACKs of devices to the bridge, heard at the device's own signal strength. The frame
  // they answer may reach this node from the other one a few milliseconds later.
  ExpiringHashes<32> answeredDirectly_;
  bool peerAnswerKnown_ = false;
  float peerAnswerRssiDbm_ = 0.0f;
  Job jobs_[kMaxJobs] = {};
  uint32_t nextOrder_ = 0;
  Job* awaiting_ = nullptr;
  uint64_t awaitDeadlineUs_ = 0;
  PendingAck acks_[kMaxPendingAcks] = {};
  size_t ackHead_ = 0;
  size_t ackCount_ = 0;
  bool wakeSeen_ = false;
  uint64_t lastWakeUs_ = 0;
  bool bridgePhaseKnown_ = false;
  uint16_t bridgePhaseValue_ = 0;
  uint64_t bridgePhaseSeenUs_ = 0;
  bool advertisementSeen_ = false;
  uint64_t lastAdvertisementUs_ = 0;
  bool solicitSeen_ = false;
  uint64_t lastSolicitUs_ = 0;
  uint16_t latencyUs_[kLatencySamples] = {};
  size_t latencyCount_ = 0;
  size_t latencyNext_ = 0;
};

}  // namespace tadogado

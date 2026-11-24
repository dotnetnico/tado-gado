// Measures every radio link. Receive only.
//
// All storage is fixed in size, so the radio task never allocates.
#pragma once

#include <cstddef>
#include <cstdint>

#include "dedup.h"
#include "duty_cycle.h"
#include "frames.h"
#include "link_window.h"
#include "rx_assembler.h"

namespace tadogado {

// Which frames of a proxied device go through the proxy. The values are kept in flash
// and sent to the other node, so they never change.
enum class ProxyDirection : uint8_t {
  Both = 0,
  ToValve = 1,   // Only the bridge's frames to the device; its own go directly.
  ToBridge = 2,  // Only the device's own frames; the bridge's go directly.
};
constexpr uint8_t kProxyDirectionLast = static_cast<uint8_t>(ProxyDirection::ToBridge);

constexpr size_t kMaxDevices = 32;
constexpr size_t kRecentFrames = 25;
// Room for the frames of three seconds, the time a pair waits for the other node.
constexpr size_t kMaxObserved = 48;
constexpr size_t kMaxProxied = 16;
constexpr size_t kHeardAckCapacity = 32;
// A wake-up train longer than this was aimed at a device whose sample time the sender
// did not know. A train for a known sample takes about 80 ms.
constexpr uint64_t kLongTrainUs = 200000;

struct DeviceLink {
  bool used;
  uint8_t prefix[kPrefixLen];
  uint32_t frames;
  float rssiDbm;
  float lqi;
  uint64_t lastSeenUs;
  uint32_t uplinks;
  uint32_t uplinksAcked;
  uint32_t uplinksUnacked;
  uint32_t downlinks;
  uint32_t downlinksAcked;
  uint32_t downlinksUnacked;
  uint64_t lastDownlinkUs;
  uint32_t retransmissions;
  uint32_t proxiedUplinks;
  uint32_t proxiedDownlinks;
  uint32_t deliveryFailures;
  uint32_t longTrainAcked;
  uint32_t longTrainUnacked;
  uint32_t ackedByPeer;        // Answers that only the other node of a pair heard.
  uint32_t alreadyAnswered;    // Frames the valve node got to deliver, which the device had already.
  HourlyAirtime<300> airtime;  // What this node spent on the device.
  LinkBucket hour[kHourBuckets];
  LinkBucket day[kDayBuckets];
};

// An ACK this node heard, for the other node of a pair.
struct HeardAck {
  uint8_t device[kPrefixLen];
  uint8_t seq;
  bool uplink;
};

// Airtime of wake-up trains, by who sent them.
struct TrainTotals {
  uint64_t bridgeUs;   // Trains followed by a frame of the bridge.
  uint64_t peerUs;     // Trains of the other node of a pair.
  uint64_t unknownUs;  // Trains without a frame this node heard after them.
};

struct RecentFrame {
  uint64_t timestampUs;
  FrameClass frameClass;
  uint8_t length;
  float rssiDbm;
  uint8_t lqi;
  bool hasAddresses;
  bool hasDestination;
  uint8_t src[kPrefixLen];
  uint8_t dst[kPrefixLen];
  bool hasSeq;
  uint8_t seq;
};

struct Counters {
  uint32_t framesRx = 0;
  uint32_t crcErrors = 0;
  uint32_t fcsErrors = 0;
  uint32_t wakeups = 0;
  uint32_t echoes = 0;
  uint32_t verifiedAcks = 0;
  uint32_t devicesNotTracked = 0;
};

class LinkMonitor {
 public:
  void onFrame(const RxFrame& rx);
  void tick(uint64_t nowUs);

  // The hour and day windows start empty again, from `nowUs`. The proxy does this when
  // it starts or stops, so a window never mixes the two modes.
  void resetWindows(uint64_t nowUs);
  uint64_t windowsSinceUs() const { return windowsSinceUs_; }
  LinkTotals hourTotals(const DeviceLink& device) const;
  LinkTotals dayTotals(const DeviceLink& device) const;
  TrainTotals hourTrains() const;
  TrainTotals dayTrains() const;

  const Counters& counters() const { return counters_; }
  const FrameDeduplicator& dedup() const { return dedup_; }
  const DeviceLink* devices() const { return devices_; }
  // Most recent first; index 0 is the newest of recentCount() frames.
  const RecentFrame& recent(size_t index) const;
  size_t recentCount() const { return recentCount_; }
  bool hasBridge() const { return hasBridge_; }
  const uint8_t* bridge() const { return bridge_; }
  void setBridge(const uint8_t* prefix);

  // The devices named for the proxy. While it runs, their ACK rates stop being
  // measured: the proxy answers them itself, and is deaf while it does. A device can be
  // proxied in one direction only; the other direction goes directly.
  bool setProxied(const uint8_t* prefix, bool enabled, ProxyDirection direction = ProxyDirection::Both);
  bool isProxied(const uint8_t* prefix) const;
  // True if the proxy answers the device's own frames.
  bool proxiesUplink(const uint8_t* prefix) const;
  // True if the proxy answers the bridge's frames to the device.
  bool proxiesDownlink(const uint8_t* prefix) const;
  // True if any device has the bridge's frames go through the proxy.
  bool proxiesAnyDownlink() const;
  void clearProxied() { proxiedCount_ = 0; }
  size_t proxiedCount() const { return proxiedCount_; }
  const uint8_t* proxied(size_t index) const { return proxied_[index]; }
  ProxyDirection proxiedDirection(size_t index) const { return proxiedDirection_[index]; }
  // In a pair both nodes answer on behalf of others, so an ACK to or from a proxied
  // device may be the other node's, and says nothing of the device or the bridge.
  void setProxyActive(bool active, bool pair) {
    proxyActive_ = active;
    pairActive_ = active && pair;
  }

  // A frame given to the other node of a pair, which will send it on. Hearing the same
  // bytes again is always that node: a sender never repeats its frame counter.
  void noteHandedOver(const uint8_t* frame, size_t length, uint64_t nowUs);
  bool sentByOtherNode(const uint8_t* frame, size_t length, uint64_t nowUs) const;

  DeviceLink* mutableLink(const uint8_t* prefix) { return link(prefix); }
  // nullptr if the device was never heard.
  const DeviceLink* findLink(const uint8_t* prefix) const;

  // Two nodes hear different ACKs: one is near the bridge, the other near the valves. In
  // a pair each tells the other what it heard, and a frame counts as answered if either
  // heard the answer. A frame without an answer waits for the other node's news.
  void setPeerLinked(bool linked) { peerLinked_ = linked; }
  bool peerLinked() const { return peerLinked_; }
  // Moves the ACKs heard since the last call to `out`. Returns how many.
  size_t takeHeardAcks(HeardAck* out, size_t capacity);
  void notePeerAck(const uint8_t* device, uint8_t seq, bool uplink, uint64_t nowUs);

 private:
  struct Observed {
    bool used;
    bool waitingForPeer;
    bool longTrain;
    uint8_t device[kPrefixLen];
    uint8_t seq;
    bool uplink;
    uint64_t timestampUs;
  };

  // The wake-up train being heard. It ends with the frame it announces.
  struct Train {
    bool open;
    bool claimed;
    uint8_t seq;
    uint8_t dstShort[2];
    uint64_t startUs;
    uint64_t lastUs;
  };

  enum class TrainSender : uint8_t { Bridge, Peer, Unknown };

  void onWakeup(const RxFrame& rx, const WakeUp& wake);
  void onData(const RxFrame& rx, const DataFrame& data);
  void onEnhAck(const RxFrame& rx, const EnhAck& ack);
  void onBroadcast(const RxFrame& rx, const BroadcastFrame& broadcast);
  void learnBridge(const RxFrame& rx, const DataFrame& data);
  // The ACK of one frame, heard here or by the other node.
  void acknowledge(const uint8_t* device, uint8_t seq, bool uplink, bool byPeer);
  void settle(Observed& entry, bool acked, bool byPeer);
  void observe(const uint8_t* device, uint8_t seq, bool uplink, bool longTrain, uint64_t nowUs);
  // True if this node cannot tell the ACK of the device or bridge from the proxy's.
  bool ambiguousAck(const uint8_t* device, bool uplink) const;
  void rememberHeardAck(const uint8_t* device, uint8_t seq, bool uplink);
  // Ends the open train, if a frame with `seq` follows it. Returns the train's length,
  // or 0 if it was not its train.
  uint64_t claimTrain(const RxFrame& rx, uint8_t seq, TrainSender sender, const uint8_t* src, const uint8_t* dst);
  void closeTrain();
  void countTrain(TrainSender sender, DeviceLink* from, DeviceLink* to, uint64_t airtimeUs);
  DeviceLink* deviceWithShort(const uint8_t* dstShort);
  void advanceWindows(uint64_t nowUs);
  void count(DeviceLink* device, uint16_t LinkBucket::* field);
  void addTime(DeviceLink* device, uint32_t LinkBucket::* field, uint64_t airtimeUs);
  DeviceLink* link(const uint8_t* prefix);
  void heard(DeviceLink* device, const RxFrame& rx);
  void remember(const RxFrame& rx, FrameClass frameClass, const uint8_t* src, const uint8_t* dst, const uint8_t* seq);

  Counters counters_;
  FrameDeduplicator dedup_;
  DeviceLink devices_[kMaxDevices] = {};
  Observed observed_[kMaxObserved] = {};
  RecentFrame recent_[kRecentFrames] = {};
  size_t recentNext_ = 0;
  size_t recentCount_ = 0;
  bool hasBridge_ = false;
  uint8_t bridge_[kPrefixLen] = {};
  uint8_t proxied_[kMaxProxied][kPrefixLen] = {};
  ProxyDirection proxiedDirection_[kMaxProxied] = {};
  size_t proxiedCount_ = 0;
  bool proxyActive_ = false;
  bool pairActive_ = false;
  // The other node holds a frame while its radio is busy, and tries three times.
  static constexpr uint64_t kHandedOverTtlUs = 300000000;
  ExpiringHashes<128> handedOver_;
  bool wakeSeen_ = false;
  uint64_t lastWakeUs_ = 0;
  uint8_t lastWakeSeq_ = 0;
  static constexpr uint8_t kBridgeVotes = 2;
  uint8_t bridgeCandidate_[kPrefixLen] = {};
  uint8_t bridgeCandidateSeq_ = 0;
  uint8_t bridgeVotes_ = 0;

  uint64_t nowUs_ = 0;  // The latest time seen, for choosing a device to forget.
  Train train_ = {};
  BucketClock<kHourBucketUs, kHourBuckets> hourClock_;
  BucketClock<kDayBucketUs, kDayBuckets> dayClock_;
  uint64_t windowsSinceUs_ = 0;
  TrainTotals hourTrains_[kHourBuckets] = {};
  TrainTotals dayTrains_[kDayBuckets] = {};

  bool peerLinked_ = false;
  // The other node reports its ACKs in batches, a few times a second.
  static constexpr uint64_t kPeerAckTtlUs = 10000000;
  ExpiringHashes<64> peerAcks_;
  HeardAck heardAcks_[kHeardAckCapacity] = {};
  size_t heardAckNext_ = 0;
  size_t heardAckCount_ = 0;
};

}  // namespace tadogado
